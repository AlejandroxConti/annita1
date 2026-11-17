#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/cache.h>
#include <stdint.h>
#include "fsl_cache.h"

#define SHM_BASE      0x202c0000u
#define SHM_MAGIC     (SHM_BASE + 0x00)
#define SHM_LEN       (SHM_BASE + 0x08)
#define SHM_SEQ       (SHM_BASE + 0x0C)
#define SHM_IDX       (SHM_BASE + 0x10)
#define SHM_CNT       (SHM_BASE + 0x14)   /* 6 x uint32 */
#define SHM_BUF0      (SHM_BASE + 0x100)
#define SHM_BUF1      (SHM_BASE + 0x2100)
#define SHM_JPEG_MAX  8000u

#define W32(a) (*(volatile uint32_t *)(a))

enum { C_RX, C_OR, C_FE, C_BADCS, C_BADLEN, C_RESYNC, C_N };
static uint32_t cnt[C_N];

enum parse_state { ST_WAIT_AA, ST_WAIT_55, ST_LEN_LO, ST_LEN_HI, ST_PAYLOAD, ST_CHECKSUM };

static void publish_counters(void)
{
	for (int i = 0; i < C_N; i++) {
		W32(SHM_CNT + 4 * i) = cnt[i];
	}
	sys_cache_data_flush_range((void *)SHM_CNT, 4 * C_N);
}

int main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(lpuart5));

	if (!device_is_ready(uart)) {
		W32(SHM_MAGIC) = 0xDEAD0002u;
		sys_cache_data_flush_range((void *)SHM_MAGIC, 4);
		for (;;) { __asm volatile("nop"); }
	}

	W32(SHM_SEQ) = 0;
	W32(SHM_LEN) = 0;
	W32(SHM_IDX) = 0;
	publish_counters();
	sys_cache_data_flush_range((void *)SHM_BASE, 0x20);
	W32(SHM_MAGIC) = 0xA5A5A5A6u;   /* 0xA5A5A5A6 = layout ping-pong */
	L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)SHM_BASE, 0x40u);
	__DSB();
	sys_cache_data_flush_range((void *)SHM_MAGIC, 4);

	enum parse_state state = ST_WAIT_AA;
	uint16_t frame_len = 0, idx = 0;
	uint8_t checksum = 0;
	uint32_t seq = 0;
	uint32_t pub = 0;                 /* buffer publicado */
	volatile uint8_t *wbuf = (volatile uint8_t *)SHM_BUF1;  /* buffer de escritura */

	for (;;) {
		unsigned char c;

		int e = uart_err_check(uart);   /* limpia OR/FE/NF/PF */
		if (e) {
			if (e & UART_ERROR_OVERRUN) { cnt[C_OR]++; }
			if (e & (UART_ERROR_FRAMING | UART_ERROR_PARITY)) { cnt[C_FE]++; }
			cnt[C_RESYNC]++;
			state = ST_WAIT_AA;
			publish_counters();
		}

		if (uart_poll_in(uart, &c) != 0) {
			continue;
		}
		cnt[C_RX]++;

		switch (state) {
		case ST_WAIT_AA:
			if (c == 0xAA) state = ST_WAIT_55;
			break;
		case ST_WAIT_55:
			state = (c == 0x55) ? ST_LEN_LO : ST_WAIT_AA;
			break;
		case ST_LEN_LO:
			frame_len = c;
			state = ST_LEN_HI;
			break;
		case ST_LEN_HI:
			frame_len |= ((uint16_t)c) << 8;
			if (frame_len == 0 || frame_len > SHM_JPEG_MAX) {
				cnt[C_BADLEN]++;
				state = ST_WAIT_AA;
			} else {
				idx = 0;
				checksum = 0;
				state = ST_PAYLOAD;
			}
			break;
		case ST_PAYLOAD:
			wbuf[idx++] = c;
			checksum ^= c;
			if (idx >= frame_len) state = ST_CHECKSUM;
			break;
		case ST_CHECKSUM:
			if (c == checksum) {
				sys_cache_data_flush_range((void *)wbuf, frame_len);
				pub ^= 1u;                       /* el recien escrito pasa a publicado */
				W32(SHM_LEN) = frame_len;
				W32(SHM_IDX) = pub;
				publish_counters();
				sys_cache_data_flush_range((void *)SHM_LEN, 4);
				sys_cache_data_flush_range((void *)SHM_IDX, 4);
				/* 1) datos + metadatos a RAM antes de publicar seq */
				L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)SHM_BASE, 0x4200u);
				__DSB();
				seq++;
				W32(SHM_SEQ) = seq;              /* ultimo */
				sys_cache_data_flush_range((void *)SHM_SEQ, 4);
				/* 2) publicar seq */
				L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)SHM_BASE, 0x20u);
				__DSB();
				/* proximo frame va al otro buffer */
				wbuf = (volatile uint8_t *)(pub ? SHM_BUF0 : SHM_BUF1);
			} else {
				cnt[C_BADCS]++;
			}
			state = ST_WAIT_AA;
			break;
		}
	}
	return 0;
}
