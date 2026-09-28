/*
 * Annita-1, Cortex-M4 side: camera frame receiver.
 *
 * Reads the framed JPEG stream coming from the ESP32 on LPUART5 and publishes
 * every frame that passes its checksum to shared memory, where the M7 picks it up.
 *
 * Wire format:  0xAA 0x55 | length (u16, little endian) | JPEG bytes | XOR checksum
 *
 * Shared memory (OCRAM2, base 0x202c0000):
 *   +0x00  magic      0xA5A5A5A5 once the M4 is running, 0xDEAD0002 if LPUART5 is not ready
 *   +0x04  reserved
 *   +0x08  frame_len  length in bytes of the latest valid frame
 *   +0x0C  frame_seq  incremented after every valid frame (the M7 watches this)
 *   +0x100 jpeg       frame data
 *
 * The receiver polls the UART and never sleeps or yields. During bring-up the
 * first thread switch on this core (k_msleep) locked the M4 up, so the main
 * loop is kept free of anything that forces a context switch.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/cache.h>
#include <stdint.h>

#define SHM_BASE          0x202c0000u
#define SHM_MAGIC_ADDR    (SHM_BASE + 0x00)
#define SHM_RESERVED_ADDR (SHM_BASE + 0x04)
#define SHM_FRAMELEN_ADDR (SHM_BASE + 0x08)
#define SHM_FRAMESEQ_ADDR (SHM_BASE + 0x0C)
#define SHM_JPEG_ADDR     (SHM_BASE + 0x100)
#define JPEG_MAX          8000u   /* the ESP32 never sends more than this */

static volatile uint32_t *const shm_magic    = (volatile uint32_t *)SHM_MAGIC_ADDR;
static volatile uint32_t *const shm_reserved = (volatile uint32_t *)SHM_RESERVED_ADDR;
static volatile uint32_t *const shm_framelen = (volatile uint32_t *)SHM_FRAMELEN_ADDR;
static volatile uint32_t *const shm_frameseq = (volatile uint32_t *)SHM_FRAMESEQ_ADDR;
static volatile uint8_t  *const shm_jpeg     = (volatile uint8_t  *)SHM_JPEG_ADDR;

enum parse_state {
	ST_WAIT_AA,
	ST_WAIT_55,
	ST_LEN_LO,
	ST_LEN_HI,
	ST_PAYLOAD,
	ST_CHECKSUM,
};

int main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(lpuart5));

	if (!device_is_ready(uart)) {
		*shm_magic = 0xDEAD0002u;
		sys_cache_data_flush_range((void *)shm_magic, 4);
		for (;;) {
			__asm volatile("nop");
		}
	}

	*shm_frameseq = 0;
	*shm_framelen = 0;
	*shm_reserved = 0;
	sys_cache_data_flush_range((void *)SHM_BASE, 0x10);
	*shm_magic = 0xA5A5A5A5u;
	sys_cache_data_flush_range((void *)shm_magic, 4);

	enum parse_state state = ST_WAIT_AA;
	uint16_t frame_len = 0;
	uint16_t idx = 0;
	uint8_t checksum = 0;
	uint32_t seq = 0;

	for (;;) {
		unsigned char c;

		if (uart_poll_in(uart, &c) != 0) {
			continue;   /* nothing received, keep polling */
		}

		switch (state) {
		case ST_WAIT_AA:
			if (c == 0xAA) {
				state = ST_WAIT_55;
			}
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
			if (frame_len == 0 || frame_len > JPEG_MAX) {
				state = ST_WAIT_AA;   /* bad length, resync on the next header */
			} else {
				idx = 0;
				checksum = 0;
				state = ST_PAYLOAD;
			}
			break;

		case ST_PAYLOAD:
			shm_jpeg[idx++] = c;
			checksum ^= c;
			if (idx >= frame_len) {
				state = ST_CHECKSUM;
			}
			break;

		case ST_CHECKSUM:
			if (c == checksum) {
				/* Publish: data first, then length, then the sequence number. */
				sys_cache_data_flush_range((void *)shm_jpeg, frame_len);
				*shm_framelen = frame_len;
				seq++;
				*shm_frameseq = seq;
				sys_cache_data_flush_range((void *)shm_framelen, 4);
				sys_cache_data_flush_range((void *)shm_frameseq, 4);
			}
			/* On a checksum mismatch the frame is dropped. */
			state = ST_WAIT_AA;
			break;
		}
	}

	return 0;
}
