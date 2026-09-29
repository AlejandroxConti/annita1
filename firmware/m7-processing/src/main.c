#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_mcux_pxp.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "tjpgd/tjpgd.h"
#include "tinyml_classifier.h"

#define LINK_UART_NODE DT_NODELABEL(lpuart5)
#define PXP_NODE      DT_NODELABEL(pxp)

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

#define MAX_PAYLOAD    8192
#define MAX_DECODED_W  480
#define MAX_DECODED_H  320

typedef enum {
	STATE_WAIT_SYNC1,
	STATE_WAIT_SYNC2,
	STATE_WAIT_LEN_LOW,
	STATE_WAIT_LEN_HIGH,
	STATE_READ_PAYLOAD,
	STATE_WAIT_CHECKSUM
} rx_state_t;

/* -------------------------------------------------------------------------- */
/* UART frame parser                                                          */
/* -------------------------------------------------------------------------- */

static rx_state_t rx_state = STATE_WAIT_SYNC1;

static uint8_t payload[MAX_PAYLOAD];
static uint16_t payload_len;
static uint16_t payload_idx;
static uint8_t running_checksum;

static uint32_t frames_ok;
static uint32_t frames_bad_checksum;
static uint32_t frames_decoded;
static uint32_t frames_decode_failed;
static uint32_t frames_dropped_busy;

static volatile bool frame_ready;
static volatile bool processing_frame;

static uint8_t decode_buf[MAX_PAYLOAD];
static uint16_t decode_len;

/*
 * Protocol:
 *
 *   0xAA 0x55 | length_low | length_high | JPEG payload | checksum
 *
 * The checksum is the 8-bit sum of all JPEG payload bytes.
 */
static void parser_reset(void)
{
	rx_state = STATE_WAIT_SYNC1;
	payload_len = 0;
	payload_idx = 0;
	running_checksum = 0;
}

static void parser_process_byte(uint8_t byte)
{
	switch (rx_state) {
	case STATE_WAIT_SYNC1:
		if (byte == SYNC_BYTE_1) {
			rx_state = STATE_WAIT_SYNC2;
		}
		break;

	case STATE_WAIT_SYNC2:
		if (byte == SYNC_BYTE_2) {
			rx_state = STATE_WAIT_LEN_LOW;
		} else if (byte == SYNC_BYTE_1) {
			rx_state = STATE_WAIT_SYNC2;
		} else {
			rx_state = STATE_WAIT_SYNC1;
		}
		break;

	case STATE_WAIT_LEN_LOW:
		payload_len = byte;
		rx_state = STATE_WAIT_LEN_HIGH;
		break;

	case STATE_WAIT_LEN_HIGH:
		payload_len |= ((uint16_t)byte << 8);

		if (payload_len == 0 || payload_len > MAX_PAYLOAD) {
			parser_reset();
			break;
		}

		payload_idx = 0;
		running_checksum = 0;
		rx_state = STATE_READ_PAYLOAD;
		break;

	case STATE_READ_PAYLOAD:
		payload[payload_idx++] = byte;
		running_checksum = (uint8_t)(running_checksum + byte);

		if (payload_idx >= payload_len) {
			rx_state = STATE_WAIT_CHECKSUM;
		}
		break;

	case STATE_WAIT_CHECKSUM:
		if (byte == running_checksum) {
			frames_ok++;

			if (!frame_ready && !processing_frame) {
				memcpy(decode_buf, payload, payload_len);
				decode_len = payload_len;
				frame_ready = true;
			} else {
				frames_dropped_busy++;
			}
		} else {
			frames_bad_checksum++;
		}

		parser_reset();
		break;

	default:
		parser_reset();
		break;
	}
}

static void uart_rx_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	uint8_t rx_buf[32];

	while (uart_irq_rx_ready(dev)) {
		int len = uart_fifo_read(dev, rx_buf, sizeof(rx_buf));

		if (len <= 0) {
			break;
		}

		for (int i = 0; i < len; i++) {
			parser_process_byte(rx_buf[i]);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Image buffers                                                              */
/* -------------------------------------------------------------------------- */

static uint8_t work_pool[TJPGD_WORKSPACE_SIZE];

static uint16_t out_buf[MAX_DECODED_W * MAX_DECODED_H]
	__attribute__((aligned(64)));

static uint16_t pxp_out_buf[MAX_DECODED_W * MAX_DECODED_H]
	__attribute__((aligned(64)));

static uint16_t out_w;
static uint16_t out_h;

static uint8_t luma_buf[MAX_DECODED_W * MAX_DECODED_H];

static volatile bool pxp_done;

/* -------------------------------------------------------------------------- */
/* JPEG decoder                                                               */
/* -------------------------------------------------------------------------- */

typedef struct {
	uint8_t *data;
	size_t size;
	size_t pos;
} jpeg_src_t;

static size_t jd_input_func(JDEC *jd, uint8_t *buf, size_t nbyte)
{
	jpeg_src_t *src = (jpeg_src_t *)jd->device;

	size_t avail = src->size - src->pos;
	size_t n = (nbyte < avail) ? nbyte : avail;

	if (buf != NULL) {
		memcpy(buf, src->data + src->pos, n);
	}

	src->pos += n;

	return n;
}

static int jd_output_func(JDEC *jd, void *bitmap, JRECT *rect)
{
	uint16_t *src = (uint16_t *)bitmap;
	uint16_t bw = rect->right - rect->left + 1;

	for (uint16_t y = rect->top;
	     y <= rect->bottom && y < MAX_DECODED_H;
	     y++) {

		for (uint16_t x = rect->left;
		     x <= rect->right && x < MAX_DECODED_W;
		     x++) {

			out_buf[y * out_w + x] =
				src[(y - rect->top) * bw + (x - rect->left)];
		}
	}

	return 1;
}

/* -------------------------------------------------------------------------- */
/* PXP                                                                        */
/* -------------------------------------------------------------------------- */

static void pxp_callback(const struct device *dev,
			 void *user_data,
			 uint32_t channel,
			 int status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);

	pxp_done = true;
}

static int run_pxp_rotate180(const struct device *pxp_dev,
			     uint16_t w,
			     uint16_t h)
{
	struct dma_config cfg = {0};
	struct dma_block_config block = {0};

	int ret;
	int64_t t0;
	int64_t t1;

	size_t image_size = (size_t)w * h * sizeof(uint16_t);

	/*
	 * The CPU produced out_buf. Flush it before the DMA/PXP reads it.
	 */
	sys_cache_data_flush_range(out_buf, image_size);

	/*
	 * The PXP writes pxp_out_buf. Invalidate the CPU cache after DMA
	 * completion before the CPU accesses the resulting image.
	 */
	sys_cache_data_flush_range(pxp_out_buf, image_size);

	block.source_address = (uint32_t)out_buf;
	block.dest_address = (uint32_t)pxp_out_buf;
	block.block_size = (uint32_t)image_size;

	cfg.channel_direction = MEMORY_TO_MEMORY;
	cfg.dma_slot =
		DMA_MCUX_PXP_FMT(DMA_MCUX_PXP_FMT_RGB565) |
		DMA_MCUX_PXP_CMD(DMA_MCUX_PXP_CMD_ROTATE_180);

	cfg.linked_channel = DMA_MCUX_PXP_FLIP(DMA_MCUX_PXP_FLIP_DISABLE);

	cfg.source_data_size = (uint32_t)w * 2;
	cfg.source_burst_length = h;

	cfg.dest_data_size = (uint32_t)w * 2;
	cfg.dest_burst_length = h;

	cfg.head_block = &block;
	cfg.dma_callback = pxp_callback;

	pxp_done = false;

	t0 = k_uptime_get();

	ret = dma_config(pxp_dev, 0, &cfg);
	if (ret != 0) {
		printk("ERROR: PXP dma_config failed: %d\n", ret);
		return -1;
	}

	ret = dma_start(pxp_dev, 0);
	if (ret != 0) {
		printk("ERROR: PXP dma_start failed: %d\n", ret);
		return -1;
	}

	int timeout = 100;

	while (!pxp_done && timeout > 0) {
		k_msleep(1);
		timeout--;
	}

	t1 = k_uptime_get();

	if (!pxp_done) {
		printk("ERROR: PXP timeout\n");
		return -1;
	}

	sys_cache_data_invd_range(pxp_out_buf, image_size);

	printk("PXP rotate180 OK: %ux%u in %lld ms\n",
	       w, h, (t1 - t0));

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Image quality assessment                                                   */
/* -------------------------------------------------------------------------- */

static void extract_luminance_green(const uint16_t *rgb565_buf,
				    uint8_t *luma_out,
				    uint16_t w,
				    uint16_t h)
{
	uint32_t n = (uint32_t)w * h;

	for (uint32_t i = 0; i < n; i++) {
		uint16_t px = rgb565_buf[i];
		uint8_t g6 = (px >> 5) & 0x3F;

		luma_out[i] = (uint8_t)(g6 << 2);
	}
}

static float compute_laplacian_variance(const uint8_t *luma,
					uint16_t w,
					uint16_t h)
{
	double sum = 0.0;
	double sum_sq = 0.0;
	uint32_t count = 0;

	for (uint16_t y = 1; y < h - 1; y++) {
		for (uint16_t x = 1; x < w - 1; x++) {
			int center = luma[y * w + x];
			int up = luma[(y - 1) * w + x];
			int down = luma[(y + 1) * w + x];
			int left = luma[y * w + (x - 1)];
			int right = luma[y * w + (x + 1)];

			int lap = up + down + left + right - (4 * center);

			sum += lap;
			sum_sq += (double)lap * (double)lap;
			count++;
		}
	}

	if (count == 0) {
		return 0.0f;
	}

	double mean = sum / count;
	double variance = (sum_sq / count) - (mean * mean);

	return (float)variance;
}

static void compute_mean_stddev(const uint8_t *luma,
				uint16_t w,
				uint16_t h,
				float *mean_out,
				float *stddev_out)
{
	uint32_t n = (uint32_t)w * h;

	double sum = 0.0;
	double sum_sq = 0.0;

	for (uint32_t i = 0; i < n; i++) {
		sum += luma[i];
		sum_sq += (double)luma[i] * (double)luma[i];
	}

	double mean = sum / n;
	double variance = (sum_sq / n) - (mean * mean);

	if (variance < 0.0) {
		variance = 0.0;
	}

	*mean_out = (float)mean;
	*stddev_out = (float)sqrt(variance);
}

static float run_image_quality_assessment(uint16_t w, uint16_t h)
{
	int64_t t0;
	int64_t t1;

	float sharpness;
	float mean_luma;
	float contrast;

	t0 = k_uptime_get();

	extract_luminance_green(pxp_out_buf, luma_buf, w, h);

	sharpness = compute_laplacian_variance(
		luma_buf, w, h);

	compute_mean_stddev(
		luma_buf, w, h,
		&mean_luma,
		&contrast);

	t1 = k_uptime_get();

	float sharpness_norm = sharpness / 5.0f;

	if (sharpness_norm > 100.0f) {
		sharpness_norm = 100.0f;
	}

	float contrast_norm = contrast;

	if (contrast_norm > 100.0f) {
		contrast_norm = 100.0f;
	}

	float quality_score =
		(sharpness_norm * 0.6f) +
		(contrast_norm * 0.4f);

	printk("IQA OK: sharpness=%d.%02u contrast=%d.%02u "
	       "mean=%d.%02u quality=%d.%02u in %lld ms\n",
	       (int)sharpness,
	       (unsigned)((sharpness - (int)sharpness) * 100),

	       (int)contrast,
	       (unsigned)((contrast - (int)contrast) * 100),

	       (int)mean_luma,
	       (unsigned)((mean_luma - (int)mean_luma) * 100),

	       (int)quality_score,
	       (unsigned)((quality_score - (int)quality_score) * 100),

	       (t1 - t0));

	return quality_score;
}

/* -------------------------------------------------------------------------- */
/* Feature extraction                                                         */
/* -------------------------------------------------------------------------- */

#define GRID_ROWS 4
#define GRID_COLS 4

static uint32_t hist_r[16];
static uint32_t hist_g[16];
static uint32_t hist_b[16];

static float roi_luma[GRID_ROWS * GRID_COLS];
static float roi_blue_dom[GRID_ROWS * GRID_COLS];

static void extract_rgb565_histograms(const uint16_t *rgb565_buf,
				      uint16_t w,
				      uint16_t h,
				      float *mean_r_out,
				      float *mean_g_out,
				      float *mean_b_out)
{
	uint32_t n = (uint32_t)w * h;

	uint32_t sum_r = 0;
	uint32_t sum_g = 0;
	uint32_t sum_b = 0;

	memset(hist_r, 0, sizeof(hist_r));
	memset(hist_g, 0, sizeof(hist_g));
	memset(hist_b, 0, sizeof(hist_b));

	for (uint32_t i = 0; i < n; i++) {
		uint16_t px = rgb565_buf[i];

		uint8_t r5 = (px >> 11) & 0x1F;
		uint8_t g6 = (px >> 5) & 0x3F;
		uint8_t b5 = px & 0x1F;

		uint8_t r8 = (uint8_t)(r5 << 3);
		uint8_t g8 = (uint8_t)(g6 << 2);
		uint8_t b8 = (uint8_t)(b5 << 3);

		sum_r += r8;
		sum_g += g8;
		sum_b += b8;

		hist_r[r5 >> 1]++;
		hist_g[g6 >> 2]++;
		hist_b[b5 >> 1]++;
	}

	*mean_r_out = (float)sum_r / n;
	*mean_g_out = (float)sum_g / n;
	*mean_b_out = (float)sum_b / n;
}

static float compute_sobel_edge_density(const uint8_t *luma,
					uint16_t w,
					uint16_t h,
					float *mean_magnitude_out)
{
	uint32_t edge_count = 0;
	uint32_t count = 0;

	double sum_mag = 0.0;

	const int threshold = 60;

	for (uint16_t y = 1; y < h - 1; y++) {
		for (uint16_t x = 1; x < w - 1; x++) {
			int p00 = luma[(y - 1) * w + (x - 1)];
			int p01 = luma[(y - 1) * w + x];
			int p02 = luma[(y - 1) * w + (x + 1)];

			int p10 = luma[y * w + (x - 1)];
			int p12 = luma[y * w + (x + 1)];

			int p20 = luma[(y + 1) * w + (x - 1)];
			int p21 = luma[(y + 1) * w + x];
			int p22 = luma[(y + 1) * w + (x + 1)];

			int gx =
				(p02 + 2 * p12 + p22) -
				(p00 + 2 * p10 + p20);

			int gy =
				(p20 + 2 * p21 + p22) -
				(p00 + 2 * p01 + p02);

			int mag =
				(gx < 0 ? -gx : gx) +
				(gy < 0 ? -gy : gy);

			sum_mag += mag;
			count++;

			if (mag > threshold) {
				edge_count++;
			}
		}
	}

	if (count == 0) {
		*mean_magnitude_out = 0.0f;
		return 0.0f;
	}

	*mean_magnitude_out =
		(float)(sum_mag / count);

	return 100.0f *
	       (float)edge_count /
	       (float)count;
}

static void compute_roi_grid(const uint16_t *rgb565_buf,
			     const uint8_t *luma,
			     uint16_t w,
			     uint16_t h)
{
	uint32_t sum_luma[GRID_ROWS * GRID_COLS];
	int32_t sum_blue_dom[GRID_ROWS * GRID_COLS];
	uint32_t count[GRID_ROWS * GRID_COLS];

	memset(sum_luma, 0, sizeof(sum_luma));
	memset(sum_blue_dom, 0, sizeof(sum_blue_dom));
	memset(count, 0, sizeof(count));

	for (uint16_t y = 0; y < h; y++) {
		uint16_t row =
			(uint16_t)((uint32_t)y * GRID_ROWS / h);

		for (uint16_t x = 0; x < w; x++) {
			uint16_t col =
				(uint16_t)((uint32_t)x * GRID_COLS / w);

			uint16_t cell =
				row * GRID_COLS + col;

			uint32_t idx =
				(uint32_t)y * w + x;

			uint16_t px = rgb565_buf[idx];

			uint8_t r8 =
				(uint8_t)(((px >> 11) & 0x1F) << 3);

			uint8_t b8 =
				(uint8_t)((px & 0x1F) << 3);

			sum_luma[cell] += luma[idx];

			sum_blue_dom[cell] +=
				(int32_t)b8 - (int32_t)r8;

			count[cell]++;
		}
	}

	for (int i = 0; i < GRID_ROWS * GRID_COLS; i++) {
		if (count[i] > 0) {
			roi_luma[i] =
				(float)sum_luma[i] / count[i];

			roi_blue_dom[i] =
				(float)sum_blue_dom[i] / count[i];
		} else {
			roi_luma[i] = 0.0f;
			roi_blue_dom[i] = 0.0f;
		}
	}
}

static void run_feature_extraction(uint16_t w, uint16_t h)
{
	int64_t t0;
	int64_t t1;

	float mean_r;
	float mean_g;
	float mean_b;
	float mean_edge_mag;

	t0 = k_uptime_get();

	extract_rgb565_histograms(
		pxp_out_buf,
		w,
		h,
		&mean_r,
		&mean_g,
		&mean_b);

	float edge_density =
		compute_sobel_edge_density(
			luma_buf,
			w,
			h,
			&mean_edge_mag);

	compute_roi_grid(
		pxp_out_buf,
		luma_buf,
		w,
		h);

	t1 = k_uptime_get();

	float roi_blue_dom_avg = 0.0f;

	for (int i = 0;
	     i < GRID_ROWS * GRID_COLS;
	     i++) {

		roi_blue_dom_avg +=
			roi_blue_dom[i];
	}

	roi_blue_dom_avg /=
		(GRID_ROWS * GRID_COLS);

	printk("FEATURES OK: R=%d G=%d B=%d "
	       "edges=%d.%02u%% blue_dom=%d "
	       "in %lld ms\n",
	       (int)mean_r,
	       (int)mean_g,
	       (int)mean_b,

	       (int)edge_density,
	       (unsigned)((edge_density -
			   (int)edge_density) * 100),

	       (int)roi_blue_dom_avg,

	       (t1 - t0));
}

/* -------------------------------------------------------------------------- */
/* SVS decision engine                                                        */
/* -------------------------------------------------------------------------- */

#define MAX_PAYLOAD_FOR_COST 8192

static void run_svs_decision_engine(float quality_score,
				    float prob_water,
				    uint16_t jpeg_len)
{
	float event_score =
		prob_water * 100.0f;

	float cost_score =
		((float)jpeg_len /
		 (float)MAX_PAYLOAD_FOR_COST) *
		100.0f;

	if (cost_score > 100.0f) {
		cost_score = 100.0f;
	}

	float svs =
		(quality_score * 0.4f) +
		(event_score * 0.4f) -
		(cost_score * 0.2f);

	if (svs < 0.0f) {
		svs = 0.0f;
	}

	if (svs > 100.0f) {
		svs = 100.0f;
	}

	const char *decision;

	if (svs < 20.0f) {
		decision = "DISCARD";
	} else if (svs < 50.0f) {
		decision = "COMPRESS";
	} else {
		decision = "PROCESS/TRANSMIT";
	}

	printk("SVS OK: quality=%d.%02u event=%d.%02u "
	       "cost=%d.%02u SVS=%d.%02u -> %s\n",

	       (int)quality_score,
	       (unsigned)((quality_score -
			   (int)quality_score) * 100),

	       (int)event_score,
	       (unsigned)((event_score -
			   (int)event_score) * 100),

	       (int)cost_score,
	       (unsigned)((cost_score -
			   (int)cost_score) * 100),

	       (int)svs,
	       (unsigned)((svs -
			   (int)svs) * 100),

	       decision);

	static uint32_t frames_discarded;
	static uint32_t frames_compressed;
	static uint32_t frames_transmitted;

	static uint32_t bytes_transmitted;
	static uint32_t bytes_saved;

	if (svs < 20.0f) {
		frames_discarded++;
		bytes_saved += jpeg_len;

		printk("ACTION: DISCARDED "
		       "(0 bytes transmitted, %u bytes saved)\n",
		       jpeg_len);

	} else if (svs < 50.0f) {
		frames_compressed++;

		uint32_t bytes_reduced =
			jpeg_len / 2;

		bytes_transmitted +=
			bytes_reduced;

		printk("ACTION: ADDITIONAL COMPRESSION "
		       "(%u bytes -> ~%u simulated bytes)\n",
		       jpeg_len,
		       bytes_reduced);

	} else {
		frames_transmitted++;
		bytes_transmitted += jpeg_len;

		printk("ACTION: TRANSMIT TO GROUND STATION "
		       "(%u bytes)\n",
		       jpeg_len);
	}

	uint32_t total_frames =
		frames_discarded +
		frames_compressed +
		frames_transmitted;

	if ((total_frames % 20) == 0) {
		printk("SUMMARY: %u frames "
		       "(discarded=%u compressed=%u transmitted=%u) "
		       "bytes_transmitted=%u bytes_saved=%u\n",

		       total_frames,
		       frames_discarded,
		       frames_compressed,
		       frames_transmitted,
		       bytes_transmitted,
		       bytes_saved);
	}
}

/* -------------------------------------------------------------------------- */
/* JPEG processing pipeline                                                   */
/* -------------------------------------------------------------------------- */

static void decode_jpeg(const struct device *pxp_dev,
			uint8_t *data,
			uint16_t len)
{
	if (len < 4) {
		printk("ERROR: JPEG frame too short: %u bytes\n", len);
		frames_decode_failed++;
		return;
	}

	printk("JPEG frame received: %u bytes\n", len);

	JDEC jd;

	jpeg_src_t src = {
		.data = data,
		.size = len,
		.pos = 0
	};

	JRESULT res;

	int64_t t0 =
		k_uptime_get();

	res = jd_prepare(
		&jd,
		jd_input_func,
		work_pool,
		sizeof(work_pool),
		&src);

	if (res != JDR_OK) {
		printk("ERROR: JPEG prepare failed: %d\n", res);
		frames_decode_failed++;
		return;
	}

	out_w = jd.width;
	out_h = jd.height;

	if (out_w == 0 ||
	    out_h == 0 ||
	    out_w > MAX_DECODED_W ||
	    out_h > MAX_DECODED_H) {

		printk("ERROR: Unsupported JPEG size: %ux%u\n",
		       out_w,
		       out_h);

		frames_decode_failed++;
		return;
	}

	res = jd_decomp(
		&jd,
		jd_output_func,
		0);

	int64_t t1 =
		k_uptime_get();

	if (res != JDR_OK) {
		frames_decode_failed++;

		printk("ERROR: JPEG decode failed: %d\n",
		       res);

		return;
	}

	frames_decoded++;

	printk("JPEG decoded OK #%u: %ux%u in %lld ms\n",
	       frames_decoded,
	       out_w,
	       out_h,
	       (t1 - t0));

	if (run_pxp_rotate180(
		    pxp_dev,
		    out_w,
		    out_h) != 0) {

		frames_decode_failed++;
		return;
	}

	float quality_score =
		run_image_quality_assessment(
			out_w,
			out_h);

	run_feature_extraction(
		out_w,
		out_h);

	float prob_water = 0.0f;

	int64_t t0_tinyml =
		k_uptime_get();

	int ret_tinyml =
		tinyml_classify_water_land(
			pxp_out_buf,
			out_w,
			out_h,
			&prob_water);

	int64_t t1_tinyml =
		k_uptime_get();

	if (ret_tinyml == 0) {
		const char *class_name =
			(prob_water > 0.5f) ?
			"WATER" :
			"LAND";

		printk("TinyML OK: water_probability=%d.%03u "
		       "-> %s in %lld ms\n",

		       (int)prob_water,

		       (unsigned)((prob_water -
				   (int)prob_water) * 1000),

		       class_name,

		       (t1_tinyml -
			t0_tinyml));
	} else {
		printk("ERROR: TinyML classification failed: %d\n",
		       ret_tinyml);
	}

	run_svs_decision_engine(
		quality_score,
		prob_water,
		len);
}

/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
	const struct device *uart_dev =
		DEVICE_DT_GET(LINK_UART_NODE);

	const struct device *pxp_dev =
		DEVICE_DT_GET(PXP_NODE);

	int ret;

	printk("\n");
	printk("========================================\n");
	printk("ANNITA - M7 Standalone\n");
	printk("========================================\n");

	if (!device_is_ready(uart_dev)) {
		printk("ERROR: LPUART5 is not ready\n");
		return 0;
	}

	if (!device_is_ready(pxp_dev)) {
		printk("ERROR: PXP is not ready\n");
		return 0;
	}

	printk("LPUART5: ready\n");
	printk("PXP: ready\n");

	ret = tinyml_init();

	if (ret != 0) {
		printk("ERROR: TinyML initialization failed: %d\n",
		       ret);
		return 0;
	}

	printk("TinyML: ready\n");

	ret = uart_irq_callback_user_data_set(
		uart_dev,
		uart_rx_callback,
		NULL);

	if (ret != 0) {
		printk("ERROR: UART callback setup failed: %d\n",
		       ret);
		return 0;
	}

	parser_reset();

	uart_irq_rx_enable(uart_dev);

	printk("UART RX: enabled\n");
	printk("Protocol: AA 55 + length + JPEG + checksum\n");
	printk("M7 standalone pipeline ready\n");

	while (1) {
		if (frame_ready && !processing_frame) {
			/*
			 * Claim the single frame buffer before starting
			 * processing. The UART ISR will drop complete frames
			 * while processing_frame is true, preventing it from
			 * overwriting decode_buf.
			 */
			unsigned int key =
				irq_lock();

			if (frame_ready &&
			    !processing_frame) {

				processing_frame = true;
				frame_ready = false;

				uint16_t len =
					decode_len;

				irq_unlock(key);

				decode_jpeg(
					pxp_dev,
					decode_buf,
					len);

				processing_frame = false;
			} else {
				irq_unlock(key);
			}
		}

		k_msleep(1);
	}

	return 0;
}
