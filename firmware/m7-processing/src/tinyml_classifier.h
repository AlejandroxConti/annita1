#ifndef TINYML_CLASSIFIER_H_
#define TINYML_CLASSIFIER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int tinyml_init(void);
int tinyml_classify_water_land(const uint16_t *rgb565_buf, uint16_t w, uint16_t h,
				float *prob_water_out);

#ifdef __cplusplus
}
#endif

#endif
