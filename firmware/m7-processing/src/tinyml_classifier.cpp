#include "tinyml_classifier.h"

#include <zephyr/kernel.h>

#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include "tflite_model/water_land_model_data.h"

#define MODEL_IMG_SIZE 64

namespace {
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;
TfLiteTensor *output = nullptr;

constexpr int kTensorArenaSize = 56 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

bool initialized = false;
}

extern "C" int tinyml_init(void)
{
	model = tflite::GetModel(g_water_land_model);

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		MicroPrintf("Model schema version %d, expected %d",
			    model->version(), TFLITE_SCHEMA_VERSION);
		return -1;
	}

	static tflite::MicroMutableOpResolver<5> resolver;
	resolver.AddConv2D();
	resolver.AddMaxPool2D();
	resolver.AddMean();
	resolver.AddFullyConnected();
	resolver.AddLogistic();

	static tflite::MicroInterpreter static_interpreter(
		model, resolver, tensor_arena, kTensorArenaSize);
	interpreter = &static_interpreter;

	TfLiteStatus allocate_status = interpreter->AllocateTensors();

	if (allocate_status != kTfLiteOk) {
		MicroPrintf("AllocateTensors() failed");
		return -1;
	}

	input = interpreter->input(0);
	output = interpreter->output(0);

	MicroPrintf("TinyML ready. Arena used: %d of %d bytes",
		    (int)interpreter->arena_used_bytes(), kTensorArenaSize);

	initialized = true;
	return 0;
}

extern "C" int tinyml_classify_water_land(
	const uint16_t *rgb565_buf,
	uint16_t w,
	uint16_t h,
	float *prob_water_out)
{
	if (!initialized) {
		return -1;
	}

	for (int oy = 0; oy < MODEL_IMG_SIZE; oy++) {
		int iy = (oy * h) / MODEL_IMG_SIZE;

		for (int ox = 0; ox < MODEL_IMG_SIZE; ox++) {
			int ix = (ox * w) / MODEL_IMG_SIZE;
			uint16_t px = rgb565_buf[iy * w + ix];

			uint8_t r8 =
				(uint8_t)(((px >> 11) & 0x1F) << 3);
			uint8_t g8 =
				(uint8_t)(((px >> 5) & 0x3F) << 2);
			uint8_t b8 =
				(uint8_t)((px & 0x1F) << 3);

			int idx =
				(oy * MODEL_IMG_SIZE + ox) * 3;

			/*
			 * Input scale ~= 1/255, zero_point = -128:
			 * quantizing (pixel / 255) to int8 is
			 * equivalent to (pixel - 128).
			 */
			input->data.int8[idx + 0] =
				(int8_t)((int)r8 - 128);
			input->data.int8[idx + 1] =
				(int8_t)((int)g8 - 128);
			input->data.int8[idx + 2] =
				(int8_t)((int)b8 - 128);
		}
	}

	TfLiteStatus invoke_status = interpreter->Invoke();

	if (invoke_status != kTfLiteOk) {
		MicroPrintf("Invoke() failed");
		return -1;
	}

	int8_t out_q = output->data.int8[0];

	float prob_water =
		(out_q - output->params.zero_point) *
		output->params.scale;

	*prob_water_out = prob_water;

	return 0;
}
