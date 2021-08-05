/* An attempt at creating C extensions to TF-Micro in order to get it to run
on a Crazyflie 2.0. The reason for this is because all of the firmware for
the crazyflie is written in C, and although we can compile TF-Micro for the
Cortex M4, we still have to link it to the main loop in the firmware.

Approach will be to expose the common functions we need in C, compile TF Micro
in C++, and then link it to the main loop later.
==============================================================================*/

#include "tensorflow/lite/experimental/micro/examples/micro_speech/recognize_commands.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "machinelearning.h"

// Our machine learning models we're putting in :)
#include "tfmicro_models.h"

#define DEBUG_MODULE "ML"

extern "C" {
	#include "stm32fxxx.h"
	#include "FreeRTOS.h"

	#include "debug.h"
}

extern "C" {
	/**
	 * TfLiteModel wrapper functions
	 */
	const CTfLiteModel * CTfLiteModel_create() {
		return reinterpret_cast<const CTfLiteModel*>(
			::tflite::GetModel(TFMICRO_MODEL));
	}
	void CTfLiteModel_destroy(CTfLiteModel* v) {
		delete reinterpret_cast<tflite::Model*>(v);
	}
	
	int CTfLiteModel_version(CTfLiteModel* v) {
		return static_cast<int>(reinterpret_cast<tflite::Model*>(v)->version());
	}	
	
	/**
	 * TF Micro Interpreter wrapper functions
	 */
	int CTfInterpreter_create_return_version(const CTfLiteModel* c_model, int alloc_size) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		uint8_t tensor_arena[alloc_size];
		tflite::SimpleTensorAllocator tensor_allocator(tensor_arena, alloc_size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);
		return model_input->dims->size;
	}
	
	int CTfLiteModel_dimensions(const CTfLiteModel* c_model, uint8_t* arena, size_t size, int dim) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		tflite::SimpleTensorAllocator tensor_allocator(arena, size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);
		return model_input->dims->data[dim];
	}

	/* Actual methods used for inferencing */
	void CTfInterpreter_simple_fc(const CTfLiteModel* c_model, 
			uint8_t* tensor, int alloc_size, uint8_t* input, int* result) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		tflite::SimpleTensorAllocator tensor_allocator(tensor, alloc_size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);

		memcpy(model_input->data.uint8, input, 20 * sizeof(uint8_t));

		TfLiteStatus invoke_status = interpreter.Invoke();
		if (invoke_status != kTfLiteOk) {
			return;
		}

		TfLiteTensor* output = interpreter.output(0);

		int NUM_CLASSES = 3;
		for (int i = 0; i < NUM_CLASSES; i++) {

			result[i] = static_cast<int>(output->data.uint8[i]);
		}
	}

	void CTfInterpreter_simple_conv(const CTfLiteModel* c_model, uint8_t* tensor, size_t alloc_size,
		model_type* input, size_t input_size, model_type* result, size_t result_size) {
		
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		tflite::SimpleTensorAllocator tensor_allocator(tensor, alloc_size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);

		memcpy(model_input->data.uint8, input, input_size * sizeof(model_type));

		TfLiteStatus invoke_status = interpreter.Invoke();
		if (invoke_status != kTfLiteOk) {
			return;
		}

		TfLiteTensor* output = interpreter.output(0);
		memcpy(result, output->data.uint8, result_size * sizeof(model_type));
	}
}


extern "C" int machine_learning_test(int n) {
	tflite::MicroErrorReporter micro_error_reporter;
	tflite::ErrorReporter* error_reporter = &micro_error_reporter;

	DEBUG_PRINT("Test ML Network.\n");

	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	const tflite::Model* model = ::tflite::GetModel(TFMICRO_MODEL);
	DEBUG_PRINT("Loaded in model small_mnist_model.\n");
	DEBUG_PRINT("Model Version: %d\n", model->version());
	DEBUG_PRINT("Model Description: %s\n", model->description());

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		error_reporter->Report(
				"Schema version %d not equal to supported version %d.\n",
				model->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}

	// This pulls in all the operation implementations we need.
	tflite::ops::micro::AllOpsResolver resolver;

	// Create an area of memory to use for input, output, and intermediate arrays.
	// The size of this will depend on the model you're using, and may need to be
	// determined by experimentation.
	const int tensor_arena_size = 1024 * 5;
	DEBUG_PRINT("Allocate tensor arena of %d bytes\n", tensor_arena_size);
	
	uint8_t* tensor_arena = (uint8_t *) pvPortMalloc(tensor_arena_size * sizeof(uint8_t));

	tflite::SimpleTensorAllocator tensor_allocator(tensor_arena, tensor_arena_size);
	DEBUG_PRINT("Allocated the tensors.\n");

	// Build an interpreter to runtiny_mnist_model the model with.
	DEBUG_PRINT("Attempting to build the interpreter.\n");
	tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
	DEBUG_PRINT("The interpreter has been built.\n");
	
	TfLiteTensor* model_input = interpreter.input(0);
	// Get information about the memory area to use for the model's input.
	DEBUG_PRINT("Dims->size: %d\n", model_input->dims->size);
	DEBUG_PRINT("Dims->data[0]: %d\n", model_input->dims->data[0]);
	DEBUG_PRINT("Dims->data[1]: %d\n", model_input->dims->data[1]);
	DEBUG_PRINT("Dims->data[2]: %d\n", model_input->dims->data[2]);

	// Run the model on the spectrogram input and make sure it succeeds.
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk) {
		error_reporter->Report("Invoke failed");
		return 1;
	}

	// The output from the model is a vector containing the scores for each
	// kind of prediction, so figure out what the highest scoring category was.
	TfLiteTensor* output = interpreter.output(0);
	DEBUG_PRINT("First byte of output: %d", output->data.uint8[0]);

	return 0;
}


extern "C" int testDoubleFunction(int x) {
	return x * 2;
};
