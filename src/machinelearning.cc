/* An attempt at creating C extensions to TF-Micro in order to get it to run
on a Crazyflie 2.0. The reason for this is because all of the firmware for
the crazyflie is written in C, and although we can compile TF-Micro for the
Cortex M4, we still have to link it to the main loop in the firmware.

Approach will be to expose the common functions we need in C, compile TF Micro
in C++, and then link it to the main loop later.
==============================================================================*/
#include "stdlib.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "machinelearning.h"

// Our machine learning models we're putting in :)
#include "tfmicro_models.h"

#define DEBUG_MODULE "ML"

// C Includes
extern "C" {
	#include "stm32fxxx.h"
	#include "FreeRTOS.h"
	#include "FreeRTOSConfig.h"
	#include "task.h"

	#include "debug.h"
	#include "cycle_counter.h"
}

// Global variables
const int kTensorArenaSize = 1024 * 5;
uint8_t tensor_arena[kTensorArenaSize];

extern "C" {
	/**
	 * TfLiteModel wrapper functions
	 */
	const CTfLiteModel * CTfLiteModel_create(const void *model_buf) {
		return reinterpret_cast<const CTfLiteModel*>(
			tflite::GetModel(model_buf)
		);
	}
	void CTfLiteModel_destroy(CTfLiteModel* c_model) {
		delete reinterpret_cast<tflite::Model*>(c_model);
	}
	
	int CTfLiteModel_version(CTfLiteModel* c_model) {
		return static_cast<int>(reinterpret_cast<tflite::Model*>(c_model)->version());
	}

	/**
	 * TfLiteInterpreter wrapper functions
	 */
	CTfLiteInterpreter* CTFLiteInterpreter_create(const CTfLiteModel* c_model) {
		static tflite::MicroErrorReporter micro_error_reporter;
		static tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		static tflite::MicroInterpreter* interpreter = nullptr;

		static const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);

		if (model->version() != TFLITE_SCHEMA_VERSION) {
			TF_LITE_REPORT_ERROR(error_reporter,
							"Model provided is schema version %d not equal "
							"to supported version %d.",
							model->version(), TFLITE_SCHEMA_VERSION
			);
			return nullptr;
		}

		// This pulls in all the operation implementations we need.
		static tflite::AllOpsResolver resolver;

		// // Build an interpreter to runtiny_mnist_model the model with.
		static tflite::MicroInterpreter static_interpreter(
			model, 
			resolver, 
			tensor_arena, 
			kTensorArenaSize, 
			error_reporter
		);
		interpreter = &static_interpreter;

		TfLiteStatus allocate_status = interpreter->AllocateTensors();
		if (allocate_status != kTfLiteOk) {
			DEBUG_PRINT("AllocateTensors() failed");
			return nullptr;
		}
		
		return reinterpret_cast<CTfLiteInterpreter*>(interpreter);
	}

	void CTFLiteInterpreter_destroy(CTfLiteInterpreter* c_interpreter) {
		delete reinterpret_cast<tflite::MicroInterpreter*>(c_interpreter);
	}
	
	/**
	 * TF Micro Interpreter wrapper functions
	 */
	int CTfLiteInterpreter_run(
		CTfLiteInterpreter* c_interpreter,
		model_type* input, 
		size_t input_size, 
		model_type* result, 
		size_t result_size
	) {
		static tflite::MicroInterpreter* interpreter = reinterpret_cast<tflite::MicroInterpreter*>(c_interpreter);
		
		// Copy input data
		static TfLiteTensor* model_input = interpreter->input(0);
		static TfLiteTensor* model_output = interpreter->output(0);

		memcpy(model_input->data.uint8, input, input_size * sizeof(model_type));

		// Reset Timer
		ResetTimer();
		StartTimer();
		// Run inference
		TfLiteStatus invoke_status = interpreter->Invoke();
		// Stop Timer
		StopTimer();
		if (invoke_status != kTfLiteOk) {
			return 0;
		}
		// Copy output data
		memcpy(result, model_output->data.uint8, result_size * sizeof(model_type));
		
		return getCycles();
	}
}


extern "C" int machine_learning_test(int n) {
	static const tflite::Model* test_model = nullptr;

	static tflite::ErrorReporter* test_error_reporter = nullptr;
	static tflite::MicroInterpreter* test_interpreter = nullptr;

	static TfLiteTensor* test_model_input = nullptr;
	static TfLiteTensor* test_model_output = nullptr;

	static tflite::MicroErrorReporter test_micro_error_reporter;
  	test_error_reporter = &test_micro_error_reporter;

	DEBUG_PRINT("Test ML Network.\n");

	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	test_model = tflite::GetModel(TFMICRO_MODEL);
	DEBUG_PRINT("Loaded in model.\n");
	DEBUG_PRINT("Model Version: %d\n", test_model->version());
	DEBUG_PRINT("Model Description: %s\n", test_model->description());

	if (test_model->version() != TFLITE_SCHEMA_VERSION) {
		TF_LITE_REPORT_ERROR(test_error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         test_model->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}

	// This pulls in all the operation implementations we need.
	static tflite::AllOpsResolver test_resolver;

	// Create an area of memory to use for input, output, and intermediate arrays.
	// The size of this will depend on the model you're using, and may need to be
	// determined by experimentation.
	DEBUG_PRINT("Allocate tensor arena of %d bytes\n", kTensorArenaSize);
	// uint8_t* tensor_arena = (uint8_t *) pvPortMalloc(kTensorArenaSize * sizeof(uint8_t));

	// // Build an interpreter to runtiny_mnist_model the model with.
	DEBUG_PRINT("Attempting to build the interpreter.\n");
	static tflite::MicroInterpreter static_interpreter(
      test_model, test_resolver, tensor_arena, kTensorArenaSize, test_error_reporter);
  	test_interpreter = &static_interpreter;
	DEBUG_PRINT("The interpreter has been built.\n");

	TfLiteStatus allocate_status = test_interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		DEBUG_PRINT("AllocateTensors() failed");
		return -1;
	}
	
	test_model_input = test_interpreter->input(0);
	// Get information about the memory area to use for the model's input.
	DEBUG_PRINT("Type: %d\n", test_model_input->type);

	TfLiteAffineQuantization* model_input_quant = (TfLiteAffineQuantization*) test_model_input->quantization.params;
	DEBUG_PRINT("Quantization: %d\n", test_model_input->quantization.type);
	if (test_model_input->quantization.type) {
		DEBUG_PRINT("Quantization->dim: %d\n", model_input_quant->quantized_dimension);
		DEBUG_PRINT("Quantization->scale[0]: %f\n", model_input_quant->scale[0]);
		DEBUG_PRINT("Quantization->zero_point[0]: %d\n", model_input_quant->zero_point[0]);
	}

	DEBUG_PRINT("Dims->size: %d\n", test_model_input->dims->size);
	DEBUG_PRINT("Dims->data[0]: %d\n", test_model_input->dims->data[0]);
	DEBUG_PRINT("Dims->data[1]: %d\n", test_model_input->dims->data[1]);
	DEBUG_PRINT("Dims->data[2]: %d\n", test_model_input->dims->data[2]);

	for (int i = 0; i < test_model_input->bytes ;++i) {
		test_model_input->data.uint8[i] = (rand() % 20) + 50;
	}

	// Run the model on the spectrogram input and make sure it succeeds.
	DEBUG_PRINT("Invoke model...\n");

	for (int test=0; test<100; ++test) {
		// Reset and start cylce counter
		ResetTimer();
		StartTimer();

		// Run inference
		TfLiteStatus invoke_status = test_interpreter->Invoke();

		// Stop Timer
		StopTimer();

		// Check for errors
		if (invoke_status != kTfLiteOk) {
			DEBUG_PRINT("Invoke failed");
			test_error_reporter->Report("Invoke failed");
			return 1;
		} else {
			unsigned int inference_cycles = getCycles();
			float inference_time = inference_cycles / (1.0f * configCPU_CLOCK_HZ) * 1000.0f;
			DEBUG_PRINT("[%03d] %.3f ms (%u CPU cycles).\r\n",test, inference_time, inference_cycles);
		}
		vTaskDelay(M2T(25));
	}

	test_model_output = test_interpreter->output(0);
	DEBUG_PRINT("Type: %d\n", test_model_output->type);

 	TfLiteAffineQuantization* model_output_quant = (TfLiteAffineQuantization*) test_model_output->quantization.params;
	DEBUG_PRINT("Quantization: %d\n", test_model_output->quantization.type);
	if (test_model_output->quantization.type) {
		DEBUG_PRINT("Quantization->dim: %d\n", model_output_quant->quantized_dimension);
		DEBUG_PRINT("Quantization->scale[0]: %f\n", model_output_quant->scale[0]);
		DEBUG_PRINT("Quantization->zero_point[0]: %d\n", model_output_quant->zero_point[0]);
	}

	DEBUG_PRINT("Dims->size: %d\n", test_model_output->dims->size);
	DEBUG_PRINT("Dims->out[0]: %d\n", test_model_output->dims->data[0]);
	DEBUG_PRINT("Dims->out[1]: %d\n", test_model_output->dims->data[1]);
	DEBUG_PRINT("First byte of output: %f\n", test_model_output->data.f[0]);

	return 0;
}
