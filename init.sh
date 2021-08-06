#!/usr/bin/env bash

git submodule update --init --recursive

# Initialize 
cd extern/tflite-micro
make -f tensorflow/lite/micro/tools/make/Makefile TARGET=disco_f746ng OPTIMIZED_KERNEL_DIR=cmsis_nn TARGET_ARCH="cortex-m4+fp" generate_hello_world_make_project
make -f tensorflow/lite/micro/tools/make/Makefile TARGET=stm32f4 OPTIMIZED_KERNEL_DIR=cmsis_nn TARGET_ARCH="cortex-m4+fp" microlite