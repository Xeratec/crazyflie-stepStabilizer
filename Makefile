# CrazyFlie's Makefile
# Copyright (c) 2011-2021 Bitcraze AB
# This Makefile compiles all the object file to ./bin/ and the resulting firmware
# image in ./cfX.elf and ./cfX.bin

CRAZYFLIE_BASE ?= ./extern/crazyflie-firmware
TFMICRO_BASE ?= ./extern/tflite-micro/tensorflow/lite/micro/tools/make/gen/stm32f4_cortex-m4+fp_default/prj/hello_world/make
CMSIS_BASE ?= ./extern/tflite-micro/tensorflow/lite/micro/tools/make/downloads/cmsis
# CMSIS_BASE ?= $(CRAZYFLIE_BASE)/vendor/CMSIS

# Put your personal build config in tools/make/config.mk and DO NOT COMMIT IT!
# Make a copy of tools/make/config.mk.example to get you started
-include ./config.mk

CFLAGS += $(EXTRA_CFLAGS)

######### JTAG and environment configuration ##########
OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2.cfg
OPENOCD_CMDS      ?=
CROSS_COMPILE     ?= arm-none-eabi-
PYTHON            ?= python3
DFU_UTIL          ?= dfu-util
CLOAD             ?= 1
DEBUG             ?= 0
CLOAD_SCRIPT      ?= python3 -m cfloader
CLOAD_CMDS        ?=
CLOAD_ARGS        ?=
PLATFORM          ?= cf2
LPS_TDMA_ENABLE   ?= 0
LPS_TDOA_ENABLE   ?= 0
LPS_TDOA3_ENABLE  ?= 0


# Platform configuration handling
-include current_platform.mk
include $(CRAZYFLIE_BASE)/tools/make/platform.mk

CFLAGS += -DCRAZYFLIE_FW

######### Stabilizer configuration ##########
## These are set by the platform (see tools/make/platforms/*.mk), can be overwritten here
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger, INDI
POWER_DISTRIBUTION ?= stock

#OpenOCD conf
RTOS_DEBUG        ?= 0

LIB = $(CRAZYFLIE_BASE)/src/lib
FREERTOS = $(CRAZYFLIE_BASE)/vendor/FreeRTOS
CFLAGS += -DBLOBS_LOC='"$(CRAZYFLIE_BASE)/blobs/"'

# Communication Link
UART2_LINK        ?= 0


############### CPU-specific build configuration ################

ifeq ($(CPU), stm32f4)
PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
LINKER_DIR = $(CRAZYFLIE_BASE)/tools/make/F405/linker
ST_OBJ_DIR  = $(CRAZYFLIE_BASE)/tools/make/F405

OPENOCD_TARGET    ?= target/stm32f4x_stlink.cfg


# St Lib
VPATH += $(LIB)/CMSIS/STM32F4xx/Source/
VPATH += $(LIB)/STM32_USB_Device_Library/Core/src
VPATH += $(LIB)/STM32_USB_OTG_Driver/src
VPATH += $(CRAZYFLIE_BASE)/src/deck/api $(CRAZYFLIE_BASE)/src/deck/core $(CRAZYFLIE_BASE)/src/deck/drivers/src $(CRAZYFLIE_BASE)/src/deck/drivers/src/test
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/tdoa $(CRAZYFLIE_BASE)/src/utils/src/lighthouse
CRT0 = startup_stm32f40xx.o system_stm32f4xx.o

# Add ST lib object files
-include $(ST_OBJ_DIR)/st_obj.mk

# USB obj
ST_OBJ += usb_core.o usb_dcd_int.o usb_dcd.o
# USB Device obj
ST_OBJ += usbd_ioreq.o usbd_req.o usbd_core.o

PROCESSOR = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DARM_MATH_DSP -DCMSIS_NN -DTF_LITE_USE_GLOBAL_CMATH_FUNCTIONS -DTF_LITE_USE_GLOBAL_MAX -DTF_LITE_USE_GLOBAL_MIN
CXXFLAGS += -DARM_MATH_DSP -DCMSIS_NN -DTF_LITE_USE_GLOBAL_CMATH_FUNCTIONS -DTF_LITE_USE_GLOBAL_MAX -DTF_LITE_USE_GLOBAL_MIN

CFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee
CXXFLAGS += -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee

#Flags required by the ST library
CFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER
CXXFLAGS += -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER

LOAD_ADDRESS_stm32f4 = 0x8000000
LOAD_ADDRESS_CLOAD_stm32f4 = 0x8004000
MEM_SIZE_FLASH_K = 1008
MEM_SIZE_RAM_K = 128
MEM_SIZE_CCM_K = 64
endif

################ Build configuration ##################

# libdw dw1000 driver
VPATH += $(CRAZYFLIE_BASE)/vendor/libdw1000/src

# vl53l1 driver
VPATH += $(LIB)/vl53l1/core/src

# FreeRTOS
VPATH += $(PORT)
PORT_OBJ = port.o
VPATH +=  $(FREERTOS)/portable/MemMang
MEMMANG_OBJ ?= heap_4.o

VPATH += $(FREERTOS)
FREERTOS_OBJ = list.o tasks.o queue.o timers.o $(MEMMANG_OBJ)

#FatFS
VPATH += $(LIB)/FatFS
PROJ_OBJ += ff.o ffunicode.o fatfs_sd.o
ifeq ($(FATFS_DISKIO_TESTS), 1)
PROJ_OBJ += diskio_function_tests.o
CFLAGS += -DUSD_RUN_DISKIO_FUNCTION_TESTS
endif

ifeq ($(APP), 1)
CFLAGS += -DAPP_ENABLED=1
endif
ifdef APP_STACKSIZE
CFLAGS += -DAPP_STACKSIZE=$(APP_STACKSIZE)
endif
ifdef APP_PRIORITY
CFLAGS += -DAPP_PRIORITY=$(APP_PRIORITY)
endif

# Crazyflie sources
VPATH += $(CRAZYFLIE_BASE)/src/init $(CRAZYFLIE_BASE)/src/hal/src $(CRAZYFLIE_BASE)/src/modules/src $(CRAZYFLIE_BASE)/src/modules/src/lighthouse $(CRAZYFLIE_BASE)/src/modules/src/kalman_core $(CRAZYFLIE_BASE)/src/utils/src $(CRAZYFLIE_BASE)/src/drivers/bosch/src $(CRAZYFLIE_BASE)/src/drivers/src $(CRAZYFLIE_BASE)/src/platform
VPATH += $(CRAZYFLIE_BASE)/src/utils/src/kve

######################### TF Micro Compilation ##################
# Need to compile TF Micro with some limited C++-11 support
# and standard libraries for math.
# Add all subdirectories to the include path so we have these headers
VPATH += $(TFMICRO_BASE)
VPATH += $(TFMICRO_BASE)/third_party
VPATH += $(TFMICRO_BASE)/third_party/flatbuffers
VPATH += $(TFMICRO_BASE)/third_party/gemmlowp
VPATH += $(TFMICRO_BASE)/tensorflow/lite/micro/memory_planner
VPATH += $(TFMICRO_BASE)/tensorflow/lite/micro/kernels/cmsis_nn
VPATH += $(TFMICRO_BASE)/tensorflow/lite/micro/kernels
VPATH += $(TFMICRO_BASE)/tensorflow/lite/micro
VPATH += $(TFMICRO_BASE)/tensorflow/lite/kernels/internal
VPATH += $(TFMICRO_BASE)/tensorflow/lite/kernels
VPATH += $(TFMICRO_BASE)/tensorflow/lite/core/api
VPATH += $(TFMICRO_BASE)/tensorflow/lite/schema
VPATH += $(TFMICRO_BASE)/tensorflow/lite/c

VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/ActivationFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/BasicMathFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/ConcatenationFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/ConvolutionFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/FullyConnectedFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/NNSupportFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/PoolingFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/ReshapeFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/SoftmaxFunctions
VPATH += $(CMSIS_BASE)/CMSIS/NN/Source/SVDFunctions

# Exclude the Excluded Files
EXCLUDES_VPATH += $(CRAZYFLIE_BASE)/src/modules/src/range.c
EXCLUDES_VPATH += $(TFMICRO_BASE)/tensorflow/lite/kernels/kernel_util.cc

VPATH := $(filter-out $(EXCLUDES_VPATH), $(VPATH))

############### Source files configuration ################

# Init
PROJ_OBJ += main.o
PROJ_OBJ += platform.o platform_utils.o platform_$(PLATFORM).o platform_$(CPU).o

# Drivers
PROJ_OBJ += exti.o nvic.o motors.o
PROJ_OBJ += led.o mpu6500.o i2cdev.o ws2812_cf2.o lps25h.o i2c_drv.o
PROJ_OBJ += ak8963.o eeprom.o maxsonar.o piezo.o
PROJ_OBJ += uart_syslink.o swd.o uart1.o uart2.o watchdog.o
PROJ_OBJ += cppm.o
PROJ_OBJ += bmi055_accel.o bmi055_gyro.o bmi160.o bmp280.o bstdr_comm_support.o bmm150.o
PROJ_OBJ += bmi088_accel.o bmi088_gyro.o bmi088_fifo.o bmp3.o
PROJ_OBJ += pca9685.o vl53l0x.o pca95x4.o pca9555.o vl53l1x.o pmw3901.o
PROJ_OBJ += amg8833.o lh_bootloader.o

# USB Files
PROJ_OBJ += usb_bsp.o usblink.o usbd_desc.o usb.o

# Hal
PROJ_OBJ += crtp.o ledseq.o freeRTOSdebug.o buzzer.o
PROJ_OBJ += pm_$(CPU).o syslink.o radiolink.o ow_syslink.o ow_common.o proximity.o usec_time.o
PROJ_OBJ += sensors.o
PROJ_OBJ += storage.o

# libdw
PROJ_OBJ += libdw1000.o libdw1000Spi.o

# vl53l1 lib
PROJ_OBJ += vl53l1_api_core.o vl53l1_api.o vl53l1_core.o vl53l1_silicon_core.o vl53l1_api_strings.o
PROJ_OBJ += vl53l1_api_calibration.o vl53l1_api_debug.o vl53l1_api_preset_modes.o vl53l1_error_strings.o
PROJ_OBJ += vl53l1_register_funcs.o vl53l1_wait.o vl53l1_core_support.o

# Modules
PROJ_OBJ += system.o comm.o console.o pid.o crtpservice.o param.o
PROJ_OBJ += log.o worker.o queuemonitor.o msp.o
PROJ_OBJ += platformservice.o sound_cf2.o extrx.o sysload.o mem.o
PROJ_OBJ += range.o app_handler.o static_mem.o app_channel.o
PROJ_OBJ += eventtrigger.o supervisor.o

# Stabilizer modules
PROJ_OBJ += commander.o crtp_commander.o crtp_commander_rpyt.o
PROJ_OBJ += crtp_commander_generic.o crtp_localization_service.o peer_localization.o
PROJ_OBJ += attitude_pid_controller.o sensfusion6.o stabilizer.o
PROJ_OBJ += position_estimator_altitude.o position_controller_pid.o position_controller_indi.o
PROJ_OBJ += estimator.o estimator_complementary.o
PROJ_OBJ += controller.o controller_pid.o controller_mellinger.o controller_indi.o
PROJ_OBJ += power_distribution_$(POWER_DISTRIBUTION).o
PROJ_OBJ += collision_avoidance.o health.o

# Kalman estimator
PROJ_OBJ += estimator_kalman.o kalman_core.o kalman_supervisor.o
PROJ_OBJ += mm_distance.o mm_absolute_height.o mm_position.o mm_pose.o mm_tdoa.o mm_flow.o mm_tof.o mm_yaw_error.o mm_sweep_angles.o
PROJ_OBJ += mm_tdoa_robust.o mm_distance_robust.o

# High-Level Commander
PROJ_OBJ += crtp_commander_high_level.o planner.o pptraj.o pptraj_compressed.o

# Deck Core
PROJ_OBJ += deck.o deck_info.o deck_drivers.o deck_test.o deck_memory.o

# Deck API
PROJ_OBJ += deck_constants.o
PROJ_OBJ += deck_digital.o
PROJ_OBJ += deck_analog.o
PROJ_OBJ += deck_spi.o
PROJ_OBJ += deck_spi3.o

# Decks
PROJ_OBJ += bigquad.o
PROJ_OBJ += ledring12.o
PROJ_OBJ += buzzdeck.o
PROJ_OBJ += gtgps.o
PROJ_OBJ += cppmdeck.o
PROJ_OBJ += usddeck.o
PROJ_OBJ += zranger.o zranger2.o
PROJ_OBJ += locodeck.o
PROJ_OBJ += clockCorrectionEngine.o
PROJ_OBJ += lpsTwrTag.o
PROJ_OBJ += lpsTdoa2Tag.o
PROJ_OBJ += lpsTdoa3Tag.o tdoaEngineInstance.o tdoaEngine.o tdoaStats.o tdoaStorage.o
PROJ_OBJ += outlierFilter.o
PROJ_OBJ += flowdeck_v1v2.o
PROJ_OBJ += oa.o
PROJ_OBJ += multiranger.o
PROJ_OBJ += lighthouse.o
PROJ_OBJ += activeMarkerDeck.o

# Adjust this flag to "force run" the file that you created in the
# src/deck/drivers/src folder. This overrides the default loading
# sequence for the Crazyflie so you can run your code there. For some
# examples look at tfmicrodemo.c or tfmicrobenchmark.c and change the
# following -DDECK_FORCE flag to your file. 
# CFLAGS += -DDECK_FORCE=tfMicroDemo

ifdef ML_MODEL
CFLAGS += -D TFMICRO_MODEL=$(ML_MODEL)
CXXFLAGS += -D TFMICRO_MODEL=$(ML_MODEL)
endif

######################### TF Micro Compilation ##################
# Need to compile TF Micro with some limited C++-11 support
# and standard libraries for math. Add all objects needed for compile.
TF_SRCS := \
debug_log.cc \
tensorflow/lite/micro/all_ops_resolver.cc \
tensorflow/lite/micro/flatbuffer_utils.cc \
tensorflow/lite/micro/memory_helpers.cc \
tensorflow/lite/micro/micro_allocator.cc \
tensorflow/lite/micro/micro_error_reporter.cc \
tensorflow/lite/micro/micro_graph.cc \
tensorflow/lite/micro/micro_interpreter.cc \
tensorflow/lite/micro/micro_profiler.cc \
tensorflow/lite/micro/micro_string.cc \
tensorflow/lite/micro/micro_time.cc \
tensorflow/lite/micro/micro_utils.cc \
tensorflow/lite/micro/mock_micro_graph.cc \
tensorflow/lite/micro/recording_micro_allocator.cc \
tensorflow/lite/micro/recording_simple_memory_allocator.cc \
tensorflow/lite/micro/simple_memory_allocator.cc \
tensorflow/lite/micro/system_setup.cc \
tensorflow/lite/micro/test_helpers.cc \
tensorflow/lite/micro/memory_planner/greedy_memory_planner.cc \
tensorflow/lite/micro/memory_planner/linear_memory_planner.cc \
tensorflow/lite/kernels/kernel_util.cc \
tensorflow/lite/kernels/internal/quantization_util.cc \
tensorflow/lite/core/api/flatbuffer_conversions.cc \
tensorflow/lite/core/api/tensor_utils.cc \
tensorflow/lite/core/api/error_reporter.cc \
tensorflow/lite/core/api/op_resolver.cc \
tensorflow/lite/schema/schema_utils.cc \
tensorflow/lite/c/common.c   \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu6_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c             \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_add_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/BasicMathFunctions/arm_elementwise_mul_s8.c         \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_w.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_x.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_y.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConcatenationFunctions/arm_concatenation_s8_z.c           \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1x1_s8_fast.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_1_x_n_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_basic.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_HWC_q7_RGB.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_convolve_wrapper_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_3x3_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_s8_opt.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_conv_wrapper_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ConvolutionFunctions/arm_nn_mat_mult_s8.c                                                       \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q15_opt.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_q7_opt.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/FullyConnectedFunctions/arm_fully_connected_s8.c                 \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_accumulate_q7_to_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_add_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_padded_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_depthwise_conv_nt_t_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_1x_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mul_core_4x_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_mult_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nntables.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_nn_vec_mat_mult_t_svdf_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_no_shift.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/NNSupportFunctions/arm_q7_to_q15_with_offset.c                                   \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_avgpool_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_max_pool_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/PoolingFunctions/arm_pool_q7_HWC.c         \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/ReshapeFunctions/arm_reshape_s8.c     \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q15.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_q7.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_s8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_u8.c \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SoftmaxFunctions/arm_softmax_with_batch_q7.c             \
tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source/SVDFunctions/arm_svdf_s8.c     \
tensorflow/lite/micro/kernels/activations.cc \
tensorflow/lite/micro/kernels/activations_common.cc \
tensorflow/lite/micro/kernels/cmsis_nn/add.cc \
tensorflow/lite/micro/kernels/add_n.cc \
tensorflow/lite/micro/kernels/arg_min_max.cc \
tensorflow/lite/micro/kernels/batch_to_space_nd.cc \
tensorflow/lite/micro/kernels/cast.cc \
tensorflow/lite/micro/kernels/ceil.cc \
tensorflow/lite/micro/kernels/circular_buffer.cc \
tensorflow/lite/micro/kernels/comparisons.cc \
tensorflow/lite/micro/kernels/concatenation.cc \
tensorflow/lite/micro/kernels/cmsis_nn/conv.cc \
tensorflow/lite/micro/kernels/conv_common.cc \
tensorflow/lite/micro/kernels/cumsum.cc \
tensorflow/lite/micro/kernels/depth_to_space.cc \
tensorflow/lite/micro/kernels/cmsis_nn/depthwise_conv.cc \
tensorflow/lite/micro/kernels/depthwise_conv_common.cc \
tensorflow/lite/micro/kernels/dequantize.cc \
tensorflow/lite/micro/kernels/detection_postprocess.cc \
tensorflow/lite/micro/kernels/elementwise.cc \
tensorflow/lite/micro/kernels/elu.cc \
tensorflow/lite/micro/kernels/ethosu.cc \
tensorflow/lite/micro/kernels/exp.cc \
tensorflow/lite/micro/kernels/expand_dims.cc \
tensorflow/lite/micro/kernels/fill.cc \
tensorflow/lite/micro/kernels/floor.cc \
tensorflow/lite/micro/kernels/floor_div.cc \
tensorflow/lite/micro/kernels/floor_mod.cc \
tensorflow/lite/micro/kernels/cmsis_nn/fully_connected.cc \
tensorflow/lite/micro/kernels/fully_connected_common.cc \
tensorflow/lite/micro/kernels/gather.cc \
tensorflow/lite/micro/kernels/gather_nd.cc \
tensorflow/lite/micro/kernels/hard_swish.cc \
tensorflow/lite/micro/kernels/hard_swish_common.cc \
tensorflow/lite/micro/kernels/if.cc \
tensorflow/lite/micro/kernels/kernel_runner.cc \
tensorflow/lite/micro/kernels/kernel_util.cc \
tensorflow/lite/micro/kernels/l2norm.cc \
tensorflow/lite/micro/kernels/l2_pool_2d.cc \
tensorflow/lite/micro/kernels/leaky_relu.cc \
tensorflow/lite/micro/kernels/logical.cc \
tensorflow/lite/micro/kernels/logical_common.cc \
tensorflow/lite/micro/kernels/logistic.cc \
tensorflow/lite/micro/kernels/logistic_common.cc \
tensorflow/lite/micro/kernels/log_softmax.cc \
tensorflow/lite/micro/kernels/maximum_minimum.cc \
tensorflow/lite/micro/kernels/cmsis_nn/mul.cc \
tensorflow/lite/micro/kernels/neg.cc \
tensorflow/lite/micro/kernels/pack.cc \
tensorflow/lite/micro/kernels/pad.cc \
tensorflow/lite/micro/kernels/cmsis_nn/pooling.cc \
tensorflow/lite/micro/kernels/pooling_common.cc \
tensorflow/lite/micro/kernels/prelu.cc \
tensorflow/lite/micro/kernels/quantize.cc \
tensorflow/lite/micro/kernels/quantize_common.cc \
tensorflow/lite/micro/kernels/reduce.cc \
tensorflow/lite/micro/kernels/reshape.cc \
tensorflow/lite/micro/kernels/resize_bilinear.cc \
tensorflow/lite/micro/kernels/resize_nearest_neighbor.cc \
tensorflow/lite/micro/kernels/round.cc \
tensorflow/lite/micro/kernels/shape.cc \
tensorflow/lite/micro/kernels/cmsis_nn/softmax.cc \
tensorflow/lite/micro/kernels/softmax_common.cc \
tensorflow/lite/micro/kernels/space_to_batch_nd.cc \
tensorflow/lite/micro/kernels/space_to_depth.cc \
tensorflow/lite/micro/kernels/split.cc \
tensorflow/lite/micro/kernels/split_v.cc \
tensorflow/lite/micro/kernels/squeeze.cc \
tensorflow/lite/micro/kernels/strided_slice.cc \
tensorflow/lite/micro/kernels/sub.cc \
tensorflow/lite/micro/kernels/cmsis_nn/svdf.cc \
tensorflow/lite/micro/kernels/svdf_common.cc \
tensorflow/lite/micro/kernels/tanh.cc \
tensorflow/lite/micro/kernels/transpose.cc \
tensorflow/lite/micro/kernels/transpose_conv.cc \
tensorflow/lite/micro/kernels/unpack.cc \
tensorflow/lite/micro/kernels/zeros_like.cc \

# TF_SRCS := $(notdir $(TF_SRCS))

PROJ_OBJ += $(patsubst %.cc,%.o,$(patsubst %.c,%.o,$(TF_SRCS)))

# Uart2 Link for CRTP communication is not compatible with decks using uart2
ifeq ($(UART2_LINK), 1)
CFLAGS += -DUART2_LINK_COMM
else
PROJ_OBJ += aideck.o
endif

ifeq ($(LPS_TDOA_ENABLE), 1)
CFLAGS += -DLPS_TDOA_ENABLE
endif

ifeq ($(LPS_TDOA3_ENABLE), 1)
CFLAGS += -DLPS_TDOA3_ENABLE
endif

ifeq ($(LPS_TDMA_ENABLE), 1)
CFLAGS += -DLPS_TDMA_ENABLE
endif

ifdef SENSORS
SENSORS_UPPER = $(shell echo $(SENSORS) | tr a-z A-Z)
CFLAGS += -DSENSORS_FORCE=SensorImplementation_$(SENSORS)

# Add sensor file to the build if needed
ifeq (,$(findstring DSENSOR_INCLUDED_$(SENSORS_UPPER),$(CFLAGS)))
CFLAGS += -DSENSOR_INCLUDED_$(SENSORS_UPPER)
CXXFLAGS += -DSENSOR_INCLUDED_$(SENSORS_UPPER)
PROJ_OBJ += sensors_$(SENSORS).o
endif
endif

#Deck tests
PROJ_OBJ += exptest.o
PROJ_OBJ += exptestRR.o
PROJ_OBJ += exptestBolt.o
#PROJ_OBJ += bigquadtest.o
#PROJ_OBJ += uarttest.o
#PROJ_OBJ += aidecktest.o


# Utilities
PROJ_OBJ += filter.o cpuid.o cfassert.o  eprintf.o crc32.o num.o debug.o
PROJ_OBJ += version.o FreeRTOS-openocd.o
PROJ_OBJ += configblockeeprom.o
PROJ_OBJ += sleepus.o statsCnt.o rateSupervisor.o
PROJ_OBJ += lighthouse_core.o pulse_processor.o pulse_processor_v1.o pulse_processor_v2.o lighthouse_geometry.o ootx_decoder.o lighthouse_calibration.o lighthouse_deck_flasher.o lighthouse_position_est.o lighthouse_storage.o
PROJ_OBJ += kve_storage.o kve.o

ifeq ($(DEBUG_PRINT_ON_SEGGER_RTT), 1)
VPATH += $(LIB)/Segger_RTT/RTT
INCLUDES += -I$(LIB)/Segger_RTT/RTT
PROJ_OBJ += SEGGER_RTT.o SEGGER_RTT_printf.o
CFLAGS += -DDEBUG_PRINT_ON_SEGGER_RTT
endif

# Libs
PROJ_OBJ += libarm_math.a

OBJ = $(FREERTOS_OBJ) $(PORT_OBJ) $(ST_OBJ) $(PROJ_OBJ) $(APP_OBJ) $(CRT0)

#Get just the paths so you can create them in advance
OBJDIRS := $(dir $(OBJ))
OBJDIRS := $(patsubst %,bin/%,$(OBJDIRS))
DUMMY:=$(shell mkdir --parents $(OBJDIRS))

############### Compilation configuration ################
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)g++
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb

INCLUDES += -I$(CMSIS_BASE)
INCLUDES += -I$(CMSIS_BASE)/CMSIS/Core/Include 
INCLUDES += -I$(CMSIS_BASE)/CMSIS/DSP/Include
INCLUDES += -I$(CMSIS_BASE)/CMSIS/NN/Include

INCLUDES += -I$(CRAZYFLIE_BASE)/vendor/libdw1000/inc
INCLUDES += -I$(FREERTOS)/include -I$(PORT)

INCLUDES += -I$(CRAZYFLIE_BASE)/src/config
INCLUDES += -I$(CRAZYFLIE_BASE)/src/platform

INCLUDES += -I$(CRAZYFLIE_BASE)/src/deck/interface -I$(CRAZYFLIE_BASE)/src/deck/drivers/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/drivers/interface -I$(CRAZYFLIE_BASE)/src/drivers/bosch/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/hal/interface
INCLUDES += -I$(CRAZYFLIE_BASE)/src/modules/interface -I$(CRAZYFLIE_BASE)/src/modules/interface/kalman_core -I$(CRAZYFLIE_BASE)/src/modules/interface/lighthouse
INCLUDES += -I$(CRAZYFLIE_BASE)/src/utils/interface -I$(CRAZYFLIE_BASE)/src/utils/interface/kve -I$(CRAZYFLIE_BASE)/src/utils/interface/lighthouse -I$(CRAZYFLIE_BASE)/src/utils/interface/tdoa

INCLUDES += -I$(LIB)/FatFS
INCLUDES += -I$(LIB)/CMSIS/STM32F4xx/Include
INCLUDES += -I$(LIB)/STM32_USB_Device_Library/Core/inc
INCLUDES += -I$(LIB)/STM32_USB_OTG_Driver/inc
INCLUDES += -I$(LIB)/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -I$(LIB)/vl53l1 -I$(LIB)/vl53l1/core/inc

# Add tfmicro library
INCLUDES += -I $(TFMICRO_BASE)/
INCLUDES += -I $(TFMICRO_BASE)
INCLUDES += -I $(TFMICRO_BASE)/third_party/flatbuffers
INCLUDES += -I $(TFMICRO_BASE)/third_party/flatbuffers/include
INCLUDES += -I $(TFMICRO_BASE)/third_party
INCLUDES += -I $(TFMICRO_BASE)/third_party/gemmlowp
INCLUDES += -I $(TFMICRO_BASE)/third_party/ruy

CFLAGS += -g3
ifeq ($(DEBUG), 1)
  CFLAGS += -Os -DDEBUG
  CXXFLAGS += -Os -DDEBUG

  # Prevent silent errors when converting between types (requires explicit casting)
#   CFLAGS += -Wconversion
#   CXXFLAGS += -Wconversion
else
  CFLAGS += -Os
  CXXFLAGS += -Os

  # Fail on warnings
  CFLAGS += -Werror
  CXXFLAGS += -Werror
endif

# Disable warnings for unaligned addresses in packed structs (added in GCC 9)
CFLAGS += -Wno-address-of-packed-member

ifeq ($(LTO), 1)
  CFLAGS += -flto
endif

CXXFLAGS += -fpermissive

CFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)
CXXFLAGS += -DBOARD_REV_$(REV) -DESTIMATOR_NAME=$(ESTIMATOR)Estimator -DCONTROLLER_NAME=ControllerType$(CONTROLLER) -DPOWER_DISTRIBUTION_TYPE_$(POWER_DISTRIBUTION)

CFLAGS += $(PROCESSOR) $(INCLUDES)
CXXFLAGS += $(PROCESSOR) $(INCLUDES)

CFLAGS += -Wall -Wmissing-braces -fno-strict-aliasing $(C_PROFILE) -std=gnu11
# Compiler flags to generate dependency files:
CFLAGS += -MD -MP -MF $(BIN)/dep/$(notdir $(@)).d -MQ $(@)
CXXFLAGS += -MD -MP -MF $(BIN)/dep/$(notdir $(@)).d -MQ $(@)

#Permits to remove un-used functions and global variables from output file
CFLAGS += -ffunction-sections -fdata-sections
CXXFLAGS += -ffunction-sections -fdata-sections
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion

ASFLAGS = $(PROCESSOR) $(INCLUDES)
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -Wl,-Map=$(PROG).map,--cref,--gc-sections,--undefined=uxTopUsedPriority
LDFLAGS += -L$(CRAZYFLIE_BASE)/tools/make/F405/linker

# Necessary for standard library support
LDFLAGS += -lstdc++

#Flags required by the ST library
ifeq ($(CLOAD), 1)
  LDFLAGS += -T $(LINKER_DIR)/FLASH_CLOAD.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_CLOAD_$(CPU))
else
  LDFLAGS += -T $(LINKER_DIR)/FLASH.ld
  LOAD_ADDRESS = $(LOAD_ADDRESS_$(CPU))
endif

ifeq ($(LTO), 1)
  LDFLAGS += -Os -flto -fuse-linker-plugin
endif

#Program name
PROG = $(PLATFORM)
#Where to compile the .o
BIN = bin
VPATH += $(BIN)

#Dependency files to include
DEPS := $(foreach o,$(OBJ),$(BIN)/dep/$(o).d)

##################### Misc. ################################
ifeq ($(SHELL),/bin/sh)
  COL_RED=\033[1;31m
  COL_GREEN=\033[1;32m
  COL_RESET=\033[m
endif

# This define n-thing is a standard hack to get newlines in GNU Make.
define n


endef

# Make sure that the submodules are up to date.
# Check if there are any files in the vendor directories, if not warn the user.
ifeq ($(wildcard $(CRAZYFLIE_BASE)/vendor/*/*),)
  $(error $n                                                                   \
    The submodules does not seem to be present, consider fetching them by:$n   \
      $$ git submodule init$n                                                  \
      $$ git submodule update$n                                                \
  )
endif

#################### Targets ###############################

all: bin/ bin/dep bin/vendor check_submodules build
build:
# Each target is in a different line, so they are executed one after the other even when the processor has multiple cores (when the -j option for the make command is > 1). See: https://www.gnu.org/software/make/manual/html_node/Parallel.html
	@$(MAKE) --no-print-directory clean_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory compile CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory print_version CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
	@$(MAKE) --no-print-directory size CRAZYFLIE_BASE=$(CRAZYFLIE_BASE)
compile: $(PROG).hex $(PROG).bin $(PROG).dfu

bin/:
	mkdir -p bin

bin/dep:
	mkdir -p bin/dep

bin/vendor:
	mkdir -p bin/vendor

libarm_math.a:
	+$(MAKE) -C $(CRAZYFLIE_BASE)/tools/make/cmsis_dsp/ CRAZYFLIE_BASE=$(abspath $(CRAZYFLIE_BASE)) PROJ_ROOT=$(CURDIR) V=$(V) CROSS_COMPILE=$(CROSS_COMPILE)

clean_version:
ifeq ($(SHELL),/bin/sh)
	@echo "  CLEAN_VERSION"
	@rm -f version.c
endif

print_version:
	@echo "Build for the $(PLATFORM_NAME_$(PLATFORM))!"
	@$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/versionTemplate.py --crazyflie-base $(CRAZYFLIE_BASE) --print-version
ifeq ($(CLOAD), 1)
	@echo "Crazyloader build!"
endif
ifeq ($(FATFS_DISKIO_TESTS), 1)
	@echo "WARNING: FatFS diskio tests enabled. Erases SD-card!"
endif

size:
	@$(PYTHON) $(CRAZYFLIE_BASE)/tools/make/size.py $(SIZE) $(PROG).elf $(MEM_SIZE_FLASH_K) $(MEM_SIZE_RAM_K) $(MEM_SIZE_CCM_K)

#Radio bootloader
cload:
ifeq ($(CLOAD), 1)
	$(CLOAD_SCRIPT) $(CLOAD_CMDS) flash $(CLOAD_ARGS) $(PROG).bin stm32-fw
else
	@echo "Only cload build can be bootloaded. Launch build and cload with CLOAD=1"
endif

#Flash the stm.
flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(PROG).bin $(LOAD_ADDRESS) bin" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

#verify only
flash_verify:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "verify_image $(PROG).bin $(LOAD_ADDRESS) bin" -c "reset run" -c shutdown

flash_dfu:
	$(DFU_UTIL) -a 0 -D $(PROG).dfu

#STM utility targets
halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$$_TARGETNAME configure -rtos auto"

trace:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -f tools/trace/enable_trace.cfg

gdb: $(PROG).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $^

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f4x mass_erase 0" -c shutdown

#Print preprocessor #defines
prep:
	@$(CC) $(CFLAGS) -dM -E - < /dev/null

check_submodules:
	@cd $(CRAZYFLIE_BASE); $(PYTHON) tools/make/check-for-submodules.py

include ./targets.mk

#include dependencies
-include $(DEPS)

unit:
# The flag "-DUNITY_INCLUDE_DOUBLE" allows comparison of double values in Unity. See: https://stackoverflow.com/a/37790196
	rake unit "DEFINES=$(CFLAGS) -DUNITY_INCLUDE_DOUBLE" "FILES=$(FILES)" "UNIT_TEST_STYLE=$(UNIT_TEST_STYLE)"

.PHONY: all clean build compile unit prep erase flash check_submodules trace openocd gdb halt reset flash_dfu flash_verify cload size print_version clean_version
