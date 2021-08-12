################ Build configuration ##################
VPATH += ./src
VPATH += ./src/app/src
VPATH += ./src/override
VPATH += ./src/util/src

############### Source files configuration ################
PROJ_OBJ += app_stepStabilizer_main.o
PROJ_OBJ += app_stepStabilizer_filter.o
PROJ_OBJ += app_stepStabilizer_ml.o
PROJ_OBJ += tfmicro_models.o
PROJ_OBJ += machinelearning.o

PROJ_OBJ += utils.o
PROJ_OBJ += cycle_counter.o

############### Compilation configuration ################
INCLUDES += -I./src/app/interface
INCLUDES += -I./src/util/interface

############### Configuration ################
ML_MODEL = NN_SSE_RMS_10_v2

# Set to ‘1’ to enable the app entry-point
APP = 1
# Set the task stack size in 32bit word (4 Bytes). The default is 300 (1.2KBytes)
APP_STACKSIZE = 1*1024

# Set the task priority between 0 and 5. Default is 0 (same as IDLE).
APP_PRIORITY = 3

# Crazyflie 2 Platform
PLATFORM=cf2

# DEBUG=1 allows to get more information from the Crazyflie console when
# it starts. Debug should not be enabled if you intend to fly the
# Crazyflie out of the lab (it disables the watchdog).
DEBUG=1

## Weight of the Crazyflie, including decks. The default setting is a Crazyflie 2.X without decks.
CFLAGS += -DCF_MASS=0.032f #// in kg

## Force device type string
# CFLAGS += -DDEVICE_TYPE_STRING_FORCE="CF20"

## Force a sensor implementation to be used
# SENSORS=bosch

## Set CRTP link to E-SKY receiver
# CFLAGS += -DUSE_ESKYLINK

## Redirect the console output to the UART
# CFLAGS += -DDEBUG_PRINT_ON_UART

## Redirect the console output to JLINK (using SEGGER RTT)
# DEBUG_PRINT_ON_SEGGER_RTT = 1

## Load a deck driver that has no OW memory
# CFLAGS += -DDECK_FORCE=bcBuzzer

## Load multiple deck drivers that has no OW memory
# CFLAGS += -DDECK_FORCE=bcBuzzer:bcLedRing

## Enable biq quad deck features
# CFLAGS += -DENABLE_BQ_DECK
# CFLAGS += -DBQ_DECK_ENABLE_PM
# CFLAGS += -DBQ_DECK_ENABLE_OSD

## Use morse when flashing the LED to indicate that the Crazyflie is calibrated
# CFLAGS += -DCALIBRATED_LED_MORSE

## Disable LEDs from turning on/flashing
# CFLAGS += -DTURN_OFF_LEDS

## Set the default LED Ring effect (if not set, effect 6 will be used)
# CFLAGS += -DLEDRING_DEFAULT_EFFECT=0

## Set LED Rings to use less LEDs
# CFLAGS += -DLED_RING_NBR_LEDS=6

## Set LED Rings to use less more LEDs (only if board is modified)
# CFLAGS += -DLED_RING_NBR_LEDS=24

## Do not send CRTP messages when parameter values are updated
# CFLAGS += -DSILENT_PARAM_UPDATES

## Turn on monitoring of queue usages
# CFLAGS += -DDEBUG_QUEUE_MONITOR

## Automatically reboot to bootloader before flashing
CLOAD_CMDS = -w radio://0/81/2M/E7E7E7E7E7
# CLOAD_CMDS = -w radio://0/80/2M/E7E7E7E7E7

# Or Run: python -m cfloader -w radio://0/80/2M/E7E7E7E7E7 flash cf2.bin stm32-fw

## Set number of anchor in LocoPositioningSystem
# CFLAGS += -DLOCODECK_NR_OF_ANCHORS=8

## Set alternative pins for LOCO deck (IRQ=IO_2, RESET=IO_3, default are RX1 and TX1)
# CFLAGS += -DLOCODECK_USE_ALT_PINS

## Set other pin for reset on the LOCO deck. Only works when LOCODECK_USE_ALT_PINS is set.
# For instance useful with Loco + Lighhouse + Flow decks where IO_3 collides with the Flow deck
# CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4

## Disable Low Interference Mode when using Loco Deck
# CFLAGS += -DLOCODECK_NO_LOW_INTERFERENCE

## Low interference communication
# Set the 'low interference' 2.4GHz TX power. This power is set when the loco deck is initialized
# Possible power are: +4, 0, -4, -8, -12, -16, -20, -30
# CFLAGS += -DPLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM="(-12)"

## Set alternative pins for uSD-deck. Patch soldering required (CS->RX2(PA3), SCLK->TX1(PC10), MISO->RX1(PC11), MOSI->IO_4(PC12))
# CFLAGS += -DUSDDECK_USE_ALT_PINS_AND_SPI

## Use J-Link as Debugger/flasher
# OPENOCD_INTERFACE ?= interface/jlink.cfg
# OPENOCD_TARGET    ?= target/stm32f4x.cfg
# OPENOCD_CMDS      ?= -c "transport select swd"

## LPS settings ----------------------------------------------------
## Set operation mode of the LPS system (auto is default)
## TWR
# CFLAGS += -DLPS_TWR_ENABLE=1
## TDoA2
# LPS_TDOA_ENABLE=1
## TDoA3
# LPS_TDOA3_ENABLE=1

## TDoA 3 - experimental
# Enable 2D positioning. The value (1.2) is the height that the tag will move at. Only use in TDoA 3
# CFLAGS += -DLPS_2D_POSITION_HEIGHT=1.2

## Enable longer range (lower bit rate). Only use in TDoA 3
# Note: Anchors must also be built with this flag
# CFLAGS += -DLPS_LONGER_RANGE

## Full LPS TX power.
# CFLAGS += -DLPS_FULL_TX_POWER

## SDCard test configuration ------------------------------------
# FATFS_DISKIO_TESTS  = 1	# Set to 1 to enable FatFS diskio function tests. Erases card.

## Brushless handling
# Start disarmed, needs to be armed before being able to fly
# CFLAGS += -DSTART_DISARMED
# IDLE motor drive when armed, 0 = 0%, 65535 = 100% (the motors runs as long as the Crazyflie is armed)
# CFLAGS += -DDEFAULT_IDLE_THRUST=5000
