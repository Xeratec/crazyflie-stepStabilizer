# crazyflie-stepStabilizer
Semesterproject for lecture *Systems-on-chip for Data Analytics and Machine Learning*.

**Height estimation with a Neural Network**  
The general approach of enabling drones to perform altitude estimation in indoor environments relies mainly on a fusion between inertial information and distance measurements to the ground. Usually, the latter provides more precise information, performing measurements relative to the ground with centimeter-precision - using the time-of-flight (ToF) principle. However, while very precise, using this type of sensor on drones comes with a limitation. If the floor is not perfectly flat, or if there are various obstacles on the floor (e.g., tables,  chairs), the drone will experience a sudden change in altitude when hovering above these obstacles. This sudden change can destabilize the drone or can have an undesired effect on flight behavior. The goal of this project is to detect these effects using not only the distance raw data but also other onboard sensors (IMU, barometer) and predict the height with an NN. Thus in this project, you will: 
1): Collect data on the sensor data and ground truth of height to form a dataset.
2): Train a simple neural network to give good height estimation
3): Port that neural network over to the MCU we have onboard (STM32). Note that this step can be done in multiple ways, TFLite/Cube.AI/CMSIS-NN, etc.


## Installation

Clone the repo and change directory:

```bash
git clone git@github.com:Xeratec/crazyflie-stepStabilizer.git
cd crazyflie-stepStabilizer
```

Install dependencies, download latest telemetry frame definitions and generate the dynamic parser:

```bash
$> chmod +x init.sh
$> ./init.sh
```

## Make targets

```
all        : Shortcut for build
compile    : Compile cflie.hex. WARNING: Do NOT update version.c
build      : Update version.c and compile cflie.elf/hex
clean_o    : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean      : Clean every compiled files
mrproper   : Clean every compiled files and the classical editors backup files

cload      : If the crazyflie-clients-python is placed on the same directory level and
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware
             using the wireless bootloader.
flash      : Flash .elf using OpenOCD
halt       : Halt the target using OpenOCD
reset      : Reset the target using OpenOCD
openocd    : Launch OpenOCD
```

## Flashing
Writing a new binary to the Crazyflie is called flashing (writing it to the flash memory). This page describes how to flash from the command line and there are a few different ways to do it.

### Using Crazyradio

The most common way to flash is probably to use the Crazyradio.

### Prerequisites
* A Crazyradio with drivers installed
* [crazyflie-clients-python](https://github.com/bitcraze/crazyflie-clients-python) placed on the same directory level in the file tree
* The firmware has been built
* The current working directory is the root of the frazyflie-firmware project

### Manually entering bootloader mode

* Turn the Crazyflie off
* Start the Crazyflie in bootloader mode by pressing the power button for 3 seconds. Both the blue LEDs will blink.
* In your terminal, run `make cload`

It will try to find a Crazyflie in bootloader mode and flash the binary to it.

Warning: if multiple Crazyflies within range are in bootloader mode the result is unpredictable. This method is not suitable in classroom situation where it is likely that several students are flashing at the same time. Also remember that the Crazyradio PA often reaches into the next room.

### Automatically enter bootloader mode

* Add the address of the crazyflie to the [`config.mk`](#configmk) file, for instance `CLOAD_CMDS = -w radio://0/80/2M`
* Make sure the Crazyflie is on
* In your terminal, run `make cload`

It will connect to the Crazyflie with the specified address, put it in bootloader mode and flash the binary. This method is suitable for classroom situations.

Note: this method does not work if the Crazyflie does not start, for instance if the current flashed binary is corrupt. You will have to fall back to manually entering bootloader mode.

## TF Lite

The tensorflow lite implementation is based on https://github.com/harvard-edge/crazyflie-firmware


## Authors
**Philip Wiese** (ETHZ ETIT)  
  *[wiesep@student.ethz.ch](mailto:wiesep@student.ethz.ch)* - [Xeratec](https://github.com/Xeratec) 

**Luca Rufer** (ETHZ ETIT)  
  *[lrufer@student.ethz.ch](mailto:lrufer@student.ethz.ch)* - [LucaRufer](https://github.com/LucaRufer) 
