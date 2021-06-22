# crazyflie-stepStabilizer
Semesterproject for lecture *Systems-on-chip for Data Analytics and Machine Learning*.

**Height estimation with a Neural Network**  
The general approach of enabling drones to perform altitude estimation in indoor environments relies mainly on a fusion between inertial information and distance measurements to the ground. Usually, the latter provides more precise information, performing measurements relative to the ground with centimeter-precision - using the time-of-flight (ToF) principle. However, while very precise, using this type of sensor on drones comes with a limitation. If the floor is not perfectly flat, or if there are various obstacles on the floor (e.g., tables,  chairs), the drone will experience a sudden change in altitude when hovering above these obstacles. This sudden change can destabilize the drone or can have an undesired effect on flight behavior. The goal of this project is to detect these effects using not only the distance raw data but also other onboard sensors (IMU, barometer) and predict the height with an NN. Thus in this project, you will: 
1): Collect data on the sensor data and ground truth of height to form a dataset.
2): Train a simple neural network to give good height estimation
3): Port that neural network over to the MCU we have onboard (STM32). Note that this step can be done in multiple ways, TFLite/Cube.AI/CMSIS-NN, etc.


## Usage
### Installation

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


## Authors
**Philip Wiese** (ETHZ ETIT)  
  *[wiesep@student.ethz.ch](mailto:wiesep@student.ethz.ch)* - [Xeratec](https://github.com/Xeratec) 

**Luca Rufer** (ETHZ ETIT)  
  *[lrufer@student.ethz.ch](mailto:lrufer@student.ethz.ch)* - [LucaRufer](https://github.com/LucaRufer) 
