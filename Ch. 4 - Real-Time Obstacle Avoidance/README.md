# Ch. 4 - Real-Time Obstacle Avoidance 

This directory contains the following folders: 

- `eval` : supporting analysis code for evaluation of method in Ch. 4
- `src` : source code for planning method and baseline in Ch. 4

## Prerequisites 

Install the following SDKs onf the Raspberry Pi:

- Servos - AlphaBot SDK https://github.com/berndporr/alphabot
- Lidar - RPLIDAR A1M8 SDK https://github.com/berndporr/rplidar_rpi

## Compile 

Add `src` folder to the Raspberry Pi, create a build directory using `mkdir build`.  

Navigate to the `build` directory and execute ```cmake ..``` to generate the build files. 

Compile the program using ```make```

## Run

```sudo ./autoctrl```

<!-- [I'm an inline-style link](https://youtu.be/FpOAJW28D9s) -->
  
