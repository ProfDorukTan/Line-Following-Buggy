# Line-Following-Buggy

## Project Overview

The project utilizes the STM32F401RE microcontroller, 6 TCRT 5000 sensors, 2 DC motors, and 2 encoders.

### Objective

The primary objective of the software is to serve as a control system, utilizing inputs from line sensors to regulate motor speeds and enable the buggy to track a white line. The implementation involves several key algorithms, each serving a distinct purpose:

1. **Sensor Reading Algorithm:** Reads sensor values at regular intervals and converts them into relative line positions, providing data for line tracking.
    
2. **Encoder Reading Algorithm:** Accurately translates encoder values into velocity values, enabling precise control over motor speeds.
    
3. **Speed Control and Direction Control Algorithms:** Two separate control algorithms regulate the speed and direction of the buggy.
    
4. **Bluetooth Control Algorithm:** Facilitates comprehensive testing and control, allowing monitoring and manipulation of various values (mainly control system tuning) and providing a means to control the buggy using Bluetooth commands exclusively.

### Implementation Structure

The algorithms are implemented using three distinct classes: the `Encoders` class, the `Sensors` class, and the `MotorControl` class. The `MotorControl` class, inheriting from the `Encoders` and `Sensors` classes, encompasses the control algorithm responsible for coordinating the buggy's movement.

To centralize common definitions and macros used throughout the code, a dedicated source file named `ESP4DEFINITIONS` was created. This file includes essential declarations, pin configurations, and variable initializations, facilitating easy management and modification.
