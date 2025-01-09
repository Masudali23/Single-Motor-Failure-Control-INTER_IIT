# IdeaForge Inter IIT 13.0 Project: Robust Position and Altitude Control Quadrotor with Single Motor Failure

## Table of Contents
- [Overview](#overview)
- [Video Demonstration](#video-demonstration)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Proof of Concept](#proof-of-concept)
- [Performance Metrics](#performance-metrics)
- [Challenges and Lessons Learned](#challenges-and-lessons-learned)
- [Future Scope](#future-scope)
- [References](#references)
- [Contributors](#contributors)

## Overview
This repository contains the implementation of a robust position and altitude control algorithm designed to handle single motor failure in quadrotors. The project, presented at **Inter IIT Tech Meet 13.0**, enhances the PX4 firmware to detect motor failures dynamically and ensure stable flight under adverse conditions.

### Modifications and Changes
Key modifications and enhancements include:
- **Motor Failure Plugin**: Integrated a plugin to simulate motor failures for testing stability and control responses.
- **EKF2 Tuning**: Fine-tuned the Extended Kalman Filter (EKF2) parameters to enhance state estimation accuracy.
- **Custom Parameters and Topics**: Introduced new parameters such as `EKF2_MOT_F` and custom uORB topics for better communication and control.
- **Control Algorithm Enhancements**: Updated control logic to maintain stability during motor failures, including adaptive control strategies using Principal Axis (PA), Nonlinear Dynamic Inversion (NDI), and Incremental NDI (INDI) algorithms.

## Video Demonstration
To see the firmware modifications in action, including the simulation of motor failure and the enhanced control algorithms, check out the video demonstration linked below:

[Video Demonstration of Modified PX4 Firmware for Motor Failure Scenarios](https://drive.google.com/drive/folders/1Y6v7U1SYYdSvZkmqinqmErPtkiqChUPh?usp=sharing)

## Features
### Motor Failure Detection
1. **RPY Error-Based Detection**: Detects sudden discrepancies in roll, pitch, and yaw during failures.
2. **ESC Data Monitoring**: Identifies abnormal current, voltage, or RPM metrics to localize failures.

### Enhanced PX4 Firmware
1. **Custom Modules**:
   - `motor_failure_detect_custom`: Detects motor failures and publishes states via `uORB`.
   - `accel_indi`: Publishes critical data for fault-tolerant control.
2. **EKF2 Enhancements**:
   - Introduced a new parameter `EKF2_MOT_F` for failure tracking.
3. **Control Logic**:
   - Adaptive control using **Principal Axis (PA)** + NDI and INDI algorithms.
   - Dynamic response through the PX4 Control Allocator.

## Installation
### Prerequisites
- **Operating System**: Ubuntu 20.04 LTS or later.
- **PX4 Autopilot**: Version 1.13.x or higher.
- **ROS Noetic**: Full installation ([Guide](http://wiki.ros.org/noetic)).
- **Gazebo**: Version 11.

### Steps
1. Clone the PX4 Autopilot repository:
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   ```
2. Run the PX4 installation script:
   ```bash
   bash Tools/setup/ubuntu.sh --no-sim-tools
   ```
3. Install additional dependencies:
   ```bash
   sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
   ```
4. Replace the following PX4 modules/files with the custom modules provided in this repository:
   - `motor_failure_detect_custom`
   - `accel_indi`
   - `mc_pos_control`
   - `mc_rate_control`

5. Rebuild the PX4 firmware:
   ```bash
   make px4_sitl gazebo-classic
   ```

---

## Usage
### Running the Simulation
1. Set up the Gazebo environment:
   ```bash
   source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
   ```
2. Launch the simulation:
   ```bash
   roslaunch px4 mavros_posix_sitl.launch
   ```
3. Start custom modules:
   ```bash
   motor_failure_detect_custom start
   accel_indi start
   ```
4. Simulate a motor failure by publishing the motor number:
   ```bash
   rostopic pub -1 /motor_failure/motor_number std_msgs/Int32 "data:<motor_number>"
   ```

---

## Proof of Concept

### Simulation in Gazebo

The verification of the fault-tolerant control system was conducted using Gazebo with the rotors simulator package. This setup allowed for the simulation of motor failure scenarios, and the results demonstrated the quadrotor's ability to maintain stability and perform controlled maneuvers despite a single motor failure.

**Key observations:**

- Exceptional stability during hovering.
- Controlled descents and landings.
- Precision in return-to-land (RTL) functionality.

These simulations confirmed the feasibility and reliability of the fault-tolerant control system under simulated real-world conditions.

---

## Performance Metrics
- **Drift Analysis**: X and Y-axis drifts minimized to ~0.15m and ~0.3m respectively.
- **Return-to-Land (RTL)**: Safe and accurate landings post-failure.
- **Angular Velocities**: Smooth compensation for disturbances.

---

## Challenges and Lessons Learned
1. **Motor Failure Dynamics**: Addressed thrust imbalance using adaptive control.
2. **Failsafe Management**: Enhanced robustness through dynamic parameter updates.
3. **Tuning Parameters**: Fine-tuned constants for improved response under varying conditions.

---

## Future Scope
1. **Upset Recovery**: Incorporate advanced techniques for rapid stabilization post-failure.
2. **Nonlinear Model Predictive Control (NMPC)**: Add an optimization layer for trajectory planning.
3. **Hardware Integration**: Deploy the solution on physical quadrotors for real-world testing.

---

## References
1. [Final Report](IdeaForge_Final.pdf)

2. [IEEE Robotics & Automation Magazine: Multirotor Aerial Vehicles](https://www.researchgate.net/publication/255786198_Multirotor_Aerial_Vehicles_Modeling_Estimation_and_Control_of_Quadrotor)
3. [IEEE RAL: High-Speed Flight of Quadrotor Despite Loss of Single Rotor](https://www.researchgate.net/publication/326021895_High-Speed_Flight_of_Quadrotor_Despite_Loss_of_Single_Rotor)


---

## Contributors
- **Team 34** - IdeaForge Inter IIT Tech Meet 13.0

