# DC Motor Velocity Controller with ROS2

This project contains ROS2 nodes meant for communicating and interacting with an ESP32 Microcontroller and a JGA25-371 Electric Motor in order to control it's rotational velocity by implementing a closed-loop PID Controller.

## Project Objectives
- Communicate an ESP32 to a Microprocessor running ROS2 via the use of MicroROS Library
- Obtain Rotational Velocity by processing encoder data in ESP32 microcontroller
- Send rotation velocity information through ROS topics
- Create a PID controller that takes sensor data and a setpoint and controls the speed to reach specified setpoint

## Package Description

