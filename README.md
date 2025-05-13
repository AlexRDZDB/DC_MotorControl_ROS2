# DC Motor Velocity Controller with ROS2

This project provides a ROS2-based framework for controlling the rotational velocity of a JGA25-371 DC motor using a closed-loop PID controller. The system integrates an ESP32 microcontroller with ROS2 via the Micro-ROS library to create a responsive and modular control system suitable for embedded robotics applications.

---

## 🚀 Project Objectives

- **Microcontroller Communication**: Interface an ESP32 with a host computer running ROS2 using the [Micro-ROS](https://micro.ros.org/) library.
- **Velocity Measurement**: Calculate the motor's rotational velocity from quadrature encoder data on the ESP32.
- **ROS2 Integration**: Stream encoder-based velocity data to ROS2 topics for real-time monitoring and control.
- **Closed-Loop Control**: Implement a PID controller in ROS2 that adjusts motor speed to match a specified setpoint.
- **Waveform Testing**: Evaluate controller response to step, sine, and square wave input profiles.

---

## 📦 Package Description

### `control_node`

This ROS2 node implements a PID controller for velocity regulation. It performs the following tasks:

- **Subscriptions**:
  - `/motor_output` (`std_msgs/msg/Float32`): Receives the current motor velocity as computed by the ESP32.
  - `/set_point` (`std_msgs/msg/Float32`): Receives the desired target velocity.

- **Publication**:
  - `/pwm` (`std_msgs/msg/Float32`): Publishes a normalized PWM command between `-1.0` and `1.0`, where the sign indicates direction and the magnitude represents speed. A value of `0.0` corresponds to zero velocity.

- **PID Control**:
  - The PID gains are configurable via ROS2 parameters.
  - The control output is bounded to prevent integral windup and excessive actuation.

---

### `input_node`

This ROS2 node generates dynamic setpoints for testing controller performance. It supports the following input profiles:

- **Step Function**
- **Sine Wave**
- **Square Wave**

- **Publication**:
  - `/set_point` (`std_msgs/msg/Float32`): Publishes the desired velocity setpoint.

- **Configuration**:
  - Setpoint type, amplitude, frequency, and update rate can be modified using ROS2 parameters.

---

## 🧰 Dependencies

- ROS2 Humble (or compatible distro)
- [micro-ROS Agent](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
- ESP32 with encoder interface and PWM output
- `rclpy`, `std_msgs`

## Running the Program

To launch the velocity control system, use the following ROS 2 launch command from your workspace root (after building and sourcing your environment):

```bash
ros2 launch control_node controller.launch.py
```
This will start both the controller node and the input node using their default parameters.
Optional: Provide Custom Parameter Files

You can optionally provide custom parameter files for each node by specifying them as launch arguments:

```bash
ros2 launch control_node controller.launch.py \
  controller_params_file:=/path/to/controller_params.yaml \
  input_params_file:=/path/to/input_params.yaml
```
- controller_params_file: Path to the YAML file containing PID controller and motor parameters.

- input_params_file: Path to the YAML file defining the setpoint signal parameters.

If no parameter files are provided, the program will use the hardcoded default values within each node.

- 🛠 Tip: Make sure you've sourced your ROS 2 workspace before launching:
  - `source install/setup.bash`

- 🧱 Build Reminder:
  - If you've made changes or added new files, rebuild with:

  - `colcon build --packages-select control_node input_node`
