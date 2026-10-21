# Gesture-Controlled Robotic Arm Using ROS 2 and OpenCV

## Project Overview

This project presents a custom-designed robotic arm developed using SolidWorks and simulated using ROS 2.

The robot's 3D parts and assembly were designed in SolidWorks. The robot model was exported using SW2URDF and integrated into ROS 2 through a URDF description package.

An OpenCV-based gesture-control system was developed to control the simulated robot and perform pick-and-place operations.

The project combines mechanical design, robot description, simulation, computer vision, and robotic manipulation in one workflow.

## Features

- Custom robotic arm designed in SolidWorks
- SolidWorks parts and assembly files included
- URDF generated using SW2URDF
- ROS 2 robot description package
- Launch and configuration files for the simulation
- OpenCV-based gesture recognition and control
- Simulated pick-and-place operation
- Custom objects and environment elements, including a table and box

## Technologies Used

- SolidWorks — 3D mechanical design and assembly
- SW2URDF — URDF generation
- ROS 2 Jazzy — Robot software framework
- URDF — Robot model description
- OpenCV — Computer vision and gesture processing
- Python — Gesture-control implementation

## Project Structure

The repository contains the robot description package, gesture-control source code, SolidWorks design files, and exported robot model files.

```text
gesture-controlled-robotic-arm/
├── robot_arm_description/
├── gesture_control/
├── SolidWorks/
├── SW2URDF_Export/
├── objects/
└── README.md
```

Note: Update the folder names above to match the actual repository structure.

## How It Works

1. The robotic arm is designed and assembled in SolidWorks.
2. The robot model is exported using SW2URDF.
3. The URDF and mesh files are integrated into a ROS 2 package.
4. The robot and environment are launched in the simulation.
5. The OpenCV-based gesture-control program processes hand gestures.
6. Gesture inputs are used to control the simulated robotic arm.
7. The robot performs the demonstrated pick-and-place operation.

## Requirements

- Ubuntu with ROS 2 Jazzy installed
- Python 3
- OpenCV
- A compatible webcam if live gesture input is used
- Required ROS 2 packages and dependencies used by this project

Additional dependencies may be needed depending on the simulation and control implementation.

## Build and Run

Clone this repository into the `src` directory of a ROS 2 workspace.

Build the packages:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Launch the robot description and simulation using the launch files included in the repository.

For example, the launch command will follow this pattern:

```bash
ros2 launch robot_arm_description gazebo.launch.py
```

Run the gesture-control Python program using the appropriate command for the file and dependencies included in this repository.

**Note:** Replace the placeholders with the actual package name, launch filename, and gesture-control command. Refer to the source files for the exact execution steps.

## Results

The robot was successfully simulated, and the OpenCV-based gesture-control system was used to demonstrate pick-and-place operation in the simulation.

## Future Improvements

- Improve gesture recognition accuracy and robustness
- Add more gesture-based robot commands
- Improve motion planning and trajectory control
- Integrate with physical robotic hardware
- Add more detailed simulation and testing scenarios

## Author

Developed as a robotics project integrating mechanical design, ROS 2, URDF, computer vision, and robotic manipulation.
