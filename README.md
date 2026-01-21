<<<<<<< HEAD
# Robotic Arm with Visual Grasping

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> An intelligent robotic arm system capable of autonomous object detection, localization, and grasping using computer vision and deep learning techniques.

## 🎯 Project Overview

This project develops a robotic manipulation system that combines computer vision with robotic control to achieve autonomous grasping of household objects. The system uses an Intel RealSense depth camera for 3D scene understanding and implements real-time object detection, pose estimation, and motion planning for reliable object manipulation.

### Key Features

- **🤖 Autonomous Operation**: Complete pipeline from object detection to grasping execution
- **👁️ Computer Vision**: YOLOv8-based object detection with 6DOF pose estimation  
- **📊 Real-time Performance**: <3 second end-to-end latency with ≥10 FPS processing
- **🛡️ Safety Systems**: Emergency stop, collision avoidance, and speed limiting
- **📈 High Accuracy**: ≥80% grasp success rate with ≤2cm positioning accuracy

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RGB-D Camera  │    │  Object Detection│    │  Pose Estimation│
│  (RealSense)    │───▶│     (YOLOv8)     │───▶│     (6DOF)      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                          │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Arm Control    │◀───│  Motion Planning │◀───│  Grasp Planning │
│   (ROS Driver)  │    │    (MoveIt!)     │    │   (Synthesis)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🛠️ Hardware Components

| Component | Model | Purpose | Cost (₹) |
|-----------|-------|---------|----------|
| RGB-D Camera | Intel RealSense D435 | 3D scene reconstruction | 18,000 |
| Robotic Arm | 6-DOF Educational Arm | Object manipulation | 45,000 |
| Gripper | 2-finger Adaptive | Object grasping | 8,000 |
| Computer | NVIDIA Jetson Xavier NX | AI inference & control | 25,000 |
| **Total** | | | **₹96,000** |

## 💻 Software Stack

### Core Technologies
- **ROS Noetic** - Robot Operating System framework
- **YOLOv8** - Real-time object detection
- **MoveIt!** - Motion planning and kinematics
- **OpenCV** - Computer vision processing
- **Open3D** - 3D point cloud processing

### Dependencies
```bash
# ROS packages
ros-noetic-moveit
ros-noetic-realsense2-camera
ros-noetic-vision-msgs

# Python packages  
torch torchvision
ultralytics
opencv-python
open3d
pyrealsense2
```

## 📋 Target Objects & Performance

### Supported Objects
- **Bottle** (cylindrical, 5-15cm height)
- **Mug** (cylindrical with handle, various sizes)  
- **Box** (rectangular, rigid materials)

### Performance Targets
| Metric | Target | Current |
|--------|--------|---------|
| Grasp Success Rate | ≥80% | TBD |
| End-to-End Latency | ≤3 seconds | TBD |
| Detection FPS | ≥10 FPS | TBD |
| Position Accuracy | ≤2cm error | TBD |

## 🚀 Quick Start

### Prerequisites
- Ubuntu 20.04 with ROS Noetic
- CUDA-capable GPU (GTX 1060+ or Jetson Xavier NX)
- Intel RealSense D435 camera
- Compatible 6-DOF robotic arm

### Installation

1. **Clone the repository**
```bash
git clone https://github.com/your-username/robotic-arm-visual-grasping.git
cd robotic-arm-visual-grasping
```

2. **Install dependencies**
```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python requirements
pip install -r requirements.txt
```

3. **Build the workspace**
```bash
catkin_make
source devel/setup.bash
```

4. **Run calibration**
```bash
roslaunch robot_grasping calibrate_camera.launch
```

### Basic Usage

1. **Start the complete system**
```bash
roslaunch robot_grasping full_system.launch
```

2. **Run a grasping demo**
```bash
rosservice call /grasp_object "object_name: 'bottle'"
```

3. **Monitor system status**
```bash
rostopic echo /system_status
```

## 📊 Development Progress

### Phase 1: Hardware Integration ✅
- [x] Camera and arm communication setup
- [x] ROS node architecture implementation
- [x] Basic safety systems integration

### Phase 2: Perception Module 🔄
- [x] YOLOv8 model training and optimization
- [x] 6DOF pose estimation implementation
- [ ] Real-time performance optimization

### Phase 3: Planning & Control 📅
- [ ] Grasp synthesis algorithm development
- [ ] MoveIt! motion planning integration
- [ ] Collision avoidance implementation

### Phase 4: System Integration 📅
- [ ] End-to-end testing and validation
- [ ] Performance benchmarking
- [ ] Safety system validation

## 🔬 Research & Innovation

### Novel Contributions
- **Efficient Real-time Pipeline**: Optimized for low-latency operation on edge hardware
- **Robust Grasp Planning**: Adaptive grasp synthesis considering object geometry
- **Safety-first Design**: Comprehensive safety monitoring and emergency response

### Future Enhancements
- Multi-object scene manipulation
- Dynamic object tracking and grasping
- Tactile feedback integration
- Mobile platform integration

## 📈 Testing & Validation

### Test Scenarios
1. **Single Object Grasping**: Individual objects in controlled conditions
2. **Multi-object Scenes**: Object selection in cluttered environments  
3. **Lighting Variations**: Performance under different illumination
4. **Error Recovery**: System response to failed grasp attempts

### Success Metrics
- Quantitative performance against KPI targets
- Qualitative assessment of system robustness
- Safety compliance validation
- User experience evaluation

## 👥 Team

- **Hezion Wilfred** (URK23CS7006) - Team Leader, System Integration
- **Jinto Joseph** (URK24CS1210) - Computer Vision & ML
- **Basil Shaji** - Project Mentor & Guide

## 📚 Documentation

- **Requirement Analysis** – Defines the project goals, hardware/software needs, dataset strategy, and evaluation metrics.  
  [📖 Read Full Document](https://docs.google.com/document/d/e/2PACX-1vTWHIKLwAYOayBMhqomi4HyCJyX6zqyl8QpjLR8-sovp_6rvmV_aca9dDCFIUuPn8mCMpcf9qqHAqK1/pub)  
- [System Design](https://docs.google.com/document/d/e/2PACX-1vQWGvF9tzwzcWJiz9-WvZGufjkWumws61UdmAHC1DF8FfClAnKO5x-FtkmKsAKcBcEzFPeYtYDkS2Y_/pub) – Technical architecture overview  
- [API Reference](docs/api.md) – ROS topics, services, and messages  
- [User Guide](docs/user_guide.md) – Operation and maintenance manual  
- [Developer Guide](docs/dev_guide.md) – Setup and development instructions

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**⭐ Star this repository if you find it useful!**

*Last updated: August 30, 2025*
=======
# ros2_control_demos

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository provides examples for functionalities and capabilities of `ros2_control` framework.
It consists of simple implementations that demonstrate different concepts. Choose the right branch of this repository matching you ROS 2 distribution as well as the full documentation on [control.ros.org](https://control.ros.org), see [this table](#build-status).

If you want to have rather step by step manual how to do things with `ros2_control` checkout the [ros-control/roscon2022_workshop](https://github.com/ros-controls/roscon2022_workshop) repository.

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/ros2_control_demos/pulls) and review it, or [create your own](https://github.com/ros-controls/ros2_control_demos/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started. We are happy to help you with your first contribution.

## Getting Started

Follow the steps provided in the [documentation](https://control.ros.org/humble/doc/ros2_control_demos/doc/index.html#installation) to install ros2_control_demos.

## Content

The following examples are part of this demo repository:

* Example 1: [*RRBot*](example_1)

   *RRBot* - or ''Revolute-Revolute Manipulator Robot'' - a simple position controlled robot with one hardware interface. This example also demonstrates the switching between different controllers.

* Example 2: [*DiffBot*](example_2)

   *DiffBot*, or ''Differential Mobile Robot'', is a simple mobile base with differential drive.
   The robot is basically a box moving according to differential drive kinematics.

* Example 3: ["RRBot with multiple interfaces"](example_3)

   *RRBot* with multiple interfaces.

* Example 4: ["Industrial robot with integrated sensor"](example_4)

   *RRBot* with an integrated sensor.

* Example 5: ["Industrial robots with externally connected sensor"](example_5)

   *RRBot* with an externally connected sensor.

* Example 6: ["Modular robots with separate communication to each actuator"](example_6)

   The example shows how to implement robot hardware with separate communication to each actuator.

* Example 7: ["6-DOF robot"](example_7)

   A full tutorial for a 6 DOF robot for intermediate ROS 2 users.

* Example 8: ["Using transmissions"](example_8)

   *RRBot* with an exposed transmission interface.

* Example 9: ["Gazebo simulation"](example_9)

   Demonstrates how to switch between simulation and hardware.

* Example 10: ["Industrial robot with GPIO interfaces"](example_10)

   *RRBot* with GPIO interfaces.

* Example 11: ["Car-like robot using steering controller library"](example_11)

* Example 12: ["Controller chaining"](example_12)

   The example shows a simple chainable controller and its integration to form a controller chain to control the joints of *RRBot*.

* Example 13: ["Multi-robot system with hardware lifecycle management"](example_13)

   This example shows how to handle multiple robots in a single controller manager instance.

* Example 14: ["Modular robots with actuators not providing states and with additional sensors"](example_14)

   The example shows how to implement robot hardware with actuators not providing states and with additional sensors.

* Example 15: ["Using multiple controller managers"](example_15)

   This example shows how to integrate multiple robots under different controller manager instances.

## Structure

The repository is structured into `example_XY` folders that fully contained packages with names `ros2_control_demos_example_XY`.

The packages have following structure of subfolders:

* `bringup` - stores launch files and runtime configurations for demo robots.
* `description` - stores URDF (and XACRO) description files, rviz configurations and meshes for the example robots.
* `hardware` - stores implementations of example hardware components (interfaces).
* `controllers` (optional) - stores implementation of example controllers.

The important files to check in each example are:

* `bringup/launch/<example_name>.launch.py` - launch file for the example
* `bringup/config/<example_name>_controllers.yaml` - parameters with controllers' setup for the example.
* `description/<example_name>.ros2_control.xacro` - XACRO file with `ros2_control`-URDF-tag with hardware setup and parameters.
* `description/<example_name>.urdf.xacro` - the main description for for the example used to generate URDF on the fly that is published on the `/robot_description` topic.
* `hardware/<example_name>.hpp` - header file of the example hardware component implementation.
* `hardware/<example_name>.cpp` - source file with the example hardware component implementation.
* `controllers/<example_name>.hpp` - header file of the example controller implementation.
* `controllers/<example_name>.cpp` - source file with the example controller implementation.

**NOTE** - The structure of packages, folders and files given in this repository is not recommended to be used for your robot. Usually you should have all of the above folders defined as separate packages with naming convention `<robot_name_or_type>/[bringup|description|hardware|controllers]`.
  More standard structure can be found in [ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate) repository from Dave Coleman or documentation on [ros_team_workspace](https://rtw.stoglrobotics.de/master/guidelines/robot_package_structure.html) from Stogl Robotics.

The concepts in this package are demonstrated on the examples of *RRBot* and *DiffBot*.
Those two world-known imaginary robots are trivial simulations to demonstrate and test `ros2_control` concepts.

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-binary-build.yml?branch=master) <br> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/rolling-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/rolling/doc/ros2_control_demos/doc/index.html)
**Kilted** | [`master`](https://github.com/ros-controls/ros2_control_demos/tree/master) | see above | [Documentation](https://control.ros.org/kilted/doc/ros2_control_demos/doc/index.html)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/ros2_control_demos/tree/jazzy) | [![Jazzy Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/jazzy-binary-build.yml?branch=master) <br> [![Jazzy Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/jazzy-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/jazzy-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/jazzy/doc/ros2_control_demos/doc/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_control_demos/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-binary-build.yml?branch=master) <br> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_control_demos/actions/workflows/humble-semi-binary-build.yml?branch=master) | [Documentation](https://control.ros.org/humble/doc/ros2_control_demos/doc/index.html)

## Acknowledgements

The project has received major contributions from companies and institutions [listed on control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
>>>>>>> humble
