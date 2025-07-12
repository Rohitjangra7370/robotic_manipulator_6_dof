# 🤖 Autonomous 6-DOF Robotic Manipulator

**A ROS2-powered robotic arm system with advanced perception, inverse kinematics, and autonomous manipulation capabilities.**

## ✨ Key Features

- 🦾 **6-DOF Articulated Arm** - Full workspace manipulation with Robotiq 85 gripper
- 🎯 **MoveIt2 Integration** - Advanced motion planning and collision avoidance
- 🌐 **Gazebo Simulation** - High-fidelity physics simulation environment
- 📷 **Computer Vision Ready** - Integrated camera system for perception tasks
- 🎮 **ros2_control** - Real-time joint control and hardware abstraction
- 🔧 **Modular Architecture** - Easy configuration and extensibility
- 🚀 **Autonomous Operation** - Pose-based goal commanding and execution

## 🛠️ Tech Stack

| **Category** | **Technologies** |
|--------------|------------------|
| **Framework** | ROS2 (Humble/Iron), MoveIt2 |
| **Simulation** | Gazebo Garden, ros_gz_sim |
| **Languages** | C++17, Python 3.8+ |
| **Control** | ros2_control, joint_trajectory_controller |
| **Build System** | CMake, ament_cmake |
| **Visualization** | RViz2, joint_state_publisher |
| **Dependencies** | geometry_msgs, moveit_core, hardware_interface |

## 📁 Project Structure

```
robotic_manipulator_6_dof/
├── README.md
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── kinematics.yaml          # MoveIt2 kinematics configuration
│   └── ros2_controllers.yaml    # Joint controllers setup
├── launch/
│   ├── controller.launch.py     # Main controller launcher
│   ├── display.launch.py        # Complete system launcher
│   ├── gazebo_launch.py         # Gazebo simulation setup
│   └── robot_state_publisher_launch.py
├── meshes/                      # STL files for gripper components
│   ├── robotiq_85_base_link_fine.STL
│   └── [other gripper meshes]
├── src/
│   └── nodes/
│       └── pose_goal_commander.cpp  # Autonomous pose control node
└── urdf/
    ├── robot.urdf.xacro         # Main robot description
    ├── gripper.urdf.xacro       # Robotiq 85 gripper definition
    ├── camera.urdf.xacro        # Camera sensor integration
    └── robot.urdf               # Compiled URDF file
```

## 🔄 System Workflow

The robotic system operates through a sophisticated **perception → planning → execution** pipeline:

1. **🔍 Perception**: Camera captures workspace environment and object positions
2. **🧠 Planning**: MoveIt2 computes optimal trajectories using inverse kinematics
3. **⚡ Execution**: ros2_control executes joint commands through hardware interface
4. **🎯 Feedback**: Joint state feedback ensures precise motion control

**Key Components:**
- **PoseGoalCommander**: Receives target poses and orchestrates motion planning
- **MoveGroupInterface**: Handles complex trajectory planning and collision checking
- **JointTrajectoryController**: Manages smooth joint interpolation and execution

## 🚀 Quick Start

### Prerequisites
```bash
# Install ROS2 Humble/Iron
sudo apt update && sudo apt install ros-humble-desktop-full

# Install MoveIt2 and dependencies
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers

# Install Gazebo Garden
sudo apt install ros-humble-ros-gz-sim
```

### Build & Launch
```bash
# Clone the repository
git clone https://github.com/rohitjangra7370/robotic_manipulator_6_dof.git
cd robotic_manipulator_6_dof

# Build the package
colcon build --packages-select robotic_manipulator_6_dof

# Source the workspace
source install/setup.bash

# Launch the complete system
ros2 launch robotic_manipulator_6_dof display.launch.py

# In a new terminal, run the pose commander
ros2 run robotic_manipulator_6_dof pose_goal_commander
```

### Send Commands
```bash
# Publish target poses to the robot
ros2 topic pub /pose_goal geometry_msgs/msg/Pose "{
  position: {x: 0.5, y: 0.2, z: 0.3},
  orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
}"
```

## 🎬 Demo


```markdown
![Robot Demo](./images/robot_demo.gif)
*6-DOF robotic arm performing autonomous pick-and-place operations*
```

## 🔮 Future Improvements

- [ ] **YOLO Integration** - Real-time object detection and classification
- [ ] **Reinforcement Learning** - Adaptive manipulation strategies
- [ ] **Multi-Robot Coordination** - Collaborative manipulation tasks
- [ ] **Force/Torque Sensing** - Compliant manipulation capabilities
- [ ] **ROS2 Navigation Integration** - Mobile manipulation platform
- [ ] **Digital Twin** - Real-time simulation synchronization
- [ ] **Voice Control Interface** - Natural language command processing
- [ ] **Performance Optimization** - Real-time trajectory optimization

## 📄 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Author

**Rohit Jangra** - *Robotics Engineer*  
📧 Email: rohitjangra7370@gmail.com  
🔗 GitHub: [@rohitjangra7370](https://github.com/rohitjangra7370)


  Built with ❤️ using ROS2 and MoveIt2
