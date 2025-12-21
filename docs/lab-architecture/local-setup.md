---
sidebar_position: 1
---

# Local Setup Architecture

## Overview

The local setup architecture provides a comprehensive framework for developing and testing physical AI and humanoid robotics applications on personal or laboratory computers. This approach offers maximum control and performance for development work.

## System Architecture

### Development Environment

#### Hardware Requirements
- **Workstation**: High-performance computer with GPU acceleration
- **Robot Interface**: Direct connection to robot hardware or simulation
- **Network**: Local network for multi-device communication
- **Peripherals**: Input devices, displays, and debugging tools

#### Software Stack
- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 Pro
- **ROS 2**: Humble Hawksbill distribution
- **NVIDIA Isaac ROS**: GPU-accelerated robotics packages
- **Simulation**: Gazebo Harmonic or Unity Robotics
- **Development Tools**: Visual Studio Code, Git, Docker

### Network Configuration

#### Local Network Setup
```
[Development Workstation]
    ├── ROS 2 Network (127.0.0.1, local network)
    ├── Robot Connection (USB/Ethernet/WiFi)
    ├── Simulation Environment (local)
    └── External Services (optional cloud access)
```

#### Network Performance
- **Bandwidth**: 1 Gbps minimum, 10 Gbps recommended
- **Latency**: &lt;1ms for real-time communication
- **Reliability**: 99.9% uptime for critical systems
- **Security**: Firewall and access control

## Installation and Configuration

### ROS 2 Setup

#### Installation Process
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

#### Environment Configuration
```bash
# Add to ~/.bashrc
source /opt/ros/humble/setup.bash
source /usr/share/colcon-bash/hook/colcon-argcomplete.bash

# Create ROS workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### NVIDIA Isaac ROS Setup

#### Prerequisites
- **NVIDIA GPU**: Compatible with CUDA 11.8+
- **NVIDIA Driver**: Version 520 or higher
- **CUDA Toolkit**: Version 11.8 or 12.x
- **Docker**: With NVIDIA Container Toolkit

#### Installation
```bash
# Install NVIDIA Isaac ROS dependencies
sudo apt install nvidia-isaac-ros-dev nvidia-isaac-ros-gxf

# Verify installation
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

## Development Workflow

### Project Structure
```
physical_ai_project/
├── src/
│   ├── robot_control/
│   ├── perception/
│   ├── planning/
│   └── vla/
├── config/
│   ├── robot.yaml
│   ├── controllers.yaml
│   └── vla_config.yaml
├── launch/
│   ├── simulation.launch.py
│   ├── real_robot.launch.py
│   └── vla_system.launch.py
├── models/
├── worlds/
├── scripts/
├── test/
└── docs/
```

### Development Cycle

#### 1. Simulation-First Development
```bash
# Launch simulation environment
ros2 launch my_robot simulation.launch.py

# Test perception nodes
ros2 run my_perception object_detector

# Validate control algorithms
ros2 run my_control controller_node
```

#### 2. Incremental Integration
```bash
# Test with simulated sensors
ros2 launch my_robot sim_with_real_perception.launch.py

# Gradual hardware integration
ros2 launch my_robot mixed_reality.launch.py

# Full hardware deployment
ros2 launch my_robot real_robot.launch.py
```

### Build System

#### Colcon Build Process
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select my_robot_package

# Parallel build
colcon build --parallel-workers 8

# Build with tests
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --ament-cmake-args --execute-tests-in-parallel
```

#### Package Management
```bash
# Create new package
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs my_robot_msgs my_perception_msgs

# List packages
ros2 pkg list

# Package info
ros2 pkg info my_robot_package
```

## Simulation Environment

### Gazebo Integration

#### Installation
```bash
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-ros

# Verify installation
gz sim --version
```

#### Configuration
```yaml
# config/gazebo_config.yaml
gazebo:
  world_file: "worlds/my_robot_world.sdf"
  physics_engine: "ode"
  max_step_size: 0.001
  real_time_factor: 1.0
  max_threads: 4
```

### Unity Integration

#### Setup Process
```bash
# Install Unity Hub and Unity 2022.3 LTS
# Install Unity Robotics Package
# Install ROS-TCP-Connector
# Configure ROS 2 bridge
```

## Performance Optimization

### Real-time Performance

#### System Configuration
```bash
# Configure real-time scheduling
echo 'ulimit -r 99' >> ~/.bashrc
echo 'echo 0 | sudo tee /proc/sys/kernel/rt_throttling_quota' >> ~/.bashrc

# Set CPU governor to performance mode
sudo cpupower frequency-set -g performance
```

#### Process Priority
```python
# Example real-time node
import rclpy
from rclpy.qos import QoSProfile
import threading

def set_realtime_priority():
    import os
    import ctypes
    from ctypes import c_int, c_ulong, POINTER

    # Set thread to real-time priority
    pid = os.getpid()
    libc = ctypes.CDLL("libc.so.6")
    sched_param = c_int * 1
    param = sched_param(99)  # Maximum real-time priority
    libc.sched_setscheduler(0, 1, param)  # SCHED_FIFO
```

### GPU Acceleration

#### CUDA Optimization
```python
# Example GPU-accelerated perception
import torch
import torchvision

# Enable GPU acceleration
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = MyPerceptionModel().to(device)

# Optimize for inference
model.eval()
with torch.no_grad():
    result = model(input_tensor)
```

## Debugging and Monitoring

### ROS 2 Tools

#### Command Line Tools
```bash
# Monitor topics
ros2 topic echo /camera/rgb/image_raw --field data --field header.stamp

# Check system status
ros2 lifecycle list
ros2 run topic_tools relay /original_topic /new_topic

# Service calls
ros2 service call /set_parameters rcl_interfaces/srv/SetParameters
```

#### Visualization Tools
```bash
# Launch RViz2
ros2 run rviz2 rviz2

# Launch rqt
ros2 run rqt rqt

# Plot data
ros2 run rqt_plot rqt_plot
```

### Custom Monitoring

#### Performance Monitoring Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import psutil
import GPUtil

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        self.cpu_pub = self.create_publisher(Float32, 'system/cpu_usage', 10)
        self.gpu_pub = self.create_publisher(Float32, 'system/gpu_usage', 10)
        self.ram_pub = self.create_publisher(Float32, 'system/ram_usage', 10)

        self.timer = self.create_timer(1.0, self.monitor_system)

    def monitor_system(self):
        cpu_msg = Float32()
        cpu_msg.data = psutil.cpu_percent()
        self.cpu_pub.publish(cpu_msg)

        ram_msg = Float32()
        ram_msg.data = psutil.virtual_memory().percent
        self.ram_pub.publish(ram_msg)

        gpus = GPUtil.getGPUs()
        if gpus:
            gpu_msg = Float32()
            gpu_msg.data = gpus[0].load * 100
            self.gpu_pub.publish(gpu_msg)
```

## Security and Safety

### Network Security
- **Firewall Configuration**: Restrict network access to necessary ports
- **Authentication**: Secure robot communication channels
- **Encryption**: Encrypt sensitive data transmission
- **Access Control**: Limit access to authorized users

### Safety Systems
- **Emergency Stop**: Hardware and software emergency stops
- **Limit Checking**: Joint position and velocity limits
- **Collision Detection**: Software-based collision prevention
- **Monitoring**: Continuous system health monitoring

## Maintenance and Updates

### System Updates
```bash
# Update ROS 2 packages
sudo apt update && sudo apt upgrade

# Update workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Backup Strategy
- **Code Backup**: Regular git commits and remote repositories
- **Configuration Backup**: Version-controlled configuration files
- **Model Backup**: Backup of trained AI models
- **Data Backup**: Regular backup of collected data