---
sidebar_position: 3
---

# Sensor Specifications

## Overview

Robotic perception relies on a variety of sensors to understand and interact with the physical world. This section details the specifications and requirements for different sensor types used in physical AI and humanoid robotics applications.

## Vision Sensors

### RGB Cameras
- **Resolution**: Minimum 1920x1080 (Full HD), recommended 4K
- **Frame Rate**: Minimum 30 FPS, recommended 60+ FPS
- **Interface**: USB 3.0+, GigE Vision, or MIPI CSI-2
- **Field of View**: 60-90° for general purpose, wider for navigation
- **Lens Quality**: Low distortion, auto-focus capability preferred

#### Recommended Models
- **Intel RealSense D435i**: RGB + depth with IMU
- **Azure Kinect**: High-quality RGB and depth sensing
- **Basler ace**: Industrial-grade camera with various options
- **FLIR Blackfly**: High-performance scientific cameras

### Depth Sensors
- **Depth Resolution**: Minimum 640x480, recommended 1280x720
- **Depth Accuracy**: Sub-centimeter accuracy at 1-3 meter range
- **Operating Range**: 0.3m to 10m effective range
- **Technology**: Stereo vision, structured light, or LiDAR
- **Update Rate**: Minimum 30 Hz, recommended 60 Hz

#### Depth Sensor Technologies
| Technology | Accuracy | Range | Speed | Cost |
|------------|----------|-------|-------|------|
| Stereo Vision | ±1cm | 0.3-8m | 30-60Hz | Medium |
| Structured Light | ±2mm | 0.3-1m | 30-90Hz | Medium |
| Time-of-Flight | ±1cm | 0.3-10m | 30-100Hz | High |
| LiDAR | ±1cm | 0.1-100m | 5-20Hz | High |

### Multi-spectral Sensors
- **RGB-NIR**: Visible and near-infrared imaging
- **Thermal**: FLIR cameras for heat signature detection
- **Hyperspectral**: For material identification
- **Polarization**: For surface property analysis

## LiDAR Sensors

### 2D LiDAR
- **Range**: Minimum 6m, recommended 10m+ range
- **Resolution**: 0.25° angular resolution minimum
- **Update Rate**: 5-20 Hz depending on application
- **Accuracy**: ±1-2cm range accuracy
- **Interface**: USB, Ethernet, or serial

#### Popular 2D LiDAR Models
- **Hokuyo UST-10LX**: Reliable indoor navigation
- **SICK TIM551**: Industrial-grade with Ethernet
- **YDLIDAR X4**: Cost-effective educational option
- **SLAMTEC RPLIDAR A3M1**: Good performance/cost ratio

### 3D LiDAR
- **Range**: 10-100m depending on model
- **Field of View**: 360° horizontal, 20-40° vertical
- **Point Rate**: 10,000-1,000,000 points per second
- **Accuracy**: ±1-3cm depending on range
- **Weight**: 1-10kg depending on performance

#### 3D LiDAR Options
- **Velodyne Puck**: Good balance of cost and performance
- **Ouster OS0**: Solid-state, no moving parts
- **Livox Mid-360**: Cost-effective rotating LiDAR
- **Hesai PandarQT**: High-performance solid-state option

## Inertial Measurement Units (IMU)

### Basic IMU Requirements
- **Accelerometer**: ±16g range, 4000 LSB/g resolution
- **Gyroscope**: ±2000°/s range, 16.4 LSB/(°/s) resolution
- **Magnetometer**: ±4800 µT range, 0.15 µT resolution
- **Update Rate**: Minimum 100 Hz, recommended 400+ Hz
- **Bias Stability**: &lt;10°/hr for gyros, &lt;1mg for accelerometers

### Advanced IMU Features
- **Temperature Compensation**: For stable operation
- **Kalman Filtering**: Integrated state estimation
- **Magnetometer Calibration**: Automatic hard/soft iron correction
- **Multiple Sensors**: Redundant sensors for reliability
- **ROS Integration**: Direct ROS message output

#### IMU Models
- **Xsens MTi**: High-precision with sensor fusion
- **VectorNav VN-100**: Good balance of features and cost
- **Adafruit BNO055**: Cost-effective for educational use
- **SparkFun IMU Breakout**: Easy integration options

## Force and Torque Sensors

### Wrist Force/Torque Sensors
- **Axes**: 6-axis (3 force + 3 torque) measurement
- **Range**: Configurable based on application (typically 50-500N, 5-50 Nm)
- **Resolution**: &lt;0.1N for forces, &lt;0.01 Nm for torques
- **Update Rate**: 1000+ Hz for real-time control
- **Accuracy**: &lt;1% of full scale

### Joint Torque Sensors
- **Type**: Strain gauge or magnetic torque sensors
- **Range**: Based on actuator specifications
- **Resolution**: &lt;0.1 Nm for precise control
- **Integration**: Built into actuators or external sensors
- **Safety**: Overload protection and fail-safe operation

## Tactile Sensors

### Fingertip Tactile Sensors
- **Resolution**: 16x16 to 64x64 taxel array
- **Force Range**: 0.1-10N per taxel
- **Update Rate**: 100+ Hz for dynamic tasks
- **Shape Adaptability**: Conform to object surfaces
- **Temperature Sensing**: Optional thermal feedback

### Tactile Skins
- **Coverage**: Large area tactile sensing
- **Resolution**: Variable depending on application
- **Flexibility**: Conform to complex geometries
- **Communication**: Distributed sensing network
- **Durability**: Robust to physical contact

## Audio Sensors

### Microphone Arrays
- **Channel Count**: 4-8 microphones for direction of arrival
- **Sample Rate**: 48 kHz minimum, 96 kHz recommended
- **Frequency Response**: 20 Hz - 20 kHz
- **Sensitivity**: -46 dBV/Pa minimum
- **Directivity**: Beamforming capability

### Audio Processing Requirements
- **Real-time Processing**: Low-latency audio processing
- **Noise Reduction**: Adaptive noise cancellation
- **Source Separation**: Multiple speaker handling
- **Keyword Spotting**: Wake word and command detection

## Environmental Sensors

### Temperature and Humidity
- **Temperature Range**: -40°C to +85°C
- **Accuracy**: ±0.3°C typical
- **Humidity Range**: 0-100% RH
- **Accuracy**: ±2% RH typical
- **Update Rate**: 1-10 Hz

### Air Quality
- **Gas Detection**: CO2, VOCs, particulates
- **Particulate Matter**: PM2.5, PM10 detection
- **Chemical Sensors**: Specific gas detection
- **Calibration**: Automatic calibration capability

## Sensor Fusion Considerations

### Synchronization
- **Hardware Sync**: Common sync signal for all sensors
- **Timestamp Accuracy**: Microsecond-level timestamping
- **Trigger Synchronization**: Coordinated sensor triggering
- **Clock Drift**: Compensation for clock variations

### Calibration
- **Intrinsic Calibration**: Internal sensor parameters
- **Extrinsic Calibration**: Sensor-to-sensor relationships
- **Temporal Calibration**: Time delay compensation
- **Automated Calibration**: Self-calibration capabilities

### Data Quality
- **Noise Characterization**: Understanding sensor noise
- **Outlier Detection**: Identifying and handling bad data
- **Data Validation**: Checking for sensor failures
- **Redundancy**: Multiple sensors for critical functions

## Integration Standards

### ROS 2 Sensor Support
- **sensor_msgs**: Standard message types
- **camera_info**: Camera calibration information
- **tf2**: Coordinate frame transformations
- **diagnostics**: Sensor health monitoring

### Communication Protocols
- **USB**: For most cameras and simple sensors
- **Ethernet**: For high-bandwidth or distributed sensors
- **CAN Bus**: For real-time sensor networks
- **SPI/I2C**: For embedded sensor integration

## Cost Considerations

### Budget Sensors (&lt;$500)
- **Cameras**: Basic RGB cameras, simple depth sensors
- **IMU**: Basic 9-axis IMU modules
- **LiDAR**: Entry-level 2D LiDAR
- **Audio**: Simple microphone arrays

### Professional Sensors ($500-$3000)
- **Cameras**: High-quality RGB-D cameras
- **IMU**: Calibrated IMU with sensor fusion
- **LiDAR**: Mid-range 2D or basic 3D LiDAR
- **Force/Torque**: Basic force/torque sensors

### Research Sensors (>$3000)
- **Cameras**: High-end thermal, hyperspectral, or scientific cameras
- **LiDAR**: High-performance 3D LiDAR
- **Force/Torque**: High-precision multi-axis sensors
- **Tactile**: Advanced tactile sensing arrays

## Selection Guidelines

### Application-Based Selection
- **Navigation**: LiDAR, RGB cameras, IMU
- **Manipulation**: Depth cameras, force/torque sensors, tactile
- **Human Interaction**: Audio sensors, RGB cameras, IMU
- **Research**: High-precision sensors with detailed specifications

### Performance Trade-offs
- **Accuracy vs. Cost**: Balance precision with budget
- **Range vs. Resolution**: Trade-offs in LiDAR selection
- **Update Rate vs. Power**: Consider power consumption
- **Size vs. Performance**: Compact vs. high-performance options