---
sidebar_position: 4
---

# Actuator Specifications

## Overview

Actuators are the muscles of robotic systems, converting control signals into physical motion. This section details the specifications and requirements for different types of actuators used in humanoid and physical AI robotics applications.

## Types of Actuators

### Servo Motors

#### Standard Hobby Servos
- **Torque**: 10-50 kg·cm at 6V
- **Speed**: 0.1-0.3 s/60° rotation
- **Control**: PWM signal (50 Hz, 1-2ms pulse width)
- **Position Feedback**: Built-in potentiometer
- **Communication**: Simple analog or digital protocols

#### High-Performance Servos
- **Torque**: 50-200 kg·cm at 7.4-12V
- **Speed**: 0.05-0.15 s/60° rotation
- **Resolution**: 1024-4096 position feedback
- **Communication**: Serial bus (RS-485, CAN)
- **Features**: Position, velocity, current control

#### Robot-Specific Servos
- **Dynamixel Series**: Industry standard for humanoid robots
- **Lynxmotion**: High-torque servo options
- **Herkulex**: Advanced control features
- **Futaba**: High-precision servos

### Brushless DC Motors

#### Gimbal Motors
- **Torque Constant**: 20-80 mNm/A
- **Speed Range**: 0-1000 RPM
- **Efficiency**: >85% efficiency
- **Control**: BLDC with ESC (Electronic Speed Controller)
- **Applications**: High-speed, precise positioning

#### Industrial BLDC Motors
- **Power Range**: 100W-5kW depending on application
- **Torque**: 0.1-50 Nm continuous torque
- **Speed**: 100-10000 RPM
- **Control**: Advanced servo drives
- **Feedback**: Encoder, resolver, or hall sensors

### Stepper Motors

#### NEMA Standard Steppers
- **Step Angle**: 1.8° (200 steps/revolution) or 0.9° (400 steps/revolution)
- **Holding Torque**: 0.1-50 Nm depending on size
- **Resolution**: Microstepping up to 256x
- **Control**: Step and direction signals
- **Positioning**: Open-loop precise positioning

#### High-Torque Steppers
- **Planetary Gearheads**: 5:1 to 100:1 reduction
- **Integrated Drivers**: Built-in microstepping controllers
- **Encoder Feedback**: Closed-loop operation
- **Brake Options**: Hold position when powered off

## Humanoid Robot Actuator Requirements

### Torque Requirements by Joint

#### Neck/Head Joints
- **Yaw**: 5-10 Nm (horizontal rotation)
- **Pitch**: 8-15 Nm (up/down movement)
- **Roll**: 5-10 Nm (tilting)
- **Speed**: 30-90°/s for natural movement

#### Arm Actuators
- **Shoulder (3 DOF)**: 20-50 Nm each axis
- **Elbow (1-2 DOF)**: 15-30 Nm
- **Wrist (2-3 DOF)**: 5-15 Nm each
- **Gripper/Hand**: 2-10 Nm for grasping

#### Torso Actuators
- **Waist (2-3 DOF)**: 30-100 Nm each
- **Chest (optional)**: 10-20 Nm
- **Balance**: High-speed response capability

#### Leg Actuators
- **Hip (3 DOF)**: 50-150 Nm each
- **Knee (1 DOF)**: 50-120 Nm
- **Ankle (2 DOF)**: 20-50 Nm each
- **Foot (optional)**: 10-20 Nm

### Speed Requirements
- **Slow Movements**: 10-30°/s (deliberate actions)
- **Normal Movements**: 30-90°/s (natural human-like motion)
- **Fast Movements**: 90-180°/s (quick reactions)
- **Emergency**: 180-360°/s (safety responses)

## Actuator Control Systems

### Control Types

#### Position Control
- **Accuracy**: ±0.1-1° depending on application
- **Response**: Fast settling time (under 100ms)
- **Smoothness**: Low ripple and vibration
- **Holding**: Maintains position under load

#### Velocity Control
- **Range**: 0.1-1000°/s continuous operation
- **Accuracy**: ±1-5% of commanded velocity
- **Response**: Fast acceleration/deceleration
- **Stability**: No oscillation at target velocity

#### Torque Control
- **Range**: 0-100% of rated torque
- **Accuracy**: ±2-5% of commanded torque
- **Response**: Fast torque response (&lt;10ms)
- **Safety**: Current limiting and thermal protection

### Feedback Systems

#### Encoders
- **Incremental**: 1000-4000 counts per revolution
- **Absolute**: Multi-turn absolute position
- **Resolution**: 12-17 bit resolution
- **Accuracy**: ±0.01-0.1° depending on type

#### Current Sensing
- **Precision**: ±1-5% current measurement
- **Bandwidth**: 1-10 kHz for control loops
- **Isolation**: Galvanic isolation for safety
- **Thermal**: Temperature compensation

## Power and Efficiency

### Power Requirements

#### Continuous Operation
- **Servos**: 5-50W depending on size and load
- **BLDC**: 50-500W for typical robotic joints
- **Steppers**: 10-100W in operation, 1-5W holding
- **Total System**: 500W-5000W for full humanoid

#### Peak Power
- **Acceleration**: 2-5x continuous power during motion
- **Stall Conditions**: 3-10x continuous power during stall
- **Efficiency**: 70-90% depending on motor type
- **Thermal**: Adequate cooling for sustained operation

### Efficiency Considerations
- **Motor Efficiency**: 75-90% for well-designed motors
- **Drive Efficiency**: 90-95% for modern servo drives
- **Overall System**: 70-85% system efficiency
- **Standby**: Low power consumption when idle

## Safety and Reliability

### Safety Features

#### Current Limiting
- **Hardware Protection**: Immediate current limiting
- **Software Protection**: Configurable current limits
- **Thermal Protection**: Temperature monitoring and shutdown
- **Stall Protection**: Automatic shutdown during stall

#### Mechanical Safety
- **Gearbox Rating**: Proper safety factor for loads
- **Brake Systems**: Holding brakes for vertical axes
- **Backdrive**: Prevention of backdriving where needed
- **Emergency Stop**: Immediate motor disable capability

### Reliability Metrics
- **MTBF**: 10,000-100,000 hours depending on quality
- **Duty Cycle**: 50-100% depending on cooling
- **Environmental**: IP54 to IP67 protection ratings
- **Lifespan**: 5-10 years under normal operation

## Communication Protocols

### Serial Communication
- **RS-485**: Multi-drop communication for multiple servos
- **CAN Bus**: Real-time communication with high reliability
- **Ethernet**: High-speed communication for advanced control
- **USB**: Configuration and debugging interface

### Protocol Features
- **Multi-turn**: Absolute position over multiple rotations
- **Real-time**: Low-latency communication for control
- **Diagnostics**: Built-in motor health monitoring
- **Synchronization**: Coordinated motion across multiple joints

## Popular Actuator Platforms

### Dynamixel Series
- **AX/MX Series**: Basic servo functionality
- **XL/XL430**: Advanced features with position feedback
- **XH/XM/XL430**: High-performance options
- **RH-P12-RN**: Robot hand with integrated control

### Herkulex Series
- **DRC-0101**: High-torque option
- **DRC-0201**: Mid-range performance
- **Advantages**: Advanced control features, daisy chain capability

### Industrial Options
- **MAXON EC Motors**: High-precision brushless motors
- **Faulhaber Motors**: Compact high-performance options
- **Oriental Motor**: Integrated stepper solutions
- **Parker Hannifin**: Industrial-grade actuators

## Control Architecture

### Master-Slave Configuration
- **Master Controller**: High-level motion planning
- **Slave Drivers**: Local servo control
- **Communication**: Real-time network communication
- **Synchronization**: Coordinated motion control

### Distributed Control
- **Local Processing**: Each joint has local intelligence
- **Network Communication**: High-speed real-time network
- **Modularity**: Easy replacement and maintenance
- **Scalability**: Add/remove joints as needed

## Cost Considerations

### Budget Actuators (&lt;$100/unit)
- **Hobby Servos**: Basic position control
- **Simple Steppers**: Open-loop control
- **DC Motors**: With basic encoders
- **Applications**: Educational projects

### Mid-Range ($100-$500/unit)
- **Robot Servos**: Position, velocity, torque control
- **Integrated Steppers**: Closed-loop operation
- **Basic BLDC**: With encoder feedback
- **Applications**: Research robots

### Professional ($500-$2000+/unit)
- **High-Torque Servos**: For humanoid robots
- **Industrial BLDC**: Precision control
- **Integrated Systems**: Motor + driver + controller
- **Applications**: Professional robotics

## Selection Guidelines

### Application-Based Selection
- **Precision**: Encoder resolution and control accuracy
- **Torque**: Required force/torque for application
- **Speed**: Required velocity and acceleration
- **Environment**: Temperature, humidity, dust protection
- **Communication**: Integration with control system
- **Safety**: Required safety features and certifications

### Performance Trade-offs
- **Cost vs. Performance**: Balance budget with requirements
- **Size vs. Power**: Compact vs. high-performance options
- **Open vs. Closed Loop**: Position feedback requirements
- **Standard vs. Custom**: Off-the-shelf vs. custom solutions