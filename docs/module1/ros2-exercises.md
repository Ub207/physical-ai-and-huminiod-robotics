---
sidebar_position: 5
---

# ROS 2 Exercises

## Exercise 1: Basic Publisher-Subscriber

Create a simple publisher-subscriber system to understand basic ROS 2 communication.

### Requirements
- Create a publisher that sends "Hello World" messages every second
- Create a subscriber that receives and prints the messages
- Use proper node lifecycle management

### Implementation Steps
1. Create a new ROS 2 package: `ros2_exercises`
2. Implement the publisher node
3. Implement the subscriber node
4. Create a launch file to start both nodes
5. Test the system and verify communication

### Expected Output
The subscriber should print received messages with timestamps.

## Exercise 2: Service Server and Client

Implement a simple calculator service to understand request-response communication.

### Requirements
- Create a service server that can add two integers
- Create a service client that sends requests to the server
- Handle service calls asynchronously

### Implementation Steps
1. Define a custom service message for addition
2. Implement the service server
3. Implement the service client
4. Test the service with multiple requests

## Exercise 3: Robot State Publisher

Create a system that publishes robot state information using the TF (Transform) system.

### Requirements
- Create a node that publishes joint states
- Use robot_state_publisher to broadcast transforms
- Visualize the robot in RViz

### Implementation Steps
1. Create a URDF model of a simple robot
2. Publish joint states messages
3. Configure robot_state_publisher
4. Visualize the robot in RViz

## Exercise 4: Parameter Server

Implement a system that uses ROS 2 parameters for configuration.

### Requirements
- Create a node that accepts parameters at runtime
- Use parameter callbacks to respond to parameter changes
- Create a configuration file for parameters

### Implementation Steps
1. Define parameters in your node
2. Implement parameter callbacks
3. Create parameter configuration files
4. Test parameter updates at runtime

## Exercise 5: Launch System

Create a comprehensive launch system for a multi-node robot application.

### Requirements
- Launch multiple nodes with a single command
- Configure nodes with parameters
- Handle node lifecycle and dependencies

### Implementation Steps
1. Create individual nodes for different robot functions
2. Create launch files to coordinate the nodes
3. Add conditional launching based on arguments
4. Test the complete system

## Assessment Criteria

Each exercise will be evaluated based on:
- **Correctness**: System functions as specified
- **Code Quality**: Clean, well-documented code
- **Architecture**: Proper ROS 2 patterns and practices
- **Testing**: Comprehensive testing of functionality
- **Documentation**: Clear explanations and usage instructions