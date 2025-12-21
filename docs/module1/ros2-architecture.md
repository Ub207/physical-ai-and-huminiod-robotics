---
sidebar_position: 3
---

# ROS 2 Architecture

## System Components

The ROS 2 architecture consists of several key components that work together to provide a complete robotic development framework.

### Client Libraries

ROS 2 provides client libraries for multiple programming languages:
- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rcl**: C client library (foundation for other libraries)
- **rclc**: C client library for microcontrollers
- **rclnodejs**: Node.js client library
- **rclrs**: Rust client library

### ROS Middleware Interface (RMW)

The ROS Middleware Interface abstracts the underlying communication middleware, allowing ROS 2 to work with different DDS implementations:
- **Fast DDS**: Default implementation from eProsima
- **Cyclone DDS**: Eclipse implementation
- **RTI Connext DDS**: Commercial implementation
- **OpenSplice DDS**: ADLINK implementation

### Launch System

The launch system provides a unified way to start and manage complex systems:
- **Launch files**: Declarative system configuration
- **Composable nodes**: Multiple nodes in a single process
- **Lifecycle nodes**: Nodes with explicit state management
- **Process management**: Monitoring and restart capabilities

## Communication Patterns

### Publisher-Subscriber Pattern

```python
# Example publisher
publisher = node.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
publisher.publish(msg)
```

```python
# Example subscriber
def callback(msg):
    print('Received: %s' % msg.data)

subscriber = node.create_subscription(
    String, 'topic_name', callback, 10)
```

### Service Pattern

```python
# Service server
def handle_request(request, response):
    response.result = request.a + request.b
    return response

service = node.create_service(AddTwoInts, 'add_two_ints', handle_request)
```

```python
# Service client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

### Action Pattern

Actions provide a more sophisticated communication pattern for long-running tasks with feedback:

- **Goal**: Request to start a long-running task
- **Feedback**: Periodic updates on task progress
- **Result**: Final outcome of the task

## Security Architecture

ROS 2 includes built-in security features:
- **Authentication**: Verify identity of nodes
- **Encryption**: Protect message contents
- **Access Control**: Limit what nodes can do
- **Audit Logging**: Track system activities