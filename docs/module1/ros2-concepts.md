---
sidebar_position: 2
---

# ROS 2 Concepts

## Core Architecture

ROS 2 is built on a distributed system architecture that enables communication between processes (nodes) running on potentially different devices. The key architectural concepts include:

### Nodes
Nodes are the fundamental execution units in ROS 2. Each node is a process that performs a specific task and communicates with other nodes through messages. Nodes are organized in a peer-to-peer network where each node can communicate directly with any other node.

### Topics and Publishers/Subscribers
Topics provide a unidirectional communication pattern where data flows from publishers to subscribers. This decoupled communication model allows for flexible system design where publishers and subscribers don't need to know about each other.

### Services
Services provide a request-response communication pattern. A service client sends a request to a service server and waits for a response. This synchronous communication is useful for operations that require immediate feedback.

### Actions
Actions provide a goal-oriented communication pattern with feedback and status updates. They're ideal for long-running tasks that need to report progress and can be canceled.

## Quality of Service (QoS)

ROS 2 introduces Quality of Service policies that allow fine-tuning communication behavior:

- **Reliability**: Whether messages must be delivered reliably or can be lost
- **Durability**: Whether late-joining subscribers receive old messages
- **History**: How many messages to store for late-joining subscribers
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to determine if a publisher is still active

## Communication Middleware

ROS 2 uses DDS (Data Distribution Service) as its default middleware, providing:
- Discovery: Automatic detection of nodes and their interfaces
- Transport: Reliable and efficient message delivery
- Security: Authentication, encryption, and access control
- Real-time support: Deterministic behavior for time-critical applications

## Package Management

ROS 2 uses the `ament` build system for package management:
- **Packages**: Collections of related functionality
- **Metapackages**: Collections of related packages
- **Dependencies**: Automatic resolution of package dependencies
- **Build tools**: Compilation and linking of source code