---
sidebar_position: 3
---

# Containerization for Robotics

## Overview

Containerization provides a powerful approach to package, deploy, and manage robotics applications. By using containers, developers can ensure consistent environments across different platforms, simplify dependency management, and enable scalable deployment of robotic systems.

## Containerization Benefits for Robotics

### Environment Consistency
- **Reproducible Builds**: Identical environments across development, testing, and deployment
- **Dependency Isolation**: Clean separation of application dependencies
- **Version Control**: Container images capture complete system state
- **Portability**: Applications run consistently across different hardware

### Deployment Flexibility
- **Edge Deployment**: Deploy to resource-constrained robot platforms
- **Cloud Integration**: Seamless integration with cloud services
- **Multi-platform**: Support for different operating systems and architectures
- **Rollback**: Easy rollback to previous versions

### Resource Management
- **Isolation**: Applications run in isolated environments
- **Resource Limits**: Control CPU, memory, and GPU usage
- **Efficiency**: Share resources while maintaining isolation
- **Scalability**: Scale applications based on demand

## Docker for Robotics

### Multi-stage Docker Builds

#### Optimized Build Process
```dockerfile
# Multi-stage build for robotics application
# Stage 1: Build environment
FROM ubuntu:22.04 as builder

# Install build dependencies
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 build tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 humble
RUN apt update && apt install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt update \
    && apt install -y ros-humble-desktop \
    && apt install -y python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool \
    && rm -rf /var/lib/apt/lists/*

# Source ROS environment
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

# Create workspace
WORKDIR /workspace
COPY . src/
WORKDIR /workspace/src

# Install dependencies
RUN apt update && rosdep install --from-paths . --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace
WORKDIR /workspace
RUN source /opt/ros/humble/setup.sh && colcon build --symlink-install

# Stage 2: Runtime environment
FROM ubuntu:22.04 as runtime

# Install runtime dependencies
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy built workspace
COPY --from=builder /workspace/install /opt/workspace/install

# Source ROS and workspace
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble:/opt/workspace/install
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/workspace/install/lib
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/workspace/install/lib/python3.10/site-packages

# Set up workspace
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
RUN echo "source /opt/workspace/install/setup.sh" >> ~/.bashrc

# Set working directory
WORKDIR /opt/workspace

# Default command
CMD ["bash"]
```

### GPU-Accelerated Robotics Containers

#### NVIDIA Container Toolkit Integration
```dockerfile
# Dockerfile for GPU-accelerated robotics
FROM nvidia/cuda:11.8-devel-ubuntu22.04

# Install ROS 2
RUN apt update && apt install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt update \
    && apt install -y ros-humble-desktop \
    && apt install -y python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install NVIDIA Isaac ROS packages
RUN apt update && apt install -y \
    nvidia-isaac-ros-dev \
    nvidia-isaac-ros-gxf \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
ENV ROS_DISTRO=humble
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=11.8 brand=tesla,driver>=470,driver&lt;471 brand=unknown,driver>=470,driver&lt;471 brand=nvidia,driver>=470,driver&lt;471 brand=nvidiartx,driver>=470,driver&lt;471 brand=geforce,driver>=470,driver&lt;471 brand=geforcertx,driver>=470,driver&lt;471 brand=quadro,driver>=470,driver&lt;471 brand=quadrortx,driver>=470,driver&lt;471 brand=titan,driver>=470,driver&lt;471 brand=titanrtx,driver>=470,driver&lt;471"

# Copy application code
COPY . /app
WORKDIR /app

# Install dependencies
RUN apt update && rosdep install --from-paths . --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build application
RUN source /opt/ros/humble/setup.bash && colcon build

# Set up entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

#### Entrypoint Script
```bash
#!/bin/bash
# entrypoint.sh

# Source ROS environment
source /opt/ros/humble/setup.bash
source /app/install/setup.bash

# Set ROS domain ID
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Set up network if needed
if [ -n "$ROS_HOSTNAME" ]; then
    export ROS_HOSTNAME
fi

if [ -n "$ROS_IP" ]; then
    export ROS_IP
fi

# Execute the command
exec "$@"
```

## Docker Compose for Robotics Systems

### Multi-container Robotics Application
```yaml
# docker-compose.yml for complete robotics system
version: '3.8'

services:
  # Robot perception system
  perception:
    build:
      context: ./perception
      dockerfile: Dockerfile
    runtime: nvidia
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - ROS_DOMAIN_ID=42
      - CUDA_VISIBLE_DEVICES=0
    volumes:
      - ./data:/data
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    network_mode: host
    privileged: true
    depends_on:
      - rosbridge

  # Motion planning system
  planning:
    build:
      context: ./planning
      dockerfile: Dockerfile
    environment:
      - ROS_DOMAIN_ID=42
    volumes:
      - ./maps:/maps
    depends_on:
      - perception

  # Robot control system
  control:
    build:
      context: ./control
      dockerfile: Dockerfile
    privileged: true
    devices:
      - /dev:/dev
      - /dev/kvm:/dev/kvm:rwm
    environment:
      - ROS_DOMAIN_ID=42
    depends_on:
      - planning

  # Visualization and monitoring
  visualization:
    build:
      context: ./visualization
      dockerfile: Dockerfile
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=$DISPLAY
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - /dev/dri:/dev/dri
    network_mode: host

  # ROS bridge for external communication
  rosbridge:
    image: rostooling/rosbridge-suite:latest
    environment:
      - ROS_DOMAIN_ID=42
    ports:
      - "9090:9090"
    network_mode: host

  # Data collection and storage
  data-collector:
    build:
      context: ./data-collector
      dockerfile: Dockerfile
    environment:
      - ROS_DOMAIN_ID=42
    volumes:
      - ./collected_data:/data
      - /var/run/docker.sock:/var/run/docker.sock
    depends_on:
      - perception
      - planning
      - control
```

## Kubernetes for Robotics

### Robot Deployment Configuration

#### Robot Service Definition
```yaml
# robot-service.yaml
apiVersion: v1
kind: Service
metadata:
  name: robot-service
  labels:
    app: robot-controller
spec:
  selector:
    app: robot-controller
  ports:
    - name: ros-communication
      protocol: TCP
      port: 11311
      targetPort: 11311
    - name: web-interface
      protocol: TCP
      port: 8080
      targetPort: 8080
    - name: monitoring
      protocol: TCP
      port: 9090
      targetPort: 9090
  type: LoadBalancer
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: robot-controller
spec:
  replicas: 1  # Only one replica for robot control
  selector:
    matchLabels:
      app: robot-controller
  template:
    metadata:
      labels:
        app: robot-controller
    spec:
      hostPID: true  # Required for direct hardware access
      containers:
      - name: controller
        image: robot-controller:latest
        resources:
          requests:
            memory: "4Gi"
            cpu: "2"
            nvidia.com/gpu: 1
          limits:
            memory: "8Gi"
            cpu: "4"
            nvidia.com/gpu: 1
        env:
        - name: ROS_DOMAIN_ID
          value: "42"
        - name: NVIDIA_VISIBLE_DEVICES
          value: "all"
        - name: HOST_IP
          valueFrom:
            fieldRef:
              fieldPath: status.hostIP
        securityContext:
          privileged: true
        volumeMounts:
        - name: robot-data
          mountPath: /data
        - name: device-access
          mountPath: /dev
          readOnly: false
        - name: shm
          mountPath: /dev/shm
        ports:
        - containerPort: 11311
          name: ros-master
        - containerPort: 8080
          name: web-interface
        - containerPort: 9090
          name: monitoring
      volumes:
      - name: robot-data
        persistentVolumeClaim:
          claimName: robot-data-pvc
      - name: device-access
        hostPath:
          path: /dev
      - name: shm
        emptyDir:
          medium: Memory
      nodeSelector:
        node-type: robot-node  # Dedicated robot nodes
---
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: robot-data-pvc
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 100Gi
```

### Robot State Management

#### Custom Resource Definition for Robots
```yaml
# robot-crd.yaml
apiVersion: apiextensions.k8s.io/v1
kind: CustomResourceDefinition
metadata:
  name: robots.robotics.example.com
spec:
  group: robotics.example.com
  versions:
  - name: v1
    served: true
    storage: true
    schema:
      openAPIV3Schema:
        type: object
        properties:
          spec:
            type: object
            properties:
              robotType:
                type: string
              hardwareConfig:
                type: object
              softwareConfig:
                type: object
              deploymentSpec:
                type: object
          status:
            type: object
            properties:
              phase:
                type: string
              conditions:
                type: array
                items:
                  type: object
  scope: Namespaced
  names:
    plural: robots
    singular: robot
    kind: Robot
---
apiVersion: robotics.example.com/v1
kind: Robot
metadata:
  name: humanoid-robot-01
spec:
  robotType: humanoid
  hardwareConfig:
    actuators: 24
    sensors: ["cameras", "lidar", "imu"]
  softwareConfig:
    perception: true
    planning: true
    control: true
  deploymentSpec:
    replicas: 1
    resources:
      requests:
        memory: "8Gi"
        cpu: "4"
        nvidia.com/gpu: 1
```

## Container Orchestration Patterns

### Robot Fleet Management

#### Fleet Deployment Strategy
```yaml
# robot-fleet.yaml
apiVersion: apps/v1
kind: DaemonSet
metadata:
  name: robot-agent
spec:
  selector:
    matchLabels:
      name: robot-agent
  template:
    metadata:
      labels:
        name: robot-agent
    spec:
      hostNetwork: true
      containers:
      - name: robot-agent
        image: robot-agent:latest
        env:
        - name: NODE_NAME
          valueFrom:
            fieldRef:
              fieldPath: spec.nodeName
        - name: POD_NAME
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        - name: POD_NAMESPACE
          valueFrom:
            fieldRef:
              fieldPath: metadata.namespace
        volumeMounts:
        - name: device-plugins
          mountPath: /var/lib/kubelet/device-plugins
        - name: robot-devices
          mountPath: /dev/robot
        securityContext:
          privileged: true
      volumes:
      - name: device-plugins
        hostPath:
          path: /var/lib/kubelet/device-plugins
      - name: robot-devices
        hostPath:
          path: /dev
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: robot-config
data:
  robot_config.yaml: |
    hardware:
      cameras: 2
      lidar: 1
      imu: 1
      actuators: 24
    software:
      perception:
        object_detection: true
        depth_estimation: true
      planning:
        path_planning: true
        motion_planning: true
      control:
        joint_control: true
        impedance_control: true
    networking:
      ros_domain: 42
      discovery_server: true
```

## Security in Containerized Robotics

### Security Best Practices

#### Secure Container Images
```dockerfile
# Secure base image with minimal privileges
FROM ubuntu:22.04

# Create non-root user
RUN groupadd -r robotuser && useradd -r -g robotuser robotuser

# Install only necessary packages
RUN apt update && apt install -y \
    ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy application as non-root user
COPY --chown=robotuser:robotuser . /app

# Switch to non-root user
USER robotuser

# Set up ROS environment
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages

# Build application
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select my_robot_package

# Expose necessary ports
EXPOSE 11311 8080

# Use exec form for CMD
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run my_robot_package main_node"]
```

#### Kubernetes Security Configuration
```yaml
# robot-security.yaml
apiVersion: v1
kind: ServiceAccount
metadata:
  name: robot-service-account
---
apiVersion: rbac.authorization.k8s.io/v1
kind: Role
metadata:
  namespace: robot-namespace
  name: robot-role
rules:
- apiGroups: [""]
  resources: ["pods", "services", "configmaps"]
  verbs: ["get", "list", "create", "update", "patch"]
---
apiVersion: rbac.authorization.k8s.io/v1
kind: RoleBinding
metadata:
  name: robot-rolebinding
  namespace: robot-namespace
subjects:
- kind: ServiceAccount
  name: robot-service-account
  namespace: robot-namespace
roleRef:
  kind: Role
  name: robot-role
  apiGroup: rbac.authorization.k8s.io
---
apiVersion: v1
kind: Namespace
metadata:
  name: robot-namespace
---
apiVersion: v1
kind: Secret
metadata:
  name: robot-credentials
type: Opaque
data:
  username: `<base64-encoded-username>`
  password: `<base64-encoded-password>`
```

## Monitoring and Logging

### Container Monitoring Setup

#### Prometheus Configuration for Robotics
```yaml
# prometheus-robot-config.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: prometheus-robot-config
data:
  prometheus.yml: |
    global:
      scrape_interval: 15s
    scrape_configs:
    - job_name: 'robot-nodes'
      kubernetes_sd_configs:
      - role: pod
      relabel_configs:
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_scrape]
        action: keep
        regex: true
      - source_labels: [__meta_kubernetes_pod_annotation_prometheus_io_path]
        action: replace
        target_label: __metrics_path__
        regex: (.+)
      - source_labels: [__address__, __meta_kubernetes_pod_annotation_prometheus_io_port]
        action: replace
        regex: ([^:]+)(?::\d+)?;(\d+)
        replacement: $1:$2
        target_label: __address__
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: prometheus-robot
spec:
  replicas: 1
  selector:
    matchLabels:
      app: prometheus-robot
  template:
    metadata:
      labels:
        app: prometheus-robot
    spec:
      containers:
      - name: prometheus
        image: prom/prometheus:latest
        args:
          - '--config.file=/etc/prometheus/prometheus.yml'
          - '--storage.tsdb.path=/prometheus/'
          - '--web.console.libraries=/etc/prometheus/console_libraries'
          - '--web.console.templates=/etc/prometheus/consoles'
        ports:
        - containerPort: 9090
        volumeMounts:
        - name: prometheus-config
          mountPath: /etc/prometheus/
        - name: prometheus-storage
          mountPath: /prometheus/
      volumes:
      - name: prometheus-config
        configMap:
          name: prometheus-robot-config
      - name: prometheus-storage
        emptyDir: {}
```

## Performance Optimization

### Resource Optimization Strategies

#### GPU Resource Management
```yaml
# gpu-robot-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: gpu-robot
spec:
  replicas: 1
  selector:
    matchLabels:
      app: gpu-robot
  template:
    metadata:
      labels:
        app: gpu-robot
    spec:
      containers:
      - name: perception
        image: robot-perception:latest
        resources:
          requests:
            nvidia.com/gpu: 1
            memory: "4Gi"
            cpu: "2"
          limits:
            nvidia.com/gpu: 1
            memory: "8Gi"
            cpu: "4"
        env:
        - name: NVIDIA_VISIBLE_DEVICES
          value: "0"
        - name: CUDA_DEVICE_ORDER
          value: "PCI_BUS_ID"
        - name: GPU_MEM_FRACTION
          value: "0.8"
        volumeMounts:
        - name: gpu-mem
          mountPath: /dev/nvidia-uvm
        - name: gpu-devices
          mountPath: /dev/nvidia0
      - name: control
        image: robot-control:latest
        resources:
          requests:
            memory: "2Gi"
            cpu: "1"
          limits:
            memory: "4Gi"
            cpu: "2"
      volumes:
      - name: gpu-mem
        hostPath:
          path: /dev/nvidia-uvm
      - name: gpu-devices
        hostPath:
          path: /dev/nvidia0
---
apiVersion: v1
kind: LimitRange
metadata:
  name: robot-limit-range
spec:
  limits:
  - type: Container
    default:
      cpu: "500m"
      memory: "1Gi"
    defaultRequest:
      cpu: "100m"
      memory: "128Mi"
  - type: Pod
    max:
      cpu: "8"
      memory: "16Gi"
    min:
      cpu: "100m"
      memory: "128Mi"
```

## Best Practices

### Container Image Optimization

#### Multi-architecture Support
```dockerfile
# Dockerfile for multi-arch support
FROM --platform=$TARGETPLATFORM ros:humble-robot

# Use build arguments for optimization
ARG BUILD_TYPE=Release
ARG ENABLE_CUDA=ON

# Install architecture-specific dependencies
RUN case $(uname -m) in \
    aarch64) echo "Installing ARM64 dependencies" ;; \
    x86_64) echo "Installing AMD64 dependencies" ;; \
    *) echo "Unsupported architecture" && exit 1 ;; \
    esac

# Multi-stage build for size optimization
FROM ros:humble-robot as runtime
COPY --from=builder /app/build /app/build
CMD ["ros2", "launch", "my_robot", "main.launch.py"]
```

#### Build and Deployment Pipeline
```yaml
# .github/workflows/robot-deployment.yaml
name: Robot Deployment
on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3

    - name: Set up Docker Buildx
      uses: docker/setup-buildx-action@v2

    - name: Login to DockerHub
      uses: docker/login-action@v2
      with:
        username: ${{ secrets.DOCKERHUB_USERNAME }}
        password: ${{ secrets.DOCKERHUB_TOKEN }}

    - name: Build and push
      uses: docker/build-push-action@v4
      with:
        context: .
        platforms: linux/amd64,linux/arm64
        push: true
        tags: |
          myorg/robot-app:latest
          myorg/robot-app:${{ github.sha }}

    - name: Run tests in container
      run: |
        docker run --rm myorg/robot-app:${{ github.sha }} \
          bash -c "source /opt/ros/humble/setup.bash && colcon test && colcon test-result --all"
```