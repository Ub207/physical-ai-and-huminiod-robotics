---
sidebar_position: 4
---

# ROS 2 Packages

## Package Structure

A ROS 2 package follows a standardized structure that promotes consistency and reusability:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata
├── src/                    # Source code
│   ├── node1.cpp
│   └── node2.cpp
├── include/                # Header files
├── launch/                 # Launch files
├── config/                 # Configuration files
├── test/                   # Test files
└── scripts/                # Standalone scripts
```

## Package Manifest (package.xml)

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>My robot package description</description>
  <maintainer email="maintainer@todo.todo">maintainer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Creating Packages

### Using colcon

```bash
# Create a new package
ros2 pkg create --build-type ament_cmake my_robot_package

# Create a package with dependencies
ros2 pkg create --build-type ament_cmake my_robot_package --dependencies rclcpp std_msgs
```

## Build System (ament)

ROS 2 uses ament as its build system:

- **ament_cmake**: For C/C++ packages
- **ament_python**: For Python packages
- **ament_index**: Runtime package discovery

## Common Package Types

### Driver Packages
- Interface with hardware sensors and actuators
- Provide standardized ROS 2 interfaces
- Handle low-level communication protocols

### Message Packages
- Define custom message types
- Provide data structures for communication
- Enable type-safe communication

### Configuration Packages
- Store launch files and parameters
- Define system configurations
- Enable system deployment

## Best Practices

1. **Single Responsibility**: Each package should have a clear, focused purpose
2. **Modularity**: Design packages to be reusable across different projects
3. **Clear Dependencies**: Explicitly declare all dependencies in package.xml
4. **Proper Naming**: Use descriptive, consistent naming conventions
5. **Documentation**: Include comprehensive documentation in packages
6. **Testing**: Include unit tests and integration tests