# ROS2_Example_pkg

## Overview
This repository contains multiple ROS2 packages designed to demonstrate various functionalities and concepts in ROS2. Each package serves as an example or a learning resource for understanding ROS2 nodes, services, messages, and more.

## Packages

### 1. `addthreenum_pkg`
This package demonstrates a simple ROS2 service and client setup to add three integers.
- **Service Node**: `add_three_ints_server.cpp`
- **Client Node**: `add_three_ints_client.cpp`

### 2. `custom_lecture_pkg`
This package showcases custom message and service definitions.
- **Custom Message**: `Hss.msg`
- **Custom Service**: `Hss.srv`

### 3. `draw_shape_with_turtlesim_pkg`
This package uses the `turtlesim` simulator to draw various shapes such as cycles, hearts, and squares.
- **Examples**:
  - `draw_cycle.cpp`
  - `draw_heart.cpp`
  - `draw_square.cpp`

### 4. `more_interfaces`
This package demonstrates the use of custom messages for publishing and subscribing.
- **Custom Message**: `AddressBook.msg`
- **Publisher Node**: `publish_address_book.cpp`

### 5. `param_pub_sub_pkg`
This package demonstrates parameterized publishing and subscribing in ROS2.
- **Launch File**: `pubsub_param.launch.py`
- **Nodes**:
  - `param_publisher.cpp`
  - `param_subscriber.cpp`

### 6. `pubsub_pkg`
This package demonstrates basic publishing and subscribing in ROS2.
- **Launch Files**:
  - `pubsub.launch.py`
  - `square_turtle.launch.py`
  - `turtlesim_mimic_launch.py`
- **Nodes**:
  - `cpp_parameters_node.cpp`
  - `publisher_member_function.cpp`
  - `subscriber_member_function.cpp`

### 7. `turtle_sim_lecture_pkg`
This package demonstrates the use of the `turtlesim` simulator for various tasks.
- **Launch Files**:
  - `launch_turtlesim.launch.py`
  - `mimic.launch.py`

## How to Use
1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```
4. Run the desired package or node using `ros2 run` or `ros2 launch`.

## Additional Documentation
- [Installation Guide](install.md)
- [Usage Guide](usage.md)

## Prerequisites
- ROS2 (e.g., Humble, Iron, or Rolling)
- Colcon build tool

## License
This project is licensed under the MIT License.