# Usage Guide for ROS2_Example_pkg

## Learning ROS2 with This Repository
This repository is structured to help you learn and practice ROS2 concepts. Each package demonstrates specific features of ROS2, such as services, messages, parameters, and the `turtlesim` simulator.

### Step 1: Setting Up Your Environment
1. Install ROS2 (e.g., Humble, Iron, or Rolling).
2. Install the `colcon` build tool:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```
3. Clone this repository:
   ```bash
   git clone <repository-url>
   ```
4. Navigate to the workspace directory and build the packages:
   ```bash
   colcon build
   ```
5. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Step 2: Running Examples
- **Services and Clients**:
  - Run the `add_three_ints_server`:
    ```bash
    ros2 run addthreenum_pkg add_three_ints_server
    ```
  - Run the `add_three_ints_client`:
    ```bash
    ros2 run addthreenum_pkg add_three_ints_client
    ```

- **Custom Messages and Services**:
  - Use the `custom_lecture_pkg` to test custom messages and services.

- **Turtlesim Examples**:
  - Draw a square:
    ```bash
    ros2 run draw_shape_with_turtlesim_pkg draw_square
    ```
  - Draw a heart:
    ```bash
    ros2 run draw_shape_with_turtlesim_pkg draw_heart
    ```

### Step 3: Exploring Code
- Open the source files in the `src/` directories to understand how nodes are implemented.
- Check the `CMakeLists.txt` and `package.xml` files to learn about package dependencies.

### Step 4: Modifying and Extending
- Create your own nodes by following the structure of existing packages.
- Add new messages or services in the `msg/` or `srv/` directories.
- Update the `CMakeLists.txt` and `package.xml` files to include your changes.

## Additional Resources
- [ROS2 Documentation](https://docs.ros.org/en/)
- [Turtlesim Tutorials](https://docs.ros.org/en/rolling/Tutorials/Turtlesim.html)

## Troubleshooting
- If you encounter build errors, ensure all dependencies are installed:
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Source the setup file after every build:
  ```bash
  source install/setup.bash
  ```