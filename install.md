# Installation Guide for ROS2_Example_pkg

## Prerequisites
Before installing and using the `ROS2_Example_pkg`, ensure you have the following:
- **Operating System**: Linux (Ubuntu 22.04 recommended)
- **ROS2 Distribution**: Humble, Iron, or Rolling
- **Colcon Build Tool**: Installed for building ROS2 workspaces

### Installing ROS2
1. Follow the official ROS2 installation guide for your distribution:
   - [ROS2 Installation Guide](https://docs.ros.org/en/)
2. Ensure ROS2 is properly sourced:
   ```bash
   source /opt/ros/<your-ros2-distro>/setup.bash
   ```

### Installing Colcon
1. Install the `colcon` build tool:
   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions
   ```

## Cloning the Repository
1. Clone the `ROS2_Example_pkg` repository:
   ```bash
   git clone <repository-url>
   ```
2. Navigate to the workspace directory:
   ```bash
   cd ROS2_Example_pkg
   ```

## Building the Workspace
1. Build the workspace using `colcon`:
   ```bash
   colcon build
   ```
2. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Installing Dependencies
1. Use `rosdep` to install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Running the Packages
1. Run any package or node using `ros2 run` or `ros2 launch`.
   - Example:
     ```bash
     ros2 run addthreenum_pkg add_three_ints_server
     ```

## Troubleshooting
- If you encounter build errors, ensure all dependencies are installed and sourced properly.
- Rebuild the workspace if necessary:
  ```bash
  colcon build --packages-select <package-name>
  ```