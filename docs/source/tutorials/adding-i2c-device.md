# Tutorial: Using a Raspberry Pi with Ubuntu 24.04 to Receive I2C Sensor Data and Publish as a ROS2 (Jazzy) Topic

This tutorial provides a step-by-step guide to configure a Raspberry Pi running Ubuntu 24.04 LTS to interface with an I2C sensor (MPU6050, a 6-axis IMU), read its data, and publish it as a ROS2 (Jazzy) topic. The process covers hardware setup, enabling I2C communication, installing ROS2 Jazzy, and developing both Python and C++ nodes to handle sensor data.

## Prerequisites

### Hardware

- Raspberry Pi (e.g., Raspberry Pi 4 or 5 with 8GB RAM recommended for ROS2)
- MPU6050 I2C sensor module (or another I2C sensor with known address and register map)
- Jumper wires and a breadboard for connections
- Power supply and microSD card (16GB or larger)
- Optional: Active cooler and NVMe SSD for improved performance on Raspberry Pi 5

### Software

- Ubuntu 24.04 LTS installed on the Raspberry Pi (required for ROS2 Jazzy Tier 1 support)
- Basic familiarity with Linux terminal commands, Python and/or C++ programming

### Tools

- Internet connection for downloading packages
- A computer for SSH access (optional, for headless setup)

## Step 0: Set up your PC according to the Getting Started

Follow the first steps of set up for Perseus development here: [Getting Started](https://roar-qutrc.github.io/home/getting-started.html)

## Step 1: Set Up the Raspberry Pi with Ubuntu 24.04

### Install Ubuntu 24.04

1. Download the Ubuntu 24.04 LTS image for Raspberry Pi from the [official Ubuntu website](https://ubuntu.com/download/raspberry-pi)
2. Use Raspberry Pi Imager (or a similar tool) to flash the image onto a microSD card
3. Insert the microSD card into the Raspberry Pi, connect peripherals (monitor, keyboard, or use SSH for headless setup), and power on the device
4. Follow the on-screen prompts to configure Ubuntu, including setting a username, password, and Wi-Fi credentials if applicable

### Update the system packages

```bash
sudo apt update && sudo apt upgrade -y
```

### Verify Ubuntu Version

Confirm the system is running Ubuntu 24.04:

```bash
lsb_release -a
```

The output should indicate Ubuntu 24.04 LTS.

## Step 2: Install Perseus Software Environment

Now log in to (via ssh or locally) the Pi and undertake the Getting Started steps on the Pi.
https://roar-qutrc.github.io/home/getting-started.html

## Step 3: Configure I2C on the Raspberry Pi

### Install I2C Tools

Install tools for interacting with I2C devices:

```bash
sudo apt install -y i2c-tools python3-smbus libi2c-dev
```

Note: `libi2c-dev` is required for C++ I2C development.

### Enable I2C Kernel Module

Load the I2C kernel module for the Raspberry Pi:

```bash
sudo modprobe i2c-bcm2835
echo "i2c-bcm2835" | sudo tee -a /etc/modules
echo "i2c-dev" | sudo tee -a /etc/modules
```

### Modify Device Tree Configuration

Enable the I2C interface by editing the bootloader configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Add or uncomment the following lines:

```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=100000
```

Save the file (Ctrl+O, Enter, Ctrl+X).

### Reboot the Raspberry Pi

Apply the changes:

```bash
sudo reboot
```

### Connect the MPU6050 Sensor

Wire the MPU6050 to the Raspberry Pi using the I2C pins:

- VCC to 3.3V (Pin 1)
- GND to Ground (Pin 9)
- SDA to SDA (Pin 3, GPIO 2)
- SCL to SCL (Pin 5, GPIO 3)

Use a breadboard or soldered connections to ensure reliability.

### Verify I2C Device Detection

Check if the MPU6050 is detected on I2C bus 1:

```bash
sudo i2cdetect -y 1
```

The MPU6050 should appear at address 0x68. If not detected, verify wiring, power, and connections.

### Fix Permissions (Recommended)

To access I2C devices without sudo, add your user to the i2c group:

```bash
sudo usermod -aG i2c $USER
```

Log out and back in for the changes to take effect.

## Step 4: Create a ROS2 Package

### Set Up a ROS2 Workspace

Create a workspace directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## Step 5: Choose Your Development Language

You can implement the I2C sensor interface in either Python or C++. Follow the steps for your preferred language.

## Option A: C++ Implementation

### Create a C++ Package

Generate a new ROS2 package for the I2C sensor using C++:

```bash
ros2 pkg create --build-type ament_cmake i2c_sensor_cpp --dependencies rclcpp sensor_msgs
cd i2c_sensor_cpp
```

### Create Header File

Create a header file for the MPU6050 publisher:

```bash
mkdir -p include/i2c_sensor_cpp
touch include/i2c_sensor_cpp/mpu6050_publisher.hpp
```

Open the header file in a text editor and add the following code:

```bash
nano include/i2c_sensor_cpp/mpu6050_publisher.hpp
```

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace i2c_sensor
{

class MPU6050Publisher : public rclcpp::Node
{
public:
    MPU6050Publisher();
    virtual ~MPU6050Publisher();

private:
    // ROS2 members
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    // I2C members
    int _i2c_fd;                    // File descriptor for I2C device
    const uint8_t _device_address;  // MPU6050 I2C address
    const std::string _i2c_device;  // I2C bus device path

    // Methods
    void _timer_callback();
    bool _initialize_mpu6050();
    int16_t _read_word_2c(uint8_t reg);
    void _write_register(uint8_t reg, uint8_t value);
    uint8_t _read_register(uint8_t reg);
};

} // namespace i2c_sensor
```

### Create Source File

Create a source file for the implementation:

```bash
mkdir -p src
touch src/mpu6050_publisher.cpp
```

Open the source file in a text editor and add the following code:

```bash
nano src/mpu6050_publisher.cpp
```

```cpp
#include "i2c_sensor_cpp/mpu6050_publisher.hpp"

namespace i2c_sensor
{

MPU6050Publisher::MPU6050Publisher()
: Node("mpu6050_publisher"),
  _i2c_fd(-1),
  _device_address(0x68),  // Default MPU6050 address
  _i2c_device("/dev/i2c-1")  // Raspberry Pi I2C bus 1
{
    // Create publisher
    _publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Initialize I2C communication with MPU6050
    if (_initialize_mpu6050()) {
        RCLCPP_INFO(this->get_logger(), "MPU6050 initialized successfully");

        // Create timer for periodic sensor reading (10Hz)
        _timer = this->create_wall_timer(
            100ms, std::bind(&MPU6050Publisher::_timer_callback, this));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6050");
    }
}

MPU6050Publisher::~MPU6050Publisher()
{
    // Close I2C device if open
    if (_i2c_fd >= 0) {
        close(_i2c_fd);
        RCLCPP_INFO(this->get_logger(), "I2C device closed");
    }
}

bool MPU6050Publisher::_initialize_mpu6050()
{
    // Open I2C device
    _i2c_fd = open(_i2c_device.c_str(), O_RDWR);
    if (_i2c_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device %s", _i2c_device.c_str());
        return false;
    }

    // Set I2C slave address
    if (ioctl(_i2c_fd, I2C_SLAVE, _device_address) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address");
        close(_i2c_fd);
        _i2c_fd = -1;
        return false;
    }

    // Wake up MPU6050 (exit sleep mode)
    _write_register(0x6B, 0x00);

    // Configure accelerometer range to ±2g (Register 0x1C)
    _write_register(0x1C, 0x00);

    // Configure gyroscope range to ±250°/s (Register 0x1B)
    _write_register(0x1B, 0x00);

    return true;
}

void MPU6050Publisher::_timer_callback()
{
    // Check if I2C connection is active
    if (_i2c_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "I2C device not available");
        return;
    }

    // Read accelerometer and gyroscope data
    float accel_x = static_cast<float>(_read_word_2c(0x3B)) / 16384.0f;  // ±2g range
    float accel_y = static_cast<float>(_read_word_2c(0x3D)) / 16384.0f;
    float accel_z = static_cast<float>(_read_word_2c(0x3F)) / 16384.0f;

    float gyro_x = static_cast<float>(_read_word_2c(0x43)) / 131.0f;  // ±250 deg/s range
    float gyro_y = static_cast<float>(_read_word_2c(0x45)) / 131.0f;
    float gyro_z = static_cast<float>(_read_word_2c(0x47)) / 131.0f;

    // Create IMU message
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = this->now();
    imu_msg->header.frame_id = "imu_link";

    // Convert acceleration from g to m/s²
    imu_msg->linear_acceleration.x = accel_x * 9.81;
    imu_msg->linear_acceleration.y = accel_y * 9.81;
    imu_msg->linear_acceleration.z = accel_z * 9.81;

    // Convert angular velocity from deg/s to rad/s
    imu_msg->angular_velocity.x = gyro_x * M_PI / 180.0;
    imu_msg->angular_velocity.y = gyro_y * M_PI / 180.0;
    imu_msg->angular_velocity.z = gyro_z * M_PI / 180.0;

    // Set covariance to unknown
    for (int i = 0; i < 9; i++) {
        imu_msg->linear_acceleration_covariance[i] = 0.0;
        imu_msg->angular_velocity_covariance[i] = 0.0;
        imu_msg->orientation_covariance[i] = 0.0;
    }
    imu_msg->linear_acceleration_covariance[0] = -1.0;
    imu_msg->angular_velocity_covariance[0] = -1.0;
    imu_msg->orientation_covariance[0] = -1.0;

    // Publish the message
    _publisher->publish(std::move(imu_msg));
    RCLCPP_INFO(this->get_logger(), "Published IMU data");
}

int16_t MPU6050Publisher::_read_word_2c(uint8_t reg)
{
    // Read two bytes (high and low) and convert to signed 16-bit integer
    uint8_t high = _read_register(reg);
    uint8_t low = _read_register(reg + 1);

    int16_t value = (high << 8) | low;

    // Convert to signed
    if (value >= 0x8000) {
        return -((65535 - value) + 1);
    }

    return value;
}

void MPU6050Publisher::_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(_i2c_fd, buffer, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to register 0x%02X", reg);
    }
}

uint8_t MPU6050Publisher::_read_register(uint8_t reg)
{
    // Write the register address
    if (write(_i2c_fd, &reg, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write register address 0x%02X", reg);
        return 0;
    }

    // Read the register value
    uint8_t value;
    if (read(_i2c_fd, &value, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read from register 0x%02X", reg);
        return 0;
    }

    return value;
}

} // namespace i2c_sensor

// Entry point
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<i2c_sensor::MPU6050Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Create a Main Source File

Create a main file to run the node:

```bash
touch src/mpu6050_node.cpp
```

Add the following code to the main file:

```bash
nano src/mpu6050_node.cpp
```

```cpp
#include "i2c_sensor_cpp/mpu6050_publisher.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<i2c_sensor::MPU6050Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Update CMakeLists.txt

!Need to align with template snippets!

Edit the CMakeLists.txt file to include the necessary build configuration:

```bash
nano CMakeLists.txt
```

Replace the content with the following:

```cmake
cmake_minimum_required(VERSION 3.8)
project(i2c_sensor_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add library for reusable code
add_library(${PROJECT_NAME} src/mpu6050_publisher.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs)

# Add executable
add_executable(mpu6050_node src/mpu6050_node.cpp)
target_link_libraries(mpu6050_node ${PROJECT_NAME})
ament_target_dependencies(mpu6050_node
  rclcpp
  sensor_msgs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  mpu6050_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
```

### Update package.xml

Edit package.xml to include necessary dependencies:

```bash
nano package.xml
```

Update the file to include:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>i2c_sensor_cpp</name>
  <version>0.0.0</version>
  <description>C++ ROS2 node for reading MPU6050 I2C sensor data</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Option B: Python Implementation

### Create a Python Package

Generate a new ROS2 package for the I2C sensor:

```bash
ros2 pkg create --build-type ament_python i2c_sensor_pkg --dependencies rclpy sensor_msgs
```

This creates a package named `i2c_sensor_pkg` with dependencies for Python ROS2 (`rclpy`) and sensor messages (`sensor_msgs`).

### Navigate to the Package

Move to the package's Python script directory:

```bash
cd ~/ros2_ws/src/i2c_sensor_pkg/i2c_sensor_pkg
```

### Create a Python Script

Create a file named `mpu6050_publisher.py`:

```bash
touch mpu6050_publisher.py
```

### Write the Node Code

Open `mpu6050_publisher.py` in a text editor (e.g., nano) and add the following code:

```bash
nano mpu6050_publisher.py
```

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.bus = smbus.SMBus(1)  # I2C bus 1
        self.address = 0x68  # MPU6050 default address
        self._initialize_mpu6050()

    def _initialize_mpu6050(self):
        # Wake up the MPU6050 (exit sleep mode)
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        # Configure accelerometer for ±2g range
        self.bus.write_byte_data(self.address, 0x1C, 0x00)
        # Configure gyroscope for ±250°/s range
        self.bus.write_byte_data(self.address, 0x1B, 0x00)
        self.get_logger().info('MPU6050 initialized')

    def timer_callback(self):
        # Read raw accelerometer and gyroscope data
        accel_x = self._read_word_2c(0x3B) / 16384.0  # ±2g range
        accel_y = self._read_word_2c(0x3D) / 16384.0
        accel_z = self._read_word_2c(0x3F) / 16384.0

        gyro_x = self._read_word_2c(0x43) / 131.0  # ±250 deg/s range
        gyro_y = self._read_word_2c(0x45) / 131.0
        gyro_z = self._read_word_2c(0x47) / 131.0

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = accel_x * 9.81  # Convert to m/s^2
        imu_msg.linear_acceleration.y = accel_y * 9.81
        imu_msg.linear_acceleration.z = accel_z * 9.81

        imu_msg.angular_velocity.x = math.radians(gyro_x)  # Convert to rad/s
        imu_msg.angular_velocity.y = math.radians(gyro_y)
        imu_msg.angular_velocity.z = math.radians(gyro_z)

        # Set covariance to unknown
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.linear_acceleration_covariance[0] = -1

        # Publish the message
        self.publisher_.publish(imu_msg)
        self.get_logger().info('Published IMU data')

    def _read_word_2c(self, reg):
        # Read two bytes and convert to signed integer
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low

        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

def main():
    rclpy.init()
    node = MPU6050Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Make the Script Executable

Set execution permissions:

```bash
chmod +x mpu6050_publisher.py
```

### Update Package Configuration

Edit `setup.py` in `~/ros2_ws/src/i2c_sensor_pkg` to include the script as an executable:

```bash
cd ..
nano setup.py
```

```python
from setuptools import setup

package_name = 'i2c_sensor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Publishes I2C sensor data to ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050_publisher = i2c_sensor_pkg.mpu6050_publisher:main',
        ],
    },
)
```

Update `package.xml` in the same directory to include dependencies:

```bash
nano package.xml
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>i2c_sensor_pkg</name>
  <version>0.0.0</version>
  <description>Publishes I2C sensor data to ROS2</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Install Python Dependencies

Ensure the smbus library is installed:

```bash
sudo apt install -y python3-smbus
```

## Step 6: Build and Test the ROS2 Package

### Build the Workspace

Navigate to the workspace root and build:

```bash
cd ~/ros2_ws
colcon build
```

### Source the Workspace

Source the local workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

### Run the Node

Launch the MPU6050 publisher node (depending on which implementation you choose):

For C++:

```bash
ros2 run i2c_sensor_cpp mpu6050_node
```

For Python:

```bash
ros2 run i2c_sensor_pkg mpu6050_publisher
```

Log messages should indicate that IMU data is being published.

### Verify the Topic

In a new terminal, list available topics:

```bash
ros2 topic list
```

Confirm that `/imu/data` is present. View the published data:

```bash
ros2 topic echo /imu/data
```

This displays IMU messages with acceleration and angular velocity values.

### Visualize in RViz2 (Optional)

Launch RViz2 to visualize the IMU data:

```bash
ros2 run rviz2 rviz2
```

Add an Imu display, set the topic to `/imu/data`, and configure the frame to `imu_link` to visualize orientation and motion.

## Step 7: Creating a Launch File

### For C++

Create a launch directory for the package:

```bash
cd ~/ros2_ws/src/i2c_sensor_cpp
mkdir -p launch
```

Create a launch file:

```bash
nano launch/mpu6050.launch.py
```

Add the following content:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_sensor_cpp',
            executable='mpu6050_node',
            name='mpu6050_publisher',
            output='screen',
            emulate_tty=True,
        )
    ])
```

Update CMakeLists.txt to install the launch files:

```bash
nano CMakeLists.txt
```

Add before the `ament_package()` line:

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

### For Python

Create a launch directory for the package:

```bash
cd ~/ros2_ws/src/i2c_sensor_pkg
mkdir -p launch
```

Create a launch file:

```bash
nano launch/mpu6050.launch.py
```

Add the following content:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='i2c_sensor_pkg',
            executable='mpu6050_publisher',
            name='mpu6050_publisher',
            output='screen',
            emulate_tty=True,
        )
    ])
```

Update setup.py to install the launch files:

```bash
nano setup.py
```

Update the `data_files` section:

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/mpu6050.launch.py']),
],
```

### Rebuild and Run with Launch File

Rebuild the workspace:

```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

Run using the launch file:

For C++:

```bash
ros2 launch i2c_sensor_cpp mpu6050.launch.py
```

For Python:

```bash
ros2 launch i2c_sensor_pkg mpu6050.launch.py
```

## Step 8: Troubleshooting

### I2C Device Not Detected

- Verify wiring and power supply to the MPU6050
- Run `sudo i2cdetect -y 1` to confirm the device appears at address 0x68
- Check `/boot/firmware/config.txt` for correct I2C settings

### ROS2 Node Fails to Publish

- Ensure the workspace is sourced (`source ~/ros2_ws/install/setup.bash`)
- Check terminal output for Python/C++ errors
- For Python: Confirm smbus is installed
- For C++: Check that libi2c-dev is installed

### Compilation Issues for C++

- Check for missing dependencies with `sudo apt install build-essential`
- Ensure proper CMakeLists.txt configuration
- Look for specific error messages in the build output

### Inaccurate Sensor Data

- Calibrate the MPU6050 by adjusting offsets or implementing a sensor fusion algorithm (e.g., Madgwick filter)
- Minimize vibrations or electromagnetic interference affecting the sensor

### Permission Issues

- If i2cdetect requires root, add your user to the i2c group:
  ```bash
  sudo usermod -aG i2c $USER
  ```
- Log out and back in to apply

## Step 9: Advanced Features

### Adding Calibration to the MPU6050

Sensor calibration can significantly improve measurement accuracy. Here's how to implement basic calibration:

#### For C++

Create a calibration method in the MPU6050Publisher class:

```cpp
// Add to the header file (mpu6050_publisher.hpp) in the private section:
bool _calibrate_sensors();
float _gyro_offset[3] = {0.0f, 0.0f, 0.0f};
float _accel_offset[3] = {0.0f, 0.0f, 0.0f};
```

Add the implementation in the source file:

```cpp
bool MPU6050Publisher::_calibrate_sensors()
{
    const int num_samples = 100;
    float gyro_temp[3] = {0.0f, 0.0f, 0.0f};
    float accel_temp[3] = {0.0f, 0.0f, 0.0f};

    RCLCPP_INFO(this->get_logger(), "Calibrating MPU6050 - keep the sensor still...");

    // Collect multiple samples
    for (int i = 0; i < num_samples; i++) {
        gyro_temp[0] += static_cast<float>(_read_word_2c(0x43)) / 131.0f;
        gyro_temp[1] += static_cast<float>(_read_word_2c(0x45)) / 131.0f;
        gyro_temp[2] += static_cast<float>(_read_word_2c(0x47)) / 131.0f;

        accel_temp[0] += static_cast<float>(_read_word_2c(0x3B)) / 16384.0f;
        accel_temp[1] += static_cast<float>(_read_word_2c(0x3D)) / 16384.0f;
        accel_temp[2] += static_cast<float>(_read_word_2c(0x3F)) / 16384.0f;

        // Short delay between readings
        std::this_thread::sleep_for(10ms);
    }

    // Calculate average offsets
    _gyro_offset[0] = gyro_temp[0] / num_samples;
    _gyro_offset[1] = gyro_temp[1] / num_samples;
    _gyro_offset[2] = gyro_temp[2] / num_samples;

    // For accelerometer, we only want to zero X and Y, but keep Z at ~1g (9.81 m/s²)
    _accel_offset[0] = accel_temp[0] / num_samples;
    _accel_offset[1] = accel_temp[1] / num_samples;
    _accel_offset[2] = (accel_temp[2] / num_samples) - 1.0f; // Subtract 1g

    RCLCPP_INFO(this->get_logger(), "Calibration complete");
    RCLCPP_INFO(this->get_logger(), "Gyroscope offsets: X: %.2f, Y: %.2f, Z: %.2f",
               _gyro_offset[0], _gyro_offset[1], _gyro_offset[2]);
    RCLCPP_INFO(this->get_logger(), "Accelerometer offsets: X: %.2f, Y: %.2f, Z: %.2f",
               _accel_offset[0], _accel_offset[1], _accel_offset[2]);

    return true;
}
```

Call this method in the constructor after initialising the sensor:

```cpp
if (_initialize_mpu6050()) {
    RCLCPP_INFO(this->get_logger(), "MPU6050 initialised successfully");

    // Add calibration
    _calibrate_sensors();

    // Create timer
    // ...
}
```

Modify the \_timer_callback method to apply offsets:

```cpp
// Read raw data
float accel_x = static_cast<float>(_read_word_2c(0x3B)) / 16384.0f - _accel_offset[0];
float accel_y = static_cast<float>(_read_word_2c(0x3D)) / 16384.0f - _accel_offset[1];
float accel_z = static_cast<float>(_read_word_2c(0x3F)) / 16384.0f - _accel_offset[2];

float gyro_x = static_cast<float>(_read_word_2c(0x43)) / 131.0f - _gyro_offset[0];
float gyro_y = static_cast<float>(_read_word_2c(0x45)) / 131.0f - _gyro_offset[1];
float gyro_z = static_cast<float>(_read_word_2c(0x47)) / 131.0f - _gyro_offset[2];
```

#### For Python

Add calibration to the Python implementation:

```python
def _calibrate_sensors(self):
    self.get_logger().info('Calibrating MPU6050 - keep the sensor still...')

    # Initialize variables
    num_samples = 100
    gyro_sum = [0, 0, 0]
    accel_sum = [0, 0, 0]

    # Collect samples
    for _ in range(num_samples):
        gyro_sum[0] += self._read_word_2c(0x43) / 131.0
        gyro_sum[1] += self._read_word_2c(0x45) / 131.0
        gyro_sum[2] += self._read_word_2c(0x47) / 131.0

        accel_sum[0] += self._read_word_2c(0x3B) / 16384.0
        accel_sum[1] += self._read_word_2c(0x3D) / 16384.0
        accel_sum[2] += self._read_word_2c(0x3F) / 16384.0

        time.sleep(0.01)

    # Calculate offsets
    self.gyro_offset = [sum_val / num_samples for sum_val in gyro_sum]

    # For accelerometer, zero X and Y, but keep Z at ~1g
    self.accel_offset = [
        accel_sum[0] / num_samples,
        accel_sum[1] / num_samples,
        (accel_sum[2] / num_samples) - 1.0  # Subtract 1g
    ]

    self.get_logger().info('Calibration complete')
    self.get_logger().info(f'Gyroscope offsets: X: {self.gyro_offset[0]:.2f}, '
                          f'Y: {self.gyro_offset[1]:.2f}, Z: {self.gyro_offset[2]:.2f}')
    self.get_logger().info(f'Accelerometer offsets: X: {self.accel_offset[0]:.2f}, '
                          f'Y: {self.accel_offset[1]:.2f}, Z: {self.accel_offset[2]:.2f}')
```

Add the method call in the `__init__` method:

```python
def __init__(self):
    super().__init__('mpu6050_publisher')
    self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
    self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
    self.bus = smbus.SMBus(1)  # I2C bus 1
    self.address = 0x68  # MPU6050 default address

    # Initialize sensor calibration variables
    self.gyro_offset = [0.0, 0.0, 0.0]
    self.accel_offset = [0.0, 0.0, 0.0]

    self._initialize_mpu6050()
    self._calibrate_sensors()  # Add calibration
```

Update the timer_callback method to use the offsets:

```python
def timer_callback(self):
    # Read raw accelerometer and gyroscope data with offsets applied
    accel_x = self._read_word_2c(0x3B) / 16384.0 - self.accel_offset[0]
    accel_y = self._read_word_2c(0x3D) / 16384.0 - self.accel_offset[1]
    accel_z = self._read_word_2c(0x3F) / 16384.0 - self.accel_offset[2]

    gyro_x = self._read_word_2c(0x43) / 131.0 - self.gyro_offset[0]
    gyro_y = self._read_word_2c(0x45) / 131.0 - self.gyro_offset[1]
    gyro_z = self._read_word_2c(0x47) / 131.0 - self.gyro_offset[2]

    # Rest of the method remains the same
    # ...
```

### Implementing Orientation Estimation

The MPU6050 doesn't directly provide orientation data, but we can calculate it using sensor fusion algorithms. Below is a simple implementation using a complementary filter:

#### For C++

Add the required variables to the header file:

```cpp
// Add to the private section of mpu6050_publisher.hpp
float _pitch = 0.0f;
float _roll = 0.0f;
float _yaw = 0.0f;
float _last_time = 0.0f;
```

Add the orientation calculation to the \_timer_callback method:

```cpp
void MPU6050Publisher::_timer_callback()
{
    // ... existing code to read accelerometer and gyroscope data ...

    // Calculate orientation using accelerometer data (simple approach)
    float accel_roll = atan2(accel_y, accel_z) * 180.0f / M_PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // Get current time for delta calculation
    float current_time = this->now().seconds();
    float dt = (current_time - _last_time > 0 && _last_time > 0) ? (current_time - _last_time) : 0.01f;
    _last_time = current_time;

    // Complementary filter - combine accelerometer and gyroscope data
    // 0.98 is the filter coefficient (adjust as needed)
    _roll = 0.98f * (_roll + gyro_x * dt) + 0.02f * accel_roll;
    _pitch = 0.98f * (_pitch + gyro_y * dt) + 0.02f * accel_pitch;
    _yaw += gyro_z * dt; // Simple integration for yaw

    // Convert to quaternion
    float roll_rad = _roll * M_PI / 180.0f;
    float pitch_rad = _pitch * M_PI / 180.0f;
    float yaw_rad = _yaw * M_PI / 180.0f;

    // Roll (x-axis rotation)
    float cy = cos(yaw_rad * 0.5f);
    float sy = sin(yaw_rad * 0.5f);
    float cp = cos(pitch_rad * 0.5f);
    float sp = sin(pitch_rad * 0.5f);
    float cr = cos(roll_rad * 0.5f);
    float sr = sin(roll_rad * 0.5f);

    float qw = cy * cp * cr + sy * sp * sr;
    float qx = cy * cp * sr - sy * sp * cr;
    float qy = cy * sp * cr + sy * cp * sr;
    float qz = sy * cp * cr - cy * sp * sr;

    // Update the IMU message with orientation data
    imu_msg->orientation.w = qw;
    imu_msg->orientation.x = qx;
    imu_msg->orientation.y = qy;
    imu_msg->orientation.z = qz;

    // ... rest of the existing code ...
}
```

#### For Python

Add orientation estimation to the Python implementation:

```python
def __init__(self):
    # ... existing initialization code ...

    # Add orientation variables
    self.pitch = 0.0
    self.roll = 0.0
    self.yaw = 0.0
    self.last_time = None

def timer_callback(self):
    # ... existing code to read accelerometer and gyroscope data ...

    # Calculate orientation using accelerometer data
    accel_roll = math.atan2(accel_y, accel_z) * 180.0 / math.pi
    accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi

    # Calculate time delta
    current_time = time.time()
    if self.last_time is None:
        self.last_time = current_time
    dt = current_time - self.last_time
    self.last_time = current_time

    # Complementary filter
    self.roll = 0.98 * (self.roll + gyro_x * dt) + 0.02 * accel_roll
    self.pitch = 0.98 * (self.pitch + gyro_y * dt) + 0.02 * accel_pitch
    self.yaw += gyro_z * dt  # Simple integration for yaw

    # Convert to quaternion
    roll_rad = math.radians(self.roll)
    pitch_rad = math.radians(self.pitch)
    yaw_rad = math.radians(self.yaw)

    # Convert Euler angles to quaternion
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = cy * sp * cr + sy * cp * sr
    qz = sy * cp * cr - cy * sp * sr

    # Update message with orientation
    imu_msg.orientation.w = qw
    imu_msg.orientation.x = qx
    imu_msg.orientation.y = qy
    imu_msg.orientation.z = qz

    # ... rest of the existing code ...
```

## Step 10: Integration with Perseus Robot

To integrate the IMU sensor with the Perseus robot system:

### 1. Create a Launch File for Integration

Create a launch file that includes both the IMU node and the necessary Perseus components:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch the IMU node
        Node(
            package='i2c_sensor_cpp',  # or 'i2c_sensor_pkg' for Python
            executable='mpu6050_node',  # or 'mpu6050_publisher' for Python
            name='mpu6050_publisher',
            output='screen',
            parameters=[
                {'frame_id': 'imu_link'}
            ]
        ),

        # Include Perseus base launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('perseus_bringup'),
                    'launch',
                    'perseus_base.launch.py'
                ])
            ])
        ),

        # TF2 static transform publisher for IMU position on the robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])
```

### 2. Add URDF Model for the IMU

Create a URDF description for the IMU and add it to the Perseus robot description:

```xml
<!-- Create a file named mpu6050.urdf.xacro in your package -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="mpu6050_imu" params="parent *origin">
    <link name="imu_link">
      <visual>
        <geometry>
          <box size="0.02 0.02 0.005"/>
        </geometry>
        <material name="green"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.02 0.02 0.005"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <joint name="${parent}_to_imu" type="fixed">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="imu_link"/>
    </joint>
  </xacro:macro>
</robot>
```

### 3. Configure Perseus to Use the IMU Data

For state estimation or navigation, you may need to configure the robot to use the IMU data:

```yaml
# Example configuration for ekf_node
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Add IMU to sensor fusion
    imu0: imu/data
    imu0_config:
      [
        false,
        false,
        false,
        true,
        true,
        true,
        false,
        false,
        false,
        true,
        true,
        true,
        true,
        true,
        true,
      ]
    imu0_differential: false
    imu0_relative: true
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true
```

## Additional Notes

### Adapting for Other I2C Sensors

- Modify the MPU6050-specific code (e.g., register addresses, scaling factors) based on your sensor's datasheet
- Use appropriate ROS2 message types (e.g., `sensor_msgs/Temperature` for a temperature sensor like LM75A)

### Performance Considerations

- For Raspberry Pi 5, an NVMe SSD can improve ROS2 performance due to faster read/write speeds
- Avoid high I2C clock speeds (>50 kHz) unless the sensor supports clock stretching, due to Raspberry Pi I2C driver limitations
- Enable real-time performance to improve timing accuracy:
  ```bash
  sudo apt install linux-lowlatency
  ```

### Integrating with Other Sensors

The Perseus robot may benefit from additional sensors:

1. GPS module for outdoor navigation
2. Distance/range sensors for obstacle detection
3. Cameras for computer vision tasks

You can integrate these using similar approaches, creating appropriate ROS2 nodes and publishing to standard message types.

### Advanced Topics

For more advanced implementations, consider exploring:

1. Madgwick or Mahony filters for better orientation estimation
2. Extended Kalman Filter (EKF) for sensor fusion
3. Using the built-in Digital Motion Processor (DMP) of the MPU6050
4. Multi-sensor fusion (IMU, encoders, GPS, etc.)

## Further Learning

- Explore ROS2 tutorials for advanced topics like sensor fusion or navigation: https://docs.ros.org/en/jazzy/Tutorials.html
- Engage with the ROS community on platforms like ROS Discourse or Reddit's r/ROS for sensor-specific guidance
- Learn more about Perseus robot architecture and development guidelines at the QUT Robotics ROAR team documentation

## References

- Ubuntu on Raspberry Pi I2C Configuration: Ubuntu Community Documentation
- ROS2 Jazzy Installation: https://docs.ros.org/en/jazzy/Installation.html
- I2C Tools and SMBus: Linux I2C Documentation
- ROS2 IMU Message Format: ROS2 sensor_msgs Documentation
- MPU6050 Datasheet: InvenSense MPU-6050 Six-Axis (Gyro + Accelerometer) MEMS MotionTracking™ Devices
- Perseus Robot Documentation: QUT Robotics ROAR team

---

This tutorial provides a comprehensive workflow for interfacing an I2C sensor with a Raspberry Pi running Ubuntu 24.04 and publishing data to a ROS2 Jazzy topic, with implementations in both Python and C++. For further assistance or adaptations for other sensors, consult the referenced resources or reach out to the QUT Robotics ROAR team.
