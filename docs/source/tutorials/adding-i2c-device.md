# Tutorial: Using a Raspberry Pi with Ubuntu 24.04 to Receive I2C Sensor Data and Publish as a ROS2 (Jazzy) Topic

This tutorial provides a step-by-step guide to configure a Raspberry Pi running Ubuntu 24.04 LTS to interface with an I2C sensor (MPU6050, a 6-axis IMU), read its data, and publish it as a ROS2 (Jazzy) topic. The process covers hardware setup, enabling I2C communication, installing ROS2 Jazzy, and developing a Python node to handle sensor data.

## Prerequisites

### Hardware

- Raspberry Pi (e.g., Raspberry Pi 4 or 5 with 8GB RAM recommended for ROS2)
- MPU6050 I2C sensor module (or another I2C sensor with known address and register map)
- Jumper wires and a breadboard for connections
- Power supply and microSD card (16GB or larger)

### Software

- Ubuntu 24.04 LTS installed on the Raspberry Pi (required for ROS2 Jazzy Tier 1 support)
- Basic familiarity with Linux terminal commands and Python programming

### Tools

- Internet connection for downloading packages
- A computer for SSH access

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
sudo apt install -y i2c-tools python3-smbus
```

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

## Step 4: Create a ROS2 Package

<todo>

### Create a Python Package (option B)

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

## Step 5: Write a Python Node to Read and Publish I2C Data

### Create a Python Script

Create a file named `mpu6050_publisher.py`:

```bash
touch mpu6050_publisher.py
```

### Write the Node Code

Open `mpu6050_publisher.py` in a text editor (e.g., nano) and add the following code:

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

```xml
<depend>sensor_msgs</depend>
<depend>rclpy</depend>
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

Launch the MPU6050 publisher node:

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

## Step 7: Troubleshooting

### I2C Device Not Detected

- Verify wiring and power supply to the MPU6050
- Run `sudo i2cdetect -y 1` to confirm the device appears at address 0x68
- Check `/boot/firmware/config.txt` for correct I2C settings

### ROS2 Node Fails to Publish

- Ensure the workspace is sourced (`source ~/ros2_ws/install/setup.bash`)
- Check terminal output for Python errors and confirm smbus is installed

### Inaccurate Sensor Data

- Calibrate the MPU6050 by adjusting offsets or implementing a sensor fusion algorithm (e.g., Madgwick filter)
- Minimize vibrations or electromagnetic interference affecting the sensor

### Permission Issues

- If i2cdetect requires root, add your user to the i2c group:
  ```bash
  sudo usermod -aG i2c $USER
  ```
- Log out and back in to apply

## Additional Notes

### Adapting for Other I2C Sensors

- Modify the MPU6050-specific code (e.g., register addresses, scaling factors) based on your sensor's datasheet
- Use appropriate ROS2 message types (e.g., `sensor_msgs/Temperature` for a temperature sensor like LM75A)

### Performance Considerations

- For Raspberry Pi 5, an NVMe SSD can improve ROS2 performance due to faster read/write speeds
- Avoid high I2C clock speeds (>50 kHz) unless the sensor supports clock stretching, due to Raspberry Pi I2C driver limitations

### Further Learning

- Explore ROS2 tutorials for advanced topics like sensor fusion or navigation: https://docs.ros.org/en/jazzy/Tutorials.html
- Engage with the ROS community on platforms like ROS Discourse or Reddit's r/ROS for sensor-specific guidance

## References

- Ubuntu on Raspberry Pi I2C Configuration: Ubuntu Community Documentation
- ROS2 Jazzy Installation: https://docs.ros.org/en/jazzy/Installation.html
- I2C Tools and SMBus: Linux I2C Documentation
- ROS2 IMU Message Format: ROS2 sensor_msgs Documentation

---

This tutorial provides a comprehensive workflow for interfacing an I2C sensor with a Raspberry Pi running Ubuntu 24.04 and publishing data to a ROS2 Jazzy topic. For further assistance or adaptations for other sensors, consult the referenced resources.
