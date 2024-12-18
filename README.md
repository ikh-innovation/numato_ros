
Add the following line to the ```/etc/udev/rules.d/99-custom.rules``

```sh
SUBSYSTEM=="tty",ACTION=="add", ATTRS{idVendor}=="2a19", ATTRS{idProduct}=="0802", MODE="0777", SYMLINK+="ttyNUMATO"
```

# Numato Driver ROS Node

## Overview

The Numato driver node is a ROS package designed to interface with a Numato USB GPIO expander. It allows for control and monitoring of GPIO pins through ROS topics, making it possible to integrate with other ROS nodes and systems.

## Requirements

- Python 2.x
- ROS (Robot Operating System)
- `ikh_ros_msgs` package
- `serial` package

## Installation

1. Ensure you have ROS installed on your system.
2. Install the `ikh_ros_msgs` package.
3. Install the `pyserial` library if not already installed:
   ```bash
   pip install pyserial
   ```

## Node Details

### Parameters

- `~port` (string, default: `/dev/ttyUSB0`): The serial port to which the Numato device is connected.
- `~baudrate` (int, default: `9600`): The baud rate for the serial communication.
- `~timeout` (float, default: `0.02`): Timeout for serial communication.
- `~poll_rate` (float, default: `30.0`): The rate at which the node reads the GPIO inputs.
- `~debounce_time` (float, default: `0.06`): Time duration during which an input must remain stable before being registered.
- `~inputs` (dict): Configuration for input pins.
- `~outputs` (dict): Configuration for output pins.
- `~constants` (dict): Configuration for constant pins.
- `~bumper_v2` (dict): Configuration for bumper pins.

### Subscribed Topics

- Configured dynamically based on the `outputs` parameter. Each output pin will subscribe to a topic of type `std_msgs/Bool`.

### Published Topics

- Configured dynamically based on the `inputs` and `bumper_v2` parameters. Each input pin will publish to a topic of type `ikh_ros_msgs/Labjack_dout` or `ikh_ros_msgs/Bumper_v2`.
- `~serial_port_state` is a latched `std_msgs/Bool` topic that reflects whether the serial port is connected (`True`) or not (`False`).

## Classes

### `SerialDevice`

Manages the serial port connection to the Numato device.

- `__init__()`: Initializes the class and starts the connection thread.
- `connect()`: Daemon thread target that constantly tries to connect to the serial port.
- `port_available()`: Returns if the serial port is available.
- `is_connected()`: Returns if the serial port is connected to.
- `set_disconnected()`: Sets the status as disconnected and notifies the connection thread to retry.
- `send_command(command)`: Sends the command to the serial port and checks if the response contains the command (Numato always echoes the command back).
- `read_serial()`: Thread target that constantly reads the state of all the GPIO inputs and performs debouncing.

### `NumatoDriver`

Main class for managing the Numato device.

- `__init__()`: Initializes the driver, loads configurations, and sets up the serial device.
- `load_configs()`: Loads configurations from ROS parameters.
- `dictionary_to_object_list(configs)`: Converts configuration dictionaries to a list of objects.

### `DigitalConstants`

Handles constant GPIO operations (either always high or always low).

- `__init__(configs, serial_device)`: Initializes the constant pin configuration.
- `update_callback()`: Writes the constant to the specified pin.

### `DigitalInput`

Handles GPIO input operations and publishes the pin state to a ROS topic.

- `__init__(configs, serial_device)`: Initializes the input pin configuration.
- `update_callback()`: Reads the state from the GPIO pin and publishes it.

### `DigitalOutput`

Handles GPIO output operations and subscribes to a ROS topic for setting the pin state.

- `__init__(configs, serial_device)`: Initializes the output pin configuration.
- `callback(data)`: Callback function for receiving data from the subscribed topic.
- `send_serial()`: Writes the state to the GPIO pin.

### `DigitalInputBumperV2`

Handles GPIO input operations for bumper sensors and publishes their states to a ROS topic.

- `__init__(configs, serial_device)`: Initializes the bumper pin configuration.
- `update_callback()`: Reads and publishes the state of multiple bumper sensors.

## Usage

1. Launch the ROS node:
   ```bash
   rosrun numato_ros numato_ros.py
   ```
2. Ensure the correct parameters are set either in a launch file or via the command line.

## Example Launch File

```xml
<launch>
    <arg name="driver_port" default="/dev/ttyNUMATO"/>
    <arg name="config_dir" default="$(find numato_ros)/config/io.yaml"/>
    <arg name="poll_rate" default="30"/>
    <arg name="baud" default="9600"/>
    <arg name="timeout" default="0.02"/>
    <arg name="debounce_time" default="0.06"/>   
    
    <node pkg="numato_ros" name="numato_ros" type="numato_ros.py" output="screen">
        <param name="port" value="$(arg driver_port)"/>
        <param name="baud" value="$(arg baud)"/>
        <param name="poll_rate" value="$(arg poll_rate)"/>
        <param name="timeout" value="$(arg timeout)"/>
        <param name="debounce_time" value="$(arg debounce_time)"/>
        <rosparam command="load" file="$(arg config_dir)" />
    </node>   
</launch>
```

## Notes

- Ensure that the Numato USB GPIO expander is properly connected to the specified serial port.
- Configure the `inputs`, `outputs`, `constants`, and `bumper_v2` parameters as needed to match your hardware setup.

