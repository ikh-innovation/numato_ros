# Numato Relay Interface ROS Node

## Overview

The Numato Relay Interface node is a ROS package designed to interface with a Numato USB GPIO expander. It allows for control and monitoring of GPIO pins through ROS topics, making it possible to integrate with other ROS nodes and systems.

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
4. Add the following line to the ```/etc/udev/rules.d/99-custom.rules``
   ```sh
   SUBSYSTEM=="tty",ACTION=="add", ATTRS{idVendor}=="2a19", ATTRS{idProduct}=="0802", MODE="0777", SYMLINK+="ttyNUMATO"
   ```

## Node Details

### Parameters

- `~port` (string, default: `/dev/ttyUSB0`): The serial port to which the Numato device is connected.
- `~baudrate` (int, default: `19200`): The baud rate for the serial communication.
- `~timeout` (float, default: `0.1`): Timeout for serial communication.
- `~rate` (float, default: `10.0`): The rate at which the node updates the GPIO states.
- `~inputs` (dict): Configuration for input pins.
- `~outputs` (dict): Configuration for output pins.
- `~constants` (dict): Configuration for constant pins.
- `~bumper_v2` (dict): Configuration for bumper pins.

### Subscribed Topics

- Configured dynamically based on the `outputs` parameter. Each output pin will subscribe to a topic of type `std_msgs/Bool`.

### Published Topics

- Configured dynamically based on the `inputs` and `bumper_v2` parameters. Each input pin will publish to a topic of type `ikh_ros_msgs/Labjack_dout` or `ikh_ros_msgs/Bumper_v2`.

## Classes

### `SerialPortManager`

Manages the serial port connection to the Numato device.

- `__init__(port, baudrate, timeout)`: Initializes and opens the serial port.
- `open_serial_port()`: Opens the serial port.
- `close_serial_port()`: Closes the serial port.
- `is_serial_port_open()`: Checks if the serial port is open.
- `ensure_serial_connection()`: Ensures the serial port is open, attempting to reopen if disconnected.

### `GPIO`

Base class for handling GPIO operations.

- `pin_to_index(pin)`: Converts a pin number to the appropriate string/character.
- `read_io(pin)`: Reads the state of a specified GPIO pin.
- `write_io(pin, state)`: Writes a specified state to a GPIO pin.
- `str_to_bool(value)`: Converts a string to a boolean value.
- `read_mult_io(pins)`: Reads the state of multiple GPIO pins.
- `read_mult_io_avg(pins, readtimes)`: Reads the state of multiple GPIO pins multiple times and averages the results.
- `get_avg_bool(arr, pin)`: Averages a list of boolean values.

### `DigitalOutput`

Handles GPIO output operations and subscribes to a ROS topic for setting the pin state.

- `__init__(id, configs, ser_port_manager)`: Initializes the output pin configuration.
- `callback(data)`: Callback function for receiving data from the subscribed topic.
- `update_publish()`: Writes the state to the GPIO pin and publishes the result.

### `DigitalInput`

Handles GPIO input operations and publishes the pin state to a ROS topic.

- `__init__(id, configs, ser_port_manager)`: Initializes the input pin configuration.
- `update_publish()`: Reads the state from the GPIO pin and publishes it.

### `DigitalInputBumperV2`

Handles GPIO input operations for bumper sensors and publishes their states to a ROS topic.

- `__init__(id, configs, ser_port_manager)`: Initializes the bumper pin configuration.
- `update_publish()`: Reads and publishes the state of multiple bumper sensors.

### `DigitalConstants`

Handles constant GPIO operations (either always high or always low).

- `__init__(id, configs, ser_port_manager)`: Initializes the constant pin configuration.

### `NumatoRelayInterface`

Main class for managing the overall Numato Relay Interface.

- `__init__()`: Initializes the interface, loads configurations, and sets up the serial port manager.
- `load_configs()`: Loads configurations from ROS parameters.
- `dictionary_to_object_list(configs)`: Converts configuration dictionaries to a list of objects.
- `update()`: Starts threads to update and publish the state of each GPIO object.
- `run()`: Main loop for running the node.

## Usage

1. Launch the ROS node:
   ```bash
   rosrun your_package_name numato_driver.py
   ```
2. Ensure the correct parameters are set either in a launch file or via the command line.

## Example Launch File

```xml
<launch>
    <node name="numato_ros" pkg="numato_ros" type="numato_ros" output="screen">
        <param name="port" value="/dev/ttyNUMATO" />
        <param name="baudrate" value="115200" />
        <param name="timeout" value="0.002" />
        <param name="rate" value="10.0" />
        <param name="inputs" value="$(arg inputs)" />
        <param name="outputs" value="$(arg outputs)" />
        <param name="constants" value="$(arg constants)" />
        <param name="bumper_v2" value="$(arg bumper_v2)" />
    </node>
</launch>
```

## Notes

- Ensure that the Numato USB GPIO expander is properly connected to the specified serial port.
- Configure the `inputs`, `outputs`, `constants`, and `bumper_v2` parameters as needed to match your hardware setup.

