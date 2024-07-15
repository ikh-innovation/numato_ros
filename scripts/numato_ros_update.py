#!/usr/bin/env python2

import os
import rospy
import serial
import sys
from std_msgs.msg import Bool, Header
from ikh_ros_msgs.msg import Labjack_dout, Bumper_v2
import threading
import time

# Command mapping for GPIO operations
CMDS = {
    0: "clear",
    1: "set",
    True: "set",
    False: "clear"
}

# Lock for ensuring thread-safe access to the serial port
serial_lock = threading.Lock()

class SerialPortManager(object):
    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.open_serial_port()

    def __enter__(self):
        self.open_serial_port()
        return self.serial_port

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close_serial_port()

    def open_serial_port(self):
        """Opens the serial port."""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            rospy.loginfo("Serial port {} opened successfully.".format(self.port))
            return True
        except serial.SerialException as e:
            rospy.logerr("Error opening serial port {}: {}".format(self.port, e))
            return False

    def close_serial_port(self):
        """Closes the serial port if it's open."""
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()

    def is_serial_port_open(self):
        """Checks if the serial port is open."""
        return self.serial_port and self.serial_port.isOpen()

    def ensure_serial_connection(self):
        """Ensures that the serial port is open, trying to reopen if disconnected."""
        start = rospy.Time.now().to_sec()
        while not self.is_serial_port_open() and (rospy.Time.now().to_sec()-sec<10):
            rospy.logwarn("Serial port {} not open. Attempting to reopen.".format(self.port))
            self.open_serial_port()
            rospy.sleep(0.1)
        return self.is_serial_port_open()
        

class GPIO(object):
    def __init__(self, ser_port_manager, logic, serial_read_size=25):
        self.ser_port_manager = ser_port_manager
        self.logic = logic
        self.serial_read_size = serial_read_size

    def pin_to_index(self, pin):
        """Converts a pin number to an appropriate string or character."""
        return str(pin) if pin < 10 else chr(55 + int(pin))

    def read_io(self, pin):
        """Reads the state of a specified GPIO pin."""
        with serial_lock:
            try:
                if (self.ser_port_manager.ensure_serial_connection()):
                    self.ser_port_manager.serial_port.write("gpio read {}\n\r".format(self.pin_to_index(pin)))
                    res = self.ser_port_manager.serial_port.read(self.serial_read_size)
                    return res[-4]
                else:
                    return None
            except Exception as e:
                rospy.logwarn("Read IO: Cannot read serial: {}".format(e))
                return False

    def write_io(self, pin, state):
        """Writes a specified state to a GPIO pin."""
        with serial_lock:
            try:
                if (self.ser_port_manager.ensure_serial_connection()):
                    self.ser_port_manager.serial_port.write("gpio {} {}\r".format(CMDS[state], self.pin_to_index(pin)))
                    res = self.ser_port_manager.serial_port.read(self.serial_read_size)
                    return res
                else:
                    return None
            except Exception as e:
                rospy.logwarn("Write IO: Cannot write to serial: {}".format(e))
                return None

    @staticmethod
    def str_to_bool(value):
        """Converts a string to a boolean value."""
        if value.lower() in ["0", "false"]:
            return False
        elif value.lower() in ["1", "true"]:
            return True
        return None

    def read_mult_io(self, pins=[]):
        """Reads the state of multiple GPIO pins."""
        try:
            with serial_lock:
                if (self.ser_port_manager.ensure_serial_connection()):
                    self.ser_port_manager.serial_port.write("gpio readall\n\r")
                    res = str(self.ser_port_manager.serial_port.read(1000))
                else: 
                    return None
            
            hx = res[res.find("\n"):res.find(">")]
            bn = bin(int(str(hx), 16))[2:][::-1]
            result = {pin: self.str_to_bool(bn[pin]) for pin in pins if pin is not None}
            return result
        except Exception as e:
            rospy.logwarn("Read Multiple IOs: Cannot read serial: {}".format(e))
            #return {pin: False for pin in pins if pin is not None}
            return None

    def read_mult_io_avg(self, pins=[], readtimes=8):
        """Reads the state of multiple GPIO pins multiple times and averages the results."""
        try:
            result = {pin: [] for pin in pins if pin is not None}
            
            for _ in range(readtimes):
                with serial_lock:
                    if( self.ser_port_manager.ensure_serial_connection()):
                        self.ser_port_manager.serial_port.write("gpio readall\n\r")
                        res = str(self.ser_port_manager.serial_port.read(1000))
                    else:
                        return None
                hx = res[res.find("\n"):res.find(">")]
                bn = bin(int(str(hx), 16))[2:][::-1]
                for pin in pins:
                    if pin is not None:
                        result[pin].append(self.str_to_bool(bn[pin]))
            result_avg = {pin: self.get_avg_bool(result[pin], pin) for pin in result.keys()}
            return result_avg
        except Exception as e:
            rospy.logwarn("Read Multiple Avg IOs: Cannot read serial: {}".format(e))
            # return {pin: False for pin in pins if pin is not None}
            return None

    def get_avg_bool(self, arr, pin):
        """Averages a list of boolean values."""
        if all(arr):
            return True
        if not any(arr):
            return False

        if pin is not None:
            rospy.logwarn(
                "PIN: {}: False trigger IO, return False | True counts: {} False counts: {} Result size: {}".format(
                    pin, arr.count(True), arr.count(False), len(arr)
                )
            )
        else:
            rospy.logwarn(
                "False trigger IO, return False | True counts: {} False counts: {} Result size: {}".format(
                    arr.count(True), arr.count(False), len(arr)
                )
            )
        return False


class DigitalOutput(GPIO):
    def __init__(self, id, configs, ser_port_manager):
        GPIO.__init__(self, ser_port_manager, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['outputs'][id]["topic_name"]
        self.pin = configs['outputs'][id]["pin"]
        self.logic = configs['outputs'][id]["logic"]
        self.instance_rate = configs['outputs'][id]["rate"]
        self.state = False
        self.sub = rospy.Subscriber(self.topic_name, Bool, self.callback)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.instance_rate), self.update_callback)

    def callback(self, data):
        """Callback function for receiving data from the subscribed topic."""
        self.state = data.data

    def update_publish(self):
        """Writes the state to the GPIO pin and publishes the result."""
        if self.sub.get_num_connections() > 0:
            state_to_write = self.state if self.logic == "Normal" else not self.state
            res = self.write_io(self.pin, state_to_write)
            rospy.logdebug("DigitalOutput {}: Result - {}".format(self.id, res))
        else:
            rospy.logdebug("DigitalOutput {}: No subscribers.".format(self.id))
    
    def update_callback(self, event):
        self.update_publish()

class DigitalInput(GPIO):
    def __init__(self, id, configs, ser_port_manager):
        GPIO.__init__(self, ser_port_manager, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['inputs'][id]["topic_name"]
        self.pub = rospy.Publisher(self.topic_name, Labjack_dout, queue_size=1)
        self.pin = configs['inputs'][id]["pin"]
        self.logic = configs['inputs'][id]["logic"]
        self.instance_rate = configs['inputs'][id]["rate"]
        self.msg = Labjack_dout()
        self.msg.header.stamp = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0/self.instance_rate), self.update_callback)

    def update_publish(self):
        """Reads the state from the GPIO pin and publishes it."""
        if self.pub.get_num_connections() > 0:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.frame_id = self.topic_name
            res = self.str_to_bool(str(self.read_io(self.pin)))
            if isinstance(res, bool):
                if self.logic == "Normal":
                    self.msg.state = res
                elif self.logic == "Inverted":
                    self.msg.state = not res
                self.pub.publish(self.msg)
    
    def update_callback(self, event):
        self.update_publish()


class DigitalInputBumperV2(GPIO):
    def __init__(self, id, configs, ser_port_manager):
        GPIO.__init__(self, ser_port_manager, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['bumper_v2'][id]["topic_name"]
        self.pub = rospy.Publisher(self.topic_name, Bumper_v2, queue_size=1)
        self.logic = configs['bumper_v2'][id]["logic"]
        self.instance_rate = configs['bumper_v2'][id]["rate"]
        self.pin_mappings = {
            'low_center_bumper': configs['bumper_v2'][id].get('low_center_bumper', None),
            'low_left_bumper': configs['bumper_v2'][id].get('low_left_bumper', None),
            'low_right_bumper': configs['bumper_v2'][id].get('low_right_bumper', None),
            'high_center_bumper': configs['bumper_v2'][id].get('high_center_bumper', None),
            'high_right_bumper': configs['bumper_v2'][id].get('high_right_bumper', None),
            'high_left_bumper': configs['bumper_v2'][id].get('high_left_bumper', None),
            'state_bumper': configs['bumper_v2'][id].get('state_bumper', None)
        }
        self.timer = rospy.Timer(rospy.Duration(1.0/self.instance_rate), self.update_callback)

    def update_publish(self):
        """Reads and publishes the state of multiple bumper sensors."""
        if self.pub.get_num_connections() > 0:
            res = self.read_mult_io_avg([self.pin_mappings[key] for key in self.pin_mappings if self.pin_mappings[key]], 3)
            if (res!=None):
                msg = Bumper_v2(
                    header=Header(frame_id=self.topic_name, stamp=rospy.Time.now()),
                    low_bumper_center=res.get(self.pin_mappings['low_center_bumper'], False),
                    low_bumper_left=res.get(self.pin_mappings['low_left_bumper'], False),
                    low_bumper_right=res.get(self.pin_mappings['low_right_bumper'], False),
                    high_bumper_center=res.get(self.pin_mappings['high_center_bumper'], False),
                    high_bumper_left=res.get(self.pin_mappings['high_left_bumper'], False),
                    high_bumper_right=res.get(self.pin_mappings['high_right_bumper'], False),
                    bumper_state=res.get(self.pin_mappings['state_bumper'], False),
                )
                self.pub.publish(msg)
    def update_callback(self, event):
        self.update_publish()

class DigitalConstants(GPIO):
    def __init__(self, id, configs, ser_port_manager):
        GPIO.__init__(self, ser_port_manager, "Normal", serial_read_size=25)
        self.id = id
        self.pin = configs['constants'][id]["pin"]
        self.logic = configs['constants'][id]["logic"]
        self.state = self.logic == "Normal"
        self.write_io(self.pin, self.state)


class NumatoRelayInterface:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 19200)
        self.timeout = rospy.get_param('~timeout', 0.01)
        self.scriptrate = rospy.get_param('~rate', 10.0)
        self.configs = self.load_configs()
        # Initialize Serial Port Manager
        self.serial_port_manager = SerialPortManager(self.port, self.baudrate, self.timeout)
        self.serial_port_manager.open_serial_port()
        self.objs = self.dictionary_to_object_list(self.configs)
        rospy.loginfo("NumatoRelayInterface initialized with serial port: {}".format(self.port))

    def load_configs(self):
        """Loads configurations from ROS parameters."""
        return {
            'inputs': rospy.get_param('~inputs', {}),
            'outputs': rospy.get_param('~outputs', {}),
            'constants': rospy.get_param('~constants', {}),
            'bumper_v2': rospy.get_param('~bumper_v2', {})
        }

    def dictionary_to_object_list(self, configs):
        """Converts configuration dictionaries to a list of objects."""
        obj_list = []
        for cls, key in [(DigitalOutput, 'outputs'), (DigitalInput, 'inputs'), (DigitalConstants, 'constants'), (DigitalInputBumperV2, 'bumper_v2')]:
            for id in configs.get(key, {}):
                obj_list.append(cls(id, configs, self.serial_port_manager))
        return obj_list


    def __del__(self):
        rospy.loginfo("Shutting down Numato relay interface node.")


if __name__ == "__main__":
    rospy.init_node('numato_driver')
    try:
        rospy.loginfo("START NUMATO ROS!")
        nri = NumatoRelayInterface()

        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo("Shutting down Numato relay interface node.")
