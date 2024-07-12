#!/usr/bin/env python2

import os
import rospy
import serial
import sys
from std_msgs.msg import Bool, Header
from ikh_ros_msgs.msg import Labjack_dout, Bumper_v2
import threading

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
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()

    def open_serial_port(self):
        """Opens the serial port."""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            rospy.loginfo("Serial port {} opened successfully.".format(self.port))
        except serial.SerialException as e:
            rospy.logerr("Error opening serial port {}: {}".format(self.port, e))
            sys.exit(1)  # Exit if the serial port cannot be opened


class GPIO(object):
    def __init__(self, ser_port, logic, serial_read_size=25):
        self.ser_port = ser_port
        self.logic = logic
        self.serial_read_size = serial_read_size
        self.lock = threading.Lock()

    def pin_to_index(self, pin):
        """Converts a pin number to an appropriate string or character."""
        return str(pin) if pin < 10 else chr(55 + int(pin))

    def read_io(self, pin):
        """Reads the state of a specified GPIO pin."""
        with serial_lock:
            try:
                self.ser_port.write("gpio read {}\n\r".format(self.pin_to_index(pin)))
                res = self.ser_port.read(self.serial_read_size)
                return res[-4]
            except Exception as e:
                rospy.logwarn("Cannot read serial: {}".format(e))
                return False

    def write_io(self, pin, state):
        """Writes a specified state to a GPIO pin."""
        with serial_lock:
            self.ser_port.write("gpio {} {}\r".format(CMDS[state], self.pin_to_index(pin)))
            res = self.ser_port.read(self.serial_read_size)
            return res

    def reset_serial(self):
        """Resets the serial port."""
        with serial_lock:
            if self.ser_port:
                self.ser_port.close()
            self.open_serial_port()
            rospy.loginfo("Serial port reset successful.")

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
                self.ser_port.write("gpio readall\n\r")
            res = str(self.ser_port.read(1000))
            hx = res[res.find("\n"):res.find(">")]
            bn = bin(int(str(hx), 16))[2:][::-1]
            result = {pin: self.str_to_bool(bn[pin]) for pin in pins if pin is not None}
            return result
        except Exception as e:
            rospy.logwarn("Cannot read serial: {}".format(e))
            return {pin: False for pin in pins if pin is not None}

    def read_mult_io_avg(self, pins=[], readtimes=8):
        """Reads the state of multiple GPIO pins multiple times and averages the results."""
        try:
            result = {pin: [] for pin in pins if pin is not None}
            
            for _ in range(readtimes):
                with serial_lock:
                    self.ser_port.write("gpio readall\n\r")
                res = str(self.ser_port.read(1000))
                hx = res[res.find("\n"):res.find(">")]
                bn = bin(int(str(hx), 16))[2:][::-1]
                for pin in pins:
                    if pin is not None:
                        result[pin].append(self.str_to_bool(bn[pin]))
            result_avg = {pin: self.get_avg_bool(result[pin], pin) for pin in result.keys()}
            return result_avg
        except Exception as e:
            rospy.logwarn("Cannot read serial: {}".format(e))
            return {pin: False for pin in pins if pin is not None}

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
    def __init__(self, id, configs, ser_port):
        GPIO.__init__(self, ser_port, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['outputs'][id]["topic_name"]
        self.pin = configs['outputs'][id]["pin"]
        self.logic = configs['outputs'][id]["logic"]
        self.state = False
        self.sub = rospy.Subscriber(self.topic_name, Bool, self.callback)

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


class DigitalInput(GPIO):
    def __init__(self, id, configs, ser_port):
        GPIO.__init__(self, ser_port, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['inputs'][id]["topic_name"]
        self.ser_port = ser_port
        self.pub = rospy.Publisher(self.topic_name, Labjack_dout, queue_size=1)
        self.pin = configs['inputs'][id]["pin"]
        self.logic = configs['inputs'][id]["logic"]
        self.msg = Labjack_dout()
        self.msg.header.stamp = rospy.Time.now()

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


class DigitalInputBumperV2(GPIO):
    def __init__(self, id, configs, ser_port):
        GPIO.__init__(self, ser_port, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['bumper_v2'][id]["topic_name"]
        self.ser_port = ser_port
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

    def update_publish(self):
        """Reads and publishes the state of multiple bumper sensors."""
        if self.pub.get_num_connections() > 0:
            res = self.read_mult_io_avg([self.pin_mappings[key] for key in self.pin_mappings if self.pin_mappings[key]],5)

            header = Header(frame_id=self.topic_name, stamp=rospy.Time.now())
            _low_bumper_center= res.get(self.pin_mappings['low_center_bumper'],False)
            _low_bumper_left= res.get(self.pin_mappings['low_left_bumper'],False)
            _low_bumper_right= res.get(self.pin_mappings['low_right_bumper'],False)
            _high_bumper_center= res.get(self.pin_mappings['high_center_bumper'],False) 
            _high_bumper_left= res.get(self.pin_mappings['high_left_bumper'],False) 
            _high_bumper_right= res.get(self.pin_mappings['high_right_bumper'],False) 
            _bumper_state= res.get(self.pin_mappings['state_bumper'],False) 

            msg = Bumper_v2()
            msg.header = Header(frame_id=self.topic_name, stamp=rospy.Time.now())
            msg.low_bumper_center = int(_low_bumper_center)
            msg = Bumper_v2(
                header = Header(frame_id=self.topic_name, stamp=rospy.Time.now()),
                low_bumper_center=_low_bumper_center,
                low_bumper_left= _low_bumper_left,
                low_bumper_right= _low_bumper_right,
                high_bumper_center= _high_bumper_center,
                high_bumper_left= _high_bumper_left,
                high_bumper_right= _high_bumper_right,
                bumper_state= _bumper_state,
            )
            self.pub.publish(msg)


class DigitalConstants(GPIO):
    def __init__(self, id, configs, ser_port):
        GPIO.__init__(self, ser_port, "Normal", serial_read_size=25)
        self.id = id
        self.pin = configs['constants'][id]["pin"]
        self.logic = configs['constants'][id]["logic"]
        self.state = self.logic == "Normal"
        self.write_io(self.pin, self.state)


class NumatoRelayInterface:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 19200)
        self.timeout = rospy.get_param('~timeout', 0.1)
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
                obj_list.append(cls(id, configs, self.serial_port_manager.serial_port))
        return obj_list


    def update(self):
        """Starts threads to update and publish the state of each GPIO object."""
        threads = []
        for obj in self.objs:
            thread = threading.Thread(target=obj.update_publish)
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()
    
    def run(self):
        while not rospy.is_shutdown():
            # print("--- Loop --- ")
            self.update()
            rospy.sleep(1.0/float(self.scriptrate))
        

    def __del__(self):
        rospy.loginfo("Shutting down Numato relay interface node.")


if __name__ == "__main__":
    rospy.init_node('numato_driver')
    try:
        rospy.loginfo("START NUMATO ROS!")
        nri = NumatoRelayInterface()
        nri.run()
        rospy.spin()

    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo("Shutting down Numato relay interface node.")
