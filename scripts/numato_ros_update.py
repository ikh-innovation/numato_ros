#!/usr/bin/env python2

import os
import rospy
import serial
import sys
from std_msgs.msg import Bool
from ikh_ros_msgs.msg import Labjack_dout, Bumper_v2
import threading

CMDS = {
    0: "clear",
    1: "set",
    True: "set",
    False: "clear"
}

class SerialPortManager(object):
    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.lock = threading.Lock()
        self.reconnect_thread = None
        self.reconnect_event = threading.Event()

    def __enter__(self):
        self.open_serial_port()
        return self.serial_port

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            rospy.loginfo("Serial port {} opened successfully.".format(self.port))
            if self.reconnect_thread and not self.reconnect_thread.is_alive():
                self.reconnect_event.set()  # Stop reconnect thread if it's running
                self.reconnect_thread.join()
                self.reconnect_event.clear()
                self.reconnect_thread = None
        except serial.SerialException as e:
            rospy.logerr("Error opening serial port {}: {}".format(self.port, e))
            self.start_reconnect_thread()

    def start_reconnect_thread(self):
        if not self.reconnect_thread or not self.reconnect_thread.is_alive():
            self.reconnect_event.set()
            self.reconnect_thread = threading.Thread(target=self.reconnect_loop)
            self.reconnect_thread.start()

    def reconnect_loop(self):
        while not rospy.is_shutdown() and self.reconnect_event.wait(1.0):
            if not self.serial_port or not self.serial_port.isOpen():
                try:
                    self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                    rospy.loginfo("Serial port {} reopened successfully.".format(self.port))
                    self.reconnect_event.clear()
                except serial.SerialException as e:
                    rospy.logwarn("Reconnect failed for serial port {}: {}".format(self.port, e))

    def reset_serial_port(self):
        with self.lock:
            if self.serial_port:
                self.serial_port.close()
            self.open_serial_port()
            rospy.loginfo("Serial port reset successful.")


class GPIO(object):
    def __init__(self, ser_port, logic, serial_read_size=25):
        self.ser_port = ser_port
        self.logic = logic
        self.serial_read_size = serial_read_size
        self.lock = threading.Lock()

    def pin_to_index(self, pin):
        return str(pin) if pin < 10 else chr(55 + int(pin))

    def read_io(self, pin):
        with self.lock:
            try:
                self.ser_port.write("gpio read {}\n\r".format(self.pin_to_index(pin)))
                res = self.ser_port.read(self.serial_read_size)
                return res[-4]
            except Exception as e:
                rospy.logwarn("Cannot read serial: {}".format(e))
                return False

    def write_io(self, pin, state):
        with self.lock:
            self.ser_port.write("gpio {} {}\r".format(CMDS[state], self.pin_to_index(pin)))
            res = self.ser_port.read(self.serial_read_size)
            return res

    def reset_serial(self):
        self.ser_port.reset_serial_port()

    @staticmethod
    def str_to_bool(value):
        if value.lower() in ["0", "false"]:
            return False
        elif value.lower() in ["1", "true"]:
            return True
        return None

    def read_mult_io(self, pins=[]):
        try:
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
        try:
            result = {pin: [] for pin in pins if pin is not None}
            for _ in range(readtimes):
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

    def get_avg_bool(self, bool_list, pin):
        return sum(bool_list) / float(len(bool_list)) > 0.5


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
        self.state = data.data

    def update_publish(self):
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
        if self.pub.get_num_connections() > 0:
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.frame_id = self.topic_name
            res = self.str_to_bool(self.read_io(self.pin))
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
        self.msg = Bumper_v2()
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
        if self.pub.get_num_connections() > 0:
            res = self.read_mult_io_avg([self.pin_mappings[key] for key in self.pin_mappings if self.pin_mappings[key]])
            msg = Bumper_v2(
                low_bumper_center=self.get_avg_bool(res[self.pin_mappings['low_center_bumper']], 'low_center_bumper'),
                low_bumper_left=self.get_avg_bool(res[self.pin_mappings['low_left_bumper']], 'low_left_bumper'),
                low_bumper_right=self.get_avg_bool(res[self.pin_mappings['low_right_bumper']], 'low_right_bumper'),
                high_bumper_center=self.get_avg_bool(res[self.pin_mappings['high_center_bumper']], 'high_center_bumper'),
                high_bumper_right=self.get_avg_bool(res[self.pin_mappings['high_right_bumper']], 'high_right_bumper'),
                high_bumper_left=self.get_avg_bool(res[self.pin_mappings['high_left_bumper']], 'high_left_bumper'),
                bumper_state=self.get_avg_bool(res[self.pin_mappings['state_bumper']], 'state_bumper')
            )
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.topic_name
            self.pub.publish(msg)


class DigitalConstants(GPIO):
    def __init__(self, id, configs, ser_port):
        GPIO.__init__(self, ser_port, "Inverted", serial_read_size=25)
        self.id = id
        self.topic_name = configs['constants'][id]["topic_name"]
        self.pin = configs['constants'][id]["pin"]
        self.logic = configs['constants'][id]["logic"]
        self.value = configs['constants'][id]["value"]
        self.ser_port = ser_port
        self.msg = Labjack_dout()
        self.msg.header.stamp = rospy.Time.now()

    def update_publish(self):
        if self.pub.get_num_connections() > 0:
            res = self.write_io(self.pin, self.value if self.logic == "Normal" else not self.value)
            rospy.logdebug("DigitalConstants {}: Result - {}".format(self.id, res))
        else:
            rospy.logdebug("DigitalConstants {}: No subscribers.".format(self.id))


class NumatoRelayInterface(object):
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyNUMATO')
        self.baud = rospy.get_param('~baud', 115200)
        self.timeout = rospy.get_param('~timeout', 1)
        self.objs = []
        self.load_configs()

    def load_configs(self):
        try:
            configs = rospy.get_param_cached("numato")
            self.objs = self.dictionary_to_object_list(configs)
        except rospy.ROSException as e:
            rospy.logerr("Failed to load Numato configuration: {}".format(e))
            sys.exit(1)

    def dictionary_to_object_list(self, configs):
        obj_list = []

        if 'outputs' in configs:
            obj_list.extend([DigitalOutput(key, configs, self._get_serial_port()) for key in configs['outputs']])

        if 'inputs' in configs:
            obj_list.extend([DigitalInput(key, configs, self._get_serial_port()) for key in configs['inputs']])

        if 'bumper_v2' in configs:
            obj_list.extend([DigitalInputBumperV2(key, configs, self._get_serial_port()) for key in configs['bumper_v2']])

        return obj_list

    def _get_serial_port(self):
        with SerialPortManager(self.port, self.baud, self.timeout) as ser_port:
            return ser_port

    def run(self):
        threads = []
        for obj in self.objs:
            thread = threading.Thread(target=obj.update_publish)
            thread.daemon = True
            thread.start()
            threads.append(thread)

        rospy.loginfo("Numato relay interface node running.")
        rospy.spin()

        for thread in threads:
            thread.join()

    def __del__(self):
        rospy.loginfo("Shutting down Numato relay interface node.")


if __name__ == "__main__":
    rospy.init_node('numato_driver')
    try:
        nri = NumatoRelayInterface()
        nri.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo("Shutting down Numato relay interface node.")
