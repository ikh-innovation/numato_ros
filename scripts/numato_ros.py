#!/usr/bin/env python2
import rospy
import serial
import serial.tools.list_ports

from math import ceil
from std_msgs.msg import Bool
from threading import Thread, Lock, Condition
from ikh_ros_msgs.msg import Labjack_dout, Bumper_v2

logger = '[NumatoDriver] '

def pin_to_index(pin):
    return str(pin) if pin < 10 else chr(55 + int(pin))

class SerialDevice(object):
    def __init__(self):
        # Serial parameters
        self.device_port = rospy.get_param('~port', '/dev/ttyNUMATO')
        self.device_baudrate = rospy.get_param('~baudrate', 19200)
        self.device_timeout = rospy.get_param('~timeout', 0.01)
        self.poll_rate = rospy.get_param('~poll_rate', 10.0)
        self.debounce_cycles = int(ceil(max(1, self.poll_rate*rospy.get_param('~debounce_time', 0.1))))  # Number of cycles a value must remain consistent
        # Publisher
        self.serial_status_pub = rospy.Publisher('~serial_port_state', Bool, queue_size= 10, latch=True)
        self.serial_status_pub.publish(False)
        # I/O read state
        self.io_read_state = [False for i in range(32)]
        self.read_state_lock = Lock()        
        # Serial connectivity
        self.device = None        
        self.lock = Lock()
        self.connection_status = False
        self.condition_variable = Condition(lock = self.lock)
        self.connection_thread = Thread(target=self.connect)
        self.connection_thread.setDaemon(True)
        self.connection_thread.start()
        # Polling thread
        self.polling_thread = Thread(target=self.read_serial)
    
    def connect(self):
        while True:
            with self.condition_variable:
                while self.connection_status:
                    self.condition_variable.wait()

            while not self.is_connected():
                if self.port_available():
                    try:
                        with self.lock:
                            self.device = serial.Serial(port=self.device_port,baudrate=self.device_baudrate,timeout=self.device_timeout,write_timeout=self.device_timeout)
                            self.device.reset_input_buffer()
                            self.device.reset_output_buffer()
                            self.connection_status = True
                        rospy.loginfo(logger + 'Connected to port {}!'.format(self.device_port))
                        self.serial_status_pub.publish(True)
                    except Exception as e:
                        rospy.logerr(logger + 'Trying to connect to Numato. Port: {}'.format(self.device_port))
                        rospy.logerr(logger + 'Error: {}'.format(str(e)))
                        rospy.sleep(0.5)
                else:
                    rospy.logerr(logger + '{} not available'.format(self.device_port))
                    rospy.sleep(3.0)       
    
    def port_available(self):
        with self.lock:
            return self.device_port in [p.device for p in serial.tools.list_ports.comports(include_links=True)]

    def is_connected(self):
        with self.lock:
            return self.connection_status
        
    def set_disconnected(self):
        with self.condition_variable:
            self.connection_status = False
            self.serial_status_pub.publish(False)
            rospy.logerr(logger + 'Disconnected')
            try:
                self.device.close()
            except Exception as e:
                rospy.logerr(logger + 'Error while attempting to close serial connection: {}'.format(str(e)))
            self.condition_variable.notify()
    
    def send_command(self, command):
        result = None
        with self.lock:
            self.device.reset_input_buffer()
            self.device.reset_output_buffer()
            self.device.write(command)
            result = self.device.read_until('>')
            # Process result
            if not result.startswith(command[:-1]):
                rospy.logerr(logger + 'Response {} did not contain command {}'.format(str(result), command[:-1]))
                return None
            return result
    
    def read_serial(self):
        stable_states = {i: {'state': False, 'stable_cycles': 0} for i in range(32)}
        rate = rospy.Rate(self.poll_rate)
        cmd = 'gpio readall\r'
        while not rospy.is_shutdown():
            if not self.is_connected():
                rate.sleep()
                continue
            
            # Send command
            try:
                result = self.send_command(cmd)
            except Exception as e:
                rospy.logerr(logger + 'Error while attempting to send serial command: {}'.format(str(e)))
                self.set_disconnected()
                continue
            
            # Check if response is ok
            if result is None:
                rate.sleep()
                continue            
            
            if len(result) <= len(cmd)+1:
                rospy.logerr(logger + 'Response {} is too short'.format(str(result)))
                rate.sleep()
                continue
            
            # Keep bytes that represent the read state of the i/o pins
            result = result[len(cmd)+1:].splitlines()[0].rstrip()            
            if len(result)!=8:
                rospy.logerr(logger + 'Response {} is not 8 bytes long'.format(str(result)))
                rate.sleep()
                continue
            
            # Turn them into binary
            result = bin(int(result, 16))[2:].zfill(32)
            new_states = [True if i=='1' else False for i in result[::-1]]
            
            # Apply debouncing
            for i, new_state in enumerate(new_states):
                if stable_states[i]['state'] == new_state:
                    stable_states[i]['stable_cycles'] = min(stable_states[i]['stable_cycles'] + 1, self.debounce_cycles + 1)
                else:
                    stable_states[i]['state'] = new_state
                    stable_states[i]['stable_cycles'] = 0                
                
                # Update state if stable for required cycles
                if stable_states[i]['stable_cycles'] == self.debounce_cycles:                                
                    # Update the I/O read state
                    with self.read_state_lock:            
                        self.io_read_state[i] = new_state
            
            rate.sleep()

class NumatoDriver(object):
    def __init__(self):
        # Serial device
        self.serial_device = SerialDevice()
        # User numato configuration
        configs = self.load_configs()
        self.objs = self.dictionary_to_object_list(configs)
        # Start reading thread only if there are classes which read from numato
        if any([type(o)==DigitalInput or type(o)==DigitalInputBumperV2 for o in self.objs]):
            self.serial_device.polling_thread.start()
    
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
        for cls, key in [(DigitalInput, 'inputs'), (DigitalConstants, 'constants'), (DigitalOutput, 'outputs'), (DigitalInputBumperV2, 'bumper_v2')]:
            for id in configs.get(key, {}):
                obj_list.append(cls(configs[key][id], self.serial_device))
        return obj_list

class DigitalInput(object):
    def __init__(self, configs, serial_device):
        self.serial_device = serial_device
        self.pub = rospy.Publisher(configs["topic_name"], Labjack_dout, queue_size=1)
        self.pin = configs["pin"]
        self.logic = configs["logic"]
        self.msg = Labjack_dout()
        self.msg.header.frame_id = configs["topic_name"]
        self.timer = rospy.Timer(rospy.Duration(1.0/configs["rate"]), self.update_callback)
    
    def update_callback(self, event):
        """Reads the state from the GPIO pin and publishes it."""
        if self.pub.get_num_connections() > 0:
            self.msg.header.stamp = rospy.Time.now()
            with self.serial_device.read_state_lock:
                self.msg.state = self.serial_device.io_read_state[self.pin]
            if self.logic == "Inverted":
                self.msg.state = not self.msg.state
            self.pub.publish(self.msg)

class DigitalConstants(object):
    def __init__(self, configs, serial_device):
        self.serial_device = serial_device
        self.pin = configs["pin"]
        value = configs["value"] if configs["logic"] == "Normal" else not configs["value"]
        self.cmd = "set" if value else "clear"
        self.timer = rospy.Timer(rospy.Duration(0.001), self.update_callback, oneshot=True)
    
    def update_callback(self, event):
        """Set constant when connected"""
        success = False
        command = 'gpio {} {}\r'.format(self.cmd, pin_to_index(self.pin))
        while not success:
            while not self.serial_device.is_connected():
                if rospy.is_shutdown():
                    return
                rospy.sleep(1.0)
            
            try:
                result = self.serial_device.send_command(command)
                # Check if response is ok
                if result is None:
                    rospy.sleep(1.0)
                    continue
                else:
                    success = True
            except Exception as e:
                rospy.logerr(logger + 'Error while attempting to send serial command: {}'.format(str(e)))
                self.serial_device.set_disconnected()
                continue

class DigitalOutput(object):
    def __init__(self, configs, serial_device):
        self.serial_device = serial_device        
        self.pin = configs["pin"]
        self.logic = configs["logic"]
        self.new = False
        self.state = False
        self.state_lock = Lock()
        self.state_condition = Condition(lock=self.state_lock)
        self.sub = rospy.Subscriber(configs["topic_name"], Bool, self.callback)
        self.send_thread = Thread(target=self.send_serial)
        self.send_thread.setDaemon(True)
        self.send_thread.start()

    def callback(self, data):
        """Callback function for receiving data from the subscribed topic."""
        with self.state_lock:
            self.state = data.data if self.logic == 'Normal' else not data.data
            self.new = True
            self.state_condition.notify()
            
    def send_serial(self):
        while True:
            with self.state_condition:
                while not self.new:
                    self.state_condition.wait()
                
                while not self.serial_device.is_connected():
                    rospy.sleep(1.0)
                    continue
                
                cmd = "set" if self.state else "clear"
                command = 'gpio {} {}\r'.format(cmd, pin_to_index(self.pin))
            
                try:
                    result = self.serial_device.send_command(command)
                    # Check if response is ok
                    if result is None:
                        rospy.sleep(1.0)
                        continue
                    else:
                        self.new = False
                except Exception as e:
                    rospy.logerr(logger + 'Error while attempting to send serial command: {}'.format(str(e)))
                    self.serial_device.set_disconnected()
                    continue

class DigitalInputBumperV2(object):
    def __init__(self, configs, serial_device):
        self.serial_device = serial_device
        self.logic = configs["logic"]
        self.pin_mappings = {
            'low_bumper_center': configs.get('low_center_bumper', None),
            'low_bumper_left': configs.get('low_left_bumper', None),
            'low_bumper_right': configs.get('low_right_bumper', None),
            'high_bumper_center': configs.get('high_center_bumper', None),
            'high_bumper_left': configs.get('high_left_bumper', None),
            'high_bumper_right': configs.get('high_right_bumper', None),            
            'bumper_state': configs.get('state_bumper', None)
        }
        # Prepare msg
        self.msg = Bumper_v2()
        self.msg.header.frame_id = configs["topic_name"]
        for key, value in self.pin_mappings.items():
            if value is None:
                setattr(self.msg, key, False)
                self.pin_mappings.pop(key)
        
        self.pub = rospy.Publisher(configs["topic_name"], Bumper_v2, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/configs["rate"]), self.update_callback)

    def update_callback(self, event):
        """Reads and publishes the state of multiple bumper sensors."""
        if self.pub.get_num_connections() > 0:
            with self.serial_device.read_state_lock:
                io_state = self.serial_device.io_read_state
            
            if not self.logic == 'Normal':
                io_state = [not i for i in io_state]
            
            self.msg.header.stamp = rospy.Time.now()
            for key, value in self.pin_mappings.items():
                setattr(self.msg, key, io_state[value])
                
            self.pub.publish(self.msg)

if __name__ == "__main__":
    rospy.init_node('numato_driver')
    try:
        rospy.loginfo(logger + 'Start Numato ROS!')
        ndr = NumatoDriver()
        rospy.spin()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        print('Shutting down Numato driver node.')