'''
A simple pymavlink based communicator. Works with Pixhawk controller.
'''
import atexit
import sys
import time
import os
import numpy
import signal
import logging
import threading

from pymavlink import mavutil

# SEQUENCE OF CONNECTION
# 1. Open Mavlink connection
# 2. Wait for heartbeat from pixhawk
# 3. Request parameter fetch all command
# 3. Wait and ensure all parameters is received without BAD_DATA
# 4. ARM Quad. Ensure the quad is ARMED before proceeding
# 5. Read messages from PX4 and check for RC_CHANNELS_RAW type of message
# 6. Send rc override message back to PX4
# 7. Repeat steps 5 and 6, N times
# 8. Send DISARM signal to PX4
# 9. Close the Mavlink connection

def detect_pixhawk():
    '''
    Detect the serial port where pixhawk is connected automatically. Adapted from Mavproxy code.
    '''
    serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*"]) 
    if len(serial_list) == 1: 
        return serial_list[0].device
    else: 
        print(''' 
Please choose a MAVLink master with --master 
For example: 
        --master=com14 
        --master=/dev/ttyUSB0 
        --master=127.0.0.1:14550 
     
Auto-detected serial ports are: 
''') 
        for port in serial_list: 
            print("%s" % port) 
            sys.exit(1)


class SimpleMAV(object):
    manual_control_string = "rc"

    def __init__(self, master, sysID=1, compoID=250):
        if master == "/dev/ttyACM0":
            master = detect_pixhawk()
            print "Pixhawk Master is", master
            baud = 115200 
            reboot = False 
        else:
            baud = 57600
            reboot = True

        self.conn = mavutil.mavlink_connection(device=master, baud=baud,
                                               autoreconnect=True)
        if reboot:
            logging.info("Reboot pixhawk !")
            self.conn.reboot_autopilot()
            time.sleep(0.5)
        self.sysID = sysID
        self.compoID = compoID
        self.heartbeat_period = mavutil.periodic_event(1)
        self.heartbeat_period.frequency = 1.0
        self.recv_msg_count = 0
        self.bad_data_count = 0
        self.param_names = {}
        for i in range(1, 8):
            self.param_names["RC%d_MIN" % i] = None
            self.param_names["RC%d_MAX" % i] = None

        self.initialize()

    def required_parameters_received(self):
        for value in self.param_names.itervalues():
            if value is None:
                return False
        return True            

    def fetch_param(self):
        """Attempts to fetch param. Resends the fetch request if params have
        not been full received without certain period."""
        self.conn.param_fetch_all()
        msg_count = 0
        attempts = 0
        bad_data = 0
        param_data = 0
        while not self.conn.param_fetch_complete:
            m = self.conn.recv_msg()
            if m is not None:
                if m.get_type() == "BAD_DATA":
                    bad_data +=  1
                if m.get_type() == "PARAM_VALUE":
                    param_data += 1
                    if str(m.param_id) in self.param_names:
                        self.param_names[str(m.param_id)] = m.param_value
                logging.debug(m)
                msg_count = msg_count + 1
            else:
                time.sleep(0.01)
            if (msg_count == 1000): ## retry request for parameters
                if not self.required_parameters_received():
                    logging.info("Received %s useful parameters" % (self.param_names))
                    logging.info("BAD_DATA : %d, PARAM = %d, Total Msg = %d" % (bad_data, param_data, msg_count))
                    print "Unable to retreive parameter from pixhawk ! Requesting parameters again !"
                    self.conn.param_fetch_all()
                    attempts += 1
                    msg_count = 0
                    param_data = 0
        logging.info("Received all parameters after %d attempts" % (attempts))
        logging.info("Received %s useful parameters" % (self.param_names))
        logging.info("Total parameters: %d" % len(self.conn.params))
        logging.info("BAD_DATA : %d, PARAM = %d, Total Msg = %d" % (bad_data, param_data, msg_count))
        self.params = self.conn.params

    def initialize(self):
        self.conn.wait_heartbeat()
        self.fetch_param()

        """Grab all needed params from the ArduCopter instance"""
        self.rc1MIN = self.params["RC1_MIN"]
        self.rc2MIN = self.params["RC2_MIN"]
        self.rc3MIN = self.params["RC3_MIN"]
        self.rc4MIN = self.params["RC4_MIN"]
        self.rc5MIN = self.params["RC5_MIN"]
        self.rc6MIN = self.params["RC6_MIN"]
        self.rc7MIN = self.params["RC7_MIN"]
        self.rc8MIN = self.params["RC8_MIN"]
        self.rc1MAX = self.params["RC1_MAX"]
        self.rc2MAX = self.params["RC2_MAX"]
        self.rc3MAX = self.params["RC3_MAX"]
        self.rc4MAX = self.params["RC4_MAX"]
        self.rc5MAX = self.params["RC5_MAX"]
        self.rc6MAX = self.params["RC6_MAX"]
        self.rc7MAX = self.params["RC7_MAX"]
        self.rc8MAX = self.params["RC8_MAX"]

    def arm(self):
        """Arm ArduCopter"""
        ack_msg = None
        while not self.conn.motors_armed():
            self.conn.arducopter_arm()
            ack_msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.2)
            if ack_msg is not None:
                print "Acknowledgement received : Command = " + str(ack_msg.command) + ", result = " + str(ack_msg.result)
            self.check_periodic_task()
            print "Sent ARM message. Ensure safety switch is SOLID RED"
            print "waiting for ARM confirmation."
            time.sleep(1.0)
        time.sleep(1.0)
        print "Quad is armed and ready to fly !"

    def disarm(self):
        """Disarm ArduCopter"""
        print "Disarm Quad manually !!!"
        ack_msg = None
        print "Sent DISARM message.."
        while (ack_msg is None or ack_msg.result != 0):
            self.conn.arducopter_disarm()
            self.check_periodic_task()
            print "waiting for acknowledgement.."
            ack_msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.2)
            time.sleep(0.5)
        print "Acknowledgement received : Command = " + str(ack_msg.command) + ", result = " + str(ack_msg.result)
        self.check_periodic_task()
        self.conn.wait_heartbeat()
        time.sleep(1.0)

    def signal_handler(self, signal, frame):
        time.sleep(0.1)
        #self.clear()
        sys.exit(0)

    def send_heartbeat(self, master):
        if master.mavlink10():
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                      0, 0, 0)
        else:
            MAV_GROUND = 5
            MAV_AUTOPILOT_NONE = 4
            master.mav.heartbeat_send(MAV_GROUND, MAV_AUTOPILOT_NONE)

    def check_periodic_task(self):
        if self.heartbeat_period.trigger() and self.heartbeat_period.frequency != 0:
            self.send_heartbeat(self.conn)

    def getControllerRCValue(self):
        while True:
            self.check_periodic_task()
            m = self.conn.recv_match(blocking=False)
            if m is None:
                time.sleep(0.1)
                continue
            self.recv_msg_count += 1
            if m.get_type() == "RC_CHANNELS_RAW":
                return [m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw, m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw]
            elif m.get_type() == "BAD_DATA":
                self.bad_data_count += 1

    def rcSend(self, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8):
        """Send raw rc values, check for valid range"""
        self.check_periodic_task()
        if ch1 != 0:
            ch1 = numpy.minimum(ch1, self.rc1MAX)
            ch1 = numpy.maximum(ch1, self.rc1MIN)
        if ch2 != 0:
            ch2 = numpy.minimum(ch2, self.rc2MAX)
            ch2 = numpy.maximum(ch2, self.rc2MIN)
        if ch3 != 0:
            ch3 = numpy.minimum(ch3, self.rc3MAX)
            ch3 = numpy.maximum(ch3, self.rc3MIN)
        if ch4 != 0:
            ch4 = numpy.minimum(ch4, self.rc4MAX)
            ch4 = numpy.maximum(ch4, self.rc4MIN)
        if ch5 != 0:
            ch5 = numpy.minimum(ch5, self.rc5MAX)
            ch5 = numpy.maximum(ch5, self.rc5MIN)
        if ch6 != 0:
            ch6 = numpy.minimum(ch6, self.rc6MAX)
            ch6 = numpy.maximum(ch6, self.rc6MIN)
        if ch7 != 0:
            ch7 = numpy.minimum(ch7, self.rc7MAX)
            ch7 = numpy.maximum(ch7, self.rc7MIN)
        if ch8 != 0:
            ch8 = numpy.minimum(ch8, self.rc8MAX)
            ch8 = numpy.maximum(ch8, self.rc8MIN)
        self.conn.mav.rc_channels_override_send(
            self.sysID, self.compoID, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8)

    def velocitySend(self):
        """Take velocity values and convert them to corresponding RC values"""
        pass

    def get_scaled_channel_value(self, channel_val, rcmin, rcmax):
        """
        if channel is configured to manual mode than send "0", else send the override data scaled to range
        """
        if channel_val == "rc":
            channel_val = 0
        else:
            channel_val = ((channel_val + 1.0) / 2.0) * (rcmax - rcmin) + rcmin

        return channel_val

    # def scaledRCSend(self, ch1, ch2, ch3, ch4, ch5 = 0.0, ch6 = 0.0, ch7 =
    # 0.0, ch8 = 0.0):
    def scaledRCSend(self, ch1, ch2, ch3, ch4, ch5="rc", ch6="rc", ch7="rc", ch8="rc"):
        """Converts a number between -1 and +1 to the proportinal valid RC value"""
        # Check if control should be given to hand held controller, or convert
        ch1 = self.get_scaled_channel_value(ch1, self.rc1MIN, self.rc1MAX)
        ch2 = self.get_scaled_channel_value(ch2, self.rc2MIN, self.rc2MAX)
        ch3 = self.get_scaled_channel_value(ch3, self.rc3MIN, self.rc3MAX)
        ch4 = self.get_scaled_channel_value(ch4, self.rc4MIN, self.rc4MAX)
        ch5 = self.get_scaled_channel_value(ch5, self.rc5MIN, self.rc5MAX)
        ch6 = self.get_scaled_channel_value(ch6, self.rc6MIN, self.rc6MAX)
        ch7 = self.get_scaled_channel_value(ch7, self.rc7MIN, self.rc7MAX)
        ch8 = self.get_scaled_channel_value(ch8, self.rc8MIN, self.rc8MAX)

        # Send PWM
        self.rcSend(ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8)

    def releaseRC(self):
        """Releases all RC channels back to the phsycial controler, if there is one"""
        self.conn.mav.rc_channels_override_send(
            self.sysID, self.compoID, 0, 0, 0, 0, 0, 0, 0, 0)
        print "ReleaseRC control !"

    def setMode(self, mode):
        pass

    def clear(self):
        print "Clearing SimpleMav.."
        self.check_periodic_task()
        print "Waiting for heartbeat..."
        self.conn.wait_heartbeat()
        print "Closing connection..."
        self.conn.close()
        time.sleep(1.0)
        print "Bad data recevied: %d, Messages received: %d" % (self.bad_data_count, self.recv_msg_count)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    s = SimpleMAV(master="/dev/ttyACM0")
    try:
        if 1:
            print "Ensure that safety switch is presed before arming !"
            time.sleep(1)
            s.arm()
            print "!!!!!!!!!!!!!!!!!!"
            for x in range(50):
                time.sleep(0.1)
                data = s.getControllerRCValue()
                print data
                s.scaledRCSend(0.0, 0.0, -0.8, 0.0)
                if data[7] > (s.rc6MIN + ((s.rc6MAX - s.rc6MIN)/2)):
                    break
            print "?????????????????"
            time.sleep(1)
            s.scaledRCSend("rc", "rc", "rc", "rc")
            s.disarm()
        s.clear()
    except:
        print "Exception raised !"
        print "Unexpected error:", sys.exc_info()[0]
        s.clear()
        time.sleep(1)
        raise
    sys.exit(0)
