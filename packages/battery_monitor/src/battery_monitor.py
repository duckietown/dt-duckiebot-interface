#!/usr/bin/env python

from duckietown import DTROS
from duckietown_msgs.msg import BatteryStatus
import serial
import re
import rospy

RETRIES = 5


class BatteryMonitor(DTROS):

    def __init__(self, node_name="battery_monitor"):
        super(BatteryMonitor, self).__init__(node_name)

        self.parameters['~device'] = None
        self.updateParameters()

        self.ser = serial.Serial(self.parameters['~device'], baudrate=9600, timeout=0.5)
        # Battery status is printed to serial as, for example, "SOC(%):   82"
        self.regex = re.compile(r"SOC\(%\):\s+(\d+)")

        self.pub = rospy.Publisher("~battery_monitor", BatteryStatus, queue_size=1)

        charge = self.read_battery_status()

        if charge is None:
            raise Exception("Unable to communicate with battery on port {}".format(self.parameters['~device']))

        self.timer = rospy.Timer(rospy.Duration.from_sec(10), self.read_battery_status)

    def read_battery_status(self, event=None):
        """
        Try a few times to read the status from the serial port. If successful, publish it on self.pub
        :param event: Event for using this as a timer callback
        :return: Percentage number, as int, or None if unsuccessful.
        """
        number = None
        # Sometimes (often) the battery just returns empty lines or partial lines. Try a few times, so we hopefully
        # get a successful read eventually.
        for i in range(0, RETRIES):
            line = self.ser.readline()
            # The battery status is including null characters for some reason.
            line = line.translate(None, b'\x00')
            line = line.decode('ascii')
            number = self.interpret_line(line)
            if number is not None:
                break

        if number is not None:
            msg = BatteryStatus()
            msg.charge = number / 100
            self.pub.publish(msg)

        return number

    def interpret_line(self, line):
        """
        Given a line of text, extract the number from it.

        Expects a format like this:
            SOC(%):   82
        :param line: Line of text, perhaps with the number in it.
        :return: The percentage number, as an int, or None if unsuccessful
        """
        try:
            result = self.regex.match(line)
            number = result.group(1)
            return int(number)
        except:
            return None


if __name__ == "__main__":
    try:
        node = BatteryMonitor()
        rospy.spin()
    except:
        print("Unable to communicate with battery. "
              "Assuming old battery is being used, and shutting down battery_monitor")
