#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED
from duckietown_msgs.msg import BoolStamped, CoordinationSignal


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.active = True
        self.pattern = [[0,0,0]]*5
        self.current_pattern_name = 'OFF'
        self.changePattern_(self.current_pattern_name)

        # Import protocol
        self.protocol = rospy.get_param("~LED_protocol")

        # If True, the LED turn on and off. Else, they are always on
        self.onOff = True

        if self.onOff:
            self.cycle       = 1.0/self.protocol['signals']['CAR_SIGNAL_A']['frequency']
            self.is_on       = False
            self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(self.cycle/(2.0)),self.cycleTimer)

        # Publish
        #self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.pub_state = rospy.Publisher("~current_led_state",String,queue_size=1)

        # Subscribe
        self.sub_pattern = rospy.Subscriber("~change_color_pattern",String,self.changePattern)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)

        # Scale intensity of the LEDs
        scale = 0.8
        for _, c in self.protocol['colors'].items():
           for i in range(3):
               c[i] = c[i]*scale

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        self.active = switch_msg.data

    def cycleTimer(self,event):
        if not self.active:
            return
        elif not self.onOff:
            # No oscillation
            for i in range(5):
                self.led.setRGB(i,[self.pattern[i][0],self.pattern[i][1],self.pattern[i][2]])
        else:
            # Oscillate
            if self.is_on:
                for i in range(5):
                    self.led.setRGB(i,[0,0,0])
                self.is_on = False
            else:
                for i in range(5):
                    self.led.setRGB(i,[self.pattern[i][0],self.pattern[i][1],self.pattern[i][2]])
                self.is_on = True

    def changePattern(self, msg):
        self.changePattern_(msg.data)

    def changePattern_(self, pattern_name):
        if pattern_name:
            if self.current_pattern_name == pattern_name:
                return
            else:
                self.current_pattern_name = pattern_name

            # With joystick
            if self.current_pattern_name == 'ON_WHITE':
                self.pattern = [self.protocol['colors']['white']]*5
            elif self.current_pattern_name == 'ON_RED':
                self.pattern = [self.protocol['colors']['red']]*5
            elif self.current_pattern_name == 'ON_BLUE':
                self.pattern = [self.protocol['colors']['blue']]*5
            elif self.current_pattern_name == 'ON_GREEN':
                self.pattern = [self.protocol['colors']['green']]*5
            elif self.current_pattern_name == 'CAR_SIGNAL_A':
                self.current_pattern_name = CoordinationSignal.SIGNAL_A

            elif self.current_pattern_name == 'SIGNAL_GREEN':
                self.current_pattern_name = CoordinationSignal.SIGNAL_GREEN
            elif self.current_pattern_name == 'OFF':
                self.current_pattern_name = CoordinationSignal.OFF
            else:
                self.pattern = [self.protocol['colors']['black']]*5

            # With coordination (new)
            if self.current_pattern_name == CoordinationSignal.SIGNAL_GREEN:
		color        = self.protocol['signals'][pattern_name]['color']
                self.pattern = [self.protocol['colors'][color]]*5
            elif self.current_pattern_name == CoordinationSignal.OFF:
                self.pattern = [self.protocol['colors']['black']]*5
            else:
                color           = self.protocol['signals'][pattern_name]['color']
                self.pattern    = [self.protocol['colors']['black']]*5
                self.pattern[2] = self.protocol['colors'][color]
                self.pattern[0] = self.protocol['colors'][color]
                self.pattern[4] = self.protocol['colors'][color]

            # Change frequency (frequency does not change)
            self.cycle = self.protocol['signals'][pattern_name]['frequency']
            self.changeFrequency()

            # Change LEDs
            if not self.onOff:
                self.cycleTimer([])

            # Loginfo
            rospy.loginfo('[%s] Pattern changed to (%r), cycle: %s ' %(self.node_name,pattern_name,self.cycle))

            # Publish current pattern
            self.pub_state.publish(self.current_pattern_name)

    def changeFrequency(self):
        try:
            #self.cycle = msg.data
            self.cycle_timer.shutdown()
            #below, convert to hz
            d = 1.0/(2.0*self.cycle)
            self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer)
        except ValueError as e:
            self.cycle = None
            self.current_pattern_name = None
        self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()
