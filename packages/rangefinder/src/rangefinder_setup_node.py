#!/usr/bin/env python3

import RPi.GPIO as GPIO
import sys, os
import VL53L1X
import time

#The purpose of this script:

#this script allows us to determine which rangefinder
#hardware is on board and then takes the steps to 
#configure it if needed.

#because each of the 4 sensors is identical, they each 
#start with the same i2c address. In order to fix this
#they must be given new addresses, however since the 
#sensors are all wired to the same connections changing
#one address would change all the others as well
#and we would have the same problem. To get arround this
#there is a special shutdown pin that we have wired to
#a different gpio for each of the sensors so what we do
#is shutdown all but one and sent the remap command
#then we shut down two of the remaining three and send
#the remap command, then one of the remaining two,
# then we remap the remaing one. 

def main():
    number_lidar_sensors_connected = 0

    path=os.path.join(os.path.dirname(__file__), '../config/lidar.yaml') 

    with open(path) as lidar_yaml:
        base = lidar_yaml.readline()
        base = lidar_yaml.readline()
        base = base.split()
        base = base[1] #get hex base address
        #stored in the lidar file
    
    i2c_channels = []
    for i in [0,1,2,3]:
        i2c = str(base)+str(i)
        i2c_channels.append(int(i2c, 16))

    GPIO.setwarnings(False)

    #these i2c_addresses 30,31,32,33 are arbitrary 
    #but they must match the ones that you reopen
    #when you spin up the lidar nodes
    GPIO.setmode(GPIO.BCM)
    mode = GPIO.getmode()
    print(mode)

    GPIO.setup([4,17,18,27], GPIO.OUT)
    GPIO.setup(4, GPIO.OUT)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(27, GPIO.OUT)
    GPIO.output(4, GPIO.LOW)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(4, GPIO.HIGH)
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(18, GPIO.HIGH)
    GPIO.output(27, GPIO.HIGH)
    #put all down except 4
    GPIO.output(4, GPIO.LOW)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    #then innit the first one


    GPIO.output(4, GPIO.HIGH)
    try:
        tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof1.open()
        tof1.change_address(i2c_channels[0])

        tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=i2c_channels[0])
        tof1.open()
        number_lidar_sensors_connected += 1
    except RuntimeError as e:
        print("Exeption for lidar sensor 4")
        print(e)



    GPIO.output(17, GPIO.HIGH)
    try:
        tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof2.open()
        tof2.change_address(i2c_channels[1])
        tof2 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=i2c_channels[1])
        tof2.open()
        number_lidar_sensors_connected += 1
    except RuntimeError as e:
        print("Exeption for lidar sensor 4")
        print(e)


    GPIO.setup(18, GPIO.HIGH)
    try:
        tof3 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof3.open()
        tof3.change_address(i2c_channels[2])
        tof3 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=i2c_channels[2])
        tof3.open()
        number_lidar_sensors_connected += 1
    except RuntimeError as e:
        print("Exeption for lidar sensor 4")
        print(e)

    GPIO.setup(27, GPIO.HIGH)
    try:
        tof4 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        tof4.open()
        tof4.change_address(i2c_channels[3])
        tof4 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=i2c_channels[3])
        tof4.open()
        number_lidar_sensors_connected += 1
    except RuntimeError as e:
        print("Exeption for lidar sensor 4")
        print(e)
    



    GPIO.cleanup()
    if number_lidar_sensors_connected == 0:
        sys.exit(10)

    
if __name__ == "__main__":
    main()
