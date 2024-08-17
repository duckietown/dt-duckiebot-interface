#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas and Stefanie Tellex"
__copyright__ = "Copyright 2014 Altax.net, 2017"

__license__ = "GPL"
__version__ = "1.5"

import asyncio
import logging
import serial
import time
import struct
import numpy as np
from asyncio import Lock

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def __str__(self):
        print(repr(self))
    def __repr__(self):
        return "PID(%f, %f, %f)" % (self.kp, self.ki, self.kd)

class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    RAW_IMU_STRUCT = struct.Struct('<hhhhhhhhh')
    POS_EST = 123
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ATTITUDE_STRUCT = struct.Struct('<hhh')
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    PID_STRUCT = struct.Struct('<BBBBBBBBBBBBBBBBBBBBBBBBBBBBBB')    
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254
    EEPROM_WRITE = 250

    SEND_ZERO_STRUCT1 = struct.Struct('<2B%dh' % 0)

    SEND_EIGHT_STRUCT1 = struct.Struct('<2B%dh' % 8)
    codeS = struct.Struct('<B')
    footerS = struct.Struct('B')
    emptyString = ""
    headerString = "$M<"

    """Class initialization"""
    def __init__(self, serPort = '/dev/ttyACM0'):

        """Global variables of data"""
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'elapsed':0,'timestamp':0}
        self.posest = {'x':0,'y':0,'z':0,'elapsed':0,'timestamp':0}
        self.MOTOR = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}

        self.pid = {'roll':None, 'pitch': None, 'yaw':None, 'alt':None ,'pos':None, 'posr':None, 'navr':None, 'level':None, 'mag':None, 'vel':None}

        self.ident = {"version":"", "multitype":"", "msp_version":"", "capability":""}
        self.status = {}
        self.ANALOG = {}
        self.boxids = []
        self.box = []
        self.elapsed = 0
        self.PRINT = 1

        self.logger = logging.getLogger(self.__class__.__name__)

        self.ser = serial.Serial(serPort,
                                 baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=1,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False,
                                 writeTimeout=2,
                             )
        self.serial_port_write_lock = Lock()
        self.serial_port_read_lock = Lock()

    def create_raw_rc_packet(self, channels):
        """
        Create a MSP packet for RAW_RC commanwith given channel values.
        channels: List of 8 integers, each between 1000 and 2000 representing RC channel values.
        """
        
        header = b'$M<'  # MSP header
        code = 200  # MSP code for RAW_RC
        data_length = 16  # Length of data for 8 channels, each 2 bytes

        # Pack the channels data into a byte string
        data = struct.pack('<8H', *channels)

        # Calculate the checksum: XOR of size, code, and all bytes in data
        checksum = data_length ^ code
        for byte in data:
            checksum ^= byte

        checksum &= 0xFF

        # Construct the complete packet
        packet = header + struct.pack('<BB', data_length, code) + data + struct.pack('<B', checksum)
        return packet


    """Function for sending a command to the board."""
    async def send_raw_command(self, data_length, code, data):
        async with self.serial_port_write_lock:
            if code is MultiWii.SET_RAW_RC:
                packet = self.create_raw_rc_packet(data)
            else:
                dl = len(data)
                if dl == 0:
                    s1 = MultiWii.SEND_ZERO_STRUCT1
                elif dl == 8:
                    s1 = MultiWii.SEND_EIGHT_STRUCT1
                else:
                    s1 = struct.Struct('<2B%dh' % len(data))

                dataString = s1.pack(data_length, code, *data)


                b = np.frombuffer(dataString, dtype=np.uint8)
                checksum = np.bitwise_xor.accumulate(b)[-1]
                footerString = MultiWii.footerS.pack(checksum)
                packet = bytes(MultiWii.headerString, 'ASCII') + dataString + footerString #+ b"\n"
            
            self.ser.write(packet)
            # self.logger.debug(f"Raw command sent {packet}")



    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition
      s_struct((uint8_t*)&att,6);
      break;

    """
    # def arm(self):
    #     timer = 0
    #     start = time.time()
    #     while timer < 0.5:
    #         data = [1500, 1500, 2000, 1000, 1500, 1500, 1500, 1500]
    #         self.sendCMD(16,MultiWii.SET_RAW_RC,data)
    #         time.sleep(0.05)
    #         timer = timer + (time.time() - start)
    #         start =  time.time()

    # def disarm(self):
    #     timer = 0
    #     start = time.time()
    #     while timer < 0.5:
    #         data = [1500,1500,1000,990, 1500, 1500, 1500, 1500]
    #         self.sendCMD(16,MultiWii.SET_RAW_RC,data)
    #         time.sleep(0.05)
    #         timer = timer + (time.time() - start)
    #         start =  time.time()

    """Function to receive a data packet from the board"""
    async def getData(self, cmd):
        await self.send_raw_command(0,cmd,[])
        return await self.receiveDataPacket()

    """ Sends a request for N commands from the board. """
    def getDataBulk(self, cmds):
        for c, args in cmds:
            self.send_raw_command(0, c, args)
        result = []
        for c, args in cmds:
            result.append(self.receiveDataPacket())
        return result

    async def receiveDataPacket(self):
        async with self.serial_port_read_lock:
            start = time.time()

            header = self.ser.read(5)
            if len(header) == 0:
                print("timeout on receiveDataPacket")
                return None
            elif header[0] != ord(b'$'):
                print("Didn't get valid header: ", header)
                raise ValueError("Invalid header")

            datalength = struct.unpack('B', header[-2:][0:1])[0]
            code = struct.unpack('B', header[-1:][0:1])[0]
            data = self.ser.read(datalength)
            #self.logger.debug(f"Received data packet: {header + data}")
            checksum = self.ser.read(1)
            self.checkChecksum(data, checksum)  # noop now.
            readTime = time.time()
            elapsed = readTime - start

            if code == MultiWii.ATTITUDE:
                temp = MultiWii.ATTITUDE_STRUCT.unpack(data)
                self.attitude['cmd'] = code
                self.attitude['angx'] = temp[0] / 10.0
                self.attitude['angy'] = temp[1] / 10.0
                self.attitude['heading'] = temp[2]
                self.attitude['elapsed'] = elapsed
                self.attitude['timestamp'] = readTime
                return self.attitude
            elif code == MultiWii.BOXIDS:
                temp = struct.unpack('<' + 'b' * datalength, data)
                self.boxids = temp
                return self.boxids
            elif code == MultiWii.SET_BOX:
                print("data", data)
                print("len", len(data))
            elif code == MultiWii.BOX:
                assert datalength % 2 == 0
                temp = struct.unpack('<' + 'H' * (datalength // 2), data)
                self.box = temp
                return self.box
            elif code == MultiWii.ANALOG:
                temp = struct.unpack('<' + 'B2HhH', data)
                self.ANALOG['voltage'] = temp[0] * 0.1
                self.ANALOG['intPowerMeterSum'] = temp[1]
                self.ANALOG['rssi'] = temp[2]
                self.ANALOG['amperage'] = temp[3]
                self.ANALOG['timestamp'] = readTime
                return self.ANALOG
            elif code == MultiWii.BOXNAMES:
                print("datalength", datalength)
                assert datalength % 2 == 0
                temp = struct.unpack('<' + 's' * datalength, data)
                temp = b"".join(temp).decode('utf-8')[:-1].split(";")
                self.boxnames = temp
                return self.boxnames
            elif code == MultiWii.STATUS:
                print(data)
                temp = struct.unpack('<' + 'HHHIb', data)
                self.status['cycleTime'] = temp[0]
                self.status['i2c_errors_count'] = temp[1]
                self.status['sensor'] = temp[2]
                self.status['flag'] = temp[3]
                self.status['global_conf.currentSet'] = temp[4]
                self.status['timestamp'] = readTime
                return self.status
            elif code == MultiWii.ACC_CALIBRATION:
                print("data", data)
                print("len", len(data))
            elif code == MultiWii.IDENT:
                temp = struct.unpack('<' + 'BBBI', data)
                self.ident["cmd"] = code
                self.ident["version"] = temp[0]
                self.ident["multitype"] = temp[1]
                self.ident["msp_version"] = temp[2]
                self.ident["capability"] = temp[3]
                self.ident['timestamp'] = readTime
                return self.ident
            elif code == MultiWii.RC:
                temp = struct.unpack('<' + 'hhhhhhhhhhhh', data)
                self.rcChannels['cmd'] = code
                self.rcChannels['roll'] = temp[0]
                self.rcChannels['pitch'] = temp[1]
                self.rcChannels['yaw'] = temp[2]
                self.rcChannels['throttle'] = temp[3]
                self.rcChannels['aux1'] = temp[4]
                self.rcChannels['aux2'] = temp[5]
                self.rcChannels['aux3'] = temp[6]
                self.rcChannels['aux4'] = temp[7]
                self.rcChannels['aux5'] = temp[8]
                self.rcChannels['aux6'] = temp[9]
                self.rcChannels['aux7'] = temp[10]
                self.rcChannels['aux8'] = temp[11]
                self.rcChannels['elapsed'] = elapsed
                self.rcChannels['timestamp'] = readTime
                return self.rcChannels
            elif code == MultiWii.PID:
                temp = MultiWii.PID_STRUCT.unpack(data)
                self.pid['roll'] = PID(temp[0], temp[1], temp[2])
                self.pid['pitch'] = PID(temp[3], temp[4], temp[5])
                self.pid['yaw'] = PID(temp[6], temp[7], temp[8])
                self.pid['alt'] = PID(temp[9], temp[10], temp[11])
                self.pid['pos'] = PID(temp[12], temp[13], temp[14])
                self.pid['posr'] = PID(temp[15], temp[16], temp[17])
                self.pid['navr'] = PID(temp[18], temp[19], temp[20])
                self.pid['level'] = PID(temp[21], temp[22], temp[23])
                self.pid['mag'] = PID(temp[24], temp[25], temp[26])
                self.pid['vel'] = PID(temp[27], temp[28], temp[29])
                print(self.pid)
            elif code == MultiWii.RAW_IMU:
                temp = MultiWii.RAW_IMU_STRUCT.unpack(data)
                self.rawIMU['cmd'] = code
                self.rawIMU['ax'] = temp[0]
                self.rawIMU['ay'] = temp[1]
                self.rawIMU['az'] = temp[2]
                self.rawIMU['gx'] = temp[3]
                self.rawIMU['gy'] = temp[4]
                self.rawIMU['gz'] = temp[5]
                self.rawIMU['timestamp'] = readTime
                return self.rawIMU
            elif code == MultiWii.POS_EST:
                temp = struct.unpack('<' + 'hhh', data)
                self.posest["cmd"] = code
                self.posest['x'] = float(temp[0])
                self.posest['y'] = float(temp[1])
                self.posest['z'] = float(temp[2])
                self.posest['elapsed'] = elapsed
                self.posest['timestamp'] = time.time()
                return self.posest
            elif code == MultiWii.MOTOR:
                temp = struct.unpack('<' + 'hhhhhhhh', data)
                self.MOTOR['cmd'] = code
                self.MOTOR['m1'] = float(temp[0])
                self.MOTOR['m2'] = float(temp[1])
                self.MOTOR['m3'] = float(temp[2])
                self.MOTOR['m4'] = float(temp[3])
                self.MOTOR['m5'] = float(temp[4])
                self.MOTOR['m6'] = float(temp[5])
                self.MOTOR['m7'] = float(temp[6])
                self.MOTOR['m8'] = float(temp[7])
                self.MOTOR['elapsed'] = elapsed
                self.MOTOR['timestamp'] = readTime
                return self.MOTOR
            elif code == MultiWii.SET_RAW_RC:
                return "Set Raw RC"
            else:
                print("No return error!: %d" % code)
                raise ValueError(f"Unknown code: {code}")


    """ Implement me to check the checksum. """
    def checkChecksum(self, data, checksum):
        pass
    def close(self):
        self.ser.close()




    """
    Calibrate the IMU and write outputs to a config file.
    """
    async def calibrate(self, fname):
        self.send_raw_command(0, MultiWii.ACC_CALIBRATION, [])
        print(await self.receiveDataPacket())

        # ignore the first 200 because it takes a while to settle.
        for i in range(200):
            raw_imu = await self.getData(MultiWii.RAW_IMU)
            time.sleep(0.01)
            print(raw_imu)


        raw_imu_totals = {}
        samples = 0.0
        for i in range(1000):
            raw_imu = await self.getData(MultiWii.RAW_IMU)
            print(raw_imu)
            if raw_imu != None:
                for key, value in raw_imu.items():
                    raw_imu_totals.setdefault(key, 0.0)
                    raw_imu_totals[key] += value
                samples += 1
                time.sleep(0.01)

        for key, value in raw_imu_totals.items():
            raw_imu_totals[key] = raw_imu_totals[key] / samples

        print(raw_imu_totals)

        import yaml
        f = open(fname, "w")
        yaml.dump(raw_imu_totals, f)
        f.close()

    def setBoxValues(self, values=(0,7,0,0)):
        self.send_raw_command(len(values) * 2, MultiWii.SET_BOX, values)

    def eepromWrite(self):
        self.send_raw_command(0, MultiWii.EEPROM_WRITE, [])
        
    async def update_battery(self):
        """
        Compute the ROS battery message by reading data from the board.
        """
        # extract voltage, amperage
        await self.getData(MultiWii.ANALOG)
        
    def motor_data(self):
        """
        Return the motor data as a list
        """
        return [self.MOTOR['m1'], self.MOTOR['m2'], self.MOTOR['m3'], self.MOTOR['m4']]