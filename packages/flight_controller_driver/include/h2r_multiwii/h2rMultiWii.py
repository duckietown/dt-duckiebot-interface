#!/usr/bin/env python3

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas and Stefanie Tellex"
__copyright__ = "Copyright 2014 Altax.net, 2017"

__license__ = "GPL"
__version__ = "1.5"

import numpy as np
import serial
import struct
import time
import yaml
import json
from typing import Literal, Optional

from .types import MultiWiiRpyPid


class MultiWii:
    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    IDENT_STRUCT = struct.Struct('<BBBI')
    STATUS = 101
    RAW_IMU = 102
    RAW_IMU_STRUCT = struct.Struct('<hhhhhhhhh')
    POS_EST = 123
    POS_EST_STRUCT = struct.Struct('<hhh')
    SERVO = 103
    MOTOR = 104
    MOTOR_STRUCT = struct.Struct('<hhhhhhhh')
    RC = 105
    RC_STRUCT = struct.Struct('<hhhhhhhhhhhh')
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ATTITUDE_STRUCT = struct.Struct('<hhh')
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    PID_STRUCT = struct.Struct('<bbbbbbbbbbbbbbbbbbbbbbbbbbbbbb')
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
    SET_PID_STRUCT = struct.Struct('<2B%db' % 30)
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
    emptyString = b""

    # python2.7: headerString = "$M<" now we convert to hex on the spot
    headerString = b"\x24\x4d\x3c"

    """Class initialization"""

    def __init__(self, serPort):
        """Global variables of data"""
        self.rcChannels = {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0, 'elapsed': 0,
                           'timestamp': 0}
        self.rawIMU = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0, 'elapsed': 0,
                       'timestamp': 0}
        self.posest = {'x': 0, 'y': 0, 'z': 0, 'elapsed': 0, 'timestamp': 0}
        self.motor = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'elapsed': 0, 'timestamp': 0}
        self.attitude = {'angx': 0, 'angy': 0, 'heading': 0, 'elapsed': 0, 'timestamp': 0}
        self.message = {'angx': 0, 'angy': 0, 'heading': 0, 'roll': 0, 'pitch': 0, 'yaw': 0,
                        'throttle': 0, 'elapsed': 0, 'timestamp': 0}

        self.ident = {"version": "", "multitype": "", "msp_version": "", "capability": ""}
        self.pids = {}
        self._desired_pids = {}
        self.status = {}
        self.analog = {}
        self.boxids = []
        self.box = []
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial(serPort,
                                 baudrate=115200,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0.1,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False,
                                 writeTimeout=0.1,
                                 )

    """Function for sending a command to the board."""

    def sendCMD(self, data_length, code, data):

        # NOTICE: the order of the yaw and thrust is switched to correspond to
        # cleanflight's standard AETR channel order
        if len(data) == 4:
            [r, p, y, t] = data
            data = [r, p, t, y]

        dl = len(data)
        if dl == 0:
            s1 = MultiWii.SEND_ZERO_STRUCT1
        elif dl == 8:
            s1 = MultiWii.SEND_EIGHT_STRUCT1
        else:
            s1 = struct.Struct('<2B%dh' % len(data))

        # code specific structs
        if code == MultiWii.SET_PID:
            s1 = MultiWii.SET_PID_STRUCT

        # print("dl", dl)
        # print("data_length", type(data_length), data_length)
        # print("code", type(code), code)
        # print("*data",  *data)
        dataString = s1.pack(data_length, code, *data)

        b = np.frombuffer(dataString, dtype=np.uint8)
        checksum = np.bitwise_xor.accumulate(b)[-1]
        footerString = MultiWii.footerS.pack(checksum)
        self.ser.write(
            MultiWii.emptyString.join((MultiWii.headerString, dataString, footerString, b"\n")))
        # return self.receiveDataPacket()

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition
      s_struct((uint8_t*)&att,6);
      break;

    """

    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500, 1500, 2000, 1000, 1500, 1500, 1500, 1500]
            self.sendCMD(16, MultiWii.SET_RAW_RC, data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500, 1500, 1000, 990, 1500, 1500, 1500, 1500]
            self.sendCMD(16, MultiWii.SET_RAW_RC, data)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start = time.time()

    """Function to receive a data packet from the board"""

    def getData(self, cmd):
        self.sendCMD(0, cmd, [])
        return self.receiveDataPacket()

    """ Sends a request for N commands from the board. """

    def getDataBulk(self, cmds):
        for c, args in cmds:
            self.sendCMD(0, c, args)
        result = []
        for c, args in cmds:
            result.append(self.receiveDataPacket())
        return result

    def _update_buffer_desired_pids(
        self,
        axis_name: Literal['ROLL', 'PITCH', 'YAW', 'ALT', 'Pos', 'PosR', 'NavR', 'LEVEL', 'MAG', 'VEL'],
        component_name: Literal['p', 'i', 'd'],
        coefficient_value: int,
    ):
        """ Before sending all desired PIDs to the FC, buffer locally in this python obj """
        if len(self.pids) == 0:
            self.getData(MultiWii.PID)

        if len(self._desired_pids) == 0 and len(self.pids) != 0:
            self._desired_pids = self.pids.copy()

        if len(self._desired_pids) == 0:
            print("Not updating desired PIDs because the original values have not been obtained")
            return

        self._desired_pids[axis_name][component_name] = coefficient_value

    def _update_pids(self):
        if len(self._desired_pids) == 0:
            print("Not sending desired PIDs to the FC. The desired PIDs are empty")
            return False

        try:
            desired = []
            for per_item_pid in self._desired_pids.values():
                for k in ['p', 'i', 'd']:
                    desired.append(per_item_pid[k])

            self.sendCMD(30, MultiWii.SET_PID, desired)
            self.receiveDataPacket()

            # update self.pids
            _ = self.getData(MultiWii.PID)

            for key_name, per_item_pid in self.pids.items():
                assert self._desired_pids[key_name]['p'] == per_item_pid['p']
                assert self._desired_pids[key_name]['i'] == per_item_pid['i']
                assert self._desired_pids[key_name]['d'] == per_item_pid['d']

            print("================================================================")
            print("PID update successful. After setting desired PIDs, the new PIDs are:")
            print(json.dumps(self.pids, indent=2))
            print("================================================================")

            return True
        except Exception as e:
            print(f"Failed to update PIDs to desired. Error: {e}")
            return False

    def get_pids_rpy(self):
        # update self pid
        _ = self.getData(MultiWii.PID)

        # const keys
        k_roll = 'ROLL'
        k_pitch = 'PITCH'
        k_yaw = 'YAW'

        # construct response
        ret = MultiWiiRpyPid(
            roll_p=self.pids[k_roll]['p'],
            roll_i=self.pids[k_roll]['i'],
            roll_d=self.pids[k_roll]['d'],
            pitch_p=self.pids[k_pitch]['p'],
            pitch_i=self.pids[k_pitch]['i'],
            pitch_d=self.pids[k_pitch]['d'],
            yaw_p=self.pids[k_yaw]['p'],
            yaw_i=self.pids[k_yaw]['i'],
            yaw_d=self.pids[k_yaw]['d'],
        )

        return ret

    def set_pids_rpy(
        self,
        roll_p: Optional[int] = None, roll_i: Optional[int] = None, roll_d: Optional[int] = None,
        pitch_p: Optional[int] = None, pitch_i: Optional[int] = None, pitch_d: Optional[int] = None,
        yaw_p: Optional[int] = None, yaw_i: Optional[int] = None, yaw_d: Optional[int] = None,
    ):
        if roll_p is not None:
            self._update_buffer_desired_pids('ROLL', 'p', roll_p)
        if roll_i is not None:
            self._update_buffer_desired_pids('ROLL', 'i', roll_i)
        if roll_d is not None:
            self._update_buffer_desired_pids('ROLL', 'd', roll_d)

        if pitch_p is not None:
            self._update_buffer_desired_pids('PITCH', 'p', pitch_p)
        if pitch_i is not None:
            self._update_buffer_desired_pids('PITCH', 'i', pitch_i)
        if pitch_d is not None:
            self._update_buffer_desired_pids('PITCH', 'd', pitch_d)

        if yaw_p is not None:
            self._update_buffer_desired_pids('YAW', 'p', yaw_p)
        if yaw_i is not None:
            self._update_buffer_desired_pids('YAW', 'i', yaw_i)
        if yaw_d is not None:
            self._update_buffer_desired_pids('YAW', 'd', yaw_d)

        return self._update_pids()

    def receiveDataPacket(self):
        start = time.time()

        header = self.ser.read(5)
        if len(header) == 0:
            print("timeout on receiveDataPacket")
            return None
        elif header[0] != ord(b'$'):
            print("Didn't get valid header: ", header, header[0])
            print(header, type(header), header[0], type(header[0]), len(header))
            print('----------------------------------------------------------')
            raise

        try:
            datalength = MultiWii.codeS.unpack(header[-2:-1])[0]
            code = MultiWii.codeS.unpack(header[-1:])[0]
        except Exception as e:
            print("Unpack issue", e)
            print(header, type(header), header[0], type(header[0]), len(header))
            print("***********************************************************")
            raise e

        data = self.ser.read(datalength)
        checksum = self.ser.read()
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
            temp = struct.unpack('<' + 'H' * (datalength / 2), data)
            self.box = temp
            return self.box
        elif code == MultiWii.ANALOG:
            temp = struct.unpack('<' + 'bHHH', data)
            self.analog['vbat'] = temp[0]
            self.analog['intPowerMeterSum'] = temp[1]
            self.analog['rssi'] = temp[2]
            self.analog['amperage'] = temp[3]
            self.analog['timestamp'] = readTime
            return self.analog
        elif code == MultiWii.BOXNAMES:
            print("datalength", datalength)
            assert datalength % 2 == 0
            temp = struct.unpack('<' + 's' * datalength, data)
            temp = "".join(temp)[:-1].split(";")
            self.boxnames = temp
            return self.boxnames
        elif code == MultiWii.STATUS:
            temp = struct.unpack('<HHHIB', data)
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
            temp = MultiWii.IDENT_STRUCT.unpack(data)
            self.ident["cmd"] = code
            self.ident["version"] = temp[0]
            self.ident["multitype"] = temp[1]
            self.ident["msp_version"] = temp[2]
            self.ident["capability"] = temp[3]
            self.ident['timestamp'] = readTime

            return self.ident
        elif code == MultiWii.RC:
            temp = MultiWii.RC_STRUCT.unpack(data)
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
            temp = MultiWii.POS_EST_STRUCT.unpack(data)
            self.posest["cmd"] = code
            self.posest['x'] = float(temp[0])
            self.posest['y'] = float(temp[1])
            self.posest['z'] = float(temp[2])
            self.posest['elapsed'] = elapsed
            self.posest['timestamp'] = time.time()
            return self.posest
        elif code == MultiWii.MOTOR:
            temp = MultiWii.MOTOR_STRUCT.unpack(data)
            self.motor['cmd'] = code
            self.motor['m1'] = float(temp[0])
            self.motor['m2'] = float(temp[1])
            self.motor['m3'] = float(temp[2])
            self.motor['m4'] = float(temp[3])
            self.motor['m5'] = float(temp[4])
            self.motor['m6'] = float(temp[5])
            self.motor['m7'] = float(temp[6])
            self.motor['m8'] = float(temp[7])
            self.motor['elapsed'] = elapsed
            self.motor['timestamp'] = readTime
            return self.motor
        elif code == MultiWii.SET_RAW_RC:
            return "Set Raw RC"
        elif code == MultiWii.PIDNAMES:
            # This only needs to be called once, which is embedded in the `code == MultiWii.PID` section
            if len(self.pids) != 0:
                print("PIDNAMES already retrieved, skipping such data.")
                return

            # `;` separated pid names
            names = data.decode().strip(';').split(';')
            for n in names:
                self.pids[n] = {'p': 0, 'i': 0, 'd': 0}
        elif code == MultiWii.SET_PID:
            return "Set PID"
        elif code == MultiWii.PID:
            if len(self.pids) == 0:
                self.getData(MultiWii.PIDNAMES)
            if len(self.pids) == 0:
                print("Failed to retrieve PIDNAMES. Skipping to retrieve PID values")
                return

            pid_val = MultiWii.PID_STRUCT.unpack(data)
            i = 0
            for per_item_pid in self.pids.values():
                per_item_pid['p'] = pid_val[0 + i * 3]
                per_item_pid['i'] = pid_val[1 + i * 3]
                per_item_pid['d'] = pid_val[2 + i * 3]
                i += 1

            return self.pids
        else:
            print("No return error!: %d" % code)
            raise

    """ Implement me to check the checksum. """

    def checkChecksum(self, data, checksum):
        pass

    def close(self):
        self.ser.close()

    """
    Calibrate the IMU and write outputs to a config file.
    """

    def calibrate(self, fname):
        self.sendCMD(0, MultiWii.ACC_CALIBRATION, [])
        print(self.receiveDataPacket())

        # ignore the first 200 because it takes a while to settle.
        for i in range(200):
            raw_imu = self.getData(MultiWii.RAW_IMU)
            time.sleep(0.01)
            print(raw_imu)

        raw_imu_totals = {}
        samples = 0.0
        for i in range(1000):
            raw_imu = self.getData(MultiWii.RAW_IMU)
            print(raw_imu)
            if raw_imu is not None:
                for key, value in raw_imu.items():
                    raw_imu_totals.setdefault(key, 0.0)
                    raw_imu_totals[key] += value
                samples += 1
                time.sleep(0.01)

        for key, value in raw_imu_totals.items():
            raw_imu_totals[key] = raw_imu_totals[key] / samples

        print(raw_imu_totals)

        f = open(fname, "w")
        yaml.dump(raw_imu_totals, f)
        f.close()

    def setBoxValues(self, values=(0, 7, 0, 0)):
        self.sendCMD(len(values) * 2, MultiWii.SET_BOX, values)

    def eepromWrite(self):
        self.sendCMD(0, MultiWii.EEPROM_WRITE, [])
