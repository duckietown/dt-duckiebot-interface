#!/usr/bin/env python3

from h2rMultiWii import MultiWii
import time

def main():
    board = MultiWii('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    print("Calibrate ACC... make sure we are level and still.")
    time.sleep(1)
    board.sendCMD(0, MultiWii.ACC_CALIBRATION, [])
    board.receiveDataPacket()
    time.sleep(2)
    print("Done!")
    
if __name__ == "__main__":
    main()
