#!/usr/bin/env python3
from fc_driver import FlightController

def main():
    # run flight controller driver communication loop
    fc = FlightController()
    fc.run()


if __name__ == '__main__':
    main()
