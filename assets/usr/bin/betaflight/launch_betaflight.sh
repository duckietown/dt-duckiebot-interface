#!/bin/bash

cd /usr/bin/betaflight

# Launch betaflight_SITL.elf
./betaflight_SITL.elf &

# Wait for betaflight_SITL.elf to start
sleep 2

# Shut down betaflight_SITL.elf
pkill betaflight_SITL.elf

# Copy eeprom.bin from config file to current folder
cp ./config/eeprom.bin ./eeprom.bin

# Launch betaflight_SITL.elf again
./betaflight_SITL.elf &
