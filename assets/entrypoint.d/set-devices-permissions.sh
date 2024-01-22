#!/bin/bash

# Find the first file with the specified prefix
first_file=$(find /dev/i2c*  -print -quit)

if [ -n "$first_file" ]; then
    echo "First i2c device found: $first_file"
    # Determine the GID of the 'i2c' group dynamically
    I2C_GID=$(stat -c %g "$first_file")
    # Add the user to the 'i2c' group with the correct GID
    usermod -aG $I2C_GID "$DT_USER_NAME"
else
    echo "No i2c devices found."
fi

# Fix permissions for serial devices
usermod -aG dialout "$DT_USER_NAME"
