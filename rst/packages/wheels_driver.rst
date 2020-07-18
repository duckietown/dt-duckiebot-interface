
wheels_driver package
*********************


Contents
^^^^^^^^

* `wheels_driver package`_

    * `WheelsDriverNode`_

    * `Included libraries`_

        * `wheels_driver`_

The *wheels_driver* package handles the actual execution of motor
commands by the motors via a motor driver. This is done by the
``WheelsDriverNode``.


WheelsDriverNode
================


Included libraries
==================


wheels_driver
-------------

The wheels driver consists of a single class that handles the
communication with the wheel drivers.

**class DaguWheelsDriver(debug=False)**

    Class handling communication with motors.

    Wraps the Adafruit API to talk to DC motors with a simpler
    interface. The class contains methods for creating PWM signals
    according to requested velocities. Also contains hardware
    addresses related to the motors.

    :Parameters:
        **debug** (`bool`_) – If *True*, will print a debug message
        every time a PWM signal is sent.

    ``LEFT_MOTOR_MAX_PWM = 255``

        Maximum speed for left motor

    ``LEFT_MOTOR_MIN_PWM = 60``

        Minimum speed for left motor

    **PWMvalue(v, minPWM, maxPWM)**

        Transforms the requested speed into an int8 number.

        :Parameters:
            * **v** (`float`_) – requested speed, should be between
                -1 and 1.

            * **minPWM** (``int8``) – minimum speed as int8

            * **maxPWM** (``int8``) – maximum speed as int8

    ``RIGHT_MOTOR_MAX_PWM = 255``

        Maximum speed for right motor

    ``RIGHT_MOTOR_MIN_PWM = 60``

        Minimum speed for right motor

    ``SPEED_TOLERANCE = 0.01``

        Speed tolerance level

    **setWheelsSpeed(left, right)**

        Sets speed of motors.

        :Parameters:
            * **left** (`float`_) – speed for the left wheel, should
                be between -1 and 1

            * **right** (`float`_) – speed for the right wheel,
                should be between -1 and 1

    **updatePWM()**

        Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to
        the linear velocities of the motors. The requested speed gets
        tresholded.
