
adafruit_drivers package
************************


Contents
^^^^^^^^

* `adafruit_drivers package`_

    * `Included libraries`_

        * `Adafruit_GPIO`_

        * `Adafruit_I2C`_

        * `Adafruit_MotorHAT`_

        * `Adafruit_PWM_Servo_Driver`_

The *adafruit_driver* package contains the drivers that run the
Duckiebot hardware.

Note:
    These drivers are developed by Adafruit Industries and their
    documentation is what they have provided with the driver code
    base.


Included libraries
==================


Adafruit_GPIO
-------------

This is the `Adafruit GPIO library
<https://github.com/adafruit/Adafruit_Python_GPIO>`_. We are not
maintaining nor documenting it. These docs are automatically built
from the docstrings provided by them.

**class BaseGPIO**

    Bases: `object`_

    Base class for implementing simple digital IO for a platform.
    Implementors are expected to subclass from this and provide an
    implementation of the setup, output, and input functions.

    **add_event_callback(pin, callback)**

        Add a callback for an event already defined using
        add_event_detect(). Pin should be type IN.

    **add_event_detect(pin, edge)**

        Enable edge detection events for a particular GPIO channel.
        Pin should be type IN.  Edge must be RISING, FALLING or BOTH.

    **cleanup(pin=None)**

        Clean up GPIO event detection for specific pin, or all pins if
        none is specified.

    **event_detected(pin)**

        Returns True if an edge has occured on a given GPIO.  You need
        to enable edge detection using add_event_detect() first.   Pin
        should be type IN.

    **input(pin)**

        Read the specified pin and return HIGH/true if the pin is
        pulled high, or LOW/false if pulled low.

    **input_pins(pins)**

        Read multiple pins specified in the given list and return list
        of pin values GPIO.HIGH/True if the pin is pulled high, or
        GPIO.LOW/False if pulled low.

    **is_high(pin)**

        Return true if the specified pin is pulled high.

    **is_low(pin)**

        Return true if the specified pin is pulled low.

    **output(pin, value)**

        Set the specified pin the provided high/low value.  Value
        should be either HIGH/LOW or a boolean (true = high).

    **output_pins(pins)**

        Set multiple pins high or low at once.  Pins should be a dict
        of pin name to pin value (HIGH/True for 1, LOW/False for 0).
        All provided pins will be set to the given values.

    **remove_event_detect(pin)**

        Remove edge detection for a particular GPIO channel.  Pin
        should be type IN.

    **set_high(pin)**

        Set the specified pin HIGH.

    **set_low(pin)**

        Set the specified pin LOW.

    **setup(pin, mode, pull_up_down=0)**

        Set the input or output mode for a specified pin.  Mode should
        be either OUT or IN.

    **setup_pins(pins)**

        Setup multiple pins as inputs or outputs at once.  Pins should
        be a dict of pin name to pin type (IN or OUT).

    **wait_for_edge(pin, edge)**

        Wait for an edge.   Pin should be type IN.  Edge must be
        RISING, FALLING or BOTH.

**class RPiGPIOAdapter(rpi_gpio, mode=None)**

    Bases: ``Adafruit_GPIO.GPIO.BaseGPIO``

    GPIO implementation for the Raspberry Pi using the RPi.GPIO
    library.

    **add_event_callback(pin, callback)**

        Add a callback for an event already defined using
        add_event_detect(). Pin should be type IN.

    **add_event_detect(pin, edge, callback=None, bouncetime=-1)**

        Enable edge detection events for a particular GPIO channel.
        Pin should be type IN.  Edge must be RISING, FALLING or BOTH.
        Callback is a function for the event.  Bouncetime is switch
        bounce timeout in ms for callback

    **cleanup(pin=None)**

        Clean up GPIO event detection for specific pin, or all pins if
        none is specified.

    **event_detected(pin)**

        Returns True if an edge has occured on a given GPIO.  You need
        to enable edge detection using add_event_detect() first.   Pin
        should be type IN.

    **input(pin)**

        Read the specified pin and return HIGH/true if the pin is
        pulled high, or LOW/false if pulled low.

    **input_pins(pins)**

        Read multiple pins specified in the given list and return list
        of pin values GPIO.HIGH/True if the pin is pulled high, or
        GPIO.LOW/False if pulled low.

    **is_high(pin)**

        Return true if the specified pin is pulled high.

    **is_low(pin)**

        Return true if the specified pin is pulled low.

    **output(pin, value)**

        Set the specified pin the provided high/low value.  Value
        should be either HIGH/LOW or a boolean (true = high).

    **output_pins(pins)**

        Set multiple pins high or low at once.  Pins should be a dict
        of pin name to pin value (HIGH/True for 1, LOW/False for 0).
        All provided pins will be set to the given values.

    **remove_event_detect(pin)**

        Remove edge detection for a particular GPIO channel.  Pin
        should be type IN.

    **set_high(pin)**

        Set the specified pin HIGH.

    **set_low(pin)**

        Set the specified pin LOW.

    **setup(pin, mode, pull_up_down=0)**

        Set the input or output mode for a specified pin.  Mode should
        be either OUTPUT or INPUT.

    **setup_pins(pins)**

        Setup multiple pins as inputs or outputs at once.  Pins should
        be a dict of pin name to pin type (IN or OUT).

    **wait_for_edge(pin, edge)**

        Wait for an edge.   Pin should be type IN.  Edge must be
        RISING, FALLING or BOTH.

**class AdafruitBBIOAdapter(bbio_gpio)**

    Bases: ``Adafruit_GPIO.GPIO.BaseGPIO``

    GPIO implementation for the Beaglebone Black using the
    Adafruit_BBIO library.

    **add_event_callback(pin, callback, bouncetime=-1)**

        Add a callback for an event already defined using
        add_event_detect(). Pin should be type IN.  Bouncetime is
        switch bounce timeout in ms for callback

    **add_event_detect(pin, edge, callback=None, bouncetime=-1)**

        Enable edge detection events for a particular GPIO channel.
        Pin should be type IN.  Edge must be RISING, FALLING or BOTH.
        Callback is a function for the event.  Bouncetime is switch
        bounce timeout in ms for callback

    **cleanup(pin=None)**

        Clean up GPIO event detection for specific pin, or all pins if
        none is specified.

    **event_detected(pin)**

        Returns True if an edge has occured on a given GPIO.  You need
        to enable edge detection using add_event_detect() first.   Pin
        should be type IN.

    **input(pin)**

        Read the specified pin and return HIGH/true if the pin is
        pulled high, or LOW/false if pulled low.

    **input_pins(pins)**

        Read multiple pins specified in the given list and return list
        of pin values GPIO.HIGH/True if the pin is pulled high, or
        GPIO.LOW/False if pulled low.

    **is_high(pin)**

        Return true if the specified pin is pulled high.

    **is_low(pin)**

        Return true if the specified pin is pulled low.

    **output(pin, value)**

        Set the specified pin the provided high/low value.  Value
        should be either HIGH/LOW or a boolean (true = high).

    **output_pins(pins)**

        Set multiple pins high or low at once.  Pins should be a dict
        of pin name to pin value (HIGH/True for 1, LOW/False for 0).
        All provided pins will be set to the given values.

    **remove_event_detect(pin)**

        Remove edge detection for a particular GPIO channel.  Pin
        should be type IN.

    **set_high(pin)**

        Set the specified pin HIGH.

    **set_low(pin)**

        Set the specified pin LOW.

    **setup(pin, mode, pull_up_down=0)**

        Set the input or output mode for a specified pin.  Mode should
        be either OUTPUT or INPUT.

    **setup_pins(pins)**

        Setup multiple pins as inputs or outputs at once.  Pins should
        be a dict of pin name to pin type (IN or OUT).

    **wait_for_edge(pin, edge)**

        Wait for an edge.   Pin should be type IN.  Edge must be
        RISING, FALLING or BOTH.

**class AdafruitMinnowAdapter(mraa_gpio)**

    Bases: ``Adafruit_GPIO.GPIO.BaseGPIO``

    GPIO implementation for the Minnowboard + MAX using the mraa
    library

    **add_event_callback(pin, callback)**

        Add a callback for an event already defined using
        add_event_detect(). Pin should be type IN.

    **add_event_detect(pin, edge, callback=None, bouncetime=-1)**

        Enable edge detection events for a particular GPIO channel.
        Pin should be type IN.  Edge must be RISING, FALLING or BOTH.
        Callback is a function for the event.  Bouncetime is switch
        bounce timeout in ms for callback

    **cleanup(pin=None)**

        Clean up GPIO event detection for specific pin, or all pins if
        none is specified.

    **event_detected(pin)**

        Returns True if an edge has occured on a given GPIO.  You need
        to enable edge detection using add_event_detect() first.   Pin
        should be type IN.

    **input(pin)**

        Read the specified pin and return HIGH/true if the pin is
        pulled high, or LOW/false if pulled low.

    **input_pins(pins)**

        Read multiple pins specified in the given list and return list
        of pin values GPIO.HIGH/True if the pin is pulled high, or
        GPIO.LOW/False if pulled low.

    **is_high(pin)**

        Return true if the specified pin is pulled high.

    **is_low(pin)**

        Return true if the specified pin is pulled low.

    **output(pin, value)**

        Set the specified pin the provided high/low value.  Value
        should be either 1 (ON or HIGH), or 0 (OFF or LOW) or a
        boolean.

    **output_pins(pins)**

        Set multiple pins high or low at once.  Pins should be a dict
        of pin name to pin value (HIGH/True for 1, LOW/False for 0).
        All provided pins will be set to the given values.

    **remove_event_detect(pin)**

        Remove edge detection for a particular GPIO channel.  Pin
        should be type IN.

    **set_high(pin)**

        Set the specified pin HIGH.

    **set_low(pin)**

        Set the specified pin LOW.

    **setup(pin, mode)**

        Set the input or output mode for a specified pin.  Mode should
        be either DIR_IN or DIR_OUT.

    **setup_pins(pins)**

        Setup multiple pins as inputs or outputs at once.  Pins should
        be a dict of pin name to pin type (IN or OUT).

    **wait_for_edge(pin, edge)**

        Wait for an edge.   Pin should be type IN.  Edge must be
        RISING, FALLING or BOTH.


Adafruit_I2C
------------

This is the Adafruit I2C library. We are not maintaining nor
documenting it.


Adafruit_MotorHAT
-----------------

This is the `Adafruit MotorHAT
<https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library>`_
library. We are not maintaining nor documenting it.


Adafruit_PWM_Servo_Driver
-------------------------

This is the Adafruit PWM library. We are not maintaining nor
documenting it.
