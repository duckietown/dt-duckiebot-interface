
led_emitter package
*******************


Contents
^^^^^^^^

* `led_emitter package`_

    * `LEDEmitterNode`_

    * `Included libraries`_

        * `rgb_led`_

            * `rgb_led`_

The *led_emitter* package provides the drivers that control the LEDs
of a Duckiebot. It provides a convenient API for turning the LEDs on
and off, changing their color and making them blink with various
frequencies. You can see examples how to do that in the description of
the *LEDEmitterNode* class bellow.


LEDEmitterNode
==============


Included libraries
==================


rgb_led
-------


rgb_led
~~~~~~~

Describe *rgb_leb* here…

**class RGB_LED(debug=False)**

    Bases: `object`_

    Object communicating to the LEDs.

    Low level class that creates the PWM messages that are sent to the
    microcontroller. It contains offset addresses relatives to the
    address of the various LEDs.

    Each LED on a Duckiebot or a watchtower is indexed by a number:

    +--------------------+--------------------------------------------+
    | Index              | Position (rel. to direction of movement)   |
    +====================+============================================+
    | 0                  | Front left                                 |
    +--------------------+--------------------------------------------+
    | 1                  | Rear left                                  |
    +--------------------+--------------------------------------------+
    | 2                  | Top / Front middle                         |
    +--------------------+--------------------------------------------+
    | 3                  | Rear right                                 |
    +--------------------+--------------------------------------------+
    | 4                  | Front right                                |
    +--------------------+--------------------------------------------+

    Setting the color of a single LED is done by setting the
    brightness of the red, green, and blue channels to a value between
    0 and 255. The communication with the hardware controller is
    abstracted through the `setRGB`_ method. By using it, you can set
    directly set the desired color to any LED.

    ``OFFSET_BLUE = 2``

        Offset address for the blue color

    ``OFFSET_GREEN = 1``

        Offset address for the green color

    ``OFFSET_RED = 0``

        Offset address for the red color

    **setLEDBrightness(led, offset, brightness)**

        Sets value for brightness for one color on one LED.

        Calls the function pwm.setPWM to set the PWM signal according
        to the input brightness.

        Typically shouldn’t be used directly. Use `setRGB`_ instead.

        :Parameters:
            * **led** (`int`_) – Index of specific LED (from the
                table above)

            * **offset** (`int`_) – Offset for color

            * **brightness** (``int8``) – Intensity of brightness
                (between 0 and 255)

    **setRGB(led, color)**

        Sets value for brightness for all channels of one LED

        Converts the input color brightness from [0,1] to [0,255] for
        all channels, then calls self.setLEDBrightness with the right
        offset corresponding to the color channel in the PWM signal
        and the color value as int8.

        :Parameters:
            * **led** (`int`_) – Index of specific LED (from the
                table above)

            * **color** (``list`` of `float`_) – Brightness for the
                three RGB channels, in interval [0,1]
