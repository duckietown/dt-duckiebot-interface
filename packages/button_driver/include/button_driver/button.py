from enum import IntEnum

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand
ROBOT_HARDWARE = get_device_hardware_brand()

if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import Jetson.GPIO as GPIO

elif ROBOT_HARDWARE == DeviceHardwareBrand.RASPBERRY_PI:
    import RPi.GPIO as GPIO

else:
    raise Exception("Undefined Hardware!")

from .led import ButtonLED


class ButtonEvent(IntEnum):
    PRESS = 0
    RELEASE = 1


class ButtonDriver:
    """Class handling communication with a GPIO button.

        Args:
            led_gpio_pin (:obj:`int`): ID of the pin the button LED is connected to.
            signal_gpio_pin (:obj:`int`): ID of the pin the button (signal) is connected to.
            callback (:obj:`callable`): callback function to receive signal events.
    """

    def __init__(self, led_gpio_pin, signal_gpio_pin, callback):
        # valid gpio pin
        if not 1 <= signal_gpio_pin <= 40:
            raise ValueError('The pin number must be within the range [1, 40].')
        # validate callback
        if not callable(callback):
            raise ValueError('The callback object must be a callable object')
        self._callback = callback
        # configure GPIO pin
        self._signal_gpio_pin = signal_gpio_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._signal_gpio_pin, GPIO.IN)
        # create LED object
        self._led = ButtonLED(led_gpio_pin)
        # attach event listeners to the signal GPIO pin
        GPIO.add_event_detect(
            self._signal_gpio_pin,
            GPIO.BOTH,
            callback=self._cb
        )

    @property
    def led(self) -> 'ButtonLED':
        return self._led

    def _cb(self, pin):
        signal = int(GPIO.input(pin))
        event = ButtonEvent(signal)
        self._callback(event)

    def shutdown(self):
        # remove event listeners
        GPIO.remove_event_detect(self._signal_gpio_pin)
        # shutdown LED controller
        if hasattr(self, '_led'):
            self._led.shutdown()
