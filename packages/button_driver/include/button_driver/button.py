from enum import IntEnum
from typing import Callable

from dt_device_utils import get_device_hardware_brand, DeviceHardwareBrand

ROBOT_HARDWARE = get_device_hardware_brand()

if ROBOT_HARDWARE == DeviceHardwareBrand.JETSON_NANO:
    import Jetson.GPIO as GPIO

elif ROBOT_HARDWARE in [DeviceHardwareBrand.RASPBERRY_PI, DeviceHardwareBrand.RASPBERRY_PI_64]:
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
            raise ValueError("The pin number must be within the range [1, 40].")
        # validate callback
        if not callable(callback):
            raise ValueError("The callback object must be a callable object")
        self._callback = callback
        # configure GPIO pin
        self._signal_gpio_pin = signal_gpio_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._signal_gpio_pin, GPIO.IN)
        # create LED object
        self._led = ButtonLED(led_gpio_pin)
        # prevent_default when performing tests
        self._prevent_default = False
        # callback when test finishes, only supplied when starting a test
        self._test_callback = None
        # attach event listeners to the signal GPIO pin
        GPIO.add_event_detect(self._signal_gpio_pin, GPIO.BOTH, callback=self._cb)


    @property
    def led(self) -> "ButtonLED":
        return self._led

    def _cb(self, pin):
        signal = int(GPIO.input(pin))
        event = ButtonEvent(signal)
        # when running tests
        if self._prevent_default:
            if event == ButtonEvent.RELEASE:
                if isinstance(self._test_callback, Callable):
                    self._test_callback()
                self.finish_test()
            return

        self._callback(event)

    def start_test(self, test_cb):
        self._test_callback = test_cb
        self._prevent_default = True
    
    def finish_test(self):
        self._prevent_default = False
        self._test_callback = None

    def shutdown(self):
        # remove event listeners
        GPIO.remove_event_detect(self._signal_gpio_pin)
        # shutdown LED controller
        if hasattr(self, "_led"):
            self._led.shutdown()
