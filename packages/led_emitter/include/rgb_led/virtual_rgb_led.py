import os
from typing import List

from dt_duckiematrix_protocols.LEDsCommand import LEDsCommand, LEDCommand
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_robot_utils import get_robot_name
from led_emitter.include.rgb_led.RGBLEDAbs import RGBLEDAbs


class VirtualRGBLED(RGBLEDAbs):

    DEFAULT_LIGHT_INTENSITY = 1.0

    def __init__(self):
        self._leds = [[0, 0, 0] for _ in range(5)]
        # prepare zmq pipeline
        self._device: DuckieMatrixSocket = DuckieMatrixSocket.create()
        if self._device is None or not self._device.connected:
            print("[VirtualLED]: No virtual LED connection established.")
        else:
            print("[VirtualLED]: Initialized.")
        # setup topic
        self._topic = os.path.join(get_robot_name(), "lights")

    def setRGB(self, led, color):
        self._leds[led] = color
        if self._device.connected:
            message = LEDsCommand({
                str(i): LEDCommand(
                    rgb_to_hex(l), self.DEFAULT_LIGHT_INTENSITY
                ) for i, l in enumerate(self._leds)
            })
            self._device.publish(self._topic, message)

    def __del__(self):
        if self._device is not None:
            print('[VirtualLED]: Releasing...')
            self._device.release()
            print('[VirtualLED]: Released.')
        self._device = None


def rgb_to_hex(rgb: List[float]) -> str:
    rgb = [max(0, min(int(x * 255), 255)) for x in rgb]
    return "#{0:02x}{1:02x}{2:02x}".format(*rgb)
