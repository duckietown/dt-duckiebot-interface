from typing import List

from .leds_driver_abs import LEDsDriverAbs


class VirtualLEDsDriver(LEDsDriverAbs):

    def set_channel_intensity(self, led: int, channel: int, intensity: float):
        pass

    def set_rgb(self, led: int, color: List[float]):
        pass

    def release(self):
        pass
