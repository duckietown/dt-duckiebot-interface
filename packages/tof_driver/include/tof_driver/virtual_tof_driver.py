from typing import Optional

from .tof_driver_abs import ToFDriverAbs, ToFAccuracy


class VirtualToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, *_, **__):
        super(VirtualToFDriver, self).__init__(name, accuracy)

    def setup(self):
        pass

    def start(self):
        pass

    def get_distance(self) -> Optional[float]:
        return None

    def stop(self):
        pass

    def release(self):
        pass
