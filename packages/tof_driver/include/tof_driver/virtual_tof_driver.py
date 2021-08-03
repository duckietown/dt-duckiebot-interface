import os

from tof_driver.include.tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy

from dt_duckiematrix_protocols.world.TimeOfFlightRange import TimeOfFlightRange
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_robot_utils import get_robot_name


class VirtualToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, *_, **__):
        super(VirtualToFDriver, self).__init__(name, accuracy)
        # prepare zmq pipeline
        self._range: float = accuracy.max_range * 1000.0
        self._device: DuckieMatrixSocket = DuckieMatrixSocket.create()
        if self._device is None or not self._device.connected:
            print("[VirtualToF]: No virtual ToF connection established.")
        else:
            print("[VirtualToF]: Initialized.")
        # setup topic
        self._topic = os.path.join(get_robot_name(), f"tof_{self._name}")

    def start(self):
        self._device.subscribe(self._topic, TimeOfFlightRange, self._cb)
        self._device.start()

    def _cb(self, msg: TimeOfFlightRange):
        # convert to millimiters
        self._range = msg.range * 1000.0

    def get_distance(self) -> float:
        return self._range

    def shutdown(self):
        if self._device is not None:
            print('[VirtualToF]: Releasing...')
            self._device.shutdown()
            print('[VirtualToF]: Released.')
        self._device = None

