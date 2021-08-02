import os
from typing import Callable

from dt_duckiematrix_protocols.world.WheelEncoder import WheelEncoder
from dt_duckiematrix_utils.socket import DuckieMatrixSocket
from dt_robot_utils import get_robot_name
from wheel_encoder.wheel_encoder_abs import WheelEncoderDriverAbs


class VirtualWheelEncoderDriver(WheelEncoderDriverAbs):
    """Class handling communication with a virtual wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            name (:obj:`str`): name of the encoder (e.g., left, right).
            _ (:obj:`int`): placeholder for the GPIO pin number.
            callback (:obj:`callable`): callback function to receive new (unique) readings.
    """

    def __init__(self, name: str, _: int, callback: Callable):
        super(VirtualWheelEncoderDriver, self).__init__(name, callback)
        # prepare zmq pipeline
        self._device: DuckieMatrixSocket = DuckieMatrixSocket.create()
        if self._device is None or not self._device.connected:
            print("[VirtualWheelEncoder]: No virtual encoder connection established.")
        else:
            print("[VirtualWheelEncoder]: Initialized.")
        # setup topic
        self._topic = os.path.join(get_robot_name(), f"wheel_encoder_{name}")
        self._device.subscribe(self._topic, WheelEncoder, self._cb)
        self._device.start()

    def _cb(self, msg: WheelEncoder):
        self._ticks = msg.ticks
        self._callback(self._ticks)

    def shutdown(self):
        if self._device is not None:
            print('[VirtualWheelEncoder]: Releasing...')
            self._device.shutdown()
            print('[VirtualWheelEncoder]: Released.')
        self._device = None
