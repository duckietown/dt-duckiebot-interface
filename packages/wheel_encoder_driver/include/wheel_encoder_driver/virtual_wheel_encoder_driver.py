from typing import Optional

from dt_duckiematrix_messages.WheelEncoderTicks import WheelEncoderTicks
from dt_duckiematrix_protocols import Matrix
from dt_duckiematrix_protocols.robot import WheeledRobot
from dt_duckiematrix_protocols.robot.features.sensors import WheelEncoder
from dt_duckiematrix_utils.ros import DuckiematrixLinkDescription, \
    on_duckiematrix_connection_request
from dt_robot_utils import get_robot_configuration
from wheel_encoder_driver.wheel_encoder_abs import WheelEncoderDriverAbs


class VirtualWheelEncoderDriver(WheelEncoderDriverAbs):
    """Class handling communication with a virtual wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            name (:obj:`str`): name of the encoder (e.g., left, right).
            _ (:obj:`int`): placeholder for the ticks GPIO pin number.
            __ (:obj:`int`): placeholder for the direction GPIO pin number.
            ___ (:obj:`bool`): placeholder for the direction inversion flag.
    """

    def __init__(self, name: str, _: int, __: int, ___: bool):
        super(VirtualWheelEncoderDriver, self).__init__(name)
        # prepare zmq pipeline
        self._reading: Optional[float] = None
        # register connection setup function
        self._matrix: Optional[Matrix] = None
        self._device: Optional[WheelEncoder] = None
        # register connection setup function
        print(f"[VirtualWheelEncoder]: Waiting for connection request...")
        self._connection_request: Optional[DuckiematrixLinkDescription] = None
        on_duckiematrix_connection_request(self.on_connection_request)

    def on_connection_request(self, link: DuckiematrixLinkDescription):
        print(f"[VirtualWheelEncoder]: Received request to connect to "
              f"Duckiematrix '{link.matrix}'.")
        # store new connection request
        self._connection_request = link
        # switch over to the new connection
        self.release()
        self.setup()

    def setup(self):
        if self._connection_request is None:
            return
        # ---
        link = self._connection_request
        configuration = get_robot_configuration()
        # prepare zmq pipeline
        self._matrix: Matrix = Matrix(link.uri)
        robot: WheeledRobot = self._matrix.robots.create(configuration.name, link.entity)
        self._device: WheelEncoder = robot.wheels.get(self._name).encoder
        # subscribe to encoder topic
        self._device.attach(self._process_reading)

    def start(self):
        if self._device is not None:
            self._device.start()

    def stop(self):
        self._device.stop()

    def _process_reading(self, msg: WheelEncoderTicks):
        self._ticks = msg.ticks
        self._callback(self._ticks)

    def release(self):
        if self._device is not None:
            print('[VirtualWheelEncoder]: Releasing...')
            self._device.detach(self._process_reading)
            self._device.release()
            print('[VirtualWheelEncoder]: Released.')
        self._device = None
