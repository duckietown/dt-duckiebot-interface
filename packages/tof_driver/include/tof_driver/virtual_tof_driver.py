# !/usr/bin/env python3
from threading import Condition
from typing import Optional

from dt_duckiematrix_messages.TimeOfFlightRange import TimeOfFlightRange
from dt_duckiematrix_protocols import Matrix
from dt_duckiematrix_protocols.robot import RobotAbs
from dt_duckiematrix_protocols.robot.features.sensors import TimeOfFlight
from dt_duckiematrix_utils.ros import \
    on_duckiematrix_connection_request, \
    DuckiematrixLinkDescription
from dt_robot_utils import get_robot_configuration
from tof_driver.tof_driver_abs import ToFDriverAbs, ToFAccuracy


class VirtualToFDriver(ToFDriverAbs):

    def __init__(self, name: str, accuracy: ToFAccuracy, *_, **__):
        super(VirtualToFDriver, self).__init__(name, accuracy)
        # prepare zmq pipeline
        self._range: Optional[float] = None
        # register connection setup function
        self._matrix: Optional[Matrix] = None
        self._device: Optional[TimeOfFlight] = None
        # register connection setup function
        print(f"[VirtualToF:{self._name}]: Waiting for connection request...")
        self._connection_request: Optional[DuckiematrixLinkDescription] = None
        self._new_connection_request = Condition()
        on_duckiematrix_connection_request(self.on_connection_request)

    def on_connection_request(self, link: DuckiematrixLinkDescription):
        print(f"[VirtualToF:{self._name}]: Received request to connect to "
              f"Duckiematrix '{link.matrix}'.")
        # store new connection request
        self._connection_request = link
        # notify node of the new connection
        with self._new_connection_request:
            self._new_connection_request.notify()

    def setup(self):
        if self._connection_request is None:
            return
        # ---
        # release existing device (if any)
        if self._device:
            self.release()
        link = self._connection_request
        configuration = get_robot_configuration()
        # prepare zmq pipeline
        self._matrix: Matrix = Matrix(link.uri)
        robot = self._matrix.robots.create(configuration.name, link.entity)
        self._device: TimeOfFlight = robot.time_of_flight("front_center")
        # subscribe to camera topic
        self._device.attach(self._process_range)

    def start(self):
        if self._connection_request is None:
                # wait for connection request
                print("[VirtualToF]: Waiting for connection request...")
                with self._new_connection_request:
                    self._new_connection_request.wait()
                print("[VirtualToF]: Connection request received")

        if self._device is not None:
            self._device.start()
        else:
            self.setup()

    def _process_range(self, msg: TimeOfFlightRange):
        # convert to millimiters
        self._range = msg.range * 1000.0

    def get_distance(self) -> Optional[float]:
        if self._range is None:
            return -1
        return self._range

    def stop(self):
        self._device.stop()
        self._range = None

    def release(self):
        if self._device is not None:
            print(f'[VirtualToF:{self._name}]: Releasing...')
            # unsubscribe from topic
            self._device.detach(self._process_range)
            self._device.release()
            print(f'[VirtualToF:{self._name}]: Released.')
        self._device = None
        self._range = None
