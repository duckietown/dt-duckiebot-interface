from wheel_encoder_driver.wheel_encoder_abs import WheelEncoderDriverAbs


class VirtualWheelEncoderDriver(WheelEncoderDriverAbs):
    """Class handling communication with a virtual wheel encoder.

    An instance of this class reads data off of a wheel encoder calls a callback function
    with the new cumulative tick number as sole argument.
    The callback is called only when the encoder fires, thus there is no constant frequency.

        Args:
            name (:obj:`str`): name of the encoder (e.g., left, right).
            resolution (:obj:`int`): number of ticks per revolution.
            _ (:obj:`int`): placeholder for the ticks GPIO pin number.
            __ (:obj:`int`): placeholder for the direction GPIO pin number.
            ___ (:obj:`bool`): placeholder for the direction inversion flag.
    """

    def __init__(self, name: str, resolution: int, _: int, __: int, ___: bool):
        super(VirtualWheelEncoderDriver, self).__init__(name, resolution)
