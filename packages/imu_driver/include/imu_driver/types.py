import dataclasses


@dataclasses.dataclass
class I2CConnector:
    bus: int
    address: int
