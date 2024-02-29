import dataclasses
from enum import IntEnum


class DisplayRegionID(IntEnum):
    FULL = 0
    HEADER = 1
    BODY = 2
    FOOTER = 3


@dataclasses.dataclass
class DisplayRegion:
    id: DisplayRegionID
    name: str
    width: int
    height: int
    x: int = 0
    y: int = 0


REGION_FULL = DisplayRegion(DisplayRegionID.FULL, "full", 128, 64)
REGION_HEADER = DisplayRegion(DisplayRegionID.HEADER, "header", 128, 16)
REGION_BODY = DisplayRegion(DisplayRegionID.BODY, "body", 128, 44, y=16)
REGION_FOOTER = DisplayRegion(DisplayRegionID.FOOTER, "footer", 128, 4, y=16 + 44)
