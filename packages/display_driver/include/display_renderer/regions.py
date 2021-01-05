import dataclasses

from duckietown_msgs.msg import DisplayFragment as DisplayFragmentMsg


@dataclasses.dataclass
class DisplayRegion:
    id: int
    name: str
    width: int
    height: int
    x: int = 0
    y: int = 0


REGION_FULL = DisplayRegion(DisplayFragmentMsg.REGION_FULL, 'full', 128, 64)
REGION_HEADER = DisplayRegion(DisplayFragmentMsg.REGION_HEADER, 'header', 128, 16)
REGION_BODY = DisplayRegion(DisplayFragmentMsg.REGION_BODY, 'body', 128, 44, y=16)
REGION_FOOTER = DisplayRegion(DisplayFragmentMsg.REGION_FOOTER, 'footer', 128, 4, y=16 + 44)
