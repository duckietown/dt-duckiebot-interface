import dataclasses

from duckietown_messages.standard.header import Header
from duckietown_messages.geometry_2d.roi import ROI


@dataclasses.dataclass
class DisplayROI:
    x: int
    y: int
    w: int
    h: int

    @staticmethod
    def from_message(msg: ROI):
        return DisplayROI(x=msg.x, y=msg.y, w=msg.width, h=msg.height)

    def to_message(self, header: Header) -> ROI:
        return ROI(header=header, x=self.x, y=self.y, width=self.w, height=self.h)
