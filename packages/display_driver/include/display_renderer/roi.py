import dataclasses


@dataclasses.dataclass
class DisplayROI:
    x: int
    y: int
    w: int
    h: int

    @staticmethod
    def from_sensor_msgs_ROI(roi):
        if roi.width + roi.height > 0:
            return DisplayROI(roi.x_offset, roi.y_offset, roi.width, roi.height)
        return None
