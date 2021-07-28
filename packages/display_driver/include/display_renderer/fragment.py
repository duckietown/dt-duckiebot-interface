import time
import dataclasses
import numpy as np

from .roi import DisplayROI


@dataclasses.dataclass
class DisplayFragment:
    data: np.ndarray
    roi: DisplayROI
    page: int
    z: int
    _time: float
    _ttl: int

    @property
    def given_ttl(self):
        return self._ttl

    def ttl(self):
        if self._ttl < 0:
            # infinite ttl
            return 1
        elapsed = time.time() - self._time
        return self._ttl - elapsed
