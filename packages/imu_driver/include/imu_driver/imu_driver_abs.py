from abc import ABC, abstractmethod
from typing import List, Optional


class IMUDriverAbs(ABC):

    @property
    @abstractmethod
    def linear_accelerations(self) -> Optional[List[float]]:
        pass

    @property
    @abstractmethod
    def angular_velocities(self) -> Optional[List[float]]:
        pass

    @property
    @abstractmethod
    def temperature(self) -> Optional[float]:
        pass
