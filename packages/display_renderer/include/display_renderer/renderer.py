#!/usr/bin/env python3

import asyncio
from abc import abstractmethod, ABC
from typing import Union, Iterable, Optional, Callable, Awaitable, List

import numpy as np

from display_driver.types.regions import DisplayRegion
from display_driver.types.roi import DisplayROI
from duckietown_messages.actuators.display_fragment import DisplayFragment
from duckietown_messages.sensors.image import Image
from duckietown_messages.standard.header import Header
from .text import monospace_screen


class AbsDisplayFragmentRenderer(ABC):

    def __init__(
        self,
        name: str,
        page: int,
        region: DisplayRegion,
        roi: DisplayROI,
        ttl: int = 10,
        z: int = 0,
        callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable] = None,
        frequency: Optional[float] = None
    ):
        self._name: str = name
        self._page: int = page
        self._region: DisplayRegion = region
        self._roi: DisplayROI = roi
        self._callback: Callable[['AbsDisplayFragmentRenderer'], Awaitable] = callback
        self._ttl: int = ttl
        self._z: int = z
        self._buffer: np.ndarray = np.zeros((self._roi.h, self._roi.w), dtype=np.uint8)
        self._frequency: Optional[float] = frequency
        # validate frequency
        if (self._frequency is not None) and (self._frequency <= 0.001):
            raise ValueError(f"Frequency must be greater than 0.001, got {self._frequency}")

    @property
    def name(self) -> str:
        return self._name

    @property
    def page(self) -> int:
        return self._page

    @property
    def region(self) -> DisplayRegion:
        return self._region

    @property
    def roi(self) -> DisplayROI:
        return self._roi

    @property
    def shape(self) -> tuple:
        return self._roi.h, self._roi.w

    @property
    def buffer(self) -> np.ndarray:
        self.render()
        return self._buffer

    async def worker(self):
        if self._frequency is None:
            # one shot
            await self.render()
            # trigger callback (if given)
            if self._callback is not None:
                await self._callback(self)
            # ---
            return
        # recurrent
        dt: float = 1.0 / self._frequency
        while True:
            self._clear_buffer()
            await self.step()
            # trigger callback (if given)
            if self._callback is not None:
                await self._callback(self)
            # ---
            await asyncio.sleep(dt)

    async def step(self):
        self.render()

    @abstractmethod
    def render(self):
        pass

    def _clear_buffer(self):
        self._buffer.fill(0)

    @property
    def fragments(self) -> List[DisplayFragment]:
        return [
            DisplayFragment(
                header=Header(),
                name=self._name,
                region=self._region.id,
                page=self._page,
                content=Image.from_mono1(self._buffer, header=Header()),
                location=self._roi.to_message(Header()),
                ttl=self._ttl,
                z=self._z,
            )
        ]


class TextFragmentRenderer(AbsDisplayFragmentRenderer):

    def __init__(
        self,
        name: str,
        page: int,
        region: DisplayRegion,
        roi: DisplayROI,
        scale: Union[str, float] = 1.0,
        **kwargs
    ):
        super(TextFragmentRenderer, self).__init__(name, page, region, roi, **kwargs)
        self._text = ""
        self._scale = scale

    def update(self, text: Union[Iterable[str], str]):
        self._text = text

    def render(self):
        self._buffer = monospace_screen(self.shape, self._text, scale=self._scale)


class NumpyArrayFragmentRenderer(AbsDisplayFragmentRenderer):

    def update(self, buffer: np.ndarray):
        np.copyto(self._buffer, buffer)

    def render(self):
        pass


MonoImageFragmentRenderer = NumpyArrayFragmentRenderer
