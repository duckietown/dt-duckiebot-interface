from .regions import REGION_FULL, REGION_HEADER, REGION_BODY, REGION_FOOTER, DisplayRegion
from .page import ALL_PAGES
from .roi import DisplayROI
from .fragment import DisplayFragment
from .text import monospace_screen
from .renderer import AbsDisplayFragmentRenderer, \
    TextFragmentRenderer, \
    NumpyArrayFragmentRenderer, \
    MonoImageFragmentRenderer

Z_SYSTEM = 200

PAGE_HOME = 0
PAGE_TOF = 1
PAGE_SHUTDOWN = 2