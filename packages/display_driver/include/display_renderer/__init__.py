from .regions import REGION_FULL, REGION_HEADER, REGION_BODY, REGION_FOOTER, DisplayRegion
from .page import ALL_PAGES
from .roi import DisplayROI
from .fragment import DisplayFragment
from .text import monospace_screen
from .renderer import (
    AbsDisplayFragmentRenderer,
    TextFragmentRenderer,
    NumpyArrayFragmentRenderer,
    MonoImageFragmentRenderer,
)

Z_SYSTEM = 200

# default pages shown
PAGE_HOME = 0
PAGE_ROBOT_INFO = 1
PAGE_TOF = 2

# pages created due to special event, and the page order doesn't matter much
PAGE_SHUTDOWN = 90