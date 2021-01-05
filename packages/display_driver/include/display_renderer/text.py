import numpy as np

from typing import Union, Iterable
from PIL import ImageFont, ImageDraw, Image

FONT_FILEPATH = '/usr/share/fonts/ubuntu-mono-regular.ttf'
FONT_SIZE = 8
# NOTE: the following are dependent on the value of FONT_SIZE
CHAR_WIDTH_PX = 4
CHAR_HEIGHT_PX = 7
LINE_WIDTH_CHARS = 32
NUM_LINES_HEADER = 2
NUM_LINES_BODY = 5
LINE_HEIGHT_PX = 7
LINE_SPACING = 2

SAMPLE_CANVAS = """\
|------------------------------|
|                              |
|                              |
|                              |
--------------------------------
"""


def monospace_screen(shape: tuple, text: Union[Iterable[str], str], scale: Union[str, float] = 1.0) -> np.ndarray:
    if isinstance(text, str):
        text = text.split('\n')
    if isinstance(scale, str):
        if scale not in ['vfill', 'hfill']:
            raise ValueError("The argument `scale` must be one of [`vfill`, `hfill`]")
    # ---
    if isinstance(scale, str):
        if scale == 'vfill':
            expected_h = len(text) * LINE_HEIGHT_PX + (len(text) - 1) * LINE_SPACING
            scale = shape[0] / expected_h
        if scale == 'hfill':
            longest_line_len = max([len(line) for line in text])
            expected_w = longest_line_len * CHAR_WIDTH_PX
            scale = shape[1] / expected_w
    # create empty numpy image
    image = np.zeros(shape)
    # pass the image to PIL
    pil_im = Image.fromarray(image)
    draw = ImageDraw.Draw(pil_im)
    # use a truetype font
    font = ImageFont.truetype(FONT_FILEPATH, int(FONT_SIZE * scale))
    # draw the text
    for i, line in enumerate(text):
        draw.text((0, -2 + int(i * scale * (LINE_HEIGHT_PX + LINE_SPACING))), line, font=font)
    # get back the image as numpy array
    return np.array(pil_im).astype(np.uint8) * 255
