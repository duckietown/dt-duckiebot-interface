import numpy as np

from typing import Union, Iterable, List, Tuple
from PIL import ImageFont, ImageDraw, Image

_FONT_FILEPATH = '/usr/share/fonts/ubuntu-mono-regular.ttf'
_FONT = {
    _s: ImageFont.truetype(_FONT_FILEPATH, _s) for _s in range(4, 40)
}
_CHAR_SIZE_PER_FONT_SIZE = {
    _s: _f.getsize("X")[::-1]
    for _s, _f in _FONT.items()
}
_DEBUG = False

DEFAULT_FONT_SIZE = 9
DEFAULT_LINE_SPACING = 2
LINE_SPACING_TEXT_HEIGHT_RATIO = 0.1
SAMPLE_CANVAS = """\
|-----------------------|
|                       |
|                       |
|-----------------------|
"""


def monospace_screen(shape: tuple, text: Union[Iterable[str], str], scale: Union[str, float] = 1.0,
                     align: str = 'left') -> np.ndarray:
    if isinstance(text, str):
        text = text.split('\n')
    if isinstance(scale, str):
        if scale not in ['fill', 'vfill', 'hfill']:
            raise ValueError("The argument `scale` must be one of [`fill`, `vfill`, `hfill`]")
    # ---
    font_size = DEFAULT_FONT_SIZE
    line_spacing = DEFAULT_LINE_SPACING
    if isinstance(scale, str):
        if scale == 'fill':
            vfont_size, vline_spacing = _compute_sizes('vfill', text, shape)
            hfont_size, hline_spacing = _compute_sizes('hfill', text, shape)
            fonts = [vfont_size, hfont_size]
            spacings = [vline_spacing, hline_spacing]
            i = int(np.argmin([vfont_size, hfont_size]))
            font_size = fonts[i]
            line_spacing = spacings[i]
        else:
            font_size, line_spacing = _compute_sizes(scale, text, shape)
    char_height, char_width = _CHAR_SIZE_PER_FONT_SIZE[font_size]
    # create empty numpy image
    image = np.zeros(shape)
    # pass the image to PIL
    pil_im = Image.fromarray(image)
    draw = ImageDraw.Draw(pil_im)
    # use a truetype font
    font = ImageFont.truetype(_FONT_FILEPATH, font_size)
    # draw the text
    for i, line in enumerate(text):
        row = -2 + int(i * (char_height + line_spacing))
        draw.text((0, row), line, font=font, align=align)
    # get back the image as numpy array
    return np.array(pil_im).astype(np.uint8) * 255


def _compute_sizes(fit: str, text: List[str], canvas_size: tuple) -> Tuple[int, int]:
    axis = {
        'vfill': 0,
        'hfill': 1
    }[fit]
    font_size = _best_fit_font_size(text, axis, canvas_size)
    line_spacing = int(_CHAR_SIZE_PER_FONT_SIZE[font_size][0] * LINE_SPACING_TEXT_HEIGHT_RATIO)
    return font_size, line_spacing


def _best_fit_font_size(text: List[str], axis: int, canvas_size: tuple):
    line_height = 1 + LINE_SPACING_TEXT_HEIGHT_RATIO
    text_size = (len(text) * line_height, max([len(line) for line in text]))
    best_fit = DEFAULT_FONT_SIZE
    for font_size, char_size in _CHAR_SIZE_PER_FONT_SIZE.items():
        free_px = canvas_size[axis] - int(np.ceil(char_size[axis] * text_size[axis]))
        if _DEBUG:
            print(f'Considering font_size of `{font_size}`, with a char_size of {char_size} '
                  f'for a text of size (in chars) {text_size} for a canvas of {canvas_size} px')
        if free_px >= 0:
            best_fit = font_size
            if _DEBUG:
                print(f'  > font_size of `{font_size}` is good enough for axis `{axis}`, leaving '
                      f'{free_px} px')
        else:
            if _DEBUG:
                print(f'< font_size of `{font_size}` is bad for axis `{axis}`, exceeding canvas '
                      f'size by {-free_px}px')
            break
    return best_fit
