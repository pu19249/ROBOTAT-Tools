# -*- coding: utf-8 -*-
"""
Module for conversion of PIL images to lists for variable initialization.
"""
from typing import Callable
from typing import List
from typing import Tuple

from mypy_extensions import TypedDict

from .utils import bytes_to_int
from .utils import rgb888_to_rgb565
from .utils import sublists_from_iterable

__all__ = ("pil_image_to_list",)


# conditional imports
try:
    from PIL.Image import Image
except ImportError:
    raise ImportError(
        "Package PIL or Pillow must be available to use this functionality."
    )

ConversionItem = TypedDict(
    "ConversionItem",
    {
        "formats": Tuple[str, str],
        "conversion": Callable,
        "bytes_per_pixel": int,
    },
)
Conversion = List[ConversionItem]

CONVERSIONS: Conversion = [
    {
        "formats": ("RGB", "RGB888"),
        "conversion": bytes_to_int,
        "bytes_per_pixel": 3,
    },
    {
        "formats": ("RGB", "RGB565"),
        "conversion": rgb888_to_rgb565,
        "bytes_per_pixel": 3,
    },
    {
        "formats": ("RGBA", "ARGB8888"),
        "conversion": bytes_to_int,
        "bytes_per_pixel": 4,
    },
    {"formats": ("L", "A8"), "conversion": bytes_to_int, "bytes_per_pixel": 1},
]


def pil_image_to_list(
    image: Image, outformat: str = "ARGB8888"
) -> List[List[int]]:
    """Convert a PIL image into an a list of `int`\\ s.

    Conversion is done from the :meth:`Image.tobytes` to `list` of `int`, based
    on the format given by the argument `format`.

    The main purpose of this is to turn :class:`Image`\\ s into nested
    `list`\\ s, which can be used as array initialization values by :class:
    csnake.variable.

    image's format is extracted from the image itself and there is only a
    limited number of conversions available.
    While the input format is fixed, the output format is supplied by
    `outformat`.

    Here are the supported mappings of formats (PIL :attr:`Image.mode` → output
    format):

    * RGB → RGB888
    * RGB → RGB565
    * RGBA → ARGB888
    * L → A8

    Args:
        image: `PIL` or `Pillow` :class:`Image` to be turned into array.
        outformat: Format of `int`'s in the output array.

    Returns:
        :obj:`list` of :obj:`list` of :obj:`int` for pixels of the image

    Raises:
        TypeError: If the `image` arg passed isn't a `PIL` `Image`.
        ValueError: If the conversion between the listed formats isn't
            possible.

    Examples:
        >>> from csnake.pil_converter import pil_image_to_list
        >>> from csnake import CodeWriter, Variable, FormattedLiteral
        >>> from PIL import Image, ImageDraw
        >>> im = Image.new('RGB', (3, 3))
        >>> dr = ImageDraw.Draw(im)
        >>> dr.line((0,0) + im.size, fill = '#8CAFAF')
        >>> im_ls = pil_image_to_list(im, "RGB888")
        >>> print(im_ls)
        [[9220015, 0, 0], [0, 9220015, 0], [0, 0, 9220015]]
        >>> var = Variable(
        ...     "pic",
        ...     primitive="int",
        ...     value=FormattedLiteral(im_ls, int_formatter=hex),
        ... )
        >>> cw = CodeWriter()
        >>> cw.add_variable_initialization(var)
        >>> print(cw)
        int pic[3][3] = {
            {0x8cafaf, 0x0, 0x0},
            {0x0, 0x8cafaf, 0x0},
            {0x0, 0x0, 0x8cafaf}
        };
    """

    if not isinstance(image, Image):
        raise TypeError("image must be an instance of PIL.Image")

    input_format = image.mode
    output_format = outformat

    iomode = input_format, output_format

    try:
        conversion = next(i for i in CONVERSIONS if i["formats"] == iomode)
    except StopIteration:
        raise ValueError(
            f"Cannot export to pixel format {output_format} from an image "
            "with pixel format {input_format}. See documenation code for "
            "valid combinations."
        )

    conversion_function = conversion["conversion"]

    bytes_per_pixel = conversion["bytes_per_pixel"]

    rows = image.height
    row_width = image.width

    image_bytes = image.tobytes()

    input_bytes = len(image_bytes)

    if not input_bytes % bytes_per_pixel == 0:
        raise ValueError(
            f"Number of picture's bytes ({input_bytes}) is not "
            "divisible by the pixel byte length for its supposed format "
            "({self.bytes_per_pixel})."
        )

    image_bytes_to_int = (int(b) for b in image_bytes)
    int_list_pixels = sublists_from_iterable(
        image_bytes_to_int, bytes_per_pixel
    )

    intpixel_iter = iter(int_list_pixels)

    output_list = []
    for _ in range(rows):
        curr_row = []
        for _ in range(row_width):
            current_pixel_ints = next(intpixel_iter)
            converted_pixel = conversion_function(current_pixel_ints)
            curr_row.append(converted_pixel)
        output_list.append(curr_row)

    return output_list
