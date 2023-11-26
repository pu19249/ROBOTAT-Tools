# -*- coding: utf-8 -*-
"""
Small utilities that simplify common tasks.
"""
from typing import Collection
from typing import Generator
from typing import Iterable
from typing import Optional
from typing import Sequence


def assure_str(supposed_str) -> str:
    if not isinstance(supposed_str, str):
        raise TypeError(f"supposed_str ({supposed_str}) must be a str")
    return supposed_str


def assure_str_or_none(supposed_str) -> Optional[str]:
    if not isinstance(supposed_str, str) and supposed_str is not None:
        raise TypeError(f"supposed_str ({supposed_str}) must be a str")
    return supposed_str


def seq_get(seq: Sequence, defaults: Sequence, index: int):
    """Get an element from a sequence by index. If the index is out of bounds,
    get it from the default sequence.

    Examples:

        >>> seq_get([1, 2], [5, 6, 7, 8], 2)
        7
        >>> seq_get([1, 2], [5, 6, 7, 8], -1)
        8
        >>> seq_get([1, 2], [5, 6], -1)
        2
    """
    if index < 0:
        length = len(defaults)
        index = length + index
        if index < 0:
            raise IndexError(
                "sequence index out of range in both seq and defaults"
            )
    try:
        return seq[index]
    except IndexError:
        try:
            return defaults[index]
        except IndexError:
            raise IndexError(
                "sequence index out of range in both seq and defaults"
            )


def rgb888_to_rgb565(rgb_tuple: Collection[int]) -> int:
    """Return a rgb595 int of an RGB888 color"""
    red, green, blue = rgb_tuple
    return (
        (int(red / 255 * 31) << 11)
        | (int(green / 255 * 63) << 5)
        | (int(blue / 255 * 31))
    )


def bytes_to_int(input_bytes: Iterable) -> int:
    """Concatenate a tuple of bytes or ints representing them into a single int.

    For example (hex for easier understanding):

        >>> hex(bytes_to_int((0xab, 0xcd, 0xef)))
        >>> '0xabcdef'
    """
    result = 0
    for byte in input_bytes:
        result <<= 8
        result |= byte

    return result


def sublists_from_iterable(
    iterable: Iterable, length: int, yield_last_incomplete: bool = True
) -> Generator:
    """Yields lists of a certain length from an iterable.

    This generator yields lists of length `length` from an interable
    `iterable`.

    Args:
        iterable: from which this generator yields lists of elements
        length: length of lists yielded
        yield_last_incomplete: if :pycode`True`, yield the last sublist even if
            it's shorter than `length`; if :pycode:`False`, raise a ValueError
            if the last sublist is shorter than `length`

    Yields:
        `list`\\ s of elements yielded from `iterable`

    Examples:
        >>> from csnake.utils import sublists_from_iterable
        >>> incomplete_ok = sublists_from_iterable(range(5), 4, True)
        >>> print(list(incomplete_ok))
        [[0, 1, 2, 3], [4]]
        >>> incomplete_notok = sublists_from_iterable(range(5), 4, False)
        >>> print(list(incomplete_notok))
        ValueError: Input iterable's length is not divisible by sublist length
    """
    currentlist = []
    index = 0

    for item in iterable:
        currentlist.append(item)
        index += 1
        if index == length:
            yield currentlist
            currentlist = []
            index = 0
    if currentlist:
        if yield_last_incomplete:
            yield currentlist
        else:
            raise ValueError(
                "Input iterable's length is not divisible by sublist length"
            )
