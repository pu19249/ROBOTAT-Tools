# -*- coding: utf-8 -*-
from itertools import chain
from typing import Iterable
from typing import List
from typing import Union

from .utils import assure_str


class NestedCommentError(Exception):
    """Comments inside of comments are not allowed."""


class NoCommentStartError(Exception):
    """Comments must be started before they are ended."""


class CodeWriterLite:
    """Simple container for C code, with indentation and blocks.

    Warning:
        You probably don't want to use :class:`CodeWriterLite` for code
        generation, but its subclass, :class:`CodeWriter`.
        :class:`CodeWriterLite` is meant for csnake internal use and is
        normally not accessible at package level.

    CodeWriterLite is used internally in cconstructs and also as a parent class
    of CodeWriter.

    CodeWriterLite (abbreviated CWL or cwl) is a container class for C code
    generation, which implements indentation and block ({}) opening and
    closing.

    The class supports str, len, getitem, iter, and contains, making it a
    virtual subclass of Collection.


    Args:
        lf: linefeed character used to separate lines of code when
            rendering code with :func:`str` or :attr:`code`.
        indent: indentation unit string or :class:`int` number of spaces,
            used to indent code.

    Attributes:

        lf(str): linefeed character used to separate lines of code when
            rendering code with :func:`str` or :attr:`code`.

        lines(list(str)): lines of code.
    """

    __slots__ = ("lf", "_indent_unit", "_indent_level", "_in_comment", "lines")

    def __init__(self, lf: str = None, indent: Union[int, str] = 4) -> None:

        if lf is None:
            lf = "\n"
        self.lf: str = assure_str(lf)

        if isinstance(indent, int):
            self._indent_unit = " " * indent
        else:
            self._indent_unit = assure_str(indent)

        # initialize values
        self._indent_level = 0
        self._in_comment = False

        self.lines: List[str] = []

    @property
    def _current_indent(self) -> str:
        return self._indent_unit * self._indent_level

    @property
    def code(self) -> str:
        """str: The contents of the codewriter rendered to a str, using :attr:`lf`
        as the linebreak.
        """
        return self.lf.join(self.lines)

    def __str__(self) -> str:
        """Return the code joined by :attr:`lf`."""
        return self.code

    def __iter__(self) -> Iterable[str]:
        """Return an iterator that iterates over the lines in the codewriter."""
        return iter(self.lines)

    def __getitem__(self, key) -> str:
        """Return the nth line of code."""
        return self.lines.__getitem__(key)

    def __len__(self):
        """Return the number of lines in the codewriter."""
        return self.lines.__len__()

    def __contains__(self, item) -> bool:
        """Return whether a :class:`str` is a substring of the codewriter.

        Args:
            item(str): a potential substring of the codewriter.

        Returns:
            Whether or not `item` is a substring of the :func:`str`
            representation of the codewriter.
        """
        return self.code.__contains__(item)

    def indent(self) -> None:
        """Increase the indentation level by 1 unit.

        Examples:
            :meth:`indent` and :meth:`dedent` in action:

        >>> from csnake import CodeWriter
        >>> cwr = CodeWriter()
        >>> cwr.add_line('line_1;')
        >>> cwr.indent()
        >>> cwr.add_line('line_2;')
        >>> cwr.add_line('line_3;')
        >>> cwr.dedent()
        >>> cwr.add_line('line_4;')
        >>> print(cwr)
        line_1;
            line_2;
            line_3;
        line_4;
        """
        self._indent_level += 1

    def dedent(self) -> None:
        """Decrease the indentation level by 1 unit.

        Examples:
            See :meth:`indent`.
        """

        if self._indent_level > 0:
            self._indent_level -= 1

    def zero_indent(self) -> None:
        """Set indentation level to zero."""
        self._indent_level = 0

    def start_comment(self, ignore_indent: bool = False) -> None:
        """Start a comment block.

        All of the lines added after this are in a block comment, until the
        block is finished by :meth:`end_comment`.

        Args:
            ignore_indent: whether to ignore the indentation and just start the
                comment from the beginning of the next line.

        Raises:
            NestedCommentError: if you attempt to start a comment block when a
                comment block has already been started.

        Examples:
            Starting and ending a comment block with :meth:`start_comment` and
            :meth:`end_comment`:

            >>> from csnake import CodeWriter
            >>> cwr = CodeWriter()
            >>> cwr.add_line('line_1;')
            >>> cwr.start_comment()
            >>> cwr.add_line('line_2;')
            >>> cwr.add_line('line_3;')
            >>> cwr.end_comment()
            >>> cwr.add_line('line_4;')
            >>> print(cwr)
            line_1;
            /*
            * line_2;
            * line_3;
            */
            line_4;
        """
        if self._in_comment:
            raise NestedCommentError()
        self.add_line("/*", ignore_indent=ignore_indent)
        self._in_comment = True

    def end_comment(self, ignore_indent: bool = False) -> None:
        """End a comment block.

        A comment block previously started by :meth:`start_comment` is ended.
        The following code is not inside a block comment.

        Raises:
            NoCommentStartError: if you attempt to end a comment block when a
                comment block hasn't been started.

        Examples:
            See :meth:`start_comment`.
        """
        if not self._in_comment:
            raise NoCommentStartError()
        self._in_comment = False
        self.add_line("*/", ignore_indent=ignore_indent)

    def open_brace(self) -> None:
        """Add an open brace and increase indentation for subsequent lines.

        See :meth:`indent` for more information on how indentation is done.

        Examples:
            Using :meth:`open_brace` and :meth:`close_brace`.

            >>> from csnake import CodeWriter
            >>> cwr = CodeWriter()
            >>> cwr.add_line('line_1;')
            >>> cwr.open_brace()
            >>> cwr.add_line('line_2;')
            >>> cwr.add_line('line_3;')
            >>> cwr.close_brace()
            >>> cwr.add_line('line_4;')
            >>> print(cwr)
            line_1;
            {
                line_2;
                line_3;
            }
            line_4;
        """
        self.add_line("{")
        self.indent()

    def close_brace(self) -> None:
        """Add a closed brace and decrease indentation for subsequent lines.

        See :meth:`dedent` for more information on how dedentation is done.

        Examples:
            See :meth:`open_brace`.
        """
        self.dedent()
        self.add_line("}")

    def add(self, text: str) -> None:
        """Append text to the end of last line.

        Args:
            text: :obj:`str` to append to the last line.

        Raises:
            ValueError: if :obj:`text` is not a :class:`str`.
        """
        try:
            self.lines[-1]
        except IndexError:
            self.lines.append("")

        self.lines[-1] += text

    def add_line(
        self,
        text: str = None,
        comment: str = None,
        ignore_indent: bool = False,
    ) -> None:
        """Add a line of text to the container after the last line.

        Args:
            text: :obj:`str` to add
            comment: optional comment
            ignore_indent: switch to ignore indentation and insert the code
                with indentation 0

        Raises:
            ValueError: if text or comment isn't a :class:`str`
        """

        # empty line
        if text is None and not comment and not self._in_comment:
            self.lines.append("")
            return

        current_line = ""

        if not ignore_indent:
            current_line += self._current_indent

        if self._in_comment:
            current_line += "*"

        # add the text (if appropriate)
        if text:
            text = assure_str(text)
            if "\n" in text:
                raise ValueError("text mustn't contain multiple lines.")
            if self._in_comment:
                current_line += " "
            current_line += text

        if comment:
            comment = assure_str(comment)
            # add a space if there is text
            if text:
                current_line += " "
            if self._in_comment:
                current_line += f"{comment}"
            else:
                current_line += f"/* {comment} */"

        self.lines.append(current_line)

    def add_lines(
        self, lines: Union[Iterable[str], str], ignore_indent: bool = False
    ) -> None:
        """Append code lines from an iterable or a multi-line str.

        If `lines` is an iterable iterating over :class:`str` objects, adds the
        lines to the codewriter in order.

        If `lines` is a multi-line string, adds its lines to the codewriter.

        Args:
            lines: an :class:`Iterable` of :class:`str` or a multi-line `str`
                to add to codewriter.
            ignore_indent: whether to ignore the current indentation level or
                not. If `ignore_indent` is true, the lines will be appended
                without indentation.

        Raises:
            ValueError: if one of the arguments is of the wrong type

        Examples:
            Adding a list of lines:

            >>> from csnake import CodeWriter
            >>> cwr = CodeWriter()
            >>> lol = ['list', 'of', 'lines']
            >>> cwr.add_lines(lol)
            >>> print(cwr)
            list
            of
            lines
        """

        if isinstance(lines, str):
            lines = lines.splitlines()
        else:
            lines = chain.from_iterable(
                supposed_line.splitlines() for supposed_line in lines
            )

        for line in lines:
            self.add_line(line, ignore_indent=ignore_indent)


assert issubclass(CodeWriterLite, Iterable)
