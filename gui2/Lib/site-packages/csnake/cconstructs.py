# -*- coding: utf-8 -*-
"""
Classes that define C constructs.
"""
# TODO: make a static default for LiteralFormatters for using as default, instead
# of using another instance every time.
# TODO: add typedefs, especially for function pointers
import collections
from abc import ABCMeta
from abc import abstractmethod
from collections import deque
from collections import namedtuple
from itertools import chain
from typing import Callable
from typing import Collection
from typing import Iterable
from typing import Iterator
from typing import List
from typing import Mapping
from typing import Tuple
from typing import Union

from .codewriterlite import CodeWriterLite
from .utils import assure_str
from .utils import seq_get


# internal types used by csnake as initialization values
class CIntLiteral(metaclass=ABCMeta):
    """ABC for all C integer literals."""


CIntLiteral.register(int)


class CFloatLiteral(metaclass=ABCMeta):
    """ABC for all C floating point literals."""


CFloatLiteral.register(float)


class CArrayLiteral(metaclass=ABCMeta):
    """ABC for array literals: sized Iterables except str, Mapping."""

    @classmethod
    def __subclasshook__(cls, subclass):
        if cls is CArrayLiteral:
            if (
                issubclass(subclass, collections.abc.Sized)
                and issubclass(subclass, collections.abc.Iterable)
                and not issubclass(subclass, (str, collections.abc.Mapping))
            ):
                return True
            return False
        return NotImplemented


CArrayLiteral.register(list)
CArrayLiteral.register(tuple)


class CStructLiteral(metaclass=ABCMeta):
    """ABC for struct literals: any Mapping (dict-like)."""

    @classmethod
    def __subclasshook__(cls, subclass):
        if cls is CStructLiteral:
            return issubclass(subclass, collections.abc.Mapping)
        return NotImplemented


CStructLiteral.register(dict)


class CBasicLiteral(metaclass=ABCMeta):
    """ABC for literals of basic C types (int, float)."""


CBasicLiteral.register(CIntLiteral)
CBasicLiteral.register(CFloatLiteral)


class CExtendedLiteral(metaclass=ABCMeta):
    """ABC for basic C literals (int, float) + bool and char[]."""


CExtendedLiteral.register(CBasicLiteral)
CExtendedLiteral.register(bool)
CExtendedLiteral.register(str)


class CLiteral(metaclass=ABCMeta):
    """ABC for any C literal."""

    @classmethod
    def __subclasshook__(cls, subclass):
        if cls is CLiteral:
            return issubclass(
                subclass, (CExtendedLiteral, CStructLiteral, CArrayLiteral)
            )
        return NotImplemented


CLiteral.register(CExtendedLiteral)
CLiteral.register(CStructLiteral)

# conditional imports
try:
    import numpy as np
except ImportError:
    pass
else:
    CIntLiteral.register(np.integer)
    CFloatLiteral.register(np.floating)

try:
    import sympy as sp
except ImportError:
    pass
else:
    CIntLiteral.register(sp.Integer)
    CFloatLiteral.register(sp.Float)


class ShapelessError(TypeError):
    """Argument doesn't have a shape."""


def _shape(array: Iterable) -> tuple:
    """Return dimensions (shape) of a multidimensional list."""

    try:
        return array.shape  # type: ignore
    except AttributeError:
        pass

    if isinstance(array, str):
        raise ShapelessError("Strings don't have a shape.")

    if isinstance(array, Mapping):
        raise ShapelessError("Mappings (like dicts) don't have a shape.")

    curr = array
    shp: List[int] = []

    while True:
        if not isinstance(curr, CArrayLiteral):
            return tuple(shp)
        try:
            shp.append(len(curr))
            itr = iter(curr)
            curr = next(itr)
        except (TypeError, IndexError):
            return tuple(shp)


# C constructs:


def simple_bool_formatter(value: bool) -> str:
    """Simple formatter that turns a bool into 'true' or 'false'.

    Args:
        value: boolean to format

    Returns:
        :obj:`str`'s 'true' or 'false'
    """
    if value:
        return "true"
    else:
        return "false"


def simple_str_formatter(value: str) -> str:
    """Simple formatter that surrounds a str with quotes.

    Args:
        value: string to format

    Returns:
        :obj:`str` that's the input string surrounded with double quotes
    """
    return f'"{value}"'


class LiteralFormatters:
    """Collection of formatters used for formatting literals.

    Any values left out from constructor call is set to defaults.

    Args:
        int_formatter: function to call when formatting :ccode:`int` literals
        float_formatter: function to call when formatting :ccode:`float` literals
        bool_formatter: function to call when formatting :ccode:`bool` literals
        string_formatter: function to call when formatting :ccode:`char[]` literals

    Attributes:
        int_formatter: function to call when formatting :ccode:`int` literals
        float_formatter: function to call when formatting :ccode:`float` literals
        bool_formatter: function to call when formatting :ccode:`bool` literals
        string_formatter: function to call when formatting :ccode:`char[]` literals
    """

    __slots__ = (
        "int_formatter",
        "float_formatter",
        "string_formatter",
        "bool_formatter",
    )

    def __init__(
        self,
        int_formatter: Callable = int,
        float_formatter: Callable = float,
        bool_formatter: Callable = simple_bool_formatter,
        string_formatter: Callable = simple_str_formatter,
    ) -> None:
        self.int_formatter = int_formatter
        self.float_formatter = float_formatter
        self.bool_formatter = bool_formatter
        self.string_formatter = string_formatter

    @classmethod
    def empty(cls):
        """Returns an empty :class: instance (`None` for every parameter)."""
        return cls(**{key: None for key in cls.__slots__})

    def __repr__(self):
        """Simple printout of parameter values."""
        lines = (f"  {slot}: {getattr(self, slot)}" for slot in self.__slots__)
        return "formatters {\n" + "\n".join(lines) + "\n}"

    def replace(self, *args, **kwargs):
        """Return a copy of collection with certain formatters replaced."""

        if len(args) == 1 and not kwargs:
            arg = next(iter(args))
            if isinstance(arg, LiteralFormatters):

                initializer = {
                    slot: getattr(arg, slot)
                    if getattr(arg, slot) is not None
                    else getattr(self, slot)
                    for slot in self.__slots__
                }

                return LiteralFormatters(**initializer)

            if isinstance(arg, Mapping):
                return self.replace(**arg)

        if kwargs and not args:

            diff = set(kwargs.keys()) - set(self.__slots__)
            if diff:
                raise TypeError(
                    f"Arguments {str(diff)} are invalid for this function"
                )

            initializer = {
                slot: kwargs[slot] if slot in kwargs else getattr(self, slot)
                for slot in self.__slots__
            }

            return LiteralFormatters(**initializer)

        raise TypeError("Invalid  arguments for this function")

    @classmethod
    def partial(cls, dictionary: Mapping):
        """Create a partial formatter collection based on a dict."""
        rtnval = cls.empty()
        return rtnval.replace(dictionary)


class FormattedLiteral:
    """A C literal with its formatter collection altered.

    The accompanying literal formatter collection will be used when generating
    the initialization :obj:`str`.

    Args:
        \\*\\*kwargs: keyword arguments for each of the formatters in
            ['int_formatter', 'float_formatter', 'bool_formatter',
            'string_formatter'] we want to change for this literal. Every
            missing formatter is inherited from the literal (or default).

    Attributes:
        value: value of literal
        formatter: collection of literal formatters
    """

    __slots__ = ("value", "formatter")

    def __init__(self, value: CLiteral, *args, **kwargs) -> None:
        self.value = value
        if args:
            raise ValueError(f"Unexpected arguments: {args}")

        self.formatter = LiteralFormatters.partial(kwargs)


class VariableValue(metaclass=ABCMeta):
    """Abstract base class for any initializer value based on a variable.

    Sometimes we want to initialize a value to another variable, but in some
    more complicated manner: using the address-of operator, dereference
    operator, subscripting, typecasting... This is an ABC for those
    modifiers.
    """

    @abstractmethod
    def init_target_code(self, formatters: LiteralFormatters = None):
        """Return code used for variable initialization, formatted with the
        supplied formatters.

        Args:
            formatters: collection of formatters used for formatting the
                initialization :obj:`str`
        """
        pass

    @abstractmethod
    def __str__(self):
        """Return :obj:`str` containing initialization code."""


def generate_c_value_initializer(
    cval: Union[CExtendedLiteral, VariableValue],
    formatters: LiteralFormatters = None,
) -> str:
    """Generate an initialization str from a literal using formatters."""
    if formatters is None:
        formatters = LiteralFormatters()

    if isinstance(cval, VariableValue):
        return str(cval.init_target_code(formatters))
    if isinstance(cval, bool):
        return str(formatters.bool_formatter(cval))
    if isinstance(cval, CIntLiteral):
        return str(formatters.int_formatter(cval))
    if isinstance(cval, CFloatLiteral):
        return str(formatters.float_formatter(cval))
    if isinstance(cval, str):
        return str(formatters.string_formatter(cval))

    raise TypeError("cval must be a CExtendedLiteral.")


class _ArrStructInitGen:
    """Classes and functions that help with initialization generation."""

    __slots__ = "type_action_pairs"

    def __registration(self):
        "Fake function to register slots with mypy. Not meant to be called."
        self.type_action_pairs = None
        raise NotImplementedError

    class OpenBrace:
        """Helper class to identify open braces while printing."""

        __slots__ = ()

    class ClosedBrace:
        """Helper class to identify closed braces while printing.

            struct_closing means the brace closes a struct initialization
            """

        __slots__ = "struct_closing"

        def __init__(self, struct_closing: bool = False) -> None:
            self.struct_closing = bool(struct_closing)

    class Designator:
        """Helper class to identify struct designators."""

        __slots__ = ("name",)

        def __init__(self, name: str) -> None:
            self.name = name

    @staticmethod
    def lookthrough_ignoring_format(
        forvar: Union[FormattedLiteral, CLiteral]
    ) -> Union[CLiteral, FormattedLiteral]:
        if isinstance(forvar, FormattedLiteral):
            return _ArrStructInitGen.lookthrough_ignoring_format(forvar.value)
        return forvar

    @staticmethod
    def lookahead_ignoring_format(
        stack: deque,
    ) -> Union[
        Designator,  # noqa
        OpenBrace,  # noqa
        ClosedBrace,  # noqa
        CArrayLiteral,
        CStructLiteral,
        CExtendedLiteral,
        VariableValue,
    ]:
        index = -1
        while True:
            try:
                lookahead = stack[index]
            except IndexError:
                raise IndexError("no next element of interest on stack.")

            if isinstance(lookahead, FormattedLiteral):
                res = _ArrStructInitGen.lookthrough_ignoring_format(lookahead)
                assert isinstance(res, CLiteral)
                return res  # type: ignore

            if isinstance(lookahead, LiteralFormatters):
                index -= 1
                continue

            return lookahead

    @staticmethod
    def c_array_literal_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        stack.append(_ArrStructInitGen.ClosedBrace())
        try:
            stack.extend(reversed(top))  # type: ignore
        except TypeError:
            tempdeq: deque = deque()
            assert isinstance(top, Iterable)
            tempdeq.extendleft(top)
            stack.extend(tempdeq)
        stack.append(_ArrStructInitGen.OpenBrace())
        return formatters

    @staticmethod
    def c_struct_literal_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        stack.append(_ArrStructInitGen.ClosedBrace(struct_closing=True))
        assert isinstance(top, Mapping)
        dict_pairs = (
            (value, _ArrStructInitGen.Designator(key))
            for key, value in reversed(list(top.items()))
        )
        flatdict = (item for sublist in dict_pairs for item in sublist)
        stack.extend(flatdict)
        stack.append(_ArrStructInitGen.OpenBrace())
        return formatters

    @staticmethod
    def closed_brace_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        writer.dedent()
        if top.struct_closing:
            writer.add_line("}")
        else:
            writer.add("}")

        try:
            lookahead = _ArrStructInitGen.lookahead_ignoring_format(stack)
        except IndexError:
            pass
        else:
            if isinstance(lookahead, _ArrStructInitGen.ClosedBrace):
                writer.dedent()
                writer.add_line("")
                writer.indent()
            elif isinstance(
                lookahead, (CArrayLiteral, _ArrStructInitGen.OpenBrace)
            ):
                writer.add(",")
                writer.add_line("")
            else:
                writer.add(",")
        return formatters

    @staticmethod
    def open_brace_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:

        writer.add("{")
        writer.indent()

        try:
            lookahead = _ArrStructInitGen.lookahead_ignoring_format(stack)
        except IndexError:
            pass
        else:
            if isinstance(
                lookahead,
                (_ArrStructInitGen.OpenBrace, CArrayLiteral, CStructLiteral),
            ):
                writer.add_line("")
        return formatters

    @staticmethod
    def literal_or_value_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        writer.add(generate_c_value_initializer(top, formatters))

        try:
            lookahead = _ArrStructInitGen.lookahead_ignoring_format(stack)
        except IndexError:
            pass
        else:
            if isinstance(
                lookahead,
                (
                    CExtendedLiteral,
                    VariableValue,
                    _ArrStructInitGen.Designator,
                ),
            ):
                writer.add(",")
                if isinstance(lookahead, (CExtendedLiteral, VariableValue)):
                    writer.add(" ")
        return formatters

    @staticmethod
    def designator_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        writer.add_line("." + top.name + " = ")
        return formatters

    @staticmethod
    def formatted_literal_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        new_formatters = formatters.replace(top.formatter)

        stack.append(formatters)
        stack.append(top.value)
        stack.append(new_formatters)
        return formatters

    @staticmethod
    def literal_formatters_handler(
        top,
        stack: deque,
        writer: CodeWriterLite,
        formatters: LiteralFormatters,
    ) -> LiteralFormatters:
        formatters = top
        return formatters

    TypeActionPair = namedtuple("TypeActionPair", ("types", "action"))


_ArrStructInitGen.type_action_pairs = (
    _ArrStructInitGen.TypeActionPair(
        types=(CArrayLiteral), action=_ArrStructInitGen.c_array_literal_handler
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(CStructLiteral),
        action=_ArrStructInitGen.c_struct_literal_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(_ArrStructInitGen.ClosedBrace),
        action=_ArrStructInitGen.closed_brace_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(_ArrStructInitGen.OpenBrace),
        action=_ArrStructInitGen.open_brace_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(CExtendedLiteral, VariableValue),
        action=_ArrStructInitGen.literal_or_value_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(_ArrStructInitGen.Designator),
        action=_ArrStructInitGen.designator_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(FormattedLiteral),
        action=_ArrStructInitGen.formatted_literal_handler,
    ),
    _ArrStructInitGen.TypeActionPair(
        types=(LiteralFormatters),
        action=_ArrStructInitGen.literal_formatters_handler,
    ),
)


class NoValueError(ValueError):
    """Variable has no value and hence cannot be initialized."""


class FuncPtr:
    """Class describing function pointer args and return type.

    This class made for declaring function pointer type specifics for use with
    :class:`Variable`\\ s as their `type` argument.

    Args:
        return_type: :obj:`str` containing the return type
        arguments: an iterable which yields one the following types:

            * a :class:`Variable`
            * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2 strings
                (`name`, `primitive`)
            * a :class:`Mapping` (:obj:`dict`-like) with keys (`name`,
                `primitive`)
    """

    __slots__ = ("return_type", "arguments")

    # TODO unify the creation to be the same as Function

    def __init__(
        self, return_type: str, arguments: Iterable = None, comment: str = None
    ) -> None:

        self.return_type = assure_str(return_type)
        self.arguments: List[Variable] = []

        if arguments is not None:
            for arg in arguments:
                self.arguments.append(_get_variable(arg))

    def get_declaration(
        self, name: str, qualifiers: str = None, array: str = None
    ) -> str:
        """Generate the whole declaration :obj:`str` according to parameters.

        This method is meant to be called from
        :meth:`Variable.generate_declaration` and
        :meth:`Variable.generate_initialization` function and is probably
        useless to You elsewhere.
        """
        jointargs = ", ".join(
            arg.generate_declaration() for arg in self.arguments
        )

        retval = "{rt} (*{qual}{name}{arr})({arguments})".format(
            rt=self.return_type,
            qual=qualifiers if qualifiers else "",
            name=name,
            arguments=jointargs if self.arguments else "",
            arr=array if array else "",
        )

        return retval


class Variable:
    """Class describing C variable contruct.

    You can generate declarations and initializations for variables, and
    variables can be initialized to very complex values (you can init an array
    variable to an array of :ccode:`struct` containing arrays that contain
    :ccode:`structs`, ...).

    The formatting of initialization :obj:`str`\\ s can be altered by encasing
    some level of the initialization with a :class:`FormattedLiteral`.

    As for function values, the following (Python input → C code) translations
    are assumed (`numpy` and `scipy` `int`\\ s and `float`\\ s are considered ints
    and floats):

    - :obj:`dict` → :ccode:`struct` literal
    - :obj:`list` or `tuple` → array literal
    - :obj:`int` → :ccode:`int` literal or :ccode:`int` literal
    - :obj:`float` → :ccode:`float` literal or :ccode:`double` literal
    - :obj:`bool` → :ccode:`bool` literal or :ccode:`int` literal
    - :class:`Variable` → name of the variable

    There is no type checking for initializations. That's what your compiler is
    for, no sense in replicating such functionality here.

    Args:
        name: name of the variable

        primitive: :obj:`str` name or :class:`FuncPtr` defining the variable's
            type

        qualifiers: :obj:`str` or :class:`Sequence` of :obj:`str` listing the
            variable's qualifiers (:ccode:`const`, :ccode:`volatile`, ...)
        array: :class:`Sequence` of :obj:`int` defining the dimensions of a
            (possibly multidimensional) array. If it's left out, it's inferred
            from `value`.

        comment: accompanying comment
        value: variable's value, used for initialization. Explained in detail
            above

    """

    __slots__ = (
        "name",
        "primitive",
        "qualifiers",
        "array",
        "comment",
        "value",
    )

    def __init__(
        self,
        name: str,
        primitive: Union[str, FuncPtr],
        qualifiers: Union[Iterable[str], str] = None,
        array: Iterable = None,
        comment: str = None,
        value: CLiteral = None,
    ) -> None:

        self.name = assure_str(name)
        if isinstance(primitive, FuncPtr):
            self.primitive: Union[FuncPtr, str] = primitive
        else:
            self.primitive = assure_str(primitive)
        self.comment = assure_str(comment) if comment is not None else None
        self.array = array
        self.qualifiers = qualifiers
        self.value = value

    @staticmethod
    def merge_formatters(
        value: CLiteral, formatters: LiteralFormatters = None
    ) -> Tuple[CLiteral, LiteralFormatters]:
        if not formatters:
            formatters = LiteralFormatters()

        while isinstance(value, FormattedLiteral):
            formatters = formatters.replace(value.formatter)
            value = value.value

        return (value, formatters)

    def __array_dimensions(self) -> str:
        value = self.value
        if value is not None:
            value, _ = self.merge_formatters(value)

        if isinstance(self.array, CArrayLiteral):
            array = "".join(f"[{dim}]" for dim in self.array)
        elif self.array is not None:
            array = f"[{self.array}]"
        elif self.array is None and isinstance(value, str):
            array = "[]"
        elif self.array is None and isinstance(value, Iterable):
            try:
                val_shape = _shape(value)
                array = "".join(f"[{dim}]" for dim in val_shape)
            except ShapelessError:
                array = ""
        else:
            array = ""

        return array

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        return self.name

    def generate_declaration(self, extern: bool = False) -> str:
        """Return a declaration :obj:`str`.

        Doesn't end with a semicolon (;).

        Args:
            extern: whether or not the value is to be declared as
                :ccode:`extern`

        Examples:
            >>> import numpy as np
            >>> from csnake import Variable
            >>> var = Variable(
            ...     "test",
            ...     primitive="int",
            ...     value=np.arange(24).reshape((2, 3, 4))
            ... )
            >>> print(var.generate_declaration())
            int test[2][3][4]
        """

        if self.qualifiers is not None:
            if isinstance(self.qualifiers, str):
                qual = self.qualifiers + " "
            else:
                qual = " ".join(self.qualifiers) + " "
        else:
            qual = ""

        array = self.__array_dimensions()

        if isinstance(self.primitive, FuncPtr):
            decl = self.primitive.get_declaration(
                name=self.name, qualifiers=qual, array=array
            )

            return "{ext}{decl}".format(
                ext="extern " if extern else "", decl=decl
            )

        return "{ext}{qual}{prim} {name}{array}".format(
            ext="extern " if extern else "",
            qual=qual,
            prim=self.primitive,
            name=self.name,
            array=array,
        )

    @property
    def declaration(self) -> str:
        """Declaration string.

        Ends with a semicolon (;).

        See Also:
            :meth:`generate_declaration` for the underlying method.
        """
        return self.generate_declaration(extern=False) + ";"

    def __generate_array_struct_initialization(
        self,
        array: Union[CArrayLiteral, CStructLiteral],
        indent: Union[int, str] = 4,
        writer: CodeWriterLite = None,
        formatters: LiteralFormatters = None,
    ) -> None:
        """Print (multi)dimensional arrays."""

        if self.value is None:
            raise NoValueError

        stack: deque = deque()
        stack.append(array)
        if not formatters:
            formatters = LiteralFormatters()
        if writer is None:
            writer = CodeWriterLite(indent=indent)

        while stack:
            top = stack.pop()
            for types, action in _ArrStructInitGen.type_action_pairs:
                if isinstance(top, types):
                    formatters = action(
                        top=top,
                        stack=stack,
                        writer=writer,
                        formatters=formatters,
                    )
                    break
            else:
                raise TypeError("Unknown type on stack")

    def generate_initialization(
        self, indent: Union[int, str] = 4
    ) -> CodeWriterLite:
        """Return a :class:`CodeWriterLite` instance containing the
        initialization code for this :class:`Variable`.

        Ends with a semicolon (;).

        Args:
            indent: indent :obj:`str` or :obj:`int` denoting the number of
                spaces for indentation


        Example:
            >>> import numpy as np
            >>> from csnake import Variable
            >>> var = Variable(
            ...     "test",
            ...     primitive="int",
            ...     value=np.arange(24).reshape((2, 3, 4))
            ... )
            >>> print(var.generate_initialization())
            int test[2][3][4] = {
                {
                    {0, 1, 2, 3},
                    {4, 5, 6, 7},
                    {8, 9, 10, 11}
                },
                {
                    {12, 13, 14, 15},
                    {16, 17, 18, 19},
                    {20, 21, 22, 23}
                }
            };

        """

        # main part: generating initializer
        if self.value is None:
            raise NoValueError

        writer = CodeWriterLite(indent=indent)

        if not isinstance(self.qualifiers, str) and isinstance(
            self.qualifiers, Iterable
        ):
            qual = " ".join(self.qualifiers) + " "
        elif self.qualifiers is not None:
            qual = assure_str(self.qualifiers) + " "
        else:
            qual = ""

        array = self.__array_dimensions()

        if isinstance(self.primitive, FuncPtr):
            decl = self.primitive.get_declaration(
                name=self.name, qualifiers=qual, array=array
            )
            writer.add_line(decl)
        else:
            writer.add_line(f"{qual}{self.primitive} {self.name}{array}")

        writer.add(" = ")

        value, formatters = self.merge_formatters(self.value)

        if isinstance(value, (CArrayLiteral, CStructLiteral)):
            self.__generate_array_struct_initialization(
                value, indent, writer, formatters
            )
        else:
            formatters = LiteralFormatters()
            assignment = generate_c_value_initializer(value, formatters)
            writer.add(assignment)

        writer.add(";")
        return writer

    @property
    def initialization(self) -> str:
        """Initialization :obj:`str`.

        Ends with a semicolon (;).

        See Also:
            :meth:`generate_initialization` for the underlying method.
        """
        return str(self.generate_initialization())

    def __str__(self):
        """Initialization (if value in not None) or declaration :obj:`str`.

        Falls back to declaration if :attr:`value` is None.

        Ends with a semicolon (;).

        See Also:
            :meth:`generate_initialization` for the initialization method.
            :meth:`generate_declaration` for the declaration method.
        """
        try:
            return self.initialization
        except NoValueError:
            return self.declaration


# no VariableValue is also a VariableValue!
VariableValue.register(Variable)


def _get_variable(variable: Union[Variable, Collection, Mapping]) -> Variable:
    """Get a Variable out of one of the following:

        * a Variable (idempotent)

        * a Collection (tuple/list-like) of 2 strings (name, primitive)

        * a Mapping (dict-like) with keys (name, primitive)
    """
    if isinstance(variable, Variable):
        return variable
    elif isinstance(variable, Mapping):
        var = Variable(**variable)
        return var
    elif isinstance(variable, Collection):
        if len(variable) != 2:
            raise TypeError(
                "variable must be a Collection with len(variable) == 2"
            )
        var = Variable(*variable)
        return var
    else:
        raise TypeError(
            "variable must be one of (Variable, Collection, Mapping)"
        )


class Struct:
    """Class describing C :ccode:`struct` construct.

    Args:
        name: name of struct
        typedef: whether or not the struct is :ccode:`typedef`'d

    Attributes:
        name: name of struct
        typedef: whether or not the struct is :ccode:`typedef`'d
        variables: :obj:`list` of :ccode:`struct`'s variables
    """

    __slots__ = ("name", "variables", "typedef")

    def __init__(self, name: str, typedef: bool = False) -> None:
        self.name = assure_str(name)
        self.variables: List[Variable] = []
        self.typedef = bool(typedef)

    def add_variable(self, variable: Union[Variable, Collection, Mapping]):
        """Add a variable to `struct`.

        Variables inside of a :class:`Struct` are ordered (added sequentially).

        Args:
            variable: variable to add. It can be defined in multiple ways.

                `variable` can be:

                * a :class:`Variable`
                * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2
                    :obj:`str`\\ s (`name`, `primitive`)
                * a :class:`Mapping` (:obj:`dict`-like) with keys ['name',
                    'primitive']
        """

        proc_var = _get_variable(variable)
        self.variables.append(proc_var)

    def generate_declaration(
        self, indent: Union[int, str] = 4
    ) -> CodeWriterLite:
        """Generate a :class:`CodeWriterLite` instance containing the
        initialization code for this :class:`Struct`.

        Args:
            indent: indent :obj:`str` or :obj:`int` denoting the number of
                spaces for indentation

        Example:
            >>> from csnake import Variable, Struct
            >>> strct = Struct("strname", typedef=False)
            >>> var1 = Variable("var1", "int")
            >>> var2 = Variable("var2", "int", value=range(10))
            >>> strct.add_variable(var1)
            >>> strct.add_variable(var2)
            >>> strct.add_variable(("var3", "int"))
            >>> strct.add_variable({"name": "var4", "primitive": "int"})
            >>> print(strct.generate_declaration())
            struct strname
            {
                int var1;
                int var2[10];
                int var3;
                int var4;
            };
        """
        writer = CodeWriterLite(indent=indent)

        if self.typedef:
            writer.add_line("typedef struct")
        else:
            writer.add_line(f"struct {self.name}")

        writer.open_brace()
        for var in self.variables:
            writer.add_line(var.declaration)
        writer.close_brace()

        if self.typedef:
            writer.add(" " + self.name + ";")
        else:
            writer.add(";")

        return writer

    @property
    def declaration(self):
        """:class:`CodeWriterLite` instance containing the
        declaration code for this :class:`Struct`.

        See Also:
            :meth:`generate_declaration` for the underlying method.
        """

        return self.generate_declaration()

    def __str__(self):
        """Generate a :obj:`str` instance containing the
        declaration code for this :class:`Struct`."""
        return str(self.generate_declaration())


class AddressOf(VariableValue):
    """Address of (&) VariableValue for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    containing the & (address of) used on a value.

    Args:
        variable: variable to return the address of.

    Attributes:
        variable: variable to return the address of.

    Examples:
        >>> from csnake import Variable, AddressOf
        >>> var1 = Variable("var1", "int")
        >>> addrof_var1 = AddressOf(var1)
        >>> var2 = Variable("var2", "int", value=addrof_var1)
        >>> print(var2)
        int var2 = &var1;
    """

    def __init__(self, variable: Union[VariableValue]) -> None:
        if not isinstance(variable, (VariableValue)):
            raise TypeError("variable must be of type VariableValue.")
        self.variable = variable

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        return f"&{generate_c_value_initializer(self.variable, formatters)}"

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class Dereference(VariableValue):
    """Dereference (*) modifier for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    containing the * (dereference) used on a value.

    Args:
        value: value to dereference.

    Attributes:
        value: value to dereference.

    Examples:
        >>> from csnake import Variable, Dereference
        >>> var1 = Variable("var1", "int")
        >>> derefd_var1 = Dereference(var1)
        >>> var2 = Variable("var2", "int", value=derefd_var1)
        >>> print(var2)
        int var2 = *var1;
        >>> derefd_number = Dereference(16)
        >>> var3 = Variable("var3", "int", value=derefd_number)
        >>> print(var3)
        int var3 = *16;
    """

    __slots__ = "value"

    def __init__(self, value: Union[CExtendedLiteral, VariableValue]) -> None:
        if not isinstance(value, (CExtendedLiteral, VariableValue)):
            raise TypeError("value must be of type VariableValue or CLiteral.")
        self.value = value

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`value`\\ s"""
        return f"*{generate_c_value_initializer(self.value, formatters)}"

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class Typecast(VariableValue):
    """Typecast modifier for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    containing the typecast to `type` used on a value.

    Args:
        value: value to be typecast
        cast: type to cast to

    Attributes:
        value: value to be typecast
        cast: type to cast to

    Examples:
        >>> from csnake import Variable, Typecast
        >>> var1 = Variable("var1", "int")
        >>> cast_var1 = Typecast(var1, 'long')
        >>> var2 = Variable("var2", "int", value=cast_var1)
        >>> print(var2)
        int var2 = (long) var1;
        >>> cast_number = Typecast(16, 'long')
        >>> var3 = Variable("var3", "int", value=cast_number)
        >>> print(var3)
        int var3 = (long) 16;
    """

    __slots__ = ("subscript", "cast")

    def __init__(
        self, value: Union[CExtendedLiteral, VariableValue], cast: str
    ) -> None:
        if not isinstance(value, (CExtendedLiteral, VariableValue)):
            raise TypeError(
                "variable must be of type VariableValue or CLiteral."
            )
        self.value = value
        self.cast = assure_str(cast)

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`value`\\ s"""
        initializer = generate_c_value_initializer(self.value, formatters)
        return f"({self.cast}) {initializer}"

    def __str__(self):
        """Initialization string."""
        return self.init_target_code


class Subscript(VariableValue):
    """Subscript ([]) modifier for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    with a subscripted value.

    Args:
        variable: variable to be typecast
        subscript: a :class:`VariableValue` or :obj:`int` or :obj:`list` or
            `tuple` of them representing the subscript[s].

    Attributes:
        variable: variable to be typecast
        subscript: a :class:`VariableValue` or :obj:`int` or :obj:`list` or
            `tuple` of them representing the subscript[s].

    Examples:
        >>> from csnake import Variable, Subscript
        >>> var1 = Variable("var1", "int")
        >>> subscr1_var1 = Subscript(var1, 3)
        >>> var2 = Variable("var2", "int", value=subscr1_var1)
        >>> print(var2)
        int var2 = var1[3];
        >>> subscr2_var1 = Subscript(var1, (3, 2))
        >>> var3 = Variable("var3", "int", value=subscr2_var1)
        >>> print(var3)
        int var3 = var1[3][2];
        >>> subscr2_var1 = Subscript(var1, (var3, 2))
        >>> var4 = Variable("var4", "int", value=subscr2_var1)
        >>> print(var4)
        int var4 = var1[var3][2];
    """

    __slots__ = ("subscript", "variable")

    def __init__(
        self, variable: Union[CLiteral, VariableValue], subscript: Iterable
    ) -> None:
        if not isinstance(variable, VariableValue):
            raise TypeError("variable must be VariableValue.")
        self.variable = variable

        if not isinstance(
            subscript, (CIntLiteral, Iterable, VariableValue, CArrayLiteral)
        ):
            raise TypeError(
                "Subscript must be an CIntLiteral, VariableValue "
                "(or Variable), list or tuple."
            )

        if not subscript:
            raise TypeError("Subscript must be non-empty.")

        try:
            self.subscript = list(subscript)
        except TypeError:
            self.subscript = [subscript]

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""

        return generate_c_value_initializer(
            self.variable, formatters
        ) + "".join(
            f"[{generate_c_value_initializer(dim, formatters)}]"
            for dim in self.subscript
        )

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class Dot(VariableValue):
    """Dot (.) VariableValue for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    for accessing a specific member of a struct.

    Args:
        variable: variable whose member we're accessing
        member: name of member we're accessing

    Attributes:
        variable: variable whose member we're accessing
        member: name of member we're accessing

    Examples:
        >>> from csnake import Variable, Dot
        >>> var1 = Variable("var1", "struct somestr")
        >>> dotvar = Dot(var1, 'some_member')
        >>> var2 = Variable("var2", "int", value=dotvar)
        >>> print(var2)
        int var2 = var1.some_member;
    """

    __slots__ = ("variable", "member")

    def __init__(
        self, variable: Union[CLiteral, VariableValue], member: str
    ) -> None:
        if not isinstance(variable, VariableValue):
            raise TypeError("variable must be VariableValue.")
        self.variable = variable

        if not isinstance(member, (VariableValue, str)):
            raise TypeError("member must be either a VariableValue or a str.")
        self.member = member

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        if isinstance(self.member, str):
            return (
                self.variable.init_target_code(formatters) + "." + self.member
            )
        elif isinstance(self.member, VariableValue):
            return (
                self.variable.init_target_code(formatters)
                + "."
                + self.member.init_target_code(formatters)
            )

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class Arrow(VariableValue):
    """Arrow (->) VariableValue for variable initialization.

    Subclass of :class:`VariableValue` that returns an initialization string
    for accessing a specific member of a struct indirectly (through a pointer).

    Args:
        variable: variable whose member we're accessing
        member: name of member we're accessing

    Attributes:
        variable: variable whose member we're accessing
        member: name of member we're accessing

    Examples:
        >>> from csnake import Variable, Arrow
        >>> var1 = Variable("var1", "struct somestr")
        >>> arrvar = Arrow(var1, 'some_member')
        >>> var2 = Variable("var2", "int", value=arrvar)
        >>> print(var2)
        int var2 = var1->some_member;
    """

    __slots__ = "variable" "item"

    def __init__(
        self, variable: Union[CLiteral, VariableValue], item: str
    ) -> None:
        if not isinstance(variable, VariableValue):
            raise TypeError("variable must be VariableValue.")
        self.variable = variable

        if not isinstance(item, (VariableValue, str)):
            raise TypeError("item must be either a VariableValue or a str.")
        self.item = item

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        if isinstance(self.item, str):
            return (
                self.variable.init_target_code(formatters) + "->" + self.item
            )
        elif isinstance(self.item, VariableValue):
            return (
                self.variable.init_target_code(formatters)
                + "->"
                + self.item.init_target_code(formatters)
            )

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class GenericModifier(VariableValue):
    """VariableValue generated by applying a function to a value.

    A value's initializer string is passed through the supplied function, which
    should return a :obj:`str`.

    Args:
        value: value whose initializer we're passing through a function
        format_function: function used for modifying a value's initializer

    Attributes:
        value: value whose initializer we're passing through a function
        format_function: function used for modifying a value's initializer

    Examples:
        >>> from csnake import Variable, GenericModifier
        >>> var1 = Variable("var1", "int")
        >>> genmod_var1 = GenericModifier(var1, lambda l: f'TRANSLATE({l})')
        >>> var2 = Variable("var2", "int", value=genmod_var1)
        >>> print(var2)
        int var2 = TRANSLATE(var1);
        >>> genmod_var2 = GenericModifier('test', lambda l: f'do_whatever({l})')
        >>> var3 = Variable("var3", "int", value=genmod_var2)
        >>> print(var3)
        int var3 = do_whatever("test");
    """

    __slots__ = ("value", "format_function")

    def __init__(
        self, value: Union[CExtendedLiteral, VariableValue], function: Callable
    ) -> None:
        if not isinstance(value, (CExtendedLiteral, VariableValue)):
            raise TypeError(
                "value must be of type VariableValue or CExtendedLiteral."
            )
        self.value = value
        self.format_function = function

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        return self.format_function(generate_c_value_initializer(self.value))

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class OffsetOf(VariableValue):
    """offsetof VariableValue modifier for initializing to offsets of struct
    members.

    Subclass of :class:`VariableValue` that returns an initialization string
    containing the offset of a specific member of a struct.


    Args:
        struct: struct whose member's offset we're using
        member: struct member whose offset we're using

    Attributes:
        struct: struct whose member's offset we're using
        member: struct member whose offset we're using

    Examples:
        >>> from csnake import Variable, OffsetOf, Struct
        >>> offs_val1 = OffsetOf('struct some', 'some_member')
        >>> var2 = Variable("var2", "int", value=offs_val1 )
        >>> print(var2)
        int var2 = offsetof(struct some, some_member);
        >>> test_struct = Struct('other')
        >>> offs_val2 = OffsetOf(test_struct, 'other_member')
        >>> var3 = Variable("var3", "int", value=offs_val2 )
        >>> print(var3)
        int var3 = offsetof(struct other, other_member);
    """

    __slots__ = ("struct", "member")

    def __init__(
        self, struct: Union[Struct, str], member: Union[VariableValue, str]
    ) -> None:
        if not isinstance(struct, (str, Struct)):
            raise TypeError("First argument must be either a Struct or a str.")
        self.struct = struct

        if not isinstance(member, (VariableValue, str)):
            raise TypeError(
                "Second argument must be either a VariableValue or a str"
            )
        self.member = member

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        if isinstance(self.struct, str):
            struct_name = self.struct
        elif isinstance(self.struct, Struct):
            if self.struct.typedef:
                struct_name = self.struct.name
            else:
                struct_name = "struct " + self.struct.name

        if isinstance(self.member, str):
            member_name = self.member
        elif isinstance(self.member, VariableValue):
            member_name = self.member.init_target_code(formatters)

        return f"offsetof({struct_name}, {member_name})"

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class TextModifier(VariableValue):
    """Generic textual VariableValue for initializing to an arbitrary
    :obj:`str`.

    The :obj:`str` supplied as the argument is output verbatim as the
    Initialization string.

    Args:
        text: initialization string

    Attributes:
        text: initialization string

    Examples:
        >>> from csnake import Variable, TextModifier
        >>> textmod1 = TextModifier('whatever + you + want')
        >>> var2 = Variable("var2", "int", value=textmod1)
        >>> print(var2)
        int var2 = whatever + you + want;
        >>> textmod2 = TextModifier(f'you_can_do_this_too + {var2.name}')
        >>> var3 = Variable("var3", "int", value=textmod2)
        >>> print(var3)
        int var3 = you_can_do_this_too + var2;
    """

    __slots__ = "text"

    def __init__(self, text: str) -> None:
        self.text = assure_str(text)

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return a :obj:`str` used for initialization of other
        :obj:`Variable`\\ s"""
        return self.text

    def __str__(self):
        """Initialization string."""
        return self.init_target_code()


class Function:
    """Class describing C function.

    You can generate a function prototype (declaration), a definition or a
    call.
    A function's body is a :class:`CodeWriterLite`, so You can add lines of
    code to it (or a whole new :class:`CodeWriter` instance).

    Args:
        name: name of the function
        return_type: :obj:`str` containing function's return type

        qualifiers: :obj:`str` or :class:`Sequence` of :obj:`str` listing the
            function's qualifiers (:ccode:`const`, :ccode:`volatile`, ...)

        arguments: an iterable which yields one the following types:

            * a :class:`Variable`
            * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2 strings
                (`name`, `primitive`)
            * a :class:`Mapping` (:obj:`dict`-like) with keys (`name`,
                `primitive`)


    Attributes:
        name: name of the function
        return_type: :obj:`str` containing function's return type

        qualifiers: :obj:`str` or :class:`Sequence` of :obj:`str` listing the
            function's qualifiers (:ccode:`const`, :ccode:`volatile`, ...)

        arguments: an iterable which yields one the following types:

            * a :class:`Variable`
            * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2 strings
                (`name`, `primitive`)
            * a :class:`Mapping` (:obj:`dict`-like) with keys (`name`,
                `primitive`)

        codewriter: internal instance of :class:`CodeWriterLite` that contains
        the funtion's body code.
    """

    __slots__ = (
        "name",
        "return_type",
        "arguments",
        "qualifiers",
        "codewriter",
    )

    def __init__(
        self,
        name: str,
        return_type: str = "void",
        qualifiers: Union[str, Iterable[str]] = None,
        arguments: Iterable = None,
    ) -> None:
        self.name = assure_str(name)
        self.return_type = assure_str(return_type)
        self.arguments: List[Variable] = []
        self.codewriter = CodeWriterLite()

        if isinstance(qualifiers, str):
            self.qualifiers = qualifiers.split()
        elif qualifiers is not None:
            self.qualifiers = [
                assure_str(qualifier) for qualifier in qualifiers
            ]
        else:
            self.qualifiers = []

        if arguments is not None:
            for arg in arguments:
                proc_arg = _get_variable(arg)
                self.add_argument(proc_arg)

    def add_argument(self, arg: Variable) -> None:
        """Add an argument to function.

        Arguments are added sequentially.

        arg: an one the following types:

            * a :class:`Variable`
            * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2 strings
                (`name`, `primitive`)
            * a :class:`Mapping` (:obj:`dict`-like) with keys (`name`,
                `primitive`)
        """

        proc_arg = _get_variable(arg)

        self.arguments.append(proc_arg)

    def add_arguments(self, args: Iterable[Variable]) -> None:
        """Add multiple arguments to function.

        Arguments are added sequentially.

        args: an iterable which yields one the following types:

            * a :class:`Variable`
            * a :class:`Collection` (:obj:`tuple`/:obj:`list`-like) of 2 strings
                (`name`, `primitive`)
            * a :class:`Mapping` (:obj:`dict`-like) with keys (`name`,
                `primitive`)
        """

        for arg in args:
            self.add_argument(arg)

    def add_code(self, code: Union[str, Iterable[str]]) -> None:
        """Add a :obj:`str` or :obj:`Iterable` of :obj:`str`\\ s to function's body

        Since a :class:`CodeWriter` is an iterable of :obj:`str`\\ s, you can
        simply add its contents to the function by passing it to this method.


        Args:
            code: :obj:`str` or :obj:`Iterable` of :obj:`str`\\ s to be added
        """

        self.codewriter.add_lines(code)

    def generate_call(self, *arg) -> str:
        """Return function calling code for specific arguments.

        Args:
            \\*arg: argument values for call, in order of appearance.

        Doesn't end with a semicolon (;).

        Examples:
            >>> from csnake import Variable, Function
            >>> arg1 = Variable("arg1", "int")
            >>> arg2 = Variable("arg2", "int", value=range(10))
            >>> arg3 = ("arg3", "int")
            >>> arg4 = {"name": "arg4", "primitive": "int"}
            >>> fun = Function(
            ...     "testfunct", "void", arguments=(arg1, arg2, arg3, arg4)
            ... )
            >>> fun.add_code(("code;", "more_code;"))
            >>> print(fun.generate_call(1, 2, 3, 4))
            testfunct(1, 2, 3, 4)
        """

        # if the arguments are all given in a single iterable
        if len(arg) == 1:
            return self.generate_call(*next(iter(arg)))

        if not len(arg) == len(self.arguments):
            raise ValueError("number of arguments must match")

        arg_str = ", ".join(str(argument) for argument in arg)
        return f"{self.name}({arg_str})"

    def generate_prototype(self, extern: bool = False) -> str:
        """Generate function prototype code.

        Args:
            extern: whether or not the function is to be declared as
                :ccode:`extern`

        Doesn't end with a semicolon (;).

        Examples:
            >>> from csnake import Variable, Function
            >>> arg1 = Variable("arg1", "int")
            >>> arg2 = Variable("arg2", "int", value=range(10))
            >>> arg3 = ("arg3", "int")
            >>> arg4 = {"name": "arg4", "primitive": "int"}
            >>> fun = Function(
            ...     "testfunct", "void", arguments=(arg1, arg2, arg3, arg4)
            ... )
            >>> fun.add_code(("code;", "more_code;"))
            >>> print(fun.generate_prototype())
            void testfunct(int arg1, int arg2[10], int arg3, int arg4)
        """
        return "{extern}{qual}{ret} {nm}({args})".format(
            extern="extern " if extern else "",
            qual=" ".join(self.qualifiers) + " " if self.qualifiers else "",
            ret=self.return_type,
            nm=self.name,
            args=", ".join(
                var.generate_declaration() for var in self.arguments
            )
            if self.arguments
            else "void",
        )

    @property
    def prototype(self) -> str:
        """Function prototype (declaration) :obj:`str`.

        Ends with a semicolon (;).

        See Also:
            :meth:`generate_prototype` for the underlying method.
        """
        return self.generate_prototype() + ";"

    def generate_definition(
        self, indent: Union[int, str] = "    "
    ) -> CodeWriterLite:
        """Return a :class:`CodeWriterLite` instance containing the
        definition code for this :class:`Function`.

        Args:
            indent: indent :obj:`str` or :obj:`int` denoting the number of
                spaces for indentation

        Examples:
            >>> from csnake import Variable, Function
            >>> arg1 = Variable("arg1", "int")
            >>> arg2 = Variable("arg2", "int", value=range(10))
            >>> arg3 = ("arg3", "int")
            >>> arg4 = {"name": "arg4", "primitive": "int"}
            >>> fun = Function(
            ...     "testfunct", "void", arguments=(arg1, arg2, arg3, arg4)
            ... )
            >>> fun.add_code(("code;", "more_code;"))
            >>> print(fun.generate_definition())
            void testfunct(int arg1, int arg2[10], int arg3, int arg4)
            {
                code;
                more_code;
            }

        """
        writer = CodeWriterLite(indent=indent)
        writer.add_line(self.generate_prototype())
        writer.open_brace()

        writer.add_lines(self.codewriter)  # type: ignore

        writer.close_brace()

        return writer

    @property
    def definition(self) -> str:
        """Function definition :obj:`str`.

        See Also:
            :meth:`generate_definition` for the underlying method.
        """
        return self.generate_definition().code

    def init_target_code(self, formatters: LiteralFormatters = None) -> str:
        """Return code used for variable initialization, formatted with the
        supplied formatters.

        Args:
            formatters: collection of formatters used for formatting the
                initialization :obj:`str`
        """
        return self.name


# TODO: allow for functions to be used to initialize function pointers

VariableValue.register(Function)


class Enum:
    """Class describing C :ccode:`enum` construct.

    Args:
        name: name of the enum
        prefix: prefix to add to every enumeration's name
        typedef: whether or not the enum is :ccode:`typedef`'d

    Attributes:
        name: name of the enum
        typedef: whether or not the enum is :ccode:`typedef`'d
        prefix: prefix to add to every enumeration's name
        values: `list` of :class:`Enum.EnumValue`
    """

    __slots__ = ("typedef", "values", "name", "prefix")

    class EnumValue:
        """Singular value of an C-style enumeration."""

        __slots__ = ("name", "value", "comment")

        def __init__(
            self,
            name: str,
            value: Union[CLiteral, VariableValue] = None,
            comment: str = None,
        ) -> None:
            self.name = assure_str(name)
            if value is not None and not isinstance(
                value, (CLiteral, VariableValue)
            ):
                raise ValueError(
                    f"value ({value}) must be one of (CLiteral, VariableValue)"
                )
            self.value = value
            self.comment = assure_str(comment) if comment is not None else None

    def __init__(
        self, name: str, prefix: str = None, typedef: bool = False
    ) -> None:

        self.typedef = bool(typedef)
        # enum values
        self.values: List[Enum.EnumValue] = []
        self.name = assure_str(name)

        if prefix is not None:
            self.prefix = assure_str(prefix)
        else:
            self.prefix = ""

    def add_value(
        self,
        name: str,
        value: Union[CLiteral, VariableValue] = None,
        comment: str = None,
    ) -> None:
        """Add a single :ccode:`name = value` pair (or just :ccode:`name`)."""
        self.values.append(self.EnumValue(name, value=value, comment=comment))

    def add_values(
        self,
        values: Union[Mapping, Collection[Union[Mapping, Collection, str]]],
    ) -> None:
        """Add multiple :ccode:`name = value` pairs (or just :ccode:`name`\\ s).

        Values can be one of:

        * a :obj:`dict` with `name` : `value` entries
        * a collection of:
            * :obj:`str`\\ s denoting names (no value)
            * :obj:`dict`\\ s with keys [`name`, `value`, `comment`], and optional
                keys [`value`, `comment`]
            * collections (:obj:`list`-like) of length 1-3 denoting [`name`,
                `value`, `comment`] respectively

        """
        if isinstance(values, Mapping):
            for name, value in values.items():
                self.add_value(name, value)

        else:
            for value in values:
                if isinstance(value, str):
                    self.add_value(value)
                elif isinstance(value, Mapping):
                    self.add_value(**value)
                # lists and tuples
                else:
                    defaults = (None, None, None)
                    name = seq_get(value, defaults, 0)
                    value = seq_get(value, defaults, 1)
                    comment = seq_get(value, defaults, 2)

                    name = assure_str(name)

                    self.add_value(name, value, comment)

    def generate_declaration(
        self, indent: Union[int, str] = 4
    ) -> CodeWriterLite:
        """Generate enum declaration code.

        Generate a :class:`CodeWriterLite` instance containing the
        initialization code for this :class:`Enum`.

        Args:
            indent: indent :obj:`str` or :obj:`int` denoting the number of
                spaces for indentation


        Examples:
            >>> from csnake import (
            ...     Enum, Variable, Dereference, AddressOf
            ... )
            >>> name = "somename"
            >>> pfx = "pfx"
            >>> typedef = False
            >>> enum = Enum(name, prefix=pfx, typedef=typedef)
            >>> cval1 = Variable("varname", "int")
            >>> enum.add_value("val1", 1)
            >>> enum.add_value("val2", Dereference(1000))
            >>> enum.add_value("val3", cval1)
            >>> enum.add_value("val4", AddressOf(cval1), "some comment")
            >>> print(enum.generate_declaration())
            enum somename
            {
                pfxval1 = 1,
                pfxval2 = *1000,
                pfxval3 = varname,
                pfxval4 = &varname /* some comment */
            };
        """

        gen = CodeWriterLite(indent=indent)

        if self.typedef:
            gen.add_line("typedef enum")
        else:
            gen.add_line("enum {name}".format(name=self.name))
        gen.open_brace()

        def _generate_lines(
            values: Iterable, lastline: bool = False
        ) -> Iterator:
            return (
                (
                    "{prefix}{name}{value}{comma}".format(
                        prefix=self.prefix,
                        name=assure_str(val.name),
                        value=(" = " + generate_c_value_initializer(val.value))
                        if val.value
                        else "",
                        comma="," if not lastline else "",
                    ),
                    val.comment,
                )
                for val in values
            )

        commalines = _generate_lines(self.values[:-1], lastline=False)
        lastline = _generate_lines(self.values[-1:], lastline=True)

        for line in chain(commalines, lastline):
            gen.add_line(*line)

        gen.close_brace()

        if self.typedef:
            gen.add(" " + self.name + ";")
        else:
            gen.add(";")

        return gen

    @property
    def declaration(self) -> str:
        """The declaration :obj:`str` (with indentation of 4 spaces).


        See Also:
            :meth:`generate_declaration` for the underlying method.
        """
        return self.generate_declaration().code

    def __str__(self) -> str:
        """
        Return the declaration :obj:`str` (with indentation of 4 spaces).
        """
        return self.declaration
