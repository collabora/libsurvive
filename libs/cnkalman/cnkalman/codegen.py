import dataclasses
import logging
import math
import sys
import types
from collections.abc import Iterable

import symengine.lib.symengine_wrapper
import sympy
import sys

from symengine import cse
#from symengine import atan2, sqrt, cos, sin, Matrix, Pow, Mul, asin, Symbol, symbols, Abs, tan, acos
from symengine import Matrix, Mul, Symbol, symbols

import logging
import os

use_symbolic_eval = False
dirty_python_hack = False

function_expose_list = [(symengine.atan2, math.atan2),
                        (symengine.sqrt, math.sqrt),
                        (symengine.cos, math.cos),
                        (symengine.sin, math.sin),
                        (symengine.Pow, math.pow),
                        (symengine.asin, math.asin),
                        (symengine.Abs, abs),
                        (symengine.tan, math.tan),
                        (symengine.acos, math.acos),
                        ]

for sym_f, f in function_expose_list:
    current_module = sys.modules[__name__]

    def capture(sym_f, f):
        def eval_f(*args, **kwargs):
            global use_symbolic_eval
            return sym_f(*args, **kwargs) if use_symbolic_eval else f(*args, **kwargs)
        return eval_f
    setattr(current_module, f.__name__, capture(sym_f, f))

def isinstance_namedtuple(obj) -> bool:
    return (
            isinstance(obj, tuple) and
            hasattr(obj, '_asdict') and
            hasattr(obj, '_fields')
    )

def flatten_args(bla, prefix=''):
    output = []
    if hasattr(bla, "__iter__"):
        for i, item in enumerate(bla):
            output += flatten_args(item, prefix=prefix+'['+str(i)+']') if hasattr(item, "__iter__") or isinstance_namedtuple(item) else [(prefix+"["+str(i)+"]", item)]
    elif hasattr(bla, "__dict__"):
        for k,vall in bla.__dict__.items():
            if hasattr(vall, "__iter__") or isinstance_namedtuple(vall):
                for ik, iv in flatten_args(vall, prefix=prefix+'.'+k):
                    output.append((ik, iv))
            else:
                output.append((prefix + "." + k, vall))

    return output


def make_sympy(expressions):
    flatten = []

    if type(expressions) == symengine.MutableDenseMatrix:
        return expressions
    if type(expressions) == sympy.Matrix:
        return expressions

    if hasattr(expressions, "atoms"):
        return [expressions]

    if isinstance(expressions, list) or isinstance(expressions, np.ndarray):
        return symengine.MutableDenseMatrix([make_sympy(a) for a in expressions])
    if hasattr(expressions, 'symengine_type'):
        return expressions.symengine_type()

    if not hasattr(expressions, "_sympy_"):
        if isinstance(expressions, Iterable):
            for col in expressions:
                if hasattr(col, '_sympy_') or hasattr(col, 'free_symbols'):
                    flatten.append(col)
                else:
                    for cell in col:
                        flatten.append(cell)
        else:
            return [expressions]
    else:
        flatten.append(expressions)
    return flatten

def expand_pow(x):
    pass

def number(x):
    if x.is_Number:
        return float(x)
    return None

def clean_parens(txt):
    if txt[0] == '(' and txt[-1] == ')':
        cnt = 0
        for c in txt[1:-1]:
            if c == '(':
                cnt += 1
            if c == ')':
                cnt -= 1
            if cnt < 0:
                return txt
        return clean_parens(txt[1:-1])
    return txt

class c_formatter:
    @staticmethod
    def format_ternary(test, true_path, false_path, note = ""):
        return "(%s ? %s : %s)%s" % (test, true_path, false_path, f" /* {note} */" if note else "")

class pyx_formatter:
    @staticmethod
    def format_ternary(test, true_path, false_path, note = ""):
        return f"({true_path} if {test} else {false_path})"

def code_wrapper(formatter, item, depth = 0):
    #return symengine.ccode(item)

    if item.is_Atom:
        if item == True:
            return "true"
        if item == False:
            return "false"
        return symengine.ccode(item)

    newargs = list(map(lambda x: code_wrapper(formatter, x, depth+1), item.args))

    infixes = {
        Mul: '*',
        symengine.Add: '+',
        symengine.GreaterThan: '>=',
        symengine.StrictGreaterThan: '>',
        symengine.LessThan: '<=',
        symengine.StrictLessThan: '<'
    }

    if item.__class__ == symengine.Pow:
        # Basically it's always faster to never call pow
        if item.args[1].is_Number and abs(item.args[1]) < 20:
            invert = item.args[1] < 0
            num, den = abs(item.args[1]).get_num_den()

            if den == 1 or den == 2:
                mul_cnt = num if den == 1 else (num - 1) / 2
                muls = [newargs[0]] * int(mul_cnt)
                if den == 2:
                    muls.append("sqrt(" + clean_parens(newargs[0]) + ")")
                v = " * ".join(muls)
                if len(muls) > 1:
                    v = "(" + v + ")"
                if invert:
                    v = "(1. / " + v + ")"
                return v
        return "pow(%s, %s)" % tuple(newargs)
    elif item.__class__ is symengine.Abs:
        return f"fabs({newargs[0]})"
    elif item.__class__ in infixes:
        return "(" + (" " + infixes[item.__class__] + " ").join(newargs) + ")"
    elif item.__class__ == symengine.Piecewise:
        if item.args[1] == True:
            return newargs[0]
        if item.args[1] == False:
            return newargs[2]
        return formatter.format_ternary(newargs[1], newargs[0], newargs[2])
    elif isinstance(item, symengine.Derivative):
        if item.args[0] == symengine.Abs(item.args[1]):
            return formatter.format_ternary(item.args[1] > 0, 1, -1, note="Note: Maybe not valid for == 0")
            #return "((%s) > 0 ? 1 : -1) /* Note: Maybe not valid for == 0 */" % (item.args[1])

    known_c_funcs = [ 'asin', 'cos', 'sin', 'atan2', 'tan', 'atan', 'acos']
    if item.__class__.__name__ not in known_c_funcs:
        print("Warning: Unhandled type " + item.__class__.__name__, file=sys.stderr)
    return item.__class__.__name__ + "(" + ", ".join(map(clean_parens, newargs)) + ")"
    #raise Exception("Unhandled type " + item.__class__.__name__)

def ccode(item):
    return clean_parens(code_wrapper(c_formatter, item))

def pyxcode(item):
    return clean_parens(code_wrapper(pyx_formatter, item))

def sanitize_name(n):
    if len(n) > 1 and n[0] == 'x' and str(n[1:]).isdigit():
        return "_" + n
    return n

class WrapIterable:
    def __init__(self):
        pass

    def apply_operator(self, op, other):
        if isinstance(other, Iterable):
            return np.array([op(x[0], x[1]) for x in zip(self, other)])
        return np.array([op(x, other) for x in self])

    def __add__(self, other): return self.apply_operator(lambda x, y: x + y, other)
    def __radd__(self, other): return self.apply_operator(lambda x, y: y + x, other)
    def __sub__(self, other): return self.apply_operator(lambda  x, y: x - y, other)
    def __rsub__(self, other): return self.apply_operator(lambda  x, y: y - x, other)
    def __sub__(self, other): return self.apply_operator(lambda x, y: x - y, other)
    def __mul__(self, other): return self.apply_operator(lambda x, y: x * y, other)
    def __div__(self, other): return self.apply_operator(lambda x, y: x / y, other)
    def __truediv__(self, other): return self.apply_operator(lambda x, y: x / y, other)

class WrapTuple(WrapIterable):
    def __init__(self, n, t):
        super().__init__()
        self.n = sanitize_name(n)
        self.t = t

    def __getitem__(self, item): return self.t[item]
    def __iter__(self): yield from self.t
    def __str__(self): return self.n
    def __repr__(self): return self.n

class WrapBase:
    def __init__(self, parent):
        self._parent = parent
    def root(self):
        if self._parent is None:
            return self
        return self._parent.root()

    def accessor(self, stopat = None):
        if self._parent is None:
            global dirty_python_hack
            return f"(*{self._name})" if not dirty_python_hack else self._name
        if self._parent is stopat:
            return self._name
        return self._parent.accessor(stopat) + "." + self._name
    def offsetof(self):
        r = self.root()
        if self == r:
            return None
        if hasattr(r, '_type'):
            if dirty_python_hack:
                return 0
            return f"offsetof({r._type.__name__}, {self.accessor(r)})/sizeof(FLT)"
        return None
    def symengine_type(self):
        return Symbol(self.accessor())
    @staticmethod
    def as_symengine(x):
        if hasattr(x, 'symengine_type'):
            return x.symengine_type()
        return x
    def __add__(self, other): return self.symengine_type() + WrapBase.as_symengine(other)
    def __radd__(self, other): return WrapBase.as_symengine(other) + self.symengine_type()
    def __sub__(self, other): return symengine.Add(self.symengine_type(), -WrapBase.as_symengine(other))
    def __rsub__(self, other): return symengine.Add(WrapBase.as_symengine(other), -self.symengine_type())
    def __mul__(self, other): return self.symengine_type() * WrapBase.as_symengine(other)
    def __div__(self, other): return self.symengine_type() / WrapBase.as_symengine(other)
    def __truediv__(self, other): return self.symengine_type() / WrapBase.as_symengine(other)
    def __lt__(self, other):
        if self._parent == other._parent:
            return self.id() < other.id()
        return self._parent < other._parent

    def __str__(self):
        return self._name

class WrapIndex(WrapBase):
    def __init__(self, parent, item):
        super().__init__(parent)
        self._item = item

    def accessor(self, stopat = None):
        return f"{self._parent.accessor(stopat)}[{self._item}]"
    def __str__(self):
        return f"{self._parent.accessor()}[{self._item}]"
    def __repr__(self): return self.__str__()

    def id(self): return self._item

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()

    def __hash__(self):
        return str(self).__hash__()

class WrapArray(WrapIterable, WrapBase, Iterable):
    def __init__(self, name, parent, default):
        WrapIterable.__init__(self)
        WrapBase.__init__(self, parent)
        self._default = None
        self._length = -1
        self._name = sanitize_name(name)
        self._array = list()
        if default is not dataclasses.MISSING and default is not None:
            self._default = default
            self._length = len(default)
            [self[x] for x in range(self._length)]

    def accessor(self, stopat = None):
        if self._parent is None:
            return f"{self._name}"
        return WrapBase.accessor(self, stopat)

    def ensure_size(self, idx):
        while len(self._array) <= idx:
            self._array.append(None)

    def __getitem__(self, item):
        if isinstance(item, slice):
            return self._array[item]

        name = f"{self.accessor()}[{item}]"
        rtn = Symbol(name)
        self.ensure_size(item)
        if self._array[item] == None:
            self._array[item] = WrapIndex(self, item)
        return rtn
    def id(self): return self._name

    def __iter__(self):
        yield from self._array
    def __len__(self):
        if self._length == -1:
            raise Exception(f"Need length annotation for {self._name}")
        return self._length

class WrapMember(WrapBase):
    def __init__(self, name, type, parent):
        super().__init__(parent)
        self._name = sanitize_name(name)
        self._type = type
    def _sympy_(self):
        return self.symengine_type()
    def __str__(self):
        return self._name
    def id(self): return self._name
    def accessor(self, stopat = None):
        if self._parent is None:
            return f"{self._name}"
        if self._parent is stopat:
            return self._name
        return self._parent.accessor(stopat) + "." + self._name
    def __repr__(self): return self.__str__()

class WrapObject(WrapBase):
    def __init__(self, name, type, parent):
        super().__init__(parent)
        self._name = sanitize_name(name)
        self._type = type
        for k,f in type.__dataclass_fields__.items():
            setattr(self, k, get_argument(k, None, f.type, parent=self, default=f.default))

    def id(self): return self._name

    def __lt__(self, other):
        if self._parent == other._parent:
            return self._name < other._name
        if self._parent is None:
            return True
        if other._parent is None:
            return False
        return self._parent < other._parent

    def __str__(self):
        return self._name

def parse_type(n, type, parent, default):
    if type is list or type is np.array:
        return WrapArray(n, parent, default)
    if hasattr(type, '__dataclass_fields__'):
        return WrapObject(n, type, parent)
    return WrapMember(n, type, parent)

def get_argument(n, argument_specs, annotation, parent=None, default = None):
    if argument_specs is not None and n in argument_specs:
        a = argument_specs[n]
        digits = math.floor(math.log(a))
        if isinstance(a, tuple):
            return WrapTuple(n, a)
        if isinstance(a, int):
            return WrapTuple(n, [ symbols(f"{sanitize_name(n)}{str(i).zfill(digits)}") for i in range(a)])
        return a
    if annotation is not None:
        return parse_type(n, annotation, parent, default)
    if n in globals():
        return globals()[n]
    return symbols(n)

def SymbolizeType(type):
    return parse_type(type.__name__, type, None, None)

def get_name(a):
    if type(a) == list:
        return "_".join(map(get_name, a))
    if hasattr(a, '__name__'):
        return a.__name__
    return str(a)

import inspect


def flatten_func(func, name=None, args=None, suffix = None, argument_specs ={}):
    if callable(func):
        name = func.__qualname__.replace(".", "_")
        annotations = inspect.getfullargspec(func).annotations
        args = [get_argument(n, argument_specs, annotations.get(n)) for n in inspect.getfullargspec(func).args]

    if suffix is not None:
        name = name + "_" + suffix

    if isinstance(func, types.FunctionType):
        #try:
        func = func(*map_arg(args))
        #except Exception as e:
        #    sys.stderr.write(f"Error evaluating {name}. Likely a variable needs a length annotation: {e}\n")
        #    traceback.print_exception(*sys.exc_info(), file=sys.stderr)
        #    return None, None

    if hasattr(func, '__dataclass_fields__'):
        return dataclass2dictionary(func), args

    return make_sympy(func), args

def dataclass2dictionary(func):
    dict = {}
    def process(prefix, obj, root = None):
        if hasattr(obj, '__dataclass_fields__'):
            for k,f in obj.__dataclass_fields__.items():
                process(prefix + "." + f.name, getattr(obj, f.name), obj if root is None else root)
        else:
            if isinstance(obj, Iterable):
                for idx, item in enumerate(obj):
                    dict[(root.__class__.__name__, f"{prefix.strip('.')}[{idx}]")] = item
            elif isinstance(obj, (symengine.MutableDenseMatrix, sympy.MutableDenseMatrix, np.ndarray)):
                for idx, item in enumerate(list(obj)):
                    dict[(root.__class__.__name__, f"{prefix.strip('.')}[{idx}]")] = item
            else:
                dict[(root.__class__.__name__, prefix.strip('.'))] = obj
    process('', func)
    dict['$original'] = func
    return dict

def get_type(a):
    if callable(a):
        return get_type(a())
    if hasattr(a, "__iter__"):
        ty = get_type(a[0])
        if ty[-1] != "*":
            ty += "*"
        return ty
    if isinstance(a, WrapObject):
        return a._type.__name__ + "*"
    if isinstance_namedtuple(a):
        return a.__class__.__name__ + "*"
    return "FLT"

def arg_str(arg):
    a = arg[1]
    return "const %s %s" % (get_type(a), get_name(a))

def arg_str_py(arg):
    a = arg[1]
    t = get_type(a)
    t = t.replace("FLT*", "floating[:]")
    t = t.replace("FLT", "floating")
    return "const %s %s" % (t, get_name(a))

def generate_args_string(args, as_call = False):
    return ", ".join(map(lambda x: get_name(x[1]) if as_call else arg_str, enumerate(args)))

def generate_pyxcode(func, name=None, args=None, suffix = None, argument_specs ={}, outputs = None, preamble = "", file=None, input_keys = None, prefix = ""):    
    def emit_code(*args, **kwargs):
        if file is not None:
            print(*args, **kwargs, file=file[0])
            file[0].flush()

    def emit_header(*args, **kwargs):
        if file is not None:
            print(*args, **kwargs, file=file[1])
            file[1].flush()
    print(f"\tGenerating {name}...", file=sys.stderr)
    flatten, args = flatten_func(func, name, args, suffix, argument_specs)
    if flatten is None:
        return None

    if outputs is None:
        if hasattr(flatten, "shape"):
            outputs = [("out", flatten.shape)]
        else:
            outputs = [("out", -1)]

    if callable(func):
        name = func.__qualname__.replace(".", "_")
        annotations = inspect.getfullargspec(func).annotations
        args = [get_argument(n, argument_specs, annotations.get(n)) for n in inspect.getfullargspec(func).args]

    if suffix is not None:
        name = name + "_" + suffix

    keys = None
    free_symbols = set()
    def update_free_symbols(v):
        if hasattr(v, 'free_symbols'):
            free_symbols.update({k.__str__() for k in v.free_symbols})
            return

        if isinstance(v, Iterable):
            for v1 in v:
                update_free_symbols(v1)

    type_name = "floating[:,:]"
    type = None
    if isinstance(flatten, dict):
        type_name = flatten["$original"].__class__.__name__
        type = flatten["$original"].__class__
        flatten.pop("$original")
        keys = list(flatten.keys())
        values = [flatten[k] for k in keys]
        keys = [k[1] for k in keys]
        values = [ a.symengine_type() if hasattr(a, 'symengine_type') else a for a in values ]
        print(f"\tCSE for {len(values)} dict values", file=sys.stderr)
        cse_output = cse(symengine.Matrix(values))
        update_free_symbols(values)

    else:
        print(f"\tCSE for {len(flatten)} values", file=sys.stderr)
        cse_output = cse(symengine.Matrix(flatten))
        update_free_symbols(flatten)

    print(f"\t{len(free_symbols)} free symbols...", file=sys.stderr)
    func_name = name
    emit_header("cdef void %s%s_nogil(%s, %s) nogil " % (prefix,
                                                  name,
                                                  ", ".join([f"{type_name} " + s[0] for s in outputs]),
                                                  ", ".join(map(arg_str_py, enumerate(args)))
                                                  ))

    emit_code("cdef void %s%s_nogil(%s, %s) nogil: " % (prefix,
                                        name,
                                        ", ".join([f"{type_name} " + s[0] for s in outputs]),
                                        ", ".join(map(arg_str_py, enumerate(args)))
                                        ))

    if preamble:
        emit_code(preamble.strip("\r\n"))

    # Unroll struct types
    for idx, a in enumerate(args):
        if callable(a):
            name = get_name(a)
            for k, v in flatten_args(a()):
                if f"{name}{k.strip('[]')}" in free_symbols:
                    emit_code("\tcdef floating %s = %s%s" % (str(v), "("+name+")" if isinstance_namedtuple(a()) else name, k))
        elif isinstance(a, WrapTuple):
            name = get_name(a)
            digits = math.floor(math.log(len(a.t)))
            for k, v in flatten_args(a.t):
                idx = k.strip('[]')
                if f"{name}{str(idx).zfill(digits)}" in free_symbols:
                    emit_code("\tcdef floating %s = %s%s" % (str(v), name, k))

    for item in cse_output[0]:
        stripped_line = pyxcode(item[1]).replace("\n", " ").replace("\t", " ")
        emit_code(f"\tcdef floating {symengine.ccode(item[0])} = {stripped_line}")

    output_idx = 0
    outputs_idx = 0

    count_zeros = 0
    for item_idx, item in enumerate(cse_output[1]):
        if item == 0:
            count_zeros += 1
    needs_set_zero = False#count_zeros > len(cse_output[1]) / 4

    current_shape = None
    if keys is None:
        current_shape = outputs[outputs_idx][1] if isinstance(outputs[outputs_idx][1], tuple) else [outputs[outputs_idx][1], 1]
        var = outputs[outputs_idx][0]
        emit_code(
            f"\t### cdef np.ndarray[float, ndim=2] {outputs[outputs_idx][0]} = np.zeros(({current_shape[0]},{current_shape[1]}), dtype=np.float32)")
    else:
        pass
    
    for item_idx, item in enumerate(cse_output[1]):
        if keys is None:
            current_shape = outputs[outputs_idx][1] if isinstance(outputs[outputs_idx][1], tuple) else [outputs[outputs_idx][1], 1]
            current_row = output_idx // current_shape[1]
            current_col = output_idx % current_shape[1]

            def get_col_str():
                if len(outputs[outputs_idx]) > 2 and hasattr(outputs[outputs_idx][2][current_col], 'offsetof'):
                    offset_of = outputs[outputs_idx][2][current_col].offsetof()
                    if offset_of is not None:
                        return offset_of
                return str(current_col)
            def get_row_str():
                if input_keys is not None:
                    root, path = input_keys[current_row]
                    return 0
                    return f"offsetof({root}, {path})/sizeof(FLT)"
                return str(current_row)
            if hasattr(item, "tolist"):
                for item1 in sum(item.tolist(), []):
                    emit_code("\t%s[%s,%s] = %s" % (outputs[outputs_idx][0], get_row_str(), get_col_str(), output_idx, pyxcode(item1).replace("\n", " ").replace("\t", " ")))
                    output_idx += 1
                    current_row = output_idx / current_shape[1]
                    current_col = output_idx % current_shape[1]
            else:
                if item != 0 or not needs_set_zero:
                    emit_code("\t%s[%s,%s] = %s" % (outputs[outputs_idx][0], get_row_str(), get_col_str(), pyxcode(item).replace("\n", " ").replace("\t", " ")))
                output_idx += 1
            if output_idx >= math.prod(current_shape) > 0:
                #emit_code(f"\treturn {outputs[outputs_idx][0]}")
                outputs_idx += 1
                output_idx = 0
        else:
            nl = "\n"
            emit_code(f"\tout.{keys[item_idx]}={pyxcode(item).replace(nl, '')}")

    emit_code("")

    release_gil = len(cse_output[1]) > 200
    emit_code("cpdef void %s%s(%s, %s): " % (prefix,
                                                  func_name,
                                             ", ".join([f"{type_name} " + s[0] for s in outputs]),
                                                  ", ".join(map(arg_str_py, enumerate(args)))
                                                  ))
    call_args = ",".join([s[0] for s in outputs]) + ", " + generate_args_string(args, as_call=True)
    emit_code(f"\t{prefix}{func_name}_nogil({call_args})")
    emit_code("")

    if current_shape:
        emit_header("cpdef np.ndarray %s%s_allocate(%s) " % (prefix,
                                                  func_name,
                                                  ", ".join(map(arg_str_py, enumerate(args)))
                                                  ))

        emit_code("cpdef np.ndarray %s%s_allocate(%s): " % (prefix,
                                                 func_name,
                                                 ", ".join(map(arg_str_py, enumerate(args)))
                                                 ))
        emit_code(f"\tcdef np.ndarray[double, ndim=2] {outputs[0][0]} = np.zeros(({abs(current_shape[0])},{current_shape[1]}), dtype=np.float64)")
        emit_code(f"\tcdef floating[:,:] _{outputs[0][0]} = {outputs[0][0]}")
        call_args = ",".join([f"_{s[0]}" for s in outputs]) + ", " + generate_args_string(args, as_call=True)
        emit_code(f"\t{'with nogil: ' if release_gil else ''}{prefix}{func_name}_nogil({call_args})")
        if current_shape[1] == 1:
            emit_code(f"\treturn {', '.join([s[0] for s in outputs])}.reshape(-1)")
        else:
            emit_code(f"\treturn {', '.join([s[0] for s in outputs])}")

        for vectorize_over in argument_specs.get('_vectorize', []):
            def get_vec_arg(a):
                t = get_type(a)
                name = get_name(a)
                if name == vectorize_over:
                    t += "*"
                t = t.replace("FLT**", "floating[:,:]")
                t = t.replace("FLT*", "floating[:]")
                t = t.replace("FLT", "floating")
                return "const %s %s" % (t, name)

            output_shape = f"{vectorize_over}.shape[0]", abs(current_shape[0]), current_shape[1]
            if output_shape[-1] == 1:
                output_shape = output_shape[:-1]

            dtype = "np.float32 if floating is float else np.float64"
            vec_args = ', '.join(map(get_vec_arg, args))
            call_args = ",".join([f"_{s[0]}[idx,:,:]" for s in outputs] + [f"{get_name(a)}[idx]" if vectorize_over == get_name(a) else get_name(a) for a in args])
            emit_header(f"cpdef np.ndarray {prefix}{func_name}_vectorize_{vectorize_over}({vec_args}) ")
            emit_code(f"""
cpdef np.ndarray {prefix}{func_name}_vectorize_{vectorize_over}({vec_args}):            
    cdef np.ndarray[floating, ndim=3] {outputs[0][0]} = np.zeros(({vectorize_over}.shape[0],{abs(current_shape[0])},{current_shape[1]}), dtype={dtype})
    cdef floating[:,:,:] _{outputs[0][0]} = {outputs[0][0]}
    cdef int idx
    with nogil:
        for idx in range({vectorize_over}.shape[0]):
            {prefix}{func_name}_nogil({call_args})
    return {outputs[0][0]}.reshape({', '.join(map(str,output_shape))})
""".replace("    ", "\t"))

    emit_header("cpdef void %s%s(%s, %s)" % (prefix,
                                             func_name,
                                             ", ".join([f"{type_name} " + s[0] for s in outputs]),
                                             ", ".join(map(arg_str_py, enumerate(args)))
                                             ))
    emit_code("")

    return flatten

def generate_ccode(func, name=None, args=None, suffix = None, argument_specs ={}, outputs = None, preamble = "", file=None, input_keys = None, prefix = ""):
    def emit_code(*args, **kwargs):
        if file is not None:
            print(*args, **kwargs, file=file)

    flatten, args = flatten_func(func, name, args, suffix, argument_specs)
    if flatten is None:
        return None

    if outputs is None:
        if hasattr(flatten, "shape"):
            outputs = [("out", flatten.shape)]
        else:
            outputs = [("out", -1)]

    if callable(func):
        name = func.__qualname__.replace(".", "_")
        annotations = inspect.getfullargspec(func).annotations
        args = [get_argument(n, argument_specs, annotations.get(n)) for n in inspect.getfullargspec(func).args]

    if suffix is not None:
        name = name + "_" + suffix

    singular_return = len(flatten) == 1

    keys = None
    free_symbols = set()
    def update_free_symbols(v):
        if hasattr(v, 'free_symbols'):
            free_symbols.update({k.__str__() for k in v.free_symbols})
            return

        if isinstance(v, Iterable):
            for v1 in v:
                update_free_symbols(v1)

    type = "CnMat"
    if isinstance(flatten, dict):
        type = flatten["$original"].__class__.__name__
        flatten.pop("$original")
        keys = list(flatten.keys())
        values = [flatten[k] for k in keys]
        keys = [k[1] for k in keys]
        values = [ a.symengine_type() if hasattr(a, 'symengine_type') else a for a in values ]
        cse_output = cse(symengine.Matrix(values))
        update_free_symbols(values)
    else:
        cse_output = cse(symengine.Matrix(flatten))
        update_free_symbols(flatten)

    if singular_return:
        emit_code("static inline FLT %s%s(%s) {" % (prefix, name, ", ".join(map(arg_str, enumerate(args)))))
    else:
        emit_code("static inline void %s%s(%s, %s) {" % (prefix, name, ", ".join([type + "* " + s[0] for s in outputs]), ", ".join(map(arg_str, enumerate(args)))))

    if preamble:
        emit_code(preamble.strip("\r\n"))

    # Unroll struct types
    for idx, a in enumerate(args):
        if callable(a):
            name = get_name(a)
            for k, v in flatten_args(a()):
                if f"{name}{k.strip('[]')}" in free_symbols:
                    emit_code("\tconst FLT %s = %s%s;" % (str(v), "(*"+name+")" if isinstance_namedtuple(a()) else name, k))
        elif isinstance(a, WrapTuple):
            name = get_name(a)
            for k, v in flatten_args(a.t):
                if f"{name}{k.strip('[]')}" in free_symbols:
                    emit_code("\tconst FLT %s = %s%s;" % (str(v), name, k))

    for item in cse_output[0]:
        stripped_line = ccode(item[1]).replace("\n", " ").replace("\t", " ")
        emit_code(f"\tconst FLT {symengine.ccode(item[0])} = {stripped_line};")

    output_idx = 0
    outputs_idx = 0

    count_zeros = 0
    for item_idx, item in enumerate(cse_output[1]):
        if item == 0:
            count_zeros += 1
    needs_set_zero = count_zeros > len(cse_output[1]) / 4

    if keys is None and not singular_return:
        current_shape = outputs[outputs_idx][1] if isinstance(outputs[outputs_idx][1], tuple) else [outputs[outputs_idx][1], 1]
        var = outputs[outputs_idx][0]
        if needs_set_zero:
            emit_code(f"\tcnSetZero({var});")
    for item_idx, item in enumerate(cse_output[1]):
        if keys is None:
            current_shape = outputs[outputs_idx][1] if isinstance(outputs[outputs_idx][1], tuple) else [outputs[outputs_idx][1], 1]
            current_row = output_idx // current_shape[1]
            current_col = output_idx % current_shape[1]

            def get_col_str():
                if len(outputs[outputs_idx]) > 2 and hasattr(outputs[outputs_idx][2][current_col], 'offsetof'):
                    offset_of = outputs[outputs_idx][2][current_col].offsetof()
                    if offset_of is not None:
                        return offset_of
                return str(current_col)
            def get_row_str():
                if input_keys is not None:
                    root, path = input_keys[current_row]
                    return f"offsetof({root}, {path})/sizeof(FLT)"
                return str(current_row)
            if hasattr(item, "tolist"):
                for item1 in sum(item.tolist(), []):
                    emit_code("\tcnMatrixOptionalSet(%s, %s, %s, %s);" % (outputs[outputs_idx][0], get_row_str(), get_col_str(), output_idx, ccode(item1).replace("\n", " ").replace("\t", " ")))
                    output_idx += 1
                    current_row = output_idx / current_shape[1]
                    current_col = output_idx % current_shape[1]
            else:
                if singular_return:
                    emit_code("\treturn %s;" % (ccode(item).replace("\n", " ").replace("\t", " ")))
                else:
                    if item != 0 or not needs_set_zero:
                        emit_code("\tcnMatrixOptionalSet(%s, %s, %s, %s);" % (outputs[outputs_idx][0], get_row_str(), get_col_str(), ccode(item).replace("\n", " ").replace("\t", " ")))
                output_idx += 1
            if output_idx >= math.prod(current_shape) > 0:
                outputs_idx += 1
                output_idx = 0
        else:
            nl = "\n"
            emit_code(f"\tout->{keys[item_idx]}={ccode(item).replace(nl, '')};")

    emit_code("}")
    emit_code("")
    return flatten


def jacobian(v, of):
    of = [ a.symengine_type() if hasattr(a, 'symengine_type') else a for a in of]
    if hasattr(v, 'jacobian'):
        return v.jacobian(symengine.Matrix(of))
    if isinstance(v, np.ndarray):
        v = list(v)
    return Matrix([v]).jacobian(symengine.Matrix(of))

def map_arg(arg):
    if callable(arg):
        return map_arg(arg())
    elif isinstance(arg, list):
        return list(map(map_arg, arg))
    elif isinstance(arg, tuple):
        return tuple(map(map_arg, arg))
    elif isinstance(arg, WrapTuple):
        return np.array([*arg])
    return arg

def flat_values(a):
    if isinstance(a, str):
        return []
    if isinstance(a, WrapArray):
        return [b for b in a]
    if isinstance(a, WrapMember):
        return [a]
    if isinstance(a, Iterable):
        return sum([flat_values(it) for it in a], [])
    if hasattr(a, '__dict__'):
        return flat_values([v for k,v in a.__dict__.items() if not k.startswith("_")])
    return [a]


def generate_jacobians(func, suffix=None,transpose=False,jac_all=False, jac_over=None, argument_specs={}, file=None, prefix="", codegen=generate_ccode):
    def emit_code(*args, **kwargs):
        if file is not None:
            print(*args, **kwargs, file=file)

    rtn = {}

    fx, func_args = flatten_func(func, argument_specs=argument_specs)

    #annotations = inspect.getfullargspec(func).annotations
    #func_args = [get_argument(n, argument_specs, annotations.get(n)) for n in inspect.getfullargspec(func).args]
    jac_of = {}
    if jac_over is not None:
        jac_of[get_name(jac_over)] = flat_values(map_arg(jac_over))
    else:
        jac_args = {get_name(arg): sorted(flat_values(map_arg(arg)), key=str) for arg in func_args}
        jac_of.update(jac_args)

    if jac_all:
        jac_of['all'] = sum(list(jac_of.values()), [])

    feval = (func(*map_arg(func_args)))

    keys = None
    if hasattr(feval, '__dataclass_fields__'):
        dict = dataclass2dictionary(feval)
        #type = dict["$original"].__class__.__name__
        dict.pop("$original")
        keys = list(dict.keys())
        values = [dict[k] for k in keys]
        feval = [ a.symengine_type() if hasattr(a, 'symengine_type') else a for a in values ]

    for name, jac_value in jac_of.items():
        fname = func.__qualname__.replace(".", "_")  + '_jac_' + name.strip('_')
        if type(feval) == list or type(feval) == tuple or type(feval) == np.ndarray:
            feval = make_sympy(feval)
        this_jac = jacobian(feval, jac_value)
        if transpose:
            this_jac = this_jac.transpose()

        jac_shape = this_jac.shape
        jac_size = this_jac.shape[0] * this_jac.shape[1]
        # 
        # if jac_size == 1:
        #     continue

        #emit_code("// Jacobian of", func.__qualname__.replace(".", "_"), "wrt", jac_value)
        codegen(this_jac, fname, func_args, argument_specs=argument_specs, suffix=suffix, outputs=[('Hx', jac_shape, jac_value)], input_keys=keys,file=file, prefix=prefix)

        #jac_with_hx = this_jac.reshape(jac_size, 1).col_join(fxm.reshape(fx_size, 1))

        #emit_code("// Full version Jacobian of", func.__qualname__.replace(".", "_"), "wrt", jac_value)

        fn_suffix = ""
        if suffix is not None:
            fn_suffix = "_" + suffix

        if keys is not None:
            continue

        gen_call = f"{prefix}{func.__qualname__.replace('.', '_')}{fn_suffix}(hx, {generate_args_string(func_args, True)});"
        if len(fx) == 1:
            gen_call = f"hx->data[0] = {prefix}{func.__qualname__.replace('.', '_')}{fn_suffix}({generate_args_string(func_args, True)});"

    #     preamble = f"""
    # if(Hx == 0) {{
    #     {gen_call}
    #     return;
    # }}
    # if(hx == 0) {{
    #     gen_{fname}{fn_suffix}(Hx, {generate_args_string(func_args, True)});
    #     return;
    # }}"""
    #     generate_ccode(jac_with_hx, fname + "_with_hx", func_args, suffix=suffix, outputs=[('Hx', jac_shape, jac_value), ('hx', this_jac.shape[0] - jac_size)], preamble=preamble, file)
        outputs = [('Hx', jac_shape, jac_value), ('hx', this_jac.shape[0] - jac_size)]
        if codegen == generate_ccode:
            emit_code(f"""
    static inline void {prefix}{fname}_with_hx({", ".join(["CnMat* " + s[0] for s in outputs])}, {", ".join(map(arg_str, enumerate(func_args)))}) {{
        if(hx != 0) {{ 
            {gen_call}
        }}
        if(Hx != 0) {{ 
            {prefix}{fname}{fn_suffix}(Hx, {generate_args_string(func_args, True)});
        }}
    }}""")

        rtn[name] = this_jac.reshape(*jac_shape)
    return rtn, func_args

def can_generate_jacobian(f):
    if hasattr(f, 'shape'):
        return f.shape[0] == 1 or f.shape[1] == 1
    if isinstance(f, dict):
        return True
    if isinstance(f, Iterable):
        return all(map(can_generate_jacobian, f))
    return True

def generate_code_and_jacobians(f,transpose=False, jac_all=False, jac_over=None, argument_specs = {}, file=None, prefix="", codegen=generate_ccode):
    f_eval = codegen(f, argument_specs = argument_specs, file=file, prefix=prefix)
    if can_generate_jacobian(f_eval):
        return generate_jacobians(f, argument_specs = argument_specs, transpose=transpose, jac_all=jac_all, jac_over=jac_over, file=file, prefix=prefix, codegen=codegen)
    return None, None

from pathlib import Path

codegen_enable_generate_pyx = False
def enable_generate_pyx():
    global codegen_enable_generate_pyx
    codegen_enable_generate_pyx = True

generate_code_files = {}
def get_pyx_file(fn):
    global codegen_enable_generate_pyx
    force_generate = '--cnkalman-generate-source-pyx' in sys.argv
    if not codegen_enable_generate_pyx and not force_generate:
        return None
    global dirty_python_hack
    dirty_python_hack = True
    if fn != sys.argv[0] and not codegen_enable_generate_pyx:
        return None
    if fn in generate_code_files:
        return generate_code_files[fn]
    path = Path(fn)
    new_stem = f"{path.stem}_gen"
    if path.stem == "__init__":
        new_stem = f"gen"

    full_path = f'{path.parent.as_posix()}/{new_stem}.pyx'
    if os.path.exists(full_path):
        #os.path.getmtime(full_path) > os.path.getmtime(__file__) and \
        if not force_generate and \
                os.path.getmtime(full_path) > os.path.getmtime(fn) and \
                os.path.getmtime(f'{path.parent.as_posix()}/{new_stem}.pxd') > os.path.getmtime(fn):
            return None

    print(f"Generating {path.parent.as_posix()}/{new_stem}.pyx...", file=sys.stderr)
    f = open(f"{path.parent.as_posix()}/{new_stem}.pyx", 'w')
    f.write(
        """# NOTE: This is a generated file; do not edit.
# cython: language_level=3
# cython: boundscheck=False, wraparound=False, initializedcheck=False, cdivision=True, nonecheck=False, overflowcheck=False
# clang-format off
import cython
import numpy as np
cimport numpy as np
from libc.math cimport *
from libc.stdint cimport *
from libc cimport *
from cython cimport floating
""")

    g = open(f"{path.parent.as_posix()}/{new_stem}.pxd", 'w')
    g.write(
        """# NOTE: This is a generated file; do not edit.
# cython: language_level=3
import numpy as np
cimport numpy as np

from cython cimport floating

""")
    generate_code_files[fn] = (f, g)
    return generate_code_files[fn]

def get_file(fn):
    if not '--cnkalman-generate-source' in sys.argv:
        return get_pyx_file(fn)
    if fn != sys.argv[0]:
        return None
    if fn in generate_code_files:
        return generate_code_files[fn]
    path = Path(fn)
    logging.info(f"Generating {path.parent.as_posix()}/{path.stem}.gen.h...", file=sys.stderr)
    f = generate_code_files[fn] = open(f"{path.parent.as_posix()}/{path.stem}.gen.h", 'w')
    f.write(
    """/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
""")
    return generate_code_files[fn]

import numpy as np
def functionify(args_info, jac):
    def f(*args, force_resolve=True):
        subset = {}
        for i, info in enumerate(args_info):
            if isinstance(info, WrapTuple):
                for j, s in enumerate(info.t):
                    v = args[i][j]
                    subset[s.__str__()] = v
            else:
                subset[info.__str__()] = args[i]
        rtn = np.array(jac.subs(subset))
        if force_resolve:
            return rtn.astype(np.float64)
        return rtn
    return f

def expand_hint(v, length):
    return [v[a] for a in range(length)]

def nan_if_complex(x):
    if hasattr(x , 'is_real') and x.is_real == False:
        return np.nan
    return float(x)

def has_free_symbols(x):
    if isinstance(x, Iterable):
        return any([has_free_symbols(y) for y in x])
    if hasattr(x, 'free_symbols'):
        return len(x.free_symbols) > 0
    return False


class LazyObject:
    def __init__(self, f):
        self.v = None
        self.eval = False
        self.f = f

    def __call__(self):
        if not self.eval:
            self.v = self.f()
        return self.v

class Jacobians(object):
    def __init__(self, func, prefix, kwargs):
        self.func = func
        self._jacs = None
        self._args = None
        self._evaled = False
        self.prefix = prefix
        self.kwargs = kwargs        
        self.f = get_file(inspect.getfile(func))
        if self.f is not None:
            self.eval()
            
    def eval(self, reeval=False):
        if self._evaled and not reeval:
            return
        self._evaled = True
        func = self.func
        f = self.f

        global use_symbolic_eval
        old_use_symbolic_eval = use_symbolic_eval
        use_symbolic_eval = True
        is_pyx = isinstance(f, tuple)
        jacs, args = generate_code_and_jacobians(func, argument_specs=self.kwargs, jac_all=self.kwargs.get('_all', False),
                                                 file=f, prefix=self.prefix, codegen = generate_pyxcode if is_pyx else generate_ccode)
        use_symbolic_eval = old_use_symbolic_eval
        self._jacs = jacs
        self._args = args

        if jacs is not None:
            for k, v in jacs.items():
                setattr(self, k, functionify(args, v))

    def __getattr__(self, item):
        self.eval()
        return super().__getattribute__(item)

def generate_code(prefix="", **kwargs):
    def f(func):
        try:            
            def g(*args, **g_kwargs):
                grtn = func(*args, **g_kwargs)
                if type(grtn) == symengine.MutableDenseMatrix:
                    return grtn
                if isinstance(grtn, Iterable) and not has_free_symbols(grtn):
                    return np.array([nan_if_complex(x) for x in grtn], dtype=np.float64)
                return grtn
            try:
                g.jacobians = Jacobians(func, prefix, kwargs)
            except TypeError as e:
                # This happens if it is compiled code
                pass
            return g        
        except Exception as e:
            logging.warning(f"Could not generate source level jacobian for {func.__qualname__}: {e}")
            return func
    return f
