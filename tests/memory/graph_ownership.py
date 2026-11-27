"""Resolve actual TAPA invoke array slices into per-FIFO owners, fail closed."""
from collections import defaultdict
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


def kernel_constants():
    """Read the PE/PEG/front counts from src/lu_kernel.h instead of hardcoding."""
    header = (ROOT / 'src/lu_kernel.h').read_text()
    values = {}
    for name in ('PAR_NUM_PEG', 'PAR_PE_PER_PEG', 'PAR_NUM_FRONT'):
        match = re.search(rf'constexpr\s+int\s+{name}\s*=\s*(\d+)\s*;', header)
        if not match: raise AssertionError(f'cannot resolve {name} in lu_kernel.h')
        values[name] = int(match.group(1))
    if not re.search(r'constexpr\s+int\s+PAR_NUM_PE\s*=\s*PAR_NUM_PEG\s*\*\s*PAR_PE_PER_PEG\s*;', header):
        raise AssertionError('PAR_NUM_PE is no longer PAR_NUM_PEG * PAR_PE_PER_PEG')
    values['PAR_NUM_PE'] = values['PAR_NUM_PEG'] * values['PAR_PE_PER_PEG']
    return values


CONSTANTS = kernel_constants()


def number(value):
    return CONSTANTS[value] if value in CONSTANTS else int(value)


def arguments(text):
    depth = 0
    start = 0
    result = []
    for i, char in enumerate(text):
        if char in '(<': depth += 1
        if char in ')>': depth -= 1
        if char == ',' and depth == 0:
            result.append(text[start:i].strip())
            start = i+1
    return result + [text[start:].strip()]


def owners():
    source = (ROOT / 'src/lu_kernel.cpp').read_text()
    definitions = source + '\n' + '\n'.join(path.read_text() for path in (ROOT / 'src').glob('salut_*.inc'))
    top = source[source.rindex('void SparseLUKernel('):]
    lengths = {}
    for match in re.finditer(r'tapa::(stream|streams)<(.+?)>\s+(\w+)\(', top):
        kind, spec, name = match.groups()
        lengths[name] = number(arguments(spec)[1]) if kind == 'streams' else 1
    ends = {f'{name}[{i}]': {} for name, length in lengths.items() for i in range(length)}
    cursors = defaultdict(int)
    instances = defaultdict(int)
    for call in re.finditer(r'\.invoke<([^>]+)>\(', top):
        start = call.end()
        level, end = 1, start
        while level:
            if top[end] == '(': level += 1
            if top[end] == ')': level -= 1
            end += 1
        actual = arguments(top[start:end-1])
        task, actual = actual[0], actual[1:]
        declaration = re.search(rf'void\s+{task}\((.*?)\)\s*\{{', definitions, re.S)
        if not declaration: raise AssertionError(f'unknown task signature {task}')
        formal = arguments(declaration.group(1))
        if len(formal) != len(actual): raise AssertionError(f'{task}: arguments {len(actual)} != {len(formal)}')
        mode = arguments(call.group(1))
        count = number(mode[1]) if len(mode) > 1 else 1
        for _ in range(count):
            instance = f'{task}_{instances[task]}'
            instances[task] += 1
            for port, value in zip(formal, actual):
                signature = re.search(r'tapa::([io]streams?)<(.+?)>\s*&', port)
                if not signature: continue  # Scalars and external mmap.
                stream_type, spec = signature.groups()
                direction = 'writer' if stream_type.startswith('o') else 'reader'
                param_count = number(arguments(spec)[1]) if stream_type.endswith('s') else 1
                named = re.fullmatch(r'(\w+)(?:\[(\d+)\])?', value)
                if not named: raise AssertionError(f'{task}: unresolved stream {value}')
                name, explicit = named.groups()
                if name not in lengths: raise AssertionError(f'undeclared stream {name}')
                index = int(explicit) if explicit is not None else cursors[(name, direction)]
                for i in range(index, index+param_count):
                    edge = f'{name}[{i}]'
                    if edge not in ends: raise AssertionError(f'out of range {edge} at {instance}')
                    if direction in ends[edge]: raise AssertionError(f'duplicate {direction} {edge}')
                    ends[edge][direction] = instance
                if explicit is None: cursors[(name, direction)] += param_count
    for edge, endpoints in ends.items():
        if set(endpoints) != {'reader', 'writer'}: raise AssertionError(f'missing endpoint {edge}: {endpoints}')
    return ends


if __name__ == '__main__':
    print('fifo\twriter\treader')
    for fifo, ports in sorted(owners().items()):
        print(f'{fifo}\t{ports["writer"]}\t{ports["reader"]}')
