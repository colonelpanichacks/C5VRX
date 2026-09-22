#!/usr/bin/env python3
"""Host oracle for issue 9. Does not emulate peripheral/FIFO timing or EOF."""
import ctypes
import re
import subprocess
import tempfile
from pathlib import Path
import numpy as np
from train_trajectory_lut import P4, words, accumulate, prior_groups, select, observations
from golden_demod import demod_production

ROOT = Path(__file__).resolve().parents[1]


def parse_asm():
    text = (ROOT / 'main/c5vrx2_wbfm_q4_trajectory_2to1.bsasm').read_text()
    lut = np.array([int(n) for n in re.search(r'^lut (.*)$', text, re.M)[1].split()], dtype=np.uint16)
    blocks = {}
    for line in text.splitlines():
        line = line.split('#')[0].strip().rstrip(',')
        if not line or line.startswith(('cfg ', 'lut ')):
            continue
        if line.endswith(':'):
            name = line[:-1]; blocks[name] = []
        else:
            blocks[name].append(line)
    assert len(blocks) == 3
    assert blocks['address_phase'][-1] == 'read 16'
    assert blocks['emit'][-3:] == ['read 16', 'write 8', 'jmp address_trajectory']
    assert all(not x.startswith(('read ', 'write ', 'jmp ')) for x in blocks['address_trajectory'])
    # This validates the real set statements, not a duplicate address formula.
    def expand(token):
        ends = token.split('..')
        prefix = ends[0][0] if ends[0][0] in 'OL' and len(ends[0]) > 1 else ''
        if token == 'L': return ['L']
        start = int(ends[0][len(prefix):])
        stop = int(ends[-1][len(prefix):])
        return [prefix + str(i) for i in range(start, stop+1)]
    compiled = {}
    for name, lines in blocks.items():
        ops = []
        used = set()
        for line in lines:
            if not line.startswith('set '): continue
            _, dst, src = line.split()
            ds, ss = expand(dst), expand(src)
            if src == 'L': ss *= len(ds)
            assert len(ds) == len(ss)
            for d, s in zip(ds, ss):
                d = int(d)
                assert d not in used; used.add(d)
                ops.append((d, s))
        compiled[name] = ops
    return lut, compiled


def simulate(raw, lut, blocks):
    outreg = 0; look = 0; pos = 0; output = []
    def step(name):
        nonlocal outreg, look, pos
        inp = int(raw[pos]) | (int(raw[pos+1]) << 8) if pos+1 < len(raw) else 0
        result = 0
        for dst, src in blocks[name]:
            if src == 'L': value = 0
            elif src[0] == 'O': value = (outreg >> int(src[1:])) & 1
            elif src[0] == 'L': value = (look >> int(src[1:])) & 1
            else: value = (inp >> int(src)) & 1
            result |= value << dst
        outreg = result
        look = int(lut[(result >> 16) & 1023])
        if name in ('address_phase', 'emit'): pos += 2
    step('address_phase')
    for _ in range(len(raw)//2):
        step('address_trajectory'); step('emit'); output.append(outreg & 255)
    return np.array(output, dtype=np.uint8)


def main():
    lut, asm = parse_asm()
    prior, _ = select(accumulate(prior_groups()))
    header = (ROOT / 'main/trajectory_lut.h').read_text().split('{', 1)[1].split('}', 1)[0]
    c_words = np.array([int(n) for n in re.findall(r'\d+', header)], dtype=np.uint16)
    assert np.array_equal(lut, c_words), 'assembly/C LUT mismatch'
    dac = lut & 63
    assert np.array_equal(lut, words(dac)), 'phase4/high-bit layout mismatch'
    is_prior = np.array_equal(dac, prior)
    assert len(np.unique(dac)) > 7
    rng = np.random.default_rng(9)
    raw = rng.integers(0, 256, 32772, dtype=np.uint8)
    with tempfile.TemporaryDirectory() as tmp:
        lib = Path(tmp) / 'reference.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                        str(ROOT/'main/trajectory_reference.c'), '-o', str(lib)], check=True)
        fn = ctypes.CDLL(str(lib)).c5vrx2_trajectory_reference
        ptr = ctypes.POINTER(ctypes.c_uint8)
        fn.argtypes = [ptr, ctypes.c_size_t, ptr, ctypes.c_size_t, ptr]
        fn.restype = ctypes.c_size_t
        def run(data, state, capacity=None):
            n = len(data)//2 if capacity is None else capacity
            out = np.zeros(max(n, 1), dtype=np.uint8)
            size = fn(data.ctypes.data_as(ptr), len(data), out.ctypes.data_as(ptr), n, ctypes.byref(state))
            return out[:size]
        state = ctypes.c_uint8(0)
        expected = run(raw, state)
        assert np.array_equal(simulate(raw, lut, asm), expected)
        # Actual 16 KiB ring-size boundaries and small pairs preserve history.
        state = ctypes.c_uint8(0)
        chunks = [raw[:2], raw[2:16384], raw[16384:32768], raw[32768:]]
        assert np.array_equal(np.concatenate([run(c, state) for c in chunks]), expected)
        assert len(run(raw[:3], ctypes.c_uint8(0))) == 1
        assert len(run(raw, ctypes.c_uint8(0), 0)) == 0
        assert fn(None, 2, None, 1, None) == 0
        # Exhaustive raw triples, excluding startup (same raw input parity).
        total = hard_old = hard_new = 0; old_sum = new_sum = 0
        for p, m, c in prior_groups():
            a, target, _, _ = observations(p, m, c)
            stream = np.column_stack((np.zeros(len(p), dtype=np.uint8),p,m,c)).astype(np.uint8).reshape(-1)
            actual = run(stream, ctypes.c_uint8(0))[1::2]
            assert np.array_equal(actual, dac[a])
            baseline = demod_production(stream)[::2]
            old = np.abs(baseline.astype(int)-target)
            new = np.abs(actual.astype(int)-target)
            total += len(target); hard_old += int((old>=16).sum()); hard_new += int((new>=16).sum())
            old_sum += int(old.sum()); new_sum += int(new.sum())
        if is_prior:
            assert hard_new < hard_old
        print(f'Exhaustive {total} triples: phase5 MAE={old_sum/total:.3f}, hard>=16={hard_old/total:.3%}; '
              f'trajectory MAE={new_sum/total:.3f}, hard>=16={hard_new/total:.3%}')
    print(f'PASS: assembly dataflow == compiled C; ring/chunk state; two bundles, read16/write8; {len(np.unique(dac))} DAC levels.')
    print('Uniform geometry is not independent RF validation. Hardware FIFO/EOF/rate/video-lock proof pending.')

if __name__ == '__main__': main()
