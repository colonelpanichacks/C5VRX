"""Verify actual 80 MS/s candidate assembly; not hardware timing evidence."""
from pathlib import Path
import random
from bs_model import parse, simulate
from validate_phase5_quality import build_lut, phase5_state

ROOT = Path(__file__).resolve().parents[1]


def interpolate(a,b):
    return [a,(3*a+b)//4,(a+b)//2,(a+3*b)//4]


def main():
    text=(ROOT/'main/c5vrx2_phase5_linear80.bsasm').read_text()
    cfg,lut,blocks,labels=parse(text)
    assert lut==build_lut(), 'must preserve Phase5 centroid/calibration table'
    assert len(blocks)==7
    for a in range(64):
        for b in range(64):
            got=simulate(text,[0]*16,4,initial=(a,0,0,b),start='load_sums')
            assert got==interpolate(a,b), (a,b,got)
            assert all(min(a,b)<=v<=max(a,b) for v in got)
    rng=random.Random(16)
    raw=[rng.randrange(256) for _ in range(65540)]
    expected=[]; prev_phase=prev_dac=0
    for packed in raw[1::2]:
        phase=phase5_state(packed)
        dac=lut[(prev_phase<<5)|phase]&63
        expected.extend(interpolate(prev_dac,dac))
        prev_phase,prev_dac=phase,dac
    got=simulate(text,raw,len(expected))
    assert got==expected, next((i for i,(a,b) in enumerate(zip(got,expected)) if a!=b),None)
    assert len(set(got))==64
    print('PASS: all 4096 endpoint pairs, exact source-mapped quarter/midpoint arithmetic')
    print('PASS: 131080 output bytes, retained state across four 16-KiB boundaries')
    print('PASS: unchanged 2048-byte Phase5 LUT, all 64 interpolated output levels')
    print('EXPERIMENTAL: six bundles/2 IQ bytes/4 DAC bytes; physical 80 MS/s not yet proven')


if __name__=='__main__': main()
