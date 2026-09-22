"""Analyze RF-off six-DAC-pad replay captures; RX40 undersamples TX80."""
import argparse
import json
import struct
from math import gcd
from pathlib import Path
from analyze_issue11_output import fnv
from bs_model import simulate


def compare(capture, template, step):
    # Search every cyclic phase; require an exact 16-sample signature first.
    n=len(template)
    candidates=[i for i in range(n) if all(
        capture[j]==template[(i+j*step)%n] for j in range(min(16,len(capture))))]
    if not candidates:
        return dict(aligned=False, exact=False, reason='no exact 16-sample cyclic signature')
    mismatch,phase=min((sum(v!=template[(i+j*step)%n] for j,v in enumerate(capture)),i)
                       for i in candidates)
    return dict(aligned=True, exact=mismatch==0, mismatch=mismatch, phase=phase,
                signature_candidates=len(candidates))


def compare_rates(capture, template, tx_hz, rx_hz):
    divisor=gcd(tx_hz,rx_hz)
    numerator,denominator=tx_hz//divisor,rx_hz//divisor
    if denominator==1: return compare(capture,template,numerator)
    n=len(template)
    candidates=[]
    # Only clock-ratio-consistent sampling phases; no arbitrary drops/warps.
    for fraction in range(denominator):
        offsets=[(j*numerator+fraction)//denominator for j in range(len(capture))]
        for phase in range(n):
            if all(capture[j]==template[(phase+offsets[j])%n] for j in range(min(16,len(capture)))):
                mismatch=sum(v!=template[(phase+offsets[j])%n] for j,v in enumerate(capture))
                candidates.append((mismatch,phase,fraction))
    if not candidates: return dict(aligned=False,exact=False,reason='no exact clock-ratio signature')
    mismatch,phase,fraction=min(candidates)
    return dict(aligned=True,exact=mismatch==0,mismatch=mismatch,phase=phase,
                fractional_phase=fraction,ratio=[numerator,denominator])


def analyze(path):
    data=path.read_bytes()
    if len(data)<4160: raise ValueError('truncated capture')
    h=struct.unpack('<16I',data[:64])
    if h[:4]!=(0x5044384c,1,64,4096): raise ValueError('not L8DP v1')
    if h[4] not in (40000000,48000000,60000000,80000000) or h[5]!=40000000:
        raise ValueError('unsupported requested rates')
    capture=data[64:4160]
    if fnv(capture)!=h[12]: raise ValueError('capture hash mismatch')
    raw=bytes((i*73+(i>>3)*29+(i>>7)*11+17)&255 for i in range(16384))
    if fnv(raw)!=h[13]: raise ValueError('input pattern hash mismatch')
    mode=h[14]
    if mode not in (0,1,2): raise ValueError('unsupported replay mode')
    if mode==1:
        template=[v&63 for v in raw]
    else:
        name='c5vrx2_phase5_linear80.bsasm' if mode==0 else 'c5vrx2_wbfm_q4_phase5_2to1.bsasm'
        source=(Path(__file__).resolve().parents[1]/'main'/name).read_text()
        period=32768 if mode==0 else 16384
        # Second input period retains discriminator/interpolator state.
        template=simulate(source,raw*3,period*2)[period:period*2]
    valid=all(h[i]==0 for i in (6,7,8))
    return dict(evidence='RF-off repeating input, physical DAC digital pad readback; not RF or analog settling',
                requested_tx_hz=h[4],requested_rx_hz=h[5],
                replay_mode=['linear80','direct','phase5'][mode],
                tx_error=h[6],rx_error=h[7],result=h[8],rx_elapsed_us=h[10],
                irq_before=h[9],irq_after=h[11],unique_codes=len(set(v&63 for v in capture)),
                tx_status=h[15],
                comparison=compare_rates([v&63 for v in capture],template,h[4],h[5]) if valid else None,
                limitation='RX40 undersamples faster TX; rates/phase must be verified independently')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    print(json.dumps(analyze(p.parse_args().capture),indent=2))
