"""Read the RF-off L80O diagnostic. No live or analog settling claims."""
import argparse
import hashlib
import json
import struct
from pathlib import Path
from analyze_issue11_output import fnv


def analyze(path, offset=0):
    data=path.read_bytes()[offset:]
    if len(data)<256: raise ValueError('truncated L80O header')
    h=struct.unpack('<64I',data[:256])
    if h[:3]!=(0x4f30384c,1,256): raise ValueError('not an L80O v1 record')
    if h[3:5]!=(16384,32768) or h[15]!=32832:
        raise ValueError('unsupported oracle geometry')
    if len(data)<256+h[15]+h[4]: raise ValueError('truncated payload')
    actual=data[256:256+h[15]]
    expected=data[256+h[15]:256+h[15]+h[4]]
    if fnv(actual)!=h[10] or fnv(expected)!=h[11]: raise ValueError('payload hash mismatch')
    count=min(h[5],h[4],h[15])
    differences=[i for i in range(count) if actual[i]!=expected[i]]
    mismatch=len(differences)+h[4]-count
    if h[6]==0 and mismatch!=h[7]: raise ValueError('firmware mismatch count disagrees with payload')
    rows=[]
    for i in range(8):
        rate,n,elapsed,status,error=h[16+i*5:21+i*5]
        rows.append(dict(rate_hz=rate,input_bytes=n,elapsed_us=elapsed,
                         midstream_irq=status,fifo_read_empty=bool(status&1),error=error))
    slopes=[]
    for i in (0,2,4,6):
        small,large=rows[i:i+2]
        if small['error']!=0 or large['error']!=0:
            slopes.append(dict(rate_hz=small['rate_hz'],incremental_output_MBps=None,
                               within_15_percent=False,reason='transfer failed or was not executed'))
            continue
        dt=large['elapsed_us']-small['elapsed_us']
        measured=2*(large['input_bytes']-small['input_bytes'])/dt if dt>0 else None
        target=large['rate_hz']/1e6
        slopes.append(dict(rate_hz=large['rate_hz'],incremental_output_MBps=measured,
                           within_15_percent=measured is not None and abs(measured/target-1)<.15))
    passed=(h[6]==0 and mismatch==0 and h[12]==0 and h[13]==0
            and all(r['error']==0 and not r['fifo_read_empty'] for r in rows)
            and all(s['within_15_percent'] for s in slopes))
    return dict(evidence='RF-off full-byte BitScrambler loopback and finite decorated TX timing; not live RX/TX or analog settling',
                sha256=hashlib.sha256(data).hexdigest(), cpu_hz=h[14],
                loop_error=h[6],bytes_written=h[5],expected_bytes=h[4],
                mismatches=mismatch,first_mismatches=differences[:32],
                differing_received_bytes=len(differences),missing_bytes=h[4]-count,
                trailing_bytes=max(0,h[5]-h[4]),
                timing=rows,incremental_timing=slopes,
                basic_hardware_gate_pass=passed)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    p.add_argument('--offset',type=lambda s:int(s,0),default=0,
                   help='record offset within flash dump (e.g. 0x1000 after trace partition)')
    a=p.parse_args()
    print(json.dumps(analyze(a.capture,a.offset),indent=2))
