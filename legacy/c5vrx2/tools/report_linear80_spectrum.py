"""Synthetic reconstruction comparison, not an RF/physical measurement."""
import json
from pathlib import Path
import numpy as np


def main():
    # Coherent tone near NTSC burst; no FFT-window leakage at the compared bins.
    n,k=4096,733
    fs=80e6
    x=np.rint(20+8*np.cos(2*np.pi*k*np.arange(n)/n)).astype(int)
    old=np.repeat(x,4)
    prev=np.roll(x,1)
    new=np.c_[prev,(3*prev+x)//4,(prev+x)//2,(prev+3*x)//4].ravel()
    def stats(y):
        spec=2*np.abs(np.fft.rfft(y-y.mean()))/len(y)
        return dict(mean_codes=float(y.mean()),fundamental_peak_codes=float(spec[k]),
                    first_image_dBc=float(20*np.log10(spec[n-k]/spec[k])))
    held,linear=stats(old),stats(new)
    report=dict(evidence='synthetic 6-bit coherent tone; no hardware/RF claim',
                tone_hz=k*20e6/n, held_20Ms=held, linear_80Ms=linear,
                extra_image_suppression_dB=held['first_image_dBc']-linear['first_image_dBc'],
                fundamental_change_dB=float(20*np.log10(linear['fundamental_peak_codes']/held['fundamental_peak_codes'])))
    out=Path(__file__).resolve().parents[1]/'measurements/pr16-linear80'
    out.mkdir(parents=True,exist_ok=True)
    (out/'synthetic-spectrum.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,7))
    for label,y in [('20 MS/s held values',old),('80 MS/s four-point candidate',new)]:
        axes[0].step(np.arange(100)*12.5,y[:100],where='post',label=label)
        spec=2*np.abs(np.fft.rfft(y-y.mean()))/len(y)
        axes[1].plot(np.fft.rfftfreq(len(y),1/fs)/1e6,20*np.log10(np.maximum(spec,1e-9)/spec[k]),lw=.6,label=label)
    axes[0].set(xlabel='Time (ns)',ylabel='DAC code',title='Synthetic reconstruction only; real DAC settling is unmeasured')
    axes[1].set(xlabel='Frequency (MHz)',ylabel='Amplitude (dBc)',ylim=(-75,5),xlim=(0,40))
    for ax in axes: ax.legend(); ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'synthetic-spectrum.png',dpi=150);plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
