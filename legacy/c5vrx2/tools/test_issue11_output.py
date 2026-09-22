import struct
import tempfile
import unittest
from pathlib import Path
import numpy as np
from analyze_issue11_output import align, fnv, load, measure


class OutputMeasurementTests(unittest.TestCase):
    def test_alignment_crosses_replay_seam(self):
        ref = np.random.default_rng(11).integers(0,64,16384,dtype=np.uint8)
        data = ref[(np.arange(12000)+13000)%16384].copy()
        self.assertEqual(align(data, ref)[:2], (13000,0))
        data[405] ^= 1
        self.assertEqual(align(data, ref)[:2], (13000,1))

    def test_known_sync_and_burst(self):
        fs, frequency, amplitude, phase = 20e6, 3579545.454545, 8, .3
        x = np.full(12000, 30.)
        for start in range(100,11500,1280):
            x[start:start+94] = 0
            a,b = start+105,min(start+165,len(x))
            x[a:b] += amplitude*np.cos(2*np.pi*frequency*np.arange(a,b)/fs+phase)
        rows, _, _ = measure(x,0,16384,fs,threshold=15)
        self.assertGreaterEqual(len(rows),8)
        for row in rows:
            if row['line_length_us'] is not None:
                self.assertAlmostEqual(row['line_length_us'],64,places=7)
            self.assertAlmostEqual(row['ntsc_burst_peak_codes'],amplitude,places=7)
            self.assertAlmostEqual(row['ntsc_burst_global_phase_deg'],np.degrees(phase),places=7)

    def test_no_sync_is_not_a_measurement(self):
        rows, _, _ = measure(np.full(12000,20.),0,16384,20e6)
        self.assertEqual(rows,[])

    def test_capture_validation(self):
        raw, out = bytes(range(256)),bytes(range(64))
        h = [0x31314f43,1,64,len(raw),len(out),40000000,20000000,0,0,0,0,0,fnv(raw),fnv(out),0,0]
        blob = struct.pack('<16I',*h)+raw+out
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'capture.bin'
            path.write_bytes(blob)
            _, acquired, actual = load(path)
            self.assertEqual(acquired.tobytes(),raw)
            self.assertEqual(actual.tobytes(),out)
            for bad in [blob[:63], blob[:-1], blob[:-1]+b'\xff']:
                path.write_bytes(bad)
                with self.assertRaises(ValueError): load(path)
            h[9] = 0xffffffff
            path.write_bytes(struct.pack('<16I',*h)+raw+out)
            with self.assertRaisesRegex(ValueError,'hardware acquisition failed'): load(path)


if __name__ == '__main__': unittest.main()
