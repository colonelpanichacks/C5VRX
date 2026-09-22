import struct
import tempfile
import unittest
from pathlib import Path
from analyze_linear80_oracle import analyze
from analyze_issue11_output import fnv


class OracleParserTests(unittest.TestCase):
    def fixture(self, fifo_error=False, corrupt=False, slow=False):
        actual=bytes(32832);expected=bytes(32768)
        h=[0]*64
        h[:9]=[0x4f30384c,1,256,16384,32768,32768,0,0,0xffffffff]
        h[10]=fnv(actual);h[11]=fnv(expected);h[14]=160000000;h[15]=32832
        for i in range(8):
            rate=40000000 if i<4 else 80000000
            n=4096 if i%2==0 else 16384
            dt=500+round(2*n*1e6/rate)*(2 if slow else 1)
            h[16+i*5:21+i*5]=[rate,n,dt,int(fifo_error),0]
        if corrupt: actual=b'\xff'+actual[1:]
        return struct.pack('<64I',*h)+actual+expected

    def test_clean_fixture_and_fifo_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'fixture.bin'
            p.write_bytes(self.fixture())
            self.assertTrue(analyze(p)['basic_hardware_gate_pass'])
            p.write_bytes(self.fixture(fifo_error=True))
            self.assertFalse(analyze(p)['basic_hardware_gate_pass'])
            p.write_bytes(self.fixture(slow=True))
            self.assertFalse(analyze(p)['basic_hardware_gate_pass'])

    def test_corruption_and_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'fixture.bin'
            for data in [self.fixture(corrupt=True),self.fixture()[:-1],bytes(20)]:
                p.write_bytes(data)
                with self.assertRaises(ValueError): analyze(p)

    def test_hardware_short_output_and_timeout(self):
        data=bytearray(self.fixture())
        struct.pack_into('<I',data,5*4,32764)
        struct.pack_into('<I',data,7*4,4)
        struct.pack_into('<I',data,20*4,0x107)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'flash.bin'
            p.write_bytes(bytes(4096)+data)
            result=analyze(p,4096)
            self.assertEqual(result['missing_bytes'],4)
            self.assertEqual(result['differing_received_bytes'],0)
            self.assertIsNone(result['incremental_timing'][0]['incremental_output_MBps'])
            self.assertFalse(result['basic_hardware_gate_pass'])


if __name__=='__main__': unittest.main()
