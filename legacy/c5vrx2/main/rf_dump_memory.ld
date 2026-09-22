/* Keep static application state below both banks ceded to MAC_DUMP_ALLOC. */
ASSERT(_bss_end <= 0x4082ffc0,
       "C5VRX-2: static DRAM overlaps MAC dump banks 0x4082ffc0..0x4085003f")
