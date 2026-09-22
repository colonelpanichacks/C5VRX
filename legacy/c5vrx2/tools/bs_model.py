"""Source-driven C5 BitScrambler dataflow model, not a timing/EOF model.

Uses physically established C5 LUT indexing at O16 (the generic prose about
the most significant bits does not describe the measured C5 16-bit mapping).
"""
import re


def parse(text):
    cfg, lut, blocks, labels = {}, [], [], {}
    current = []
    for line in text.splitlines():
        line = line.split('#')[0].strip().lower()
        if not line: continue
        if line.startswith('cfg '):
            _, k, v = line.split(); cfg[k] = v; continue
        if line.startswith('lut '):
            lut.extend(int(n,0) for n in re.split(r'[ ,]+',line[4:])); continue
        if line.endswith(':'):
            labels[line[:-1]] = len(blocks); continue
        current.append(line.rstrip(','))
        if not line.endswith(','):
            blocks.append(current); current=[]
    if current: raise ValueError('unterminated bundle')
    width = int(cfg.get('lut_width_bits',32))
    if len(lut)*width > 16384: raise ValueError('C5 LUT exceeds 2048 bytes')
    lut += [0]*((16384//width)-len(lut))
    return cfg, lut, blocks, labels


def simulate(text, raw, count, *, initial=None, start=None):
    cfg, lut, blocks, labels = parse(text)
    out = a = b = look = pos = pc = 0
    if initial is not None: out, a, b, look = initial
    if start is not None: pc = labels[start]
    result = []
    def expand(token):
        if '..' not in token: return [token]
        lo, hi = token.split('..')
        prefix = lo[0] if lo[0].isalpha() else ''
        return [prefix+str(i) for i in range(int(lo[len(prefix):]),int(hi[len(prefix):])+1)]
    for _ in range((count+32)*32):
        new = 0; read = write = 0; opcode = ['nop']
        used = set()
        for line in blocks[pc]:
            bits = line.split()
            if bits[0] == 'set':
                dst, src = expand(bits[1]), expand(bits[2])
                if len(src) == 1: src *= len(dst)
                if len(dst) != len(src): raise ValueError('mapping length')
                for d,s in zip(dst,src):
                    d=int(d)
                    if d in used: raise ValueError('duplicate output assignment')
                    used.add(d)
                    if s in ('l','h'): v = s == 'h'
                    elif s[0] in 'olab':
                        v=({'o':out,'l':look,'a':a,'b':b}[s[0]] >> int(s[1:])) & 1
                    else:
                        bit=int(s); ix=pos+bit//8
                        v=(int(raw[ix]) >> (bit%8)) & 1 if ix < len(raw) else 0
                    new |= int(v)<<d
            elif bits[0] == 'read': read=int(bits[1])
            elif bits[0] == 'write': write=int(bits[1])
            else: opcode=bits
        op = opcode[0]
        next_pc=pc+1
        if op == 'jmp': next_pc=labels[opcode[1]]
        elif op.startswith(('ldcti','addcti','ldctd')):
            kind = 'addcti' if op.startswith('addcti') else 'ldcti' if op.startswith('ldcti') else 'ldctd'
            suffix=op[len(kind):]; ctr=suffix[0]; half=suffix[1:]
            old=a if ctr=='a' else b
            operand=(new>>16)&65535 if kind!='ldctd' else int(opcode[1],0)
            value=old+operand if kind=='addcti' else operand
            if half=='l': value=(old&0xff00)|(value&255)
            elif half=='h': value=(old&255)|(value&0xff00)
            else: value &= 65535
            if ctr=='a': a=value
            else: b=value
        elif op != 'nop': raise ValueError(f'unsupported opcode {op}')
        out=new
        look=lut[(out>>16)&(len(lut)-1)]
        pos+=read//8
        result.extend((out>>(8*j))&255 for j in range(write//8))
        if len(result)>=count: return result[:count]
        pc=next_pc
        if pc>=len(blocks): raise ValueError('fell off ROM')
    raise ValueError('no progress')
