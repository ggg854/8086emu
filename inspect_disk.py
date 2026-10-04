import struct, os

def inspect(path):
    print('===' * 3, path)
    if not os.path.exists(path):
        print('  MISSING')
        return
    d = open(path, 'rb').read()
    print('  size', len(d), '=', len(d) // 1024, 'K')
    if len(d) < 512:
        return
    if d[510:512] != b'\x55\xaa':
        print('  no boot signature at offset 0 -> not an MBR/boot sector')
        return

    for i in range(4):
        e = d[0x1BE + i * 16: 0x1BE + i * 16 + 16]
        typ = e[4]
        lba = struct.unpack('<I', e[8:12])[0]
        nsec = struct.unpack('<I', e[12:16])[0]
        if typ == 0 or nsec == 0:
            continue
        print('  part%d: type=%02X lba=%d sectors=%d' % (i, typ, lba, nsec))
        off = lba * 512
        bs = d[off:off + 512]
        if bs[510:512] != b'\x55\xaa':
            print('    partition boot sector has no 55AA')
            continue
        bps = struct.unpack('<H', bs[11:13])[0]
        spc = bs[13]
        res = struct.unpack('<H', bs[14:16])[0]
        nf = bs[16]
        root = struct.unpack('<H', bs[17:19])[0]
        fsz = struct.unpack('<H', bs[22:24])[0]
        print('    BPB: bps=%d sec/clus=%d reserved=%d numFAT=%d rootent=%d fatsec=%d'
              % (bps, spc, res, nf, root, fsz))
        root_off = off + (res + nf * fsz) * bps
        names = []
        for k in range(root):
            ent = d[root_off + k * 32: root_off + k * 32 + 32]
            if ent[0] == 0:
                break
            if ent[11] == 0x0F:
                continue
            nm = ent[0:8].decode('latin1').rstrip()
            ext = ent[8:11].decode('latin1').rstrip()
            names.append(nm + ('.' + ext if ext else ''))
        print('    root entries:', names[:50])

for p in [r'd:\dos86\data\dos.img', r'd:\dos86\dos.img', r'd:\dos86\disk.img']:
    inspect(p)
