import struct, os

SRC = r'd:\dos86\disk.img'
DST = r'd:\dos86\disk_ega.img'
COM = r'd:\dos86\data\EGA.COM'

data = open(COM, 'rb').read()
print('EGA.COM size:', len(data))

NAME = b'EGA     COM'   # 8 + 3 = 11 字节


def dir_entry(cluster, size):
    e = bytearray(32)
    e[0:11] = NAME
    e[11] = 0x20                      # ATTR_ARCHIVE
    struct.pack_into('<H', e, 26, cluster)
    struct.pack_into('<I', e, 28, size)
    return e


# ============================================================
# 1) 注入硬盘镜像副本（FAT16）
# ============================================================
d = bytearray(open(SRC, 'rb').read())
part_off = 17 * 512
bs = d[part_off:part_off + 512]
bps = struct.unpack('<H', bs[11:13])[0]
spc = bs[13]
res = struct.unpack('<H', bs[14:16])[0]
nf = bs[16]
rootent = struct.unpack('<H', bs[17:19])[0]
fsz = struct.unpack('<H', bs[22:24])[0]
fat_off = part_off + res * bps
root_off = part_off + (res + nf * fsz) * bps
data_off = root_off + rootent * 32
print('HD FAT16: bps=%d sec/clus=%d fat_off=%d root_off=%d data_off=%d'
      % (bps, spc, fat_off, root_off, data_off))

slot = None
for k in range(rootent):
    if d[root_off + k * 32] in (0x00, 0xE5):
        slot = k
        break
if slot is None:
    raise SystemExit('no free root dir slot')

nclusters = (fsz * bps) // 2
cl = None
for c in range(2, nclusters):
    if struct.unpack('<H', d[fat_off + c * 2: fat_off + c * 2 + 2])[0] == 0:
        cl = c
        break
print('free root slot=%d, free cluster=%d (clusters=%d)' % (slot, cl, nclusters))

needed = (len(data) + spc * bps - 1) // (spc * bps)
print('clusters needed:', needed)
assert needed == 1, 'only single-cluster supported here'

# FAT 链：两处 FAT 都要写
for base in (fat_off, fat_off + fsz * bps):
    struct.pack_into('<H', d, base + cl * 2, 0xFFFF)

off = data_off + (cl - 2) * spc * bps
d[off:off + len(data)] = data
d[root_off + slot * 32: root_off + slot * 32 + 32] = dir_entry(cl, len(data))

open(DST, 'wb').write(bytes(d))
print('wrote', DST, os.path.getsize(DST))

# 校验
d2 = open(DST, 'rb').read()
names = []
for k in range(rootent):
    ent = d2[root_off + k * 32: root_off + k * 32 + 32]
    if ent[0] == 0:
        break
    if ent[11] == 0x0F:
        continue
    nm = ent[0:8].decode('latin1').rstrip()
    ext = ent[8:11].decode('latin1').rstrip()
    if nm == 'EGA':
        names.append('%s.%s clus=%d size=%d' % (
            nm, ext, struct.unpack('<H', ent[26:28])[0],
            struct.unpack('<I', ent[28:32])[0]))
print('HD root contains:', names)

# ============================================================
# 2) 生成 360K FAT12 软盘（数据盘，从硬盘启动 DOS 后可读 A:）
# ============================================================
bps, spc, res, nf = 512, 2, 1, 2
rootent, ts, media, fsz, spt, heads = 112, 720, 0xFD, 2, 9, 2
f = bytearray(368640)
f[0:3] = b'\xEB\x3C\x90'          # jmp 到 0x3E
f[3:11] = b'DOS86   '
struct.pack_into('<H', f, 11, bps)
f[13] = spc
struct.pack_into('<H', f, 14, res)
f[16] = nf
struct.pack_into('<H', f, 17, rootent)
struct.pack_into('<H', f, 19, ts)
f[21] = media
struct.pack_into('<H', f, 22, fsz)
struct.pack_into('<H', f, 24, spt)
struct.pack_into('<H', f, 26, heads)
struct.pack_into('<I', f, 28, 0)
f[0x3E] = 0xEB                     # 非启动盘：原地死循环
f[0x3F] = 0xFE
f[510] = 0x55
f[511] = 0xAA

fat1 = res * bps
fat2 = (res + fsz) * bps
root = (res + 2 * fsz) * bps
dstart = root + rootent * 32

f[fat1 + 0] = media
f[fat1 + 1] = 0xFF
f[fat1 + 2] = 0xFF
f[fat2:fat2 + fsz * bps] = f[fat1:fat1 + fsz * bps]


def set_fat12(base, c, v):
    off = base + c + (c // 2)
    cur = f[off] | (f[off + 1] << 8)
    if c & 1:
        cur = (cur & 0x000F) | ((v & 0x0FFF) << 4)
    else:
        cur = (cur & 0xF000) | (v & 0x0FFF)
    f[off] = cur & 0xFF
    f[off + 1] = (cur >> 8) & 0xFF


set_fat12(fat1, 2, 0xFFF)
set_fat12(fat2, 2, 0xFFF)
off = dstart + (2 - 2) * spc * bps
f[off:off + len(data)] = data
f[root:root + 32] = dir_entry(2, len(data))

out = r'd:\dos86\data\ega360.img'
open(out, 'wb').write(bytes(f))
print('wrote', out, os.path.getsize(out), '=', os.path.getsize(out) // 1024, 'K')

# 校验
g = open(out, 'rb').read()
ent = g[root:root + 32]
print('floppy root: %s clus=%d size=%d' % (
    ent[0:11].decode('latin1'),
    struct.unpack('<H', ent[26:28])[0],
    struct.unpack('<I', ent[28:32])[0]))
print('floppy file data match:', g[dstart:dstart + len(data)] == data)
