import os

SRC = r'D:\Bochs-3.0\VGABIOS-lgpl-latest.bin'
OUT = r'd:\dos86\src\video\vga_rom_vga.h'

d = open(SRC, 'rb').read()
assert len(d) == 0x8000, len(d)

with open(OUT, 'w', encoding='utf-8') as f:
    f.write('// vga_rom_vga.h - embedded standard VGA BIOS option ROM (LGPL, generated, do not edit by hand)\n')
    f.write('//   Source: D:\\Bochs-3.0\\VGABIOS-lgpl-latest.bin  (Bochs VGABios 0.9c, 2025, LGPL)\n')
    f.write('//   Identity: IBM.Bochs VGABios 0.9c 08 Jan 2025 ... LGPL VGABios developers\n')
    f.write('//   Licensed under GNU LGPL - freely redistributable. Supports legacy VGA modes\n')
    f.write('//   0x00-0x13 plus VBE (gracefully degrades to plain VGA if VBE is absent).\n')
    f.write('//   Size 32768 bytes (0x8000) == VIDEO_ROM_SIZE, installed at C0000.\n')
    f.write('#ifndef VGA_ROM_VGA_H\n')
    f.write('#define VGA_ROM_VGA_H\n')
    f.write('#include <stdint.h>\n\n')
    f.write('#define VGA_ROM_VGA_SIZE 0x8000u\n\n')
    f.write('static const uint8_t vga_rom_vga[VGA_ROM_VGA_SIZE] = {\n')
    for i in range(0, len(d), 16):
        chunk = d[i:i + 16]
        f.write('  ' + ', '.join('0x%02X' % b for b in chunk) + ',\n')
    f.write('};\n\n')
    f.write('#endif\n')

print('wrote', OUT, 'bytes=', len(d))
