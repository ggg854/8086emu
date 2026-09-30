\# 8086/80286 PC Emulator



An 8086/80286 PC emulator written in C. Runs BIOS POST, boots DOS,

supports hard disk, floppy, FPU, and 80286 protected mode.



\*\*All code in this project is AI-generated.\*\*



\## Features



\- 8086/80286 instruction set, including the original 8088 cycle table

\- 80286 protected mode: descriptors, gates, TSS task switching

\- 8259A PIC, 8253 PIT, 8237 DMA, 8042 keyboard, 8250 serial port

\- CGA text/graphics, NTSC artifact colors (composite video)

\- IDE hard disk, NEC uPD765 floppy controller

\- x87 FPU (8087/80287/80387 instructions)



\## Compatibility



\- ✅ \*\*PCXTBIOS (Turbo XT BIOS v2.5)\*\* — works

\- ❌ \*\*IBM PC/AT BIOS\*\* — currently fails with a #NP fault storm. \*\*Help wanted.\*\*



\## Build



(fill in your build steps, for example:)



```bash

make

./8086emu -bios data/PCXTBIOS.BIN -disk dos.img

