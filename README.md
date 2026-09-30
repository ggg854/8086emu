<img width="646" height="479" alt="image" src="https://github.com/user-attachments/assets/b87f7a1d-c861-42bf-b779-c354f4568fc1" />
<img width="636" height="481" alt="image" src="https://github.com/user-attachments/assets/49ad3c76-14d3-4bd8-ba45-544c0ad076d8" />
<img width="644" height="421" alt="image" src="https://github.com/user-attachments/assets/cc5f6999-dffb-46a8-a3b4-a760db3400ab" />
<img width="644" height="484" alt="image" src="https://github.com/user-attachments/assets/29e62587-a5b2-4bce-845f-0ca8e165ed2b" />
it's ONLY can select CGA driver(VGA driver is black screen)

\# 8086/80286 PC Emulator
WARNING: This binary is NOT statically linked.
The binary CANNOT be launched by double-clicking.


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

./8086emu

