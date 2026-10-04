cd /d/dos86 || exit 9
make
gcc -o build/8086emu_trace.exe src/main/dos86.o src/cpu/cpu.o src/cpu/cpu386.o src/cpu/protect.o src/fpu/fpu.o src/io/io.o src/io/ide.o src/io/dma.o src/io/fdc.o src/main/config.o src/video/vga.o -lgtk-3 -lgdk-3 -lgdi32 -limm32 -lshell32 -lole32 -luuid -lwinmm -ldwmapi -lsetupapi -lcfgmgr32 -lhid -lwinspool -lcomctl32 -lcomdlg32 -lpangocairo-1.0 -lpangowin32-1.0 -lpango-1.0 -lcairo-gobject -lcairo -lharfbuzz -lgdk_pixbuf-2.0 -latk-1.0 -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lintl -lz -ljson-c -lm
ls -la build/8086emu_trace.exe
echo DONE
