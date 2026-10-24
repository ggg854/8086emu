@echo off
set PATH=D:\msys2\mingw64\bin;%PATH%
cd /d d:\dos86
for /f "delims=" %%c in ('pkg-config --cflags gtk+-3.0 json-c') do set CF=%%c
set CFLAGS=-O2 -Wall -Wextra -std=c11 -Isrc/cpu -Isrc/fpu -Isrc/io -Isrc/video -Isrc/main %CF%
gcc %CFLAGS% -c src/cpu/cpu.c        -o src/cpu/cpu.o
gcc %CFLAGS% -c src/cpu/cpu386.c     -o src/cpu/cpu386.o
gcc %CFLAGS% -c src/cpu/protect.c    -o src/cpu/protect.o
gcc %CFLAGS% -c src/fpu/fpu.c        -o src/fpu/fpu.o
gcc %CFLAGS% -c src/io/io.c          -o src/io/io.o
gcc %CFLAGS% -c src/io/ide.c         -o src/io/ide.o
gcc %CFLAGS% -c src/io/dma.c         -o src/io/dma.o
gcc %CFLAGS% -c src/io/fdc.c         -o src/io/fdc.o
gcc %CFLAGS% -c src/main/config.c    -o src/main/config.o
gcc %CFLAGS% -c src/video/vga.c      -o src/video/vga.o
gcc %CFLAGS% -c src/main/dos86.c     -o src/main/dos86.o
set LIBS=-LD:/msys2/mingw64/bin/../lib -lgtk-3 -lgdk-3 -lgdi32 -limm32 -lshell32 -lole32 -luuid -lwinmm -ldwmapi -lsetupapi -lcfgmgr32 -lhid -lwinspool -lcomctl32 -lcomdlg32 -lpangocairo-1.0 -lpangowin32-1.0 -lpango-1.0 -lcairo-gobject -lcairo -lharfbuzz -lgdk_pixbuf-2.0 -latk-1.0 -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lintl -lz -ljson-c -lm
gcc -o build\8086emu_dbg.exe src\main\dos86.o src\cpu\cpu.o src\cpu\cpu386.o src\cpu\protect.o src\fpu\fpu.o src\io\io.o src\io\ide.o src\io\dma.o src\io\fdc.o src\main\config.o src\video\vga.o %LIBS%
echo LINK_EXIT=%ERRORLEVEL%
dir build\8086emu_dbg.exe
REM 控制台子系统变体：自带真实控制台，GLib 不再因 stdin 重定向而硬退出，可无头跑
gcc -mconsole -o build\8086emu_headless.exe src\main\dos86.o src\cpu\cpu.o src\cpu\cpu386.o src\cpu\protect.o src\fpu\fpu.o src\io\io.o src\io\ide.o src\io\dma.o src\io\fdc.o src\main\config.o src\video\vga.o %LIBS%
echo LINK_HEADLESS_EXIT=%ERRORLEVEL%
dir build\8086emu_headless.exe
