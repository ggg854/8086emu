@echo off
set PATH=D:\msys2\mingw64\bin;%PATH%
cd /d d:\dos86
for /f "delims=" %%c in ('pkg-config --cflags gtk+-3.0 json-c') do set CF=%%c
REM 新分层目录：core 内核 / devices 外设 / ui 前端 / machine 主板
set CFLAGS=-O2 -Wall -Wextra -std=c11 -Isrc/core -Isrc/devices -Isrc/ui -Isrc/machine %CF%
gcc %CFLAGS% -c src/ui/dos86.c        -o src/ui/dos86.o
gcc %CFLAGS% -c src/core/cpu.c        -o src/core/cpu.o
gcc %CFLAGS% -c src/core/cpu386.c     -o src/core/cpu386.o
gcc %CFLAGS% -c src/core/protect.c    -o src/core/protect.o
gcc %CFLAGS% -c src/core/fpu.c        -o src/core/fpu.o
gcc %CFLAGS% -c src/core/bus.c        -o src/core/bus.o
gcc %CFLAGS% -c src/devices/io.c      -o src/devices/io.o
gcc %CFLAGS% -c src/devices/ide.c     -o src/devices/ide.o
gcc %CFLAGS% -c src/devices/dma.c     -o src/devices/dma.o
gcc %CFLAGS% -c src/devices/fdc.c     -o src/devices/fdc.o
gcc %CFLAGS% -c src/ui/config.c       -o src/ui/config.o
gcc %CFLAGS% -c src/devices/vga.c     -o src/devices/vga.o
set LIBS=-LD:/msys2/mingw64/bin/../lib -lgtk-3 -lgdk-3 -lgdi32 -limm32 -lshell32 -lole32 -luuid -lwinmm -ldwmapi -lsetupapi -lcfgmgr32 -lhid -lwinspool -lcomctl32 -lcomdlg32 -lpangocairo-1.0 -lpangowin32-1.0 -lpango-1.0 -lcairo-gobject -lcairo -lharfbuzz -lgdk_pixbuf-2.0 -latk-1.0 -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lintl -lz -ljson-c -lm
set OBJS=src\ui\dos86.o src\core\cpu.o src\core\cpu386.o src\core\protect.o src\core\fpu.o src\core\bus.o src\devices\io.o src\devices\ide.o src\devices\dma.o src\devices\fdc.o src\ui\config.o src\devices\vga.o
gcc -o build\8086emu_dbg.exe %OBJS% %LIBS%
echo LINK_EXIT=%ERRORLEVEL%
dir build\8086emu_dbg.exe
REM 控制台子系统变体：自带真实控制台，GLib 不再因 stdin 重定向而硬退出，可无头跑
gcc -mconsole -o build\8086emu_headless.exe %OBJS% %LIBS%
echo LINK_HEADLESS_EXIT=%ERRORLEVEL%
dir build\8086emu_headless.exe
