# Makefile - DOS Emulator (GTK, BIOS boot)
#
# 依赖: gcc, pkg-config, gtk+-3.0
# 用法:
#   make          - 编译
#   make clean    - 清理
#   make run      - 编译并运行

CC       = gcc
PKGCONF  = pkg-config
TARGET   = build/8086emu

# 源文件
SRCS = \
	src/main/dos86.c \
	src/cpu/cpu.c \
	src/cpu/cpu386.c \
	src/cpu/protect.c \
	src/fpu/fpu.c \
	src/io/io.c \
	src/io/ide.c \
	src/io/dma.c \
	src/io/fdc.c \
	src/main/config.c \
	src/video/vga.c

OBJS = $(SRCS:.c=.o)

# 头文件搜索路径
INCLUDES = \
	-Isrc/cpu \
	-Isrc/fpu \
	-Isrc/io \
	-Isrc/video \
	-Isrc/main

GTK_CFLAGS = $(shell $(PKGCONF) --cflags gtk+-3.0 json-c)
GTK_LIBS   = $(shell $(PKGCONF) --libs gtk+-3.0 json-c)

CFLAGS  = -O2 -Wall -Wextra -std=c11 $(INCLUDES) $(GTK_CFLAGS)
LDFLAGS = $(GTK_LIBS) -lm

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p build
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 头文件依赖
src/main/dos86.o: src/main/dos86.c src/main/dos86.h src/cpu/cpu.h src/video/vga.h src/io/ide.h src/io/io.h
src/cpu/cpu.o:    src/cpu/cpu.c    src/cpu/cpu.h src/cpu/cpu386.h src/cpu/protect.h src/io/io.h src/fpu/fpu.h
src/cpu/cpu386.o: src/cpu/cpu386.c src/cpu/cpu386.h src/cpu/protect.h src/cpu/cpu.h
src/cpu/protect.o: src/cpu/protect.c src/cpu/protect.h src/cpu/cpu.h src/cpu/cpu386.h
src/fpu/fpu.o:    src/fpu/fpu.c    src/fpu/fpu.h src/cpu/cpu.h
src/io/io.o:      src/io/io.c      src/io/io.h src/cpu/cpu.h src/io/ide.h src/video/vga.h src/io/dma.h src/io/fdc.h
src/io/ide.o:     src/io/ide.c     src/io/ide.h src/cpu/cpu.h
src/io/dma.o:     src/io/dma.c     src/io/dma.h
src/io/fdc.o:     src/io/fdc.c     src/io/fdc.h src/io/dma.h src/io/ide.h src/cpu/cpu.h
src/video/vga.o:  src/video/vga.c  src/video/vga.h src/cpu/cpu.h src/video/font8x16.h

clean:
	rm -f $(OBJS) $(TARGET)

run: $(TARGET)
	cd data && ../$(TARGET)
