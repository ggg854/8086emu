# Makefile - DOS Emulator (GTK, BIOS boot)
#
# 依赖: gcc, pkg-config, gtk+-3.0
# 用法:
#   make          - 编译（动态链接，build/8086emu.exe）
#   make clean    - 清理
#   make run      - 编译并运行
#   make bundle   - 生成便携包 dist/（exe + 依赖 DLL + data/，可拷到别的机器直接跑）

CC       = gcc
PKGCONF  = pkg-config
TARGET   = build/8086emu

# 源文件
SRCS = \
	src/ui/dos86.c \
	src/core/cpu.c \
	src/core/cpu386.c \
	src/core/protect.c \
	src/core/fpu.c \
	src/core/bus.c \
	src/devices/io.c \
	src/devices/ide.c \
	src/devices/dma.c \
	src/devices/fdc.c \
	src/ui/config.c \
	src/devices/vga.c

OBJS = $(SRCS:.c=.o)

# 头文件搜索路径（新分层目录：core 内核 / devices 外设 / ui 前端 / machine 主板）
INCLUDES = \
	-Isrc/core \
	-Isrc/devices \
	-Isrc/ui \
	-Isrc/machine

GTK_CFLAGS = $(shell $(PKGCONF) --cflags gtk+-3.0 json-c)
GTK_LIBS   = $(shell $(PKGCONF) --libs gtk+-3.0 json-c)

CFLAGS  = -O2 -Wall -Wextra -std=c11 $(INCLUDES) $(GTK_CFLAGS)
LDFLAGS = $(GTK_LIBS) -lm

# 便携打包：MSYS2 的 GTK3/glib 只提供动态导入库，无法真正静态链接。
#   改用「动态链接 + 依赖 DLL 随包」：make bundle 生成 dist/，把 exe 及其全部
#   依赖 DLL 拷到同目录，再把运行时 data/ 拷进去，可整个目录拷到别的机器直接跑。
DIST = dist

.PHONY: all clean run bundle

all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p build
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

# 便携打包：exe + 依赖 DLL + 运行时 data/
bundle: $(TARGET)
	@rm -rf $(DIST)
	@mkdir -p $(DIST)/data
	@cp -f $(TARGET).exe $(DIST)/
	@echo "复制依赖 DLL ..."
	@for dll in $$(ldd $(TARGET).exe 2>/dev/null | sed -n 's/.*=> \([^ ]*\) .*/\1/p'); do \
		case "$$dll" in \
			/mingw64/bin/*|/d/msys2/mingw64/bin/*|/msys2/mingw64/bin/*) cp -f "$$dll" $(DIST)/ ;; \
		esac; \
	done
	@echo "复制运行时 data/ (排除调试日志与大磁盘镜像) ..."
	@tar -C data -cf - --exclude='con_err.txt' --exclude='con_out.txt' \
		--exclude='*.img' . | tar -C $(DIST)/data -xf -
	@echo "复制 GTK 运行时模块 (pixbuf 加载器 / 主题 / immodules) ..."
	@mkdir -p $(DIST)/lib
	@cp -rf /mingw64/lib/gdk-pixbuf-2.0 $(DIST)/lib/ 2>/dev/null || true
	@cp -rf /mingw64/lib/gtk-3.0 $(DIST)/lib/ 2>/dev/null || true
	@echo "已生成便携包: $(DIST)/  (整个目录拷到别的机器，运行 $(DIST)/8086emu.exe)"

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 头文件依赖（自动按 SRCS 的同名 .o 依赖同名 .c；跨层头文件由 -I 覆盖）
# 需要精确依赖时可在此追加，例如：
# src/devices/io.o: src/devices/io.h src/core/bus.h src/devices/ide.h

clean:
	rm -f $(OBJS) $(TARGET)
	rm -rf $(DIST)

run: $(TARGET)
	cd data && ../$(TARGET)
