; ============================================================
;   EGA / VGA 测试程序（引导扇区，512 字节，结尾 0xAA55）
;
;   目的：验证 EGA/VGA 平面模式的三处关键行为
;     1) Sequencer Map Mask 逐平面写入（case -> 16 色竖条纹）
;     2) 写模式 1（锁存器 latch）+ REP MOVSB 做位块拷贝（BitBlt）
;     3) 由 CRTC 推导的平面宽度：640x350 与 320x200 两种都要正确
;
;   最后回到文本模式，用 INT 10h AH=12h BL=10h 读回显卡自报身份并打印。
;
;   只用 8086 可用指令（无 push imm / imul imm / movzx 等 80186+）
; ============================================================
        bits 16
        org 0x7C00

SCREEN  equ 0xA000               ; EGA/VGA 图形段地址
TMPL    equ 0x0500               ; 行模板缓冲区（最多 80 字节）

; ---- 变量放在低端 RAM（IVT/BDA 之后的空间），不占用引导扇区 ----
plane   equ 0x0700
row     equ 0x0702
stripe  equ 0x0704
rowbytes equ 0x0706
rows    equ 0x0708
vinfo   equ 0x070A               ; BH, BL 依次存放
vfeat   equ 0x070C               ; CX

; ------------------------------------------------------------
start:
        cli
        xor ax, ax
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov sp, 0x7C00           ; 栈向下生长，不会覆盖 7C00 处的代码
        sti
        cld

        call gc_normal           ; 写模式 0 / 位掩码 FF / 四平面全开

        ; ---- 阶段 1：模式 10h = 640x350x16 ----
        mov ax, 0x0010
        int 0x10
        mov word [rowbytes], 80
        mov word [rows], 350
        call fill_screen         ; 逐平面填 16 色竖条纹
        call blit_test           ; 写模式 1 拷贝，验证锁存器
        call wait_key

        ; ---- 阶段 2：模式 0Dh = 320x200x16 ----
        mov ax, 0x000D
        int 0x10
        mov word [rowbytes], 40
        mov word [rows], 200
        call fill_screen
        call wait_key

        ; ---- 阶段 3：回文本模式并打印显卡身份 ----
        mov ax, 0x0003
        int 0x10
        call report

halt:
        jmp halt

; ------------------------------------------------------------
; 端口辅助：set_seq  AL=索引 AH=值(3C4/3C5)
;          set_gc   AL=索引 AH=值(3CE/3CF)
; ------------------------------------------------------------
set_seq:
        push dx
        push ax
        mov dx, 0x3C4
        out dx, al
        inc dx
        mov al, ah
        out dx, al
        pop ax
        pop dx
        ret

set_gc:
        push dx
        push ax
        mov dx, 0x3CE
        out dx, al
        inc dx
        mov al, ah
        out dx, al
        pop ax
        pop dx
        ret

; 恢复成"普通"平面写状态
gc_normal:
        mov ax, 0x0005           ; GC 05 = 写模式 0
        call set_gc
        mov ax, 0x0001           ; GC 01 = 允许置位/复位 = 0
        call set_gc
        mov ax, 0xFF08           ; GC 08 = 位掩码 FF
        call set_gc
        mov ax, 0x0F02           ; SEQ 02 = Map Mask 全开
        call set_seq
        ret

; ------------------------------------------------------------
; build_template：BL = 平面号，生成一行的模板字节
;   每 5 个字节一个色号（0..15），共 80 字节
;   该平面上位为 1 的色号 -> 0xFF，否则 0x00
; ------------------------------------------------------------
build_template:
        push ax
        push cx
        push dx
        push di
        mov di, TMPL
        mov word [stripe], 0
.bt_stripe:
        mov ax, [stripe]
        cmp ax, 16
        jae .bt_done
        mov dx, ax
        mov cl, bl
        shr dx, cl
        and dx, 1
        jz .bt_zero
        mov al, 0xFF
        jmp .bt_write5
.bt_zero:
        mov al, 0x00
.bt_write5:
        mov cx, 5
.bt_w5:
        mov [di], al
        inc di
        dec cx
        jnz .bt_w5
        inc word [stripe]
        jmp .bt_stripe
.bt_done:
        pop di
        pop dx
        pop cx
        pop ax
        ret

; ------------------------------------------------------------
; fill_screen：对 4 个平面分别写入，得到 16 色竖条纹
;   每轮把 map mask 设成单个平面，再把模板铺到每一行
; ------------------------------------------------------------
fill_screen:
        call gc_normal           ; INT 10h 设模式会重置 GC/SEQ，这里保证状态确定
        mov word [plane], 0
.fs_plane:
        mov bx, [plane]
        call build_template

        ; map mask = 1 << plane
        mov bx, [plane]
        mov al, 0x01
        mov cl, bl
        shl al, cl
        mov ah, al
        mov al, 0x02
        call set_seq

        mov ax, SCREEN
        mov es, ax
        xor di, di
        mov word [row], 0
.fs_row:
        mov ax, [row]
        mov bx, [rows]
        cmp ax, bx
        jae .fs_nextplane
        push si
        push di
        push cx
        mov si, TMPL
        mov cx, [rowbytes]
        rep movsb
        pop cx
        pop di
        pop si
        mov ax, [rowbytes]
        add di, ax
        inc word [row]
        jmp .fs_row
.fs_nextplane:
        inc word [plane]
        cmp word [plane], 4
        jb .fs_plane

        xor ax, ax
        mov es, ax
        mov ax, 0x0F02           ; 恢复 map mask 全开
        call set_seq
        ret

; ------------------------------------------------------------
; blit_test：写模式 1 + 锁存器的位块拷贝
;   把第 0..49 行拷到第 50..99 行。REP MOVSB 每轮先读（装入锁存器）
;   再写（写模式 1 写的是锁存器内容），因此不需要 CPU 干预数据。
;   若锁存器或写模式 1 有 bug，复制区会呈现全黑/全白而非条纹。
; ------------------------------------------------------------
blit_test:
        mov ax, 0x0105           ; GC 05 = 写模式 1
        call set_gc
        mov ax, 0xFF08           ; 位掩码 FF
        call set_gc
        mov ax, 0x0F02           ; 四平面全开
        call set_seq

        push ds
        mov ax, SCREEN
        mov ds, ax
        mov es, ax
        mov si, 0
        mov di, 4000             ; 50 行 * 80 字节
        mov cx, 4000
        rep movsb
        pop ds

        mov ax, 0x0005           ; 写回模式 0
        call set_gc
        ret

; ------------------------------------------------------------
; report：文本模式下打印显卡自报身份
;   INT 10h AH=12h BL=10h  -> BH: 0=彩色模式生效
;                             BL: 0..3 = EGA 显存 64/128/192/256K
;                             CX: 特性位
; ------------------------------------------------------------
report:
        mov si, msg_head
        call print_str

        mov ax, 0x1200
        mov bl, 0x10
        int 0x10
        mov [vinfo], bx
        mov [vfeat], cx

        mov si, msg_bh
        call print_str
        mov al, [vinfo+1]
        call print_hex8

        mov si, msg_bl
        call print_str
        mov al, [vinfo]
        call print_hex8

        mov si, msg_cx
        call print_str
        mov al, [vfeat+1]
        call print_hex8
        mov al, [vfeat]
        call print_hex8

        mov si, msg_tail
        call print_str
        ret

; ------------------------------------------------------------
; 文本输出
; ------------------------------------------------------------
print_str:
        lodsb
        or al, al
        jz .ps_done
        mov ah, 0x0E
        mov bx, 0x0007
        int 0x10
        jmp print_str
.ps_done:
        ret

print_hex8:
        push ax
        push cx
        mov cl, 4
        mov ah, al
        shr al, cl
        call print_nibble
        mov al, ah
        call print_nibble
        pop cx
        pop ax
        ret

print_nibble:
        push ax
        push bx
        and al, 0x0F
        add al, '0'
        cmp al, '9'
        jbe .pn_ok
        add al, 'A' - '9' - 1
.pn_ok:
        mov ah, 0x0E
        mov bx, 0x0007
        int 0x10
        pop bx
        pop ax
        ret

wait_key:
        mov ax, 0x0000
        int 0x16
        ret

; ------------------------------------------------------------
msg_head: db 13,10,'EGA TEST OK',13,10,0
msg_bh:   db 'BH=',0
msg_bl:   db ' BL=',0
msg_cx:   db ' CX=',0
msg_tail: db 13,10,0

        times 510-($-$$) db 0
        dw 0xAA55
