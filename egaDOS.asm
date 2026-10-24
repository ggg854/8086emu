; ============================================================
;   EGA / VGA 测试程序 —— DOS .COM 版
;
;   用法：在 DOS 下运行 EGA
;
;   三段测试：
;     1) 模式 10h (640x350x16)：Sequencer Map Mask 逐平面填 16 色竖条纹
;     2) 写模式 1 + 锁存器：REP MOVSB 把 0..49 行拷到 50..99 行
;     3) 模式 0Dh (320x200x16)：验证由 CRTC 推导的 320 宽平面渲染
;
;   每次设完模式都用 INT 10h AH=0Fh 读回"实际生效的模式与列数"并保存，
;   最后回到文本模式统一打印 —— 用来判断 BIOS 到底有没有接受所请求的模式
;   （例如请求 0x10 却落回 0x01 即 40x25 文本）。
; ============================================================
        org 0x0100

SCREEN  equ 0xA000

start:
        cld

        ; ============================================================
        ; 先做"模式探测"并立刻在文本模式打印结果，
        ; 这样即便后面的图形测试卡住 / 没按键，诊断信息也已经出来了。
        ; ============================================================
        mov ax, 0x0003
        int 0x10
        mov si, msg_head
        call print_str

        ; ---- 逐个探测模式表：每个都"设置 -> 查询 -> 回文本 -> 打印" ----
        mov word [midx], 0
        mov word [accmask], 0        ; 位掩码：第 N 位 = 第 N 个模式被接受
        mov word [bitv], 1
probe_loop:
        mov bx, [midx]
        mov al, [mode_table + bx]
        or al, al                    ; 0 结束
        jz probe_done
        mov ah, 0x00
        int 0x10                     ; 设置模式
        mov di, m1
        call query_mode              ; [m1]=实际模式, [m1+1]=列数
        mov ax, 0x0003
        int 0x10                     ; 立即回文本模式再打印

        ; 该模式是否被接受？got == req 则置对应位（顺序同 mode_table）
        mov bx, [midx]
        mov al, [mode_table + bx]
        cmp al, [m1]
        jne .noacc
        mov ax, [bitv]
        or [accmask], ax
.noacc:
        shl word [bitv], 1

        mov si, msg_req
        call print_str
        mov bx, [midx]
        mov al, [mode_table + bx]
        call print_hex8
        mov si, msg_got
        call print_str
        mov al, [m1]
        call print_hex8
        mov si, msg_c1
        call print_str
        mov al, [m1+1]
        call print_hex8

        inc word [midx]
        jmp probe_loop
probe_done:
        ; 一行紧凑摘要，避免被后续输出滚动掉
        mov si, msg_acc
        call print_str
        mov al, [accmask+1]
        call print_hex8
        mov al, [accmask]
        call print_hex8

        ; 显卡自报身份
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

        ; INT 10h AH=1Ah：显示组合码（AL=1Ah 表示支持；BL=活动显示码）
        mov ax, 0x1A00
        xor bx, bx
        int 0x10
        mov [q_ax], ax
        mov [q_bx], bx
        mov si, msg_1a
        call print_str
        mov al, [q_ax]
        call print_hex8
        mov si, msg_bl
        call print_str
        mov al, [q_bx]
        call print_hex8

        ; INT 10h AH=1Bh：功能/状态（AL=1Bh 支持；BH=CRT 类型；BL=显示组合；CH/CL=特性）
        mov ax, 0x1B00
        xor bx, bx
        int 0x10
        mov [q_ax], ax
        mov [q_bx], bx
        mov [q_cx], cx
        mov si, msg_1b
        call print_str
        mov al, [q_ax]
        call print_hex8
        mov si, msg_bhx
        call print_str
        mov al, [q_bx+1]
        call print_hex8
        mov si, msg_bl
        call print_str
        mov al, [q_bx]
        call print_hex8
        mov si, msg_chx
        call print_str
        mov al, [q_cx+1]
        call print_hex8
        mov si, msg_clx
        call print_str
        mov al, [q_cx]
        call print_hex8

        ; BIOS 数据区（0040:xxxx）视频相关字段 —— 显卡 ROM 判定显示类型的依据
        ;   注意：设 DS=0x40 后不能再往本段变量写内存，否则会写进 BDA 段。
        ;   所以先把 6 个字节读进 BL/BH/CL/CH/DL/DH，恢复 DS 后再落盘。
        push ds
        mov ax, 0x0040
        mov ds, ax
        mov bl, [0x0010]        ; 设备字低字节
        mov bh, [0x0011]        ; 设备字高字节
        mov cl, [0x0049]        ; 当前视频模式
        mov ch, [0x0065]        ; CRT 模式控制字节
        mov dl, [0x0087]        ; EGA/VGA 信息字节 1
        mov dh, [0x0088]        ; EGA/VGA 信息字节 2
        pop ds
        mov [bdaeq], bx
        mov [bdaeq+2], cx
        mov [bdaeq+4], dx

        mov si, msg_bda
        call print_str
        mov al, [bdaeq+1]
        call print_hex8
        mov al, [bdaeq]
        call print_hex8
        mov si, msg_m49
        call print_str
        mov al, [bdaeq+2]
        call print_hex8
        mov si, msg_m65
        call print_str
        mov al, [bdaeq+3]
        call print_hex8
        mov si, msg_m87
        call print_str
        mov al, [bdaeq+4]
        call print_hex8
        mov al, [bdaeq+5]
        call print_hex8

        mov si, msg_tail
        call print_str

        ; ============================================================
        ; 诊断打完，再提示按键进入实际图形测试
        ; ============================================================
        mov si, msg_press
        call print_str
        call wait_key

        ; ---- 阶段 1：模式 10h = 640x350x16 ----
        mov ax, 0x0010
        int 0x10
        mov word [rowbytes], 80
        mov word [rows], 350
        call fill_screen
        call blit_test
        call wait_key

        ; ---- 阶段 2：模式 0Dh = 320x200x16 ----
        mov ax, 0x000D
        int 0x10
        mov word [rowbytes], 40
        mov word [rows], 200
        call fill_screen
        call wait_key

        mov ax, 0x0003
        int 0x10
        mov ax, 0x4C00
        int 0x21

; ------------------------------------------------------------
; query_mode：INT 10h AH=0Fh -> AL=模式, AH=屏幕列数
;   结果存到 DI 指向的 2 字节：[DI]=模式, [DI+1]=列数
; ------------------------------------------------------------
query_mode:
        push ax
        push bx
        mov ax, 0x0F00
        int 0x10
        mov [di], ax
        pop bx
        pop ax
        ret

; ------------------------------------------------------------
; 端口辅助：set_seq AL=索引 AH=值 (3C4/3C5)
;          set_gc  AL=索引 AH=值 (3CE/3CF)
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
; build_template：BL = 平面号
;   每 5 字节一个色号(0..15)，共 80 字节；该平面位为 1 的色号 -> FF，否则 00
; ------------------------------------------------------------
build_template:
        push ax
        push cx
        push dx
        push di
        mov di, tmpl
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
; fill_screen：逐平面写入，得到 16 色竖条纹
;   DS:SI = 本段内的模板；ES:DI = 显存
; ------------------------------------------------------------
fill_screen:
        call gc_normal
        mov ax, SCREEN
        mov es, ax
        mov word [plane], 0
.fs_plane:
        mov bx, [plane]
        call build_template

        mov bx, [plane]
        mov al, 0x01
        mov cl, bl
        shl al, cl
        mov ah, al
        mov al, 0x02
        call set_seq

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
        mov si, tmpl
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

        ; ES 恢复为本段（DS），避免影响后续
        push ds
        pop es
        mov ax, 0x0F02
        call set_seq
        ret

; ------------------------------------------------------------
; blit_test：写模式 1 + 锁存器位块拷贝（0..49 行 -> 50..99 行）
;   REP MOVSB 每轮"先读(装 latch) 再写(写模式 1 写 latch)"
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
        push ds
        pop es

        mov ax, 0x0005           ; 回到写模式 0
        call set_gc
        ret

; ------------------------------------------------------------
; report：文本模式下打印全部诊断
; ------------------------------------------------------------
report:
        mov si, msg_head
        call print_str

        mov si, msg_m1
        call print_str
        mov al, [m1]
        call print_hex8
        mov si, msg_c1
        call print_str
        mov al, [m1+1]
        call print_hex8

        mov si, msg_m2
        call print_str
        mov al, [m2]
        call print_hex8
        mov si, msg_c2
        call print_str
        mov al, [m2+1]
        call print_hex8

        ; EGA 身份：AH=12h BL=10h
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
; 数据（都在本段内，DS 不需要改）
; ------------------------------------------------------------
plane    dw 0
row      dw 0
stripe   dw 0
rowbytes dw 0
rows     dw 0
m1       dw 0
m2       dw 0
midx     dw 0
vinfo    dw 0
vfeat    dw 0
q_ax     dw 0
q_bx     dw 0
q_cx     dw 0
bdaeq    times 6 db 0      ; [0..1]=设备字 [2]=0x49 [3]=0x65 [4]=0x87 [5]=0x88
accmask  dw 0
bitv     dw 1
tmpl:    times 80 db 0

; 要探测的模式号（0 结束）：文本 + CGA + EGA + VGA 全覆盖
mode_table: db 0x01, 0x03, 0x04, 0x06, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x00

msg_head: db 13,10,'EGA TEST OK',13,10,0
msg_req:  db 13,10,'req=',0
msg_got:  db ' got=',0
msg_1a:   db 13,10,'1A AL=',0
msg_1b:   db 13,10,'1B AL=',0
msg_bhx:  db ' BH=',0
msg_chx:  db ' CH=',0
msg_clx:  db ' CL=',0
msg_bda:  db 13,10,'BDA eq=',0
msg_acc:  db 13,10,'ACCMASK=',0
msg_m49:  db ' 49=',0
msg_m65:  db ' 65=',0
msg_m87:  db ' 87/88=',0
msg_m1:   db 13,10,'M1(10h req) mode=',0
msg_c1:   db ' cols=',0
msg_m2:   db 13,10,'M2(0Dh req) mode=',0
msg_c2:   db ' cols=',0
msg_bh:   db 13,10,'BH=',0
msg_bl:   db ' BL=',0
msg_cx:   db ' CX=',0
msg_tail: db 13,10,0
msg_press: db 13,10,'PRESS ANY KEY TO RUN GRAPHIC TESTS...',13,10,0
