// config.h - 模拟器配置（JSON 持久化）
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stddef.h>

typedef struct _GList GList;   // 前向声明：config.c 会 include <gtk/gtk.h> 拿到完整定义

typedef struct {
    char bios[512];
    char disk[512];
    char floppy_a[512];
    char floppy_b[512];
    int  cpu_mhz;
    int  ram_mb;     // 内存容量（MB），"随便调"：1..256，缺省 8
    bool composite;
    bool console;
    bool debug;
} Config;

// 从 JSON 加载；成功返回 true
bool config_load(const char* path, Config* c);

// 保存到 JSON；成功返回 true
bool config_save(const char* path, const Config* c);

// 弹 GTK 配置窗口；用户点 Start 返回 true
bool config_dialog(Config* c);

// ============================================================
// 机器管理：每台机器 = machines/<name>.json（name 即文件名，不允许空格/非法字符）
// ============================================================
#define MACHINE_RAM_MIN  1
#define MACHINE_RAM_MAX  256
#define MACHINE_RAM_DEF  8

// 机器名是否合法（非空、不含空格与 / \ : * ? " < > |）
bool machine_name_valid(const char* name);

// 拼出机器文件路径 machines/<name>.json，写入 buf（n 字节）；返回 buf
char* machine_path(const char* name, char* buf, size_t n);

// 机器是否已存在
bool machine_exists(const char* name);

// 列出所有机器名（已排序，小写优先）；用 g_list_free_full(list, g_free) 释放
GList* machine_list(void);

// 加载/保存/删除一台机器；成功返回 true
bool machine_load(const char* name, Config* c);
bool machine_save(const char* name, const Config* c);
bool machine_delete(const char* name);

// 机器编辑对话框（新建/修改共用）。
//   is_new=true  ：name_buf 初始可为空，用户填写；创建后写入 machines/<name>.json
//   is_new=false ：name_buf 传入已存在机器名（编辑时锁定不可改），保存覆盖原文件
// 用户点"保存/创建"且校验通过返回 true，并把机器名写入 name_buf、配置写入 c。
bool machine_edit_dialog(Config* c, char* name_buf, size_t name_bufsz, bool is_new);

#endif