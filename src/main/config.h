// config.h - 模拟器配置（JSON 持久化）
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

typedef struct {
    char bios[512];
    char disk[512];
    char floppy_a[512];
    char floppy_b[512];
    int  cpu_mhz;
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

#endif