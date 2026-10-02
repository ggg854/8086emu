// config.c - 模拟器配置（JSON 持久化 + GTK 配置窗口）
#include "config.h"
#include "cpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <glib/gstdio.h>
#include <json-c/json.h>

#define MACHINES_DIR "machines"

bool config_load(const char* path, Config* c) {
    FILE* fp = fopen(path, "r");
    if (!fp) return false;
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char* buf = malloc(sz + 1);
    if (!buf) { fclose(fp); return false; }
    size_t rd = fread(buf, 1, sz, fp);
    buf[rd] = 0;
    fclose(fp);

    memset(c, 0, sizeof(*c));
    struct json_object* root = json_tokener_parse(buf);
    free(buf);
    if (!root) return false;

    struct json_object* v;
    if (json_object_object_get_ex(root, "bios", &v))
        strncpy(c->bios, json_object_get_string(v), sizeof(c->bios) - 1);
    if (json_object_object_get_ex(root, "disk", &v))
        strncpy(c->disk, json_object_get_string(v), sizeof(c->disk) - 1);
    if (json_object_object_get_ex(root, "floppy_a", &v) && !json_object_is_type(v, json_type_null))
        strncpy(c->floppy_a, json_object_get_string(v), sizeof(c->floppy_a) - 1);
    if (json_object_object_get_ex(root, "floppy_b", &v) && !json_object_is_type(v, json_type_null))
        strncpy(c->floppy_b, json_object_get_string(v), sizeof(c->floppy_b) - 1);
    if (json_object_object_get_ex(root, "cpu_mhz", &v))
        c->cpu_mhz = json_object_get_int(v);
    if (json_object_object_get_ex(root, "ram_mb", &v))
        c->ram_mb = json_object_get_int(v);
    if (json_object_object_get_ex(root, "composite", &v))
        c->composite = json_object_get_boolean(v);
    if (json_object_object_get_ex(root, "console", &v))
        c->console = json_object_get_boolean(v);
    if (json_object_object_get_ex(root, "debug", &v))
        c->debug = json_object_get_boolean(v);

    json_object_put(root);
    return true;
}

bool config_save(const char* path, const Config* c) {
    struct json_object* root = json_object_new_object();
    json_object_object_add(root, "bios", json_object_new_string(c->bios));
    json_object_object_add(root, "disk", json_object_new_string(c->disk));
    json_object_object_add(root, "floppy_a",
        c->floppy_a[0] ? json_object_new_string(c->floppy_a) : NULL);
    json_object_object_add(root, "floppy_b",
        c->floppy_b[0] ? json_object_new_string(c->floppy_b) : NULL);
    json_object_object_add(root, "cpu_mhz", json_object_new_int(c->cpu_mhz));
    json_object_object_add(root, "ram_mb", json_object_new_int(c->ram_mb));
    json_object_object_add(root, "composite", json_object_new_boolean(c->composite));
    json_object_object_add(root, "console", json_object_new_boolean(c->console));
    json_object_object_add(root, "debug", json_object_new_boolean(c->debug));

    const char* s = json_object_to_json_string_ext(root, JSON_C_TO_STRING_PRETTY);
    FILE* fp = fopen(path, "w");
    if (!fp) { json_object_put(root); return false; }
    fputs(s, fp);
    fclose(fp);
    json_object_put(root);
    return true;
}

// 文件选择回调：点 "..." 按钮时调用
typedef struct {
    GtkWidget* entry;
    const char* title;
    const char* filter_name;
    const char* filter_pattern;
} FilePickCtx;

static void on_pick_file(GtkWidget* btn, gpointer data) {
    (void)btn;
    FilePickCtx* ctx = (FilePickCtx*)data;
    GtkWidget* dlg = gtk_file_chooser_dialog_new(
        ctx->title, NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Cancel", GTK_RESPONSE_CANCEL,
        "_Open",   GTK_RESPONSE_ACCEPT,
        NULL);
    if (ctx->filter_name) {
        GtkFileFilter* f = gtk_file_filter_new();
        gtk_file_filter_set_name(f, ctx->filter_name);
        gtk_file_filter_add_pattern(f, ctx->filter_pattern);
        gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dlg), f);
    }
    GtkFileFilter* all = gtk_file_filter_new();
    gtk_file_filter_set_name(all, "All files");
    gtk_file_filter_add_pattern(all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dlg), all);

    if (gtk_dialog_run(GTK_DIALOG(dlg)) == GTK_RESPONSE_ACCEPT) {
        char* f = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dlg));
        if (f) {
            gtk_entry_set_text(GTK_ENTRY(ctx->entry), f);
            g_free(f);
        }
    }
    gtk_widget_destroy(dlg);
}

// 清空路径回调：点 "Clear" 按钮时调用
static void on_clear_path(GtkWidget* btn, gpointer data) {
    (void)btn;
    GtkWidget* entry = (GtkWidget*)data;
    gtk_entry_set_text(GTK_ENTRY(entry), "");
}

// 一行：标签 + 输入框 + "..." 按钮 + "Clear" 按钮
static GtkWidget* add_file_row(GtkGrid* grid, int row, const char* label,
                               const char* initial,
                               const char* dlg_title,
                               const char* filter_name,
                               const char* filter_pattern,
                               GtkWidget** out_entry) {
    GtkWidget* lbl = gtk_label_new(label);
    gtk_widget_set_halign(lbl, GTK_ALIGN_START);
    GtkWidget* ent = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(ent), initial);
    gtk_widget_set_hexpand(ent, TRUE);

    GtkWidget* btn = gtk_button_new_with_label("...");
    FilePickCtx* ctx = g_new0(FilePickCtx, 1);
    ctx->entry = ent;
    ctx->title = dlg_title;
    ctx->filter_name = filter_name;
    ctx->filter_pattern = filter_pattern;
    g_signal_connect_data(btn, "clicked", G_CALLBACK(on_pick_file),
                          ctx, (GClosureNotify)g_free, 0);

    GtkWidget* clear_btn = gtk_button_new_with_label("Clear");
    g_signal_connect(clear_btn, "clicked", G_CALLBACK(on_clear_path), ent);

    gtk_grid_attach(grid, lbl, 0, row, 1, 1);
    gtk_grid_attach(grid, ent, 1, row, 1, 1);
    gtk_grid_attach(grid, btn, 2, row, 1, 1);
    gtk_grid_attach(grid, clear_btn, 3, row, 1, 1);

    *out_entry = ent;
    return ent;
}

bool config_dialog(Config* c) {
    GtkWidget* dlg = gtk_dialog_new_with_buttons(
        "IBM PC Emulator - Configuration",
        NULL,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        "_Cancel", GTK_RESPONSE_CANCEL,
        "_Start",  GTK_RESPONSE_ACCEPT,
        NULL);
    gtk_window_set_default_size(GTK_WINDOW(dlg), 640, 400);

    GtkWidget* grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 8);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 12);

    int row = 0;
    GtkWidget *ent_bios = NULL, *ent_disk = NULL, *ent_fa = NULL, *ent_fb = NULL;

    add_file_row(GTK_GRID(grid), row++, "BIOS ROM:",
                 c->bios, "Select BIOS ROM",
                 "BIOS ROM (*.bin, *.rom)", "*.bin",
                 &ent_bios);
    add_file_row(GTK_GRID(grid), row++, "Hard disk:",
                 c->disk, "Select Hard Disk Image",
                 "Disk image (*.img, *.bin)", "*.img",
                 &ent_disk);
    add_file_row(GTK_GRID(grid), row++, "Floppy A:",
                 c->floppy_a, "Select Floppy A: Image",
                 "Floppy image (*.img, *.ima)", "*.img",
                 &ent_fa);
    add_file_row(GTK_GRID(grid), row++, "Floppy B:",
                 c->floppy_b, "Select Floppy B: Image",
                 "Floppy image (*.img, *.ima)", "*.img",
                 &ent_fb);

    GtkWidget* lbl_mhz = gtk_label_new("CPU clock (MHz):");
    gtk_widget_set_halign(lbl_mhz, GTK_ALIGN_START);
    GtkWidget* spin_mhz = gtk_spin_button_new_with_range(1, 100, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_mhz), c->cpu_mhz ? c->cpu_mhz : 20);
    gtk_grid_attach(GTK_GRID(grid), lbl_mhz, 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), spin_mhz, 1, row, 1, 1);
    row++;

    GtkWidget* chk_comp = gtk_check_button_new_with_label("CGA composite colors");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_comp), c->composite);
    gtk_grid_attach(GTK_GRID(grid), chk_comp, 1, row, 1, 1);
    row++;

    GtkWidget* chk_dbg = gtk_check_button_new_with_label("Debug log");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_dbg), c->debug);
    gtk_grid_attach(GTK_GRID(grid), chk_dbg, 1, row, 1, 1);
    row++;

    gtk_container_add(GTK_CONTAINER(gtk_dialog_get_content_area(GTK_DIALOG(dlg))), grid);
    gtk_widget_show_all(dlg);

    bool ok = (gtk_dialog_run(GTK_DIALOG(dlg)) == GTK_RESPONSE_ACCEPT);
    if (ok) {
        strncpy(c->bios,     gtk_entry_get_text(GTK_ENTRY(ent_bios)), sizeof(c->bios) - 1);
        strncpy(c->disk,     gtk_entry_get_text(GTK_ENTRY(ent_disk)), sizeof(c->disk) - 1);
        strncpy(c->floppy_a, gtk_entry_get_text(GTK_ENTRY(ent_fa)),   sizeof(c->floppy_a) - 1);
        strncpy(c->floppy_b, gtk_entry_get_text(GTK_ENTRY(ent_fb)),   sizeof(c->floppy_b) - 1);
        c->cpu_mhz   = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_mhz));
        c->composite = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_comp));
        c->debug     = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_dbg));
    }
    gtk_widget_destroy(dlg);
    return ok;
}

// ============================================================
// 机器管理：文件 = machines/<name>.json
// ============================================================
bool machine_name_valid(const char* name) {
    if (!name || !*name) return false;
    if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) return false;
    for (const char* p = name; *p; p++) {
        if (*p == ' ') return false;                       // 不许空格
        if (strchr("/\\:*?\"<>|", *p)) return false;        // 文件名非法字符
    }
    return true;
}

char* machine_path(const char* name, char* buf, size_t n) {
    snprintf(buf, n, "%s/%s.json", MACHINES_DIR, name);
    return buf;
}

bool machine_exists(const char* name) {
    char path[1024];
    machine_path(name, path, sizeof(path));
    FILE* fp = fopen(path, "r");
    if (fp) { fclose(fp); return true; }
    return false;
}

GList* machine_list(void) {
    GList* list = NULL;
    g_mkdir(MACHINES_DIR, 0755);                 // 目录不存在就建
    GDir* dir = g_dir_open(MACHINES_DIR, 0, NULL);
    if (!dir) return NULL;
    const char* nm;
    while ((nm = g_dir_read_name(dir))) {
        size_t L = strlen(nm);
        if (L > 5 && strcmp(nm + L - 5, ".json") == 0) {
            char* base = g_strndup(nm, L - 5);
            list = g_list_append(list, base);
        }
    }
    g_dir_close(dir);
    list = g_list_sort(list, (GCompareFunc)g_ascii_strcasecmp);
    return list;
}

bool machine_load(const char* name, Config* c) {
    char path[1024];
    machine_path(name, path, sizeof(path));
    return config_load(path, c);
}

bool machine_save(const char* name, const Config* c) {
    char path[1024];
    machine_path(name, path, sizeof(path));
    g_mkdir(MACHINES_DIR, 0755);
    return config_save(path, c);
}

bool machine_delete(const char* name) {
    char path[1024];
    machine_path(name, path, sizeof(path));
    return remove(path) == 0;
}

// 简易错误弹窗
static void machine_err(const char* msg) {
    GtkWidget* d = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                                          GTK_MESSAGE_ERROR, GTK_BUTTONS_OK, "%s", msg);
    gtk_dialog_run(GTK_DIALOG(d));
    gtk_widget_destroy(d);
}

bool machine_edit_dialog(Config* c, char* name_buf, size_t name_bufsz, bool is_new) {
    GtkWidget* dlg = gtk_dialog_new_with_buttons(
        is_new ? "新建机器" : "修改机器配置",
        NULL,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        "_取消", GTK_RESPONSE_CANCEL,
        is_new ? "_创建" : "_保存", GTK_RESPONSE_ACCEPT,
        NULL);
    gtk_window_set_default_size(GTK_WINDOW(dlg), 700, 520);

    GtkWidget* grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 12);

    int row = 0;
    GtkWidget *ent_bios = NULL, *ent_disk = NULL, *ent_fa = NULL, *ent_fb = NULL;

    // ---- 机器名 ----
    GtkWidget* lbl_name = gtk_label_new("机器名 (文件名，不许空格):");
    gtk_widget_set_halign(lbl_name, GTK_ALIGN_START);
    GtkWidget* ent_name = gtk_entry_new();
    if (name_buf && name_buf[0])
        gtk_entry_set_text(GTK_ENTRY(ent_name), name_buf);
    gtk_widget_set_hexpand(ent_name, TRUE);
    if (!is_new) gtk_widget_set_sensitive(ent_name, FALSE);   // 修改时锁定文件名
    gtk_grid_attach(GTK_GRID(grid), lbl_name, 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), ent_name, 1, row, 3, 1);
    row++;

    add_file_row(GTK_GRID(grid), row++, "BIOS ROM:",
                 c->bios, "Select BIOS ROM",
                 "BIOS ROM (*.bin, *.rom)", "*.bin", &ent_bios);
    add_file_row(GTK_GRID(grid), row++, "Hard disk:",
                 c->disk, "Select Hard Disk Image",
                 "Disk image (*.img, *.bin)", "*.img", &ent_disk);
    add_file_row(GTK_GRID(grid), row++, "Floppy A:",
                 c->floppy_a, "Select Floppy A: Image",
                 "Floppy image (*.img, *.ima)", "*.img", &ent_fa);
    add_file_row(GTK_GRID(grid), row++, "Floppy B:",
                 c->floppy_b, "Select Floppy B: Image",
                 "Floppy image (*.img, *.ima)", "*.img", &ent_fb);

    // ---- 内存（MB，随便调）----
    GtkWidget* lbl_ram = gtk_label_new("内存 (MB):");
    gtk_widget_set_halign(lbl_ram, GTK_ALIGN_START);
    GtkWidget* spin_ram = gtk_spin_button_new_with_range(MACHINE_RAM_MIN, MACHINE_RAM_MAX, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_ram),
                              c->ram_mb > 0 ? c->ram_mb : MACHINE_RAM_DEF);
    gtk_grid_attach(GTK_GRID(grid), lbl_ram, 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), spin_ram, 1, row, 1, 1);
    row++;

    // ---- CPU 主频 ----
    GtkWidget* lbl_mhz = gtk_label_new("CPU 主频 (MHz):");
    gtk_widget_set_halign(lbl_mhz, GTK_ALIGN_START);
    GtkWidget* spin_mhz = gtk_spin_button_new_with_range(1, 100, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_mhz), c->cpu_mhz ? c->cpu_mhz : 20);
    gtk_grid_attach(GTK_GRID(grid), lbl_mhz, 0, row, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), spin_mhz, 1, row, 1, 1);
    row++;

    GtkWidget* chk_comp = gtk_check_button_new_with_label("CGA 复合伪色 (composite)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_comp), c->composite);
    gtk_grid_attach(GTK_GRID(grid), chk_comp, 1, row, 1, 1);
    row++;

    GtkWidget* chk_dbg = gtk_check_button_new_with_label("调试日志 (debug)");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk_dbg), c->debug);
    gtk_grid_attach(GTK_GRID(grid), chk_dbg, 1, row, 1, 1);
    row++;

    gtk_container_add(GTK_CONTAINER(gtk_dialog_get_content_area(GTK_DIALOG(dlg))), grid);
    gtk_widget_show_all(dlg);

    // 校验循环：非法名/已存在 → 提示并继续等用户输入
    while (true) {
        if (gtk_dialog_run(GTK_DIALOG(dlg)) != GTK_RESPONSE_ACCEPT) {
            gtk_widget_destroy(dlg);
            return false;
        }
        const char* nm = gtk_entry_get_text(GTK_ENTRY(ent_name));
        if (!machine_name_valid(nm)) {
            machine_err("Invalid machine name: no spaces or / \\ : * ? \" < > | allowed");
            continue;
        }
        if (is_new && machine_exists(nm)) {
            machine_err("Machine name already exists; pick another or delete it first.");
            continue;
        }
        strncpy(name_buf, nm, name_bufsz - 1);
        name_buf[name_bufsz - 1] = 0;
        strncpy(c->bios,     gtk_entry_get_text(GTK_ENTRY(ent_bios)),   sizeof(c->bios) - 1);
        strncpy(c->disk,     gtk_entry_get_text(GTK_ENTRY(ent_disk)),   sizeof(c->disk) - 1);
        strncpy(c->floppy_a, gtk_entry_get_text(GTK_ENTRY(ent_fa)),     sizeof(c->floppy_a) - 1);
        strncpy(c->floppy_b, gtk_entry_get_text(GTK_ENTRY(ent_fb)),     sizeof(c->floppy_b) - 1);
        c->ram_mb    = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_ram));
        c->cpu_mhz   = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_mhz));
        c->composite = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_comp));
        c->debug     = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(chk_dbg));
        break;
    }
    gtk_widget_destroy(dlg);
    return true;
}