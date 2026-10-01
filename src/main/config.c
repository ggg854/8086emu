// config.c - 模拟器配置（JSON 持久化 + GTK 配置窗口）
#include "config.h"
#include "cpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <json-c/json.h>

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