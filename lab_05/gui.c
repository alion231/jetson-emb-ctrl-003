#include <gtk/gtk.h>
#include <stdatomic.h>
#include <stdio.h>

#include "balance_loop.h"

static GtkLabel *pitch_label;
static GtkLabel *rpm0_label;
static GtkLabel *rpm1_label;

static GtkScale *kp_scale;
static GtkScale *ki_scale;
static GtkScale *kd_scale;

static GtkSpinButton *kp_spin;
static GtkSpinButton *ki_spin;
static GtkSpinButton *kd_spin;

static GMainLoop *main_loop;

static void update_labels(void)
{
    double pitch = atomic_load(&g_pitch_deg);
    double rpm0 = atomic_load(&g_motor_rpm[0]);
    double rpm1 = atomic_load(&g_motor_rpm[1]);

    char buf[64];

    snprintf(buf, sizeof(buf), "Pitch: %.2f deg", pitch);
    gtk_label_set_text(pitch_label, buf);

    snprintf(buf, sizeof(buf), "Motor 0: %.1f RPM", rpm0);
    gtk_label_set_text(rpm0_label, buf);

    snprintf(buf, sizeof(buf), "Motor 1: %.1f RPM", rpm1);
    gtk_label_set_text(rpm1_label, buf);
}

static gboolean gui_timer_cb(gpointer user_data)
{
    (void)user_data;
    update_labels();
    return TRUE; /* keep timer */
}

static void gains_changed_cb(GtkWidget *widget, gpointer user_data)
{
    (void)widget;
    (void)user_data;

    double kp = gtk_range_get_value(GTK_RANGE(kp_scale));
    double ki = gtk_range_get_value(GTK_RANGE(ki_scale));
    double kd = gtk_range_get_value(GTK_RANGE(kd_scale));

    gtk_spin_button_set_value(kp_spin, kp);
    gtk_spin_button_set_value(ki_spin, ki);
    gtk_spin_button_set_value(kd_spin, kd);

    balance_loop_set_gains(kp, ki, kd);
}

static void reset_gains_cb(GtkButton *button, gpointer user_data)
{
    (void)button;
    (void)user_data;

    double kp = 0.05;
    double ki = 0.0;
    double kd = 0.0;

    gtk_range_set_value(GTK_RANGE(kp_scale), kp);
    gtk_range_set_value(GTK_RANGE(ki_scale), ki);
    gtk_range_set_value(GTK_RANGE(kd_scale), kd);

    gtk_spin_button_set_value(kp_spin, kp);
    gtk_spin_button_set_value(ki_spin, ki);
    gtk_spin_button_set_value(kd_spin, kd);

    balance_loop_set_gains(kp, ki, kd);
}

static void quit_cb(GtkButton *button, gpointer user_data)
{
    (void)button;
    (void)user_data;
    g_main_loop_quit(main_loop);
}

void gui_init(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    gtk_init();

    GtkWidget *window = gtk_window_new();
    gtk_window_set_title(GTK_WINDOW(window), "Self-Balancing Robot");
    gtk_window_set_default_size(GTK_WINDOW(window), 400, 300);

    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 8);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
    gtk_window_set_child(GTK_WINDOW(window), grid);

    pitch_label = GTK_LABEL(gtk_label_new("Pitch: 0.00 deg"));
    rpm0_label = GTK_LABEL(gtk_label_new("Motor 0: 0.0 RPM"));
    rpm1_label = GTK_LABEL(gtk_label_new("Motor 1: 0.0 RPM"));

    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(pitch_label), 0, 0, 2, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(rpm0_label), 0, 1, 2, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(rpm1_label), 0, 2, 2, 1);

    GtkAdjustment *kp_adj = gtk_adjustment_new(0.05, 0.0, 1.0, 0.005, 0.05, 0.0);
    GtkAdjustment *ki_adj = gtk_adjustment_new(0.0, 0.0, 1.0, 0.005, 0.05, 0.0);
    GtkAdjustment *kd_adj = gtk_adjustment_new(0.0, 0.0, 1.0, 0.005, 0.05, 0.0);    

    kp_scale = GTK_SCALE(gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, kp_adj));
    ki_scale = GTK_SCALE(gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, ki_adj));
    kd_scale = GTK_SCALE(gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, kd_adj));

    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(gtk_label_new("Kp")), 0, 3, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(kp_scale), 1, 3, 1, 1);

    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(gtk_label_new("Ki")), 0, 4, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(ki_scale), 1, 4, 1, 1);

    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(gtk_label_new("Kd")), 0, 5, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(kd_scale), 1, 5, 1, 1);

    kp_spin = GTK_SPIN_BUTTON(gtk_spin_button_new(kp_adj, 0.0, 3));
    ki_spin = GTK_SPIN_BUTTON(gtk_spin_button_new(ki_adj, 0.0, 3));
    kd_spin = GTK_SPIN_BUTTON(gtk_spin_button_new(kd_adj, 0.0, 3));

    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(kp_spin), 2, 3, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(ki_spin), 2, 4, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), GTK_WIDGET(kd_spin), 2, 5, 1, 1);

    GtkWidget *reset_button = gtk_button_new_with_label("Reset Gains");
    GtkWidget *quit_button = gtk_button_new_with_label("Quit");

    gtk_grid_attach(GTK_GRID(grid), reset_button, 0, 6, 1, 1);
    gtk_grid_attach(GTK_GRID(grid), quit_button, 1, 6, 1, 1);

    g_signal_connect(kp_scale, "value-changed", G_CALLBACK(gains_changed_cb), NULL);
    g_signal_connect(ki_scale, "value-changed", G_CALLBACK(gains_changed_cb), NULL);
    g_signal_connect(kd_scale, "value-changed", G_CALLBACK(gains_changed_cb), NULL);

    g_signal_connect(reset_button, "clicked", G_CALLBACK(reset_gains_cb), NULL);
    g_signal_connect(quit_button, "clicked", G_CALLBACK(quit_cb), NULL);

    g_timeout_add(50, gui_timer_cb, NULL); /* ~20 Hz GUI update */

    gtk_widget_show(window);

    main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(main_loop);
    g_main_loop_unref(main_loop);
}
