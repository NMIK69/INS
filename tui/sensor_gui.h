#ifndef SENSOR_GUI_H
#define SENSOR_GUI_H

#include <curses.h>
#include "vec.h"

struct display {
	WINDOW *win;
	int height;
	int width;
	int xloc;
	int yloc;
	int xpad;
	int ypad;
};

struct sensor_gui
{
	struct display accel_display;
	struct display gyro_display;
	struct display angle_display;
	struct display avg_display;
};

struct sensor_gui *init_gui(void);
void free_gui(struct sensor_gui *gui);
void update_angle_window(struct sensor_gui *gui, struct vec3f *angle);
void update_gyro_window(struct sensor_gui *gui, struct vec3f *gyro);
void update_accel_window(struct sensor_gui *gui, struct vec3f *accel);
void update_avg_window(struct sensor_gui *gui, struct vec3f *avg);

#endif
