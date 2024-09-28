#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "sensor_gui.h"

#define ARR_SIZE(arr) (sizeof(arr) / sizeof(*arr))
#define UNUSED(v) ((void)(v))

static WINDOW *main_win;

struct sensor_gui *init_gui(void)
{
	main_win = initscr();
	refresh();
	nodelay(main_win, 1);
	noecho();
	curs_set(0);

	start_color();
	use_default_colors();


	struct sensor_gui *gui = malloc(sizeof(*gui));
	assert(gui != NULL);

	//attron(A_STANDOUT); 
	//mvvline(1, 1, ACS_VLINE, 11);
	//mvvline(1, 25 * 3 + 5, ACS_VLINE, 11);
	//mvhline(1, 1, ACS_HLINE, 25 * 3 + 5);
	//mvhline(11 , 1, ACS_HLINE, 25 * 3 + 5);
	//mvaddch(1, 1, ACS_ULCORNER);
	//mvaddch(1, 25 * 3 + 5, ACS_URCORNER);
	//mvaddch(11, 1, ACS_LLCORNER);
	//mvaddch(11, 25 * 3 + 5, ACS_LRCORNER);
	//mvaddstr(1, 35, "Live-Data");
	//attroff(A_STANDOUT); 
	//refresh();

	gui->accel_display.height = 7;
	gui->accel_display.width = 20;
	gui->accel_display.xloc = 5;
	gui->accel_display.yloc = 3;
	gui->accel_display.xpad = 2;
	gui->accel_display.ypad = 2;

	gui->accel_display.win = newwin(gui->accel_display.height,
					gui->accel_display.width,
					gui->accel_display.yloc,
					gui->accel_display.xloc);

	assert(gui->accel_display.win != NULL);


	gui->gyro_display.height = 7;
	gui->gyro_display.width = 20;
	gui->gyro_display.xloc = gui->accel_display.xloc + gui->accel_display.width + 5;
	gui->gyro_display.yloc = gui->accel_display.yloc;
	gui->gyro_display.xpad = 2;
	gui->gyro_display.ypad = 2;

	gui->gyro_display.win = newwin(gui->gyro_display.height,
				       gui->gyro_display.width,
				       gui->gyro_display.yloc,
				       gui->gyro_display.xloc);

	assert(gui->gyro_display.win != NULL);


	gui->angle_display.height = 7;
	gui->angle_display.width = 20;
	gui->angle_display.xloc = gui->gyro_display.xloc + gui->gyro_display.width + 5;
	gui->angle_display.yloc = gui->accel_display.yloc;
	gui->angle_display.xpad = 2;
	gui->angle_display.ypad = 2;

	gui->angle_display.win = newwin(gui->angle_display.height,
				       gui->angle_display.width,
				       gui->angle_display.yloc,
				       gui->angle_display.xloc);

	assert(gui->angle_display.win != NULL);

	gui->avg_display.height = 3;
	gui->avg_display.width = 44;
	gui->avg_display.xloc = gui->accel_display.xloc + gui->accel_display.width /2;
	gui->avg_display.yloc = gui->accel_display.yloc + gui->accel_display.height + 2;
	gui->avg_display.xpad = 2;
	gui->avg_display.ypad = 1;

	gui->avg_display.win = newwin(gui->avg_display.height,
				      gui->avg_display.width,
				      gui->avg_display.yloc,
				      gui->avg_display.xloc);


	return gui;
}

void free_gui(struct sensor_gui *gui)
{
	if(gui == NULL)
		return;

	free(gui);
	endwin();
}

void update_avg_window(struct sensor_gui *gui, struct vec3f *avg)
{
	static int cnt = 0;

	werase(gui->avg_display.win);
	wattron(gui->avg_display.win, A_BOLD);

	box(gui->avg_display.win, 0, 0);
	mvwprintw(gui->avg_display.win, 0, 2, "Saved average: #%d ", cnt++);

	int i = 0;
	mvwprintw(gui->avg_display.win,
		  gui->avg_display.ypad,
		  gui->avg_display.xpad + i,
		  "x: %+.4f", avg->x);
	
	i+=15;

	mvwprintw(gui->avg_display.win,
		  gui->avg_display.ypad,
		  gui->avg_display.xpad + i,
		  "y: %+.4f", avg->y);

	i+=15;

	mvwprintw(gui->avg_display.win,
		  gui->avg_display.ypad,
		  gui->avg_display.xpad + i,
		  "z: %+.4f", avg->z);
		  

	wattroff(gui->avg_display.win, A_BOLD);
	wrefresh(gui->avg_display.win);
}

void update_angle_window(struct sensor_gui *gui, struct vec3f *angle)
{
	werase(gui->angle_display.win);
	wattron(gui->angle_display.win, A_BOLD);

	box(gui->angle_display.win, 0, 0);
	mvwprintw(gui->angle_display.win, 0, 2, "Rotation Angle");

	int i = 0;
	mvwprintw(gui->angle_display.win,
		  gui->angle_display.ypad + i++,
		  gui->angle_display.xpad,
		  "x: %+9.4f", angle->x);

	mvwprintw(gui->angle_display.win,
		  gui->angle_display.ypad + i++,
		  gui->angle_display.xpad,
		  "y: %+9.4f", angle->y);

	mvwprintw(gui->angle_display.win,
		  gui->angle_display.ypad + i++,
		  gui->angle_display.xpad,
		  "z: %+9.4f", angle->z);
		  

	wattroff(gui->angle_display.win, A_BOLD);
	wrefresh(gui->angle_display.win);
}


void update_gyro_window(struct sensor_gui *gui, struct vec3f *gyro)
{
	werase(gui->gyro_display.win);
	wattron(gui->gyro_display.win, A_BOLD);

	box(gui->gyro_display.win, 0, 0);
	mvwprintw(gui->gyro_display.win, 0, 2, "Angular Velocity");

	int i = 0;
	mvwprintw(gui->gyro_display.win,
		  gui->gyro_display.ypad + i++,
		  gui->gyro_display.xpad,
		  "x: %+9.4f", gyro->x);

	mvwprintw(gui->gyro_display.win,
		  gui->gyro_display.ypad + i++,
		  gui->gyro_display.xpad,
		  "y: %+9.4f", gyro->y);

	mvwprintw(gui->gyro_display.win,
		  gui->gyro_display.ypad + i++,
		  gui->gyro_display.xpad,
		  "z: %+9.4f", gyro->z);

	wattroff(gui->gyro_display.win, A_BOLD);
	wrefresh(gui->gyro_display.win);
}


void update_accel_window(struct sensor_gui *gui, struct vec3f *accel)
{
	werase(gui->accel_display.win);
	wattron(gui->accel_display.win, A_BOLD);
	
	box(gui->accel_display.win, 0, 0);
	mvwprintw(gui->accel_display.win, 0, 2, "Acceleration");

	int i = 0;
	mvwprintw(gui->accel_display.win,
		  gui->accel_display.ypad + i++,
		  gui->accel_display.xpad,
		  "x: %+9.4f", accel->x);

	mvwprintw(gui->accel_display.win,
		  gui->accel_display.ypad + i++,
		  gui->accel_display.xpad,
		  "y: %+9.4f", accel->y);

	mvwprintw(gui->accel_display.win,
		  gui->accel_display.ypad + i++,
		  gui->accel_display.xpad,
		  "z: %+9.4f", accel->z);

	wattroff(gui->accel_display.win, A_BOLD);
	wrefresh(gui->accel_display.win);
}
