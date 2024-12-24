#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <ctype.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <ncurses.h>

#include "serial_io.h"
#include "sensor_gui.h"
#include "quaternion.h"

#define MPU_MEA_START 0xAA
#define MPU_MEA_LEN ((sizeof(uint8_t)) +\
		     (sizeof(float)* 6) +\
		     (sizeof(float)* 4) +\
		     (sizeof(uint32_t)))

#define ACCEL_CONV 8192.0
#define GYRO_CONV 16.4 
#define AVG_CNT_NUM 250

static void left_shift(size_t n);
static void fetch_start(void);
static void convert(size_t i);
static void display(void);
static void save_to_file(void);
static void process(const uint8_t *tmp, size_t tmp_len);
static void read_inf(int serial_fd);
static int handle_cmdln_args(int argc, char **argv);
static void print_usage(const char *name);

static void save_vec_to_file(struct vec3f *v);

struct mpu6050_measurement
{
	struct vec3f accel;
	struct vec3f gyro;
	struct quaternion q;
	float temperature;
	uint32_t timestamp;

};

static uint8_t raw_data[5000];
static size_t raw_data_cap = sizeof(raw_data) / sizeof(*raw_data);
static size_t raw_data_len = 0;
static char *out_file_name;
static FILE *out_file;

/* a single measurement. */
static struct mpu6050_measurement mea;

static uint8_t save_all = 0;
static uint8_t save_for_cal = 0;
static uint8_t save_static = 0;
static char *dev_name = NULL;


static struct sensor_gui *gui;


int main(int argc, char **argv)
{
	if(handle_cmdln_args(argc, argv) != 0) 
		return 0;
	
	if(dev_name == NULL) {
		print_usage(argv[0]);
		return 0;
	}


	Serial_Settings ser_set = {
    			.portname = dev_name,
			.baudrate = B230400,
			.hw_flowcontrol = 0,
			.n_stop_bits = 1,
			.parity_bit = 0,
			.bit_per_byte = 8,
			.nbytes_wait = 1,
			.timeout = 0
			};

	int serial_fd = serial_open_port(&ser_set);

	if(out_file_name != NULL)
		out_file = fopen(out_file_name, "w");
	else
		out_file = fopen("imu_out.txt", "w");
	assert(out_file != NULL);

	gui = init_gui();

	read_inf(serial_fd);

	free_gui(gui);

	fclose(out_file);

	close(serial_fd);
	
	return 0;
}

static void read_inf(int serial_fd)
{
	uint8_t tmp[1024];
	size_t tmp_size = sizeof(tmp);
	ssize_t rdlen;

	char c = 0;
	while(1) {
		c = getch();
		if(c == 'q')
			break;
		if(c == 'c')
			save_for_cal = 1;
		if(c == 's')
			save_static = 1;
		
		rdlen = read(serial_fd, tmp, tmp_size);               

		assert(rdlen > 0);	

		process(tmp, rdlen);
	}
}


static void write_static_marker(void)
{
	fprintf(out_file, "XXX\n");

	assert(ferror(out_file) == 0);

}
static void save_cal(void)
{
	static int avg_cnt = 0;
	static struct vec3f aavg = {0};

	if(save_static == 1) {
		if(avg_cnt == AVG_CNT_NUM) {
			aavg.x /= (float)AVG_CNT_NUM;
			aavg.y /= (float)AVG_CNT_NUM;
			aavg.z /= (float)AVG_CNT_NUM;

			save_vec_to_file(&aavg);
			write_static_marker();

			update_avg_window(gui, &aavg);

			avg_cnt = 0;
			save_static = 0;
			aavg.x = 0.0f;
			aavg.y = 0.0f;
			aavg.z = 0.0f;
		}
		else {
			avg_cnt += 1;
			aavg.x += mea.accel.x;
			aavg.y += mea.accel.y;
			aavg.z += mea.accel.z;
		}
	}

	else {
		save_vec_to_file(&(mea.gyro));
	}
}

static void process(const uint8_t *tmp, size_t tmp_len)
{
	assert(tmp_len + raw_data_len < raw_data_cap);	

	memcpy(&raw_data[raw_data_len], tmp, tmp_len);

	raw_data_len += tmp_len;

	fetch_start();

	
	size_t i = 0;
	while(i + MPU_MEA_LEN <= raw_data_len && raw_data[i] == MPU_MEA_START) {
		
		convert(i + 1);

		display();
	
		if(save_all == 1)
			save_to_file();

		if(save_for_cal == 1)
			save_cal();

		

		i += MPU_MEA_LEN;
	}

	left_shift(i);
}


static void save_vec_to_file(struct vec3f *v)
{
	fprintf(out_file, 
		"%.3f,%.3f,%.3f\n",
		v->x, v->y, v->z);

	assert(ferror(out_file) == 0);
}

static void save_to_file()
{

	fprintf(out_file, 
		"%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
		mea.timestamp,
		mea.accel.x, mea.accel.y, mea.accel.z,
		mea.gyro.x, mea.gyro.y, mea.gyro.z);

	assert(ferror(out_file) == 0);
}



static void display()
{
	struct vec3f a = {.x=mea.accel.x, .y=mea.accel.y, .z=mea.accel.z};
	struct vec3f g = {.x=mea.gyro.x, .y=mea.gyro.y, .z=mea.gyro.z};
	
	struct vec3f o = quat_to_euler(&mea.q);

	update_accel_window(gui, &a);
	update_gyro_window(gui, &g);
	update_angle_window(gui, &o);
}

static void left_shift(size_t n)
{
	for(size_t i = n; i < raw_data_len; i++) {
		raw_data[i - n] = raw_data[i];		
	}
	raw_data_len -= n;
}

static void fetch_start()
{
	size_t i = 0;
	while(i < raw_data_len && raw_data[i] != MPU_MEA_START) {
		i += 1;
	}

	if(i > 0) {
		assert(i <= raw_data_len);
		left_shift(i);
	}
}

static void convert(size_t i)
{
	float *fp = (float *)(&raw_data[i]);
	uint32_t *up = (uint32_t *)(&raw_data[i]);

	mea.accel.x = fp[0];
	mea.accel.y = fp[1];
	mea.accel.z = fp[2];
	mea.gyro.x = fp[3];
	mea.gyro.y = fp[4];
	mea.gyro.z = fp[5];

	mea.q.w = fp[6];
	mea.q.x = fp[7];
	mea.q.y = fp[8];
	mea.q.z = fp[9];
	
	mea.timestamp = up[10];
}

static void print_usage(const char *name)
{
	fprintf(stderr, "Usage: %s [options] \n"
			"options:\n"
	                "\t -d \t Device driver of the connected STM32 (-d /dev/tty...)\n\n"
			"\t -s \t Save all the received data. By default a file named \n"
			"\t\t imu_out.txt is created. use -o to specify a file name.\n"
			"\t\t Format: ax,ay,az,gx,gy,gz\\n \n\n"
			"\t -o \t The name of the ouput file. (-o my_file.txt)\n\n"
			"\t -h \t Print this text.\n",
			name);
}

static int handle_cmdln_args(int argc, char **argv)
{
	opterr = 0;
	int op;

	while((op = getopt(argc, argv, "h:o:d:s")) != -1) {
		switch(op) {
		case 'o':
			out_file_name = optarg;
			break;
		case 'd':
			dev_name = optarg;
			break;
		case 's':
			save_all = 1;
			break;
		case 'h':
			goto usage_err;
			break;
		default:
			goto usage_err;

		}
	}

	return 0;

usage_err:
	print_usage(argv[0]);
	return -1;
}

