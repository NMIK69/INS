#include "serial_io.h"
#include <fcntl.h> 
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <assert.h>

static void set_bit_per_byte(struct termios *tty, uint8_t bpb);
static void set_parity_bit(struct termios *tty, uint8_t pb);
static void set_stop_bits(struct termios *tty, uint8_t sb);
static void set_hardware_flowcontroll(struct termios *tty, uint8_t hwfc);
static void set_non_canonical_mode(struct termios *tty);

/* most of the tty settings are taken from:
 * https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c?noredirect=1&lq=1
 * */
int serial_open_port(Serial_Settings *set)
{
	struct termios tty;

	int fd = open(set->portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0) {
		fprintf(stderr, "ERROR: can't open file: %s\n", set->portname);
	}
	assert(fd >= 0);

	assert(tcgetattr(fd, &tty) == 0);
	
	assert(cfsetospeed(&tty, set->baudrate) == 0);
	assert(cfsetispeed(&tty, set->baudrate) == 0);

	/* ignore modem controls */
	tty.c_cflag |= (CLOCAL | CREAD);

	set_bit_per_byte(&tty, set->bit_per_byte);
	set_parity_bit(&tty, set->parity_bit);
	set_stop_bits(&tty, set->n_stop_bits);
	set_hardware_flowcontroll(&tty, set->hw_flowcontrol);
	set_non_canonical_mode(&tty);
		
	/* block until there are at least *nbytes_wait* bytes to read */
	tty.c_cc[VMIN] = set->nbytes_wait;
	/* set wait timeout. 0 means no timeout. */
	tty.c_cc[VTIME] = set->timeout;
	
	assert(tcsetattr(fd, TCSANOW, &tty) == 0);

	return fd;
}


static void set_bit_per_byte(struct termios *tty, uint8_t bpb)
{
	assert(bpb == 5 ||
	       bpb == 6 ||
	       bpb == 7 ||
	       bpb == 8);

	tty->c_cflag &= ~CSIZE;

	switch(bpb) {
	case 5:
		tty->c_cflag |= CS5;
		break;
	case 6:
		tty->c_cflag |= CS6;
		break;
	case 7:
		tty->c_cflag |= CS7;
		break;
	case 8:
		tty->c_cflag |= CS8;
		break;
	default:
		assert(0);
	}
}

static void set_parity_bit(struct termios *tty, uint8_t pb)
{
	assert(pb == 1 || pb == 0);

	if(pb == 1) {
		tty->c_cflag |= PARENB;
	} else {
		tty->c_cflag &= ~PARENB;
	}
}

static void set_stop_bits(struct termios *tty, uint8_t sb)
{
	assert(sb == 1 || sb == 2);

	if(sb == 1) {
		tty->c_cflag &= ~CSTOPB;
	} else {
		/* two stop bits */
		tty->c_cflag |= CSTOPB;
	}
}

static void set_hardware_flowcontroll(struct termios *tty, uint8_t hwfc)
{
	assert(hwfc == 1 || hwfc == 0);

	if(hwfc == 1) {
		tty->c_cflag |= CRTSCTS;
	} else {
		tty->c_cflag &= ~CRTSCTS;
	}
}

static void set_non_canonical_mode(struct termios *tty)
{
	tty->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty->c_oflag &= ~OPOST;
}

