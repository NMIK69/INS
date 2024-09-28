#ifndef SERIAL_IO_H
#define SERIAL_IO_H
#include <termios.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>

typedef struct
{
	const char *portname;
	speed_t baudrate;
	uint8_t hw_flowcontrol;
	uint8_t n_stop_bits;
	uint8_t parity_bit;
	uint8_t bit_per_byte;
	size_t nbytes_wait;
	size_t timeout;

} Serial_Settings;

int serial_open_port(Serial_Settings *set);

#endif
