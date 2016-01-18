#ifndef _SERIAL_H
#define _SERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

#endif
