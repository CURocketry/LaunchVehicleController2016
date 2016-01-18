#include "serial.h"
/**
The values for speed are B115200, B230400, B9600, B19200, B38400, B57600, B1200, 
B2400, B4800, etc. The values for parity are 0 (meaning no parity), 
PARENB|PARODD (enable parity and use odd), PARENB (enable parity and use even), 
PARENB|PARODD|CMSPAR (mark parity), and PARENB|CMSPAR (space parity).

"Blocking" sets whether a read() on the port waits for the specified number of 
characters to arrive. Setting no blocking means that a read() returns however 
many characters are available without waiting for more, up to the buffer limit.

Addendum:

CMSPAR is needed only for choosing mark and space parity, which is uncommon. 
For most applications, it can be omitted. My header file /usr/include/bits/termios.h 
enables definition of CMSPAR only if the preprocessor symbol __USE_MISC is defined. 
That definition occurs (in features.h) with

#if defined _BSD_SOURCE || defined _SVID_SOURCE
 #define __USE_MISC     1
#endif

The introductory comments of <features.h> says:

/* These are defined by the user (or the compiler)
   to specify the desired environment:

...
   _BSD_SOURCE          ISO C, POSIX, and 4.3BSD things.
   _SVID_SOURCE         ISO C, POSIX, and SVID things.
...
 */

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
		                        // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
		                        // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            printf ("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}
