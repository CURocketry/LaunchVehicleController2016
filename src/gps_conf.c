#include <string.h> 
#include "gps_conf.h"
#include <serial.h>

void sendCommand(int handle, const char* cmd) {
    int size = strlen(cmd);
    
    // write must be split up to work properly
    int r = write(handle, cmd, size);
    write(handle, "\r\n",2);
    
    if (r != size) {
        printf("ERROR: did not correctly write %s\n", cmd);
    }
    else {
        printf("SET: %s\n",cmd);
    }
    usleep(10000);
}

int main() {
    char *portname = "/dev/adagps";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }

    set_interface_attribs (fd, B9600, 0); // set speed to 9600 bps, 8n1 (no parity)
    set_blocking (fd, 0);                 // set no blocking

    sendCommand (fd, PMTK_SET_NMEA_OUTPUT_RMCGGA); //RMC (recommended minimum) and GGA (fix data) including altitude 
    sendCommand (fd, PMTK_SET_NMEA_UPDATE_5HZ); // 5 HZ update frequency 
    sendCommand (fd, PGCMD_ANTENNA); //request updates on antenna status
    
    printf("Done!\n");

    /*char buf [100];
    int n = read (fd, buf, sizeof buf);  	// read up to 100 characters if ready to read
    */
    
    close(fd);
    return 0;
}
