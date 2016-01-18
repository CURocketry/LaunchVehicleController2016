#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <crt_common.h>
#include <xbee.h>
#include <gps.h>

#include "packet_def.h"

#define IGNORE_GPS true;
typedef unsigned char byte;

#define LAUNCH_TX_PD 250;
#define STANDBY_TX_PD 3000;

byte* stream(byte marker, void* ptr, void *n, int n_size); 
void set_gps_payload(Payload *payload, struct gps_fix_t data);

//Payload variables
#define MAX_BUF 64  // Maximum payload size 

Payload payload;
Payload* ptr_payload = &payload;

int tx_pd = STANDBY_TX_PD; //time between message transmission, in ms


//receiving callback
void cbReceive(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
    if ((*pkt)->dataLen > 0) {
		if ((*pkt)->data[0] == 0xAA) {
			xbee_conCallbackSet(con, NULL, NULL);
			printf("*** DISABLED CALLBACK... ***\n");
		}
		printf("rx: [0x%02X]\n", (unsigned int)((*pkt)->data[0]));
	}
	printf("tx: %d\n", xbee_conTx(con, NULL, "ACK\r\n"));
}

int main(void) {
    
    int buf_size; //actual size of buffer
    byte* buf_start = (byte*) malloc( MAX_BUF * sizeof(byte) ); //allocate starting payload pointer
    byte* buf_curr; //current position of buffer
    struct gps_data_t gps_data; 
    struct xbee *xbee;
    struct xbee_con *con;
    struct xbee_conAddress address;
    xbee_err ret;

#ifndef IGNORE_GPS
    //connect to the gps
    if ((ret = gps_open("localhost", "2947", &gps_data)) == -1) {
        printf("code: %d, reason: %s\n", ret, gps_errstr(ret));
        return EXIT_FAILURE;
    }

    gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
#endif
    // connect to the xbee
    if ((ret = xbee_setup(&xbee, "xbee3", "/dev/xbee", 57600)) != XBEE_ENONE) {
        printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

	memset(&address, 0, sizeof(address));
	address.addr64_enabled = 1;
	address.addr64[0] = 0x00;
	address.addr64[1] = 0x13;
	address.addr64[2] = 0xA2;
	address.addr64[3] = 0x00;
	address.addr64[4] = 0x40;
	address.addr64[5] = 0xBF;
	address.addr64[6] = 0x56;
	address.addr64[7] = 0xA5;
	if ((ret = xbee_conNew(xbee, &con, "Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(con, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return ret;
	}

	if ((ret = xbee_conCallbackSet(con, cbReceive, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return ret;
	}
    
    for (;;) {
        void *p;


        //break if receiving callback is disconnected
		if ((ret = xbee_conCallbackGet(con, (xbee_t_conCallback*)&p)) != XBEE_ENONE) {
			xbee_log(xbee, -1, "xbee_conCallbackGet() returned: %d", ret);
			return ret;
		}

		if (p == NULL) break;
        
        payload.latitude = 0;
        payload.longitude = 0;
        payload.altitude = 0;
        payload.flags &= ~FG_GPS_FIX;
       
#ifndef IGNORE_GPS
        // wait for 200ms to receive data 
        if (gps_waiting (&gps_data, 200000)) {
        /* read data */
        if ((ret = gps_read(&gps_data)) == -1) {
            printf("error occured reading gps data. code: %d, reason: %s\n", ret, gps_errstr(ret));
        } else {
            /* Display data from the GPS receiver. */
            if ((gps_data.status == STATUS_FIX) && 
                (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
                !isnan(gps_data.fix.latitude) && 
                !isnan(gps_data.fix.longitude)) {
                    printf("latitude: %f, longitude: %f, speed: %f, altitude: %f\n", gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, gps_data.fix.altitude);
                set_gps_payload(&payload, gps_data.fix);
            } else {
                printf("no GPS data available\n");
            }
        }
    }
#endif

        //clear buffer
        memset( buf_start, 0, MAX_BUF);


        //buf_start is pointer to first allocated space
        //buf_curr is pointer to next free space
        //if (payload.latitude != 0 && payload.longitude != 0 && payload.altitude != 0) {
        buf_curr = stream(MARKER_LAT, buf_start, &(ptr_payload->latitude), sizeof(payload.latitude));
        buf_curr = stream(MARKER_LON, buf_curr,&(ptr_payload->longitude), sizeof(payload.longitude));
        buf_curr = stream(MARKER_ALT, buf_curr,&(ptr_payload->altitude), sizeof(payload.altitude));
          //}
          
          buf_curr = stream(MARKER_FLAG, buf_curr,&(ptr_payload->flags), sizeof(payload.flags));

        //buffer size is the difference between the current and start pointers
        buf_size = buf_curr - buf_start;
        
        printh(buf_start, buf_size);

        //rpi is little endian (LSB first)
        unsigned char retVal;
        if ((ret = xbee_connTx(con, &retVal, buf_start, buf_size)) != XBEE_ENONE) {
            if (ret == XBEE_ETX) {
                printf("tx error (0x%02X)\n", retVal);
            } else {
                printf("error: %s\n", xbee_errorToStr(ret));
            }
        }
        msleep(50);
    }


	if ((ret = xbee_conEnd(con)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return ret;
	}


#ifndef IGNORE_GPS
    gps_stream(&gps_data, WATCH_DISABLE, NULL);
    gps_close (&gps_data);
#endif
	xbee_shutdown(xbee);

	return 0;
}


//stream n_size bytes in n to mem loc ptr
byte* stream(byte marker, void* ptr, void *n, int n_size) {
  //cast ptr to a byte to allow arithmetic
  byte* p = (byte*)ptr;

  *p = marker; //insert marker
  p++;

  memcpy( p, n, n_size ); //copy size bytes of n into p
  p += n_size; //increment pointer p

  return( p );
}


// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);

  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );

  return decDeg;
}


void set_gps_payload(Payload *payload, struct gps_fix_t data) {
  payload->latitude = (uint32_t) data.latitude*10000;
  payload->longitude = (uint32_t) data.longitude*10000;
  payload->altitude = (uint16_t) data.altitude;
}

