#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include <crt_common.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <xbee.h>
#include <gps.h>
#include <zlog.h>

#include "packet_def.h"

typedef unsigned char byte;

//initialization status flags
#define INIT_GPS 0
#define INIT_GYRO 1
#define INIT_ACCEL 2
#define INIT_BMP 3

//state status flags

// function prototypes
byte* stream(void* ptr, void *n, int n_size); 
void set_gps_payload(Payload *payload, struct gps_fix_t data);
void set_imu_payload(Payload *payload, struct gyro_t *gyro, struct accel_t *accel, struct bmp_t *bmp); 
int  _xbee_startup(struct xbee **xbee, struct xbee_con **con,struct  xbee_conAddress *address);
bool enable_camera();
bool disable_camera();

