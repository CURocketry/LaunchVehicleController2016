#include "main.h"
#include <pthread.h>

//#define DEBUG_IGNORE_GPS
//#define DEBUG_IGNORE_IMU

int zl_conf;
zlog_category_t *zl_prog;
zlog_category_t *zl_data;

#define STANDBY_TIME_MIN_MS 450 //delay between tranmissions in ms
#define STANDBY_TIME_MAX_MS 500 //5000 
int standby_time = STANDBY_TIME_MAX_MS;

//Payload variables
#define MAX_BUF 32  // Maximum payload size 

init_ok_t init_status;
Payload payload;
flags_t flags;
Payload* ptr_payload = &payload;
bool connected;
bool transmit = true; //whether to transmit data

int xbee_ret;
//camera stuff
pid_t cam_pid = -1;
bool cam_started;
FILE* cam_f;
char *cam_fpath;

// landing autodetection
#define ALT_OFFSET_CEIL_M 800 //ceiling offset from current altitude
#define ALT_DELTA_MAX_M 10 //max fluctuation to determine landing condition 
bool calibrate = false;
int prev_alt = 0; //previous altitude
int alt_calib = 0;
int mov_avg_delta = 0; //moving average of change in altitude
bool en_auto_detect_landing  = false;
bool exceed_ceiling = false;

bool has_gps;

//receiving callback
void cbReceive(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {
    if ((*pkt)->dataLen > 0) {
        byte cmd = (*pkt)->data[0];	
        zlog_debug(zl_prog,"rx: [0x%02X]", (unsigned int)(cmd));
        
        if (cmd >> CD_EN_CAMERA & 1) {
            zlog_debug(zl_prog,"CMD: Enable camera.");
            if (!cam_started) {
                cam_pid = fork();
                //child process      
                if (cam_pid == 0) {        
                    zlog_debug(zl_prog,"Child process running camera script");
                    Py_Initialize();
                    PyRun_SimpleFile(cam_f, cam_fpath);
                    Py_Finalize();
                    zlog_debug(zl_prog,"Child process done.");
                    exit(0);
                }
                cam_started = true;
            }
            else {
                zlog_debug(zl_prog,"Camera already started");
            }
            flags |= FG_CAMERA;
        }
        if (cmd >> CD_DS_CAMERA & 1) {
            cam_started = false;
            int res = kill(cam_pid, SIGKILL);
            zlog_info(zl_prog, "Kill child process with result %i",res);
            flags &= ~FG_CAMERA;
        }
        if (cmd >> CD_EN_TRANS & 1) {
            transmit = true;
            zlog_debug(zl_prog,"CMD: Enable transmission.");
        }
        if (cmd >> CD_DS_TRANS & 1) {
            transmit = false;
            zlog_debug(zl_prog,"CMD: Disable transmission.");
        }
        if (cmd >> CD_MAX_FREQ & 1) {
            standby_time = STANDBY_TIME_MIN_MS; 
            flags |= FG_TRANS_FREQ; 
            zlog_debug(zl_prog,"CMD: Max frequency.");
        }
        if (cmd >> CD_MIN_FREQ & 1) {
            standby_time = STANDBY_TIME_MAX_MS;
            flags &= ~FG_TRANS_FREQ;
            zlog_debug(zl_prog,"CMD: Min frequency.");
        }
        if (cmd >> CD_EN_LAUNCH & 1) {
            calibrate = true; 
            flags |= FG_LAUNCH;
            zlog_debug(zl_prog,"CMD: Enable launch.");
        }
        if (cmd >> CD_DS_LAUNCH & 1) {
            calibrate = false;
            prev_alt = 0; //previous altitude
            alt_calib = 0;
            mov_avg_delta = 0; //moving average of change in altitude
            en_auto_detect_landing  = false;
            exceed_ceiling = false;
            alt_calib = 0;
            flags &= ~FG_LAUNCH;
            zlog_debug(zl_prog,"CMD: Disable launch.");
        }
	}
}

int main_xbee(void *threadid) {
    byte* buf_start = (byte*) malloc( MAX_BUF * sizeof(byte) ); //allocate starting payload pointer
    int buf_size; //actual size of buffer
    struct xbee *xbee;
    struct xbee_con *con;
    struct xbee_conAddress address;
    xbee_err ret;
    connected = false;
    
    Payload* send_payload = malloc(sizeof(Payload));
    flags_t send_flags;

    // initialize the address of the remote xbee
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
    
    if ((xbee_ret = _xbee_startup(&xbee, &con, &address)) != XBEE_ENONE) {
        pthread_exit(&xbee_ret);
    }
    
    for (;;) {
        send_flags = flags;
        memcpy( send_payload, ptr_payload, sizeof(Payload));
        //if (!(send_payload->latitude == 0 && send_payload->longitude == 0)) {

        //clear buffer
        memset( buf_start, 0, MAX_BUF);

        //buf_start is pointer to first allocated space
        byte* buf_curr; //current position of buffer
        buf_curr = stream(buf_start, &(send_payload->latitude), sizeof(payload.latitude));
        buf_curr = stream(buf_curr,&(send_payload->longitude), sizeof(payload.longitude));
        buf_curr = stream(buf_curr,&(send_payload->altitude), sizeof(payload.altitude));
        buf_curr = stream(buf_curr,&(send_flags), sizeof(payload.flags));
        buf_curr = stream(buf_curr,&(send_payload->gyro_x), sizeof(payload.gyro_z));
        buf_curr = stream(buf_curr,&(send_payload->acc_z), sizeof(payload.acc_z));
        buf_curr = stream(buf_curr,&(send_payload->acc_x), sizeof(payload.acc_x));
        buf_curr = stream(buf_curr,&(send_payload->acc_y), sizeof(payload.acc_y));
        buf_curr = stream(buf_curr,&(send_payload->temp), sizeof(payload.temp));

        //buffer size is the difference between the current and start pointers
        buf_size = buf_curr - buf_start;
        //rpi is little endian (LSB first)
        unsigned char retVal;
        if (connected) {
            if (transmit) {
                if ((ret = xbee_connTx(con, &retVal, buf_start, buf_size)) != XBEE_ENONE) {
                    if (ret == XBEE_ETX) {
                        zlog_error(zl_prog, "xbee tx error: 0x%02X",retVal);
                    } 
                    else if (ret == XBEE_ETIMEOUT || ret == XBEE_EINUSE || ret == XBEE_EINVAL) {
                        // timeout workaround - disconnect and reconnect
                        zlog_error(zl_prog, "xbee tx error: %s", xbee_errorToStr(ret));
                        //xbee_conEnd(con);
                        //xbee_shutdown(xbee);
                        //connected = false;
                        //xbee_conCallbackSet(con, NULL, NULL);
                        ////usleep(50000);
                        //sleep(5);
                        //_xbee_startup(&xbee,&con,&address);
                    }
                    else {
                        zlog_error(zl_prog, "xbee tx error: %s", xbee_errorToStr(ret));
                    }
                }
           // }
        }
        else {
            _xbee_startup(&xbee,&con,&address);
        }
    
        //don't loop if child
        if (cam_pid == 0) {
            pthread_exit(NULL);
            //for(;;) {
                //sleep(10);
            //}
        }

    }    
        //printh(buf_start, buf_size);
        msleep(standby_time);
    }

	if ((ret = xbee_conEnd(con)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return ret;
	}
}

int main(void) { 
    struct gps_data_t gps_data; 
    struct gyro_t *gyro;
    struct accel_t *accel;
    struct bmp_t *bmp;

    int ret;
    long t;
    pthread_t xbee_th;
    // Init logger
    zl_conf = zlog_init("zlog.conf");
    if (zl_conf) {
        //try the installation location
        zl_conf = zlog_init("/var/lib/crtlvc/zlog.conf");
        if (zl_conf) {
            printf("Could not find zlog conf!\n");
            //return -2;
        }
    }

    zl_data = zlog_get_category("data");
    zl_prog = zlog_get_category("prog");
    if (!zl_data || !zl_prog) {
        printf("Error getting zlog categories!\n");
        zlog_fini();
        //return -2;
    }
    else {
        printf("zlog initialized\n");
        zlog_info(zl_prog, "### CRTLVC initialized. ###");
        zlog_info(zl_data, "### Started new data logging session ###");
    }
    
    // configure the attached gps
    if (system("/var/lib/crtlvc/gps_conf") != 0) {
        zlog_error(zl_prog,"Could not configure GPS.");
    }

    //initialize the camera stuff
    cam_fpath = malloc(35);
    cam_fpath = "camera.py";
    if ((cam_f = fopen(cam_fpath,"r")) == NULL) {
        cam_fpath = "/var/lib/crtlvc/camera.py";
        if ((cam_f = fopen(cam_fpath,"r")) == NULL) {
            zlog_error(zl_prog,"Could not find camera script!");
            printf("script not found");
        }
    }

    if (cam_f != NULL) {
        zlog_info(zl_prog,"Camera script located.");
        init_status |= (1 << INIT_CAMERA);
    }

    int rc = pthread_create(&xbee_th, NULL, (void*)&main_xbee, (void*)&t);

#ifndef DEBUG_IGNORE_IMU
    if (gyro_create(&gyro, 0, GYRO_RANGE_2000DPS)) {
        gyro_enableAutoRange(gyro, true);
        gyro_useRadians(gyro,false);
        zlog_info(zl_prog, "Gyroscope initialized.");
        init_status |= (1 << INIT_GYRO);
    } else {
        zlog_error(zl_prog, "Unable to initialize gyroscope! Skipping...");
    }

    
    if (accel_create(&accel, 1)) {
        accel_useEarthGravity(accel, false);
        zlog_info(zl_prog, "Accelerometer initialized.");
        init_status |= (1 << INIT_ACCEL);
    } else {
        zlog_error(zl_prog,"Unable to initialize accelerometer! Skipping...");
    }

    if (bmp_create(&bmp, 2)) {
        zlog_info(zl_prog, "Barometer initialized.");
        init_status |= (1 << INIT_BMP);
    } else {
        zlog_error(zl_prog,"Unable to initialize barometer! Skipping..."); 
    }

#endif

#ifndef DEBUG_IGNORE_GPS
    //connect to the gps
    if ((ret = gps_open("localhost", "2947", &gps_data)) == -1) {
        zlog_error(zl_prog, "Unable to connect to GPS! Skipping...");
        zlog_error(zl_prog, "code: %d, reason: %s\n", ret, gps_errstr(ret)); 
    } else {
        gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
        zlog_info(zl_prog, "GPSD daemon successfully connected.");
        init_status |= (1 << INIT_GPS);
    }

#endif
   
   
  
    for (;;) {

        memset( ptr_payload, 0, sizeof(Payload));
        payload.flags &= ~FG_GPS_FIX;
        //set the init status flag
        if ((init_status >> INIT_GPS) & (init_status >> INIT_GYRO)
            & (init_status >> INIT_ACCEL) & (init_status >> INIT_BMP) 
            & (init_status >> INIT_CAMERA) & 1){
            flags |= FG_INIT_OK; 
        }

#ifndef DEBUG_IGNORE_GPS
        if ( init_status >> INIT_GPS & 1) {
            // wait for 200ms to receive data 
            if (gps_waiting (&gps_data, 200000)) {
                /* read data */
                if ((ret = gps_read(&gps_data)) == -1) {
                    zlog_warn(zl_prog,"error occured reading gps data. code: %d, reason: %s\n", ret, gps_errstr(ret));
                } else {
                    /* Display data from the GPS receiver. */
                    if ((gps_data.status == STATUS_FIX) && 
                        (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
                        !isnan(gps_data.fix.latitude) && 
                        !isnan(gps_data.fix.longitude)) {
                            flags |= FG_GPS_FIX;
                            printf("latitude: %f, longitude: %f, speed: %f, altitude: %f\n", gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, gps_data.fix.altitude);
                            set_gps_payload(&payload, gps_data.fix);
                            has_gps = true;
                    } else {
                        zlog_debug(zl_prog,"no GPS data available.\n");
                        has_gps = false;
                    }
                }
            }
        }
        if (payload.latitude == 0 || payload.longitude == 0) {
            has_gps = false;
        }
#endif

#ifndef DEBUG_IGNORE_IMU
        set_imu_payload(&payload, gyro, accel, bmp);
#endif    

        //log flight data
        //if (has_gps) {
        zlog_info(zl_data,"%i,%i,%i,%i,%i,%i,%i,%i,%i,%i",payload.latitude,payload.longitude,
                payload.altitude,payload.gyro_x,payload.gyro_y,payload.gyro_z,payload.acc_x,
                payload.acc_y,payload.acc_z,payload.temp);
        //}

        //calibrate - set the altitude ceiling
        if (calibrate) {
            alt_calib = payload.altitude + ALT_OFFSET_CEIL_M;
            zlog_debug(zl_prog,"Calibrating altitude: New ceiling %i",alt_calib);
            exceed_ceiling = false;
            en_auto_detect_landing = true;
            calibrate = false;
        }
        
        //auto-detect landing is enabled
        if (en_auto_detect_landing) {
            //ceiling exceeded
            if (!exceed_ceiling && payload.altitude > alt_calib) {
                exceed_ceiling = true;
            }
            if (exceed_ceiling) {
                const int mov_avg_pd = 128;
                mov_avg_delta = mov_avg_delta + (payload.altitude - prev_alt) - mov_avg_delta / mov_avg_pd;
                mov_avg_delta = mov_avg_delta / mov_avg_pd;
                
                //altitude has stabilized
                if (mov_avg_delta < ALT_DELTA_MAX_M) {
                    standby_time = STANDBY_TIME_MAX_MS;
                    flags |= FG_LANDED;
                    en_auto_detect_landing = false;
                }
                prev_alt = payload.altitude;
            }
        }


        //don't loop if child
        if (cam_pid == 0) {
            for(;;) {
                //sleep(10);
            }
        }

        msleep(250);
        
    }




#ifndef DEBUG_IGNORE_GPS
    if (init_status >> INIT_GPS & 1) {
        gps_stream(&gps_data, WATCH_DISABLE, NULL);
        gps_close (&gps_data);
    }
#endif
	
    //xbee_shutdown(xbee);
    zlog_fini();
	return 0;
}


//stream n_size bytes in n to mem loc ptr
byte* stream(void* ptr, void *n, int n_size) {
  //cast ptr to a byte to allow arithmetic
  byte* p = (byte*)ptr;
  memcpy( p, n, n_size ); //copy size bytes of n into p
  p += n_size; //increment pointer p

  return( p );
}


int  _xbee_startup(struct xbee **xbee, struct xbee_con **con, struct xbee_conAddress *address) {
    xbee_err ret;
    struct xbee_conSettings conset;
    zlog_debug(zl_prog, "xbee start setup");
    // connect to the xbee
    if ((ret = xbee_setup(xbee, "xbee3", "/dev/xbee", 57600)) != XBEE_ENONE) {
        zlog_error(zl_prog, "ret: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
    }
	
    xbee_logLevelSet(*xbee,5);
	
    if ((ret = xbee_conNew(*xbee, con, "Data", address)) != XBEE_ENONE) {
		xbee_log(*xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	if ((ret = xbee_conDataSet(*con, *xbee, NULL)) != XBEE_ENONE) {
		xbee_log(*xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return ret;
	}

    xbee_conSettings(*con, NULL, &conset);
    conset.noWaitForAck = 1;
    xbee_conSettings(*con,  &conset, NULL);

	if ((ret = xbee_conCallbackSet(*con, cbReceive, NULL)) != XBEE_ENONE) {
		xbee_log(*xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return ret;
	}
    zlog_debug(zl_prog, "xbee setup done\n");
    connected = true;
    return XBEE_ENONE;
}

void set_gps_payload(Payload *payload, struct gps_fix_t data) {
  payload->flags |= FG_GPS_FIX; 
  payload->latitude = (int32_t) (data.latitude*10000);
  payload->longitude = (int32_t) (-1*data.longitude*10000);
  payload->altitude = (int16_t) data.altitude;
}


void set_imu_payload(Payload *payload, struct gyro_t *gyro, struct accel_t *accel, struct bmp_t *bmp) { 
    sensors_event_t event;
    float temp;

    if (init_status >> INIT_GYRO & 1) {
        gyro_getEvent(gyro, &event);
        payload->gyro_x = (int16_t)(event.gyro.x);
        payload->gyro_y = (int16_t)(event.gyro.y);
        payload->gyro_z = (int16_t)(event.gyro.z);
    }

    if (init_status >> INIT_ACCEL & 1) {
        accel_getEvent(accel, &event);
        payload->acc_x = (int16_t)(event.acceleration.x*100);
        payload->acc_y = (int16_t)(event.acceleration.y*100);
        payload->acc_z = (int16_t)(event.acceleration.z*100);
    }

    if (init_status >> INIT_BMP & 1) {
        bmp_getEvent(bmp, &event);
        bmp_getTemperature(bmp, &temp);
        payload->temp = bmp_celsiusToFahrenheit(temp);
        if (payload->altitude == 0 && event.pressure) {
            //fallback: approximate altitude using average sea level pressure
            float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
            payload->altitude = (int16_t) bmp_pressureToAltitude(seaLevelPressure,event.pressure);
        }
    }

}
