#ifndef __PACKET_DEF_H__
#define __PACKET_DEF_H__

#include <stdint.h>


// commands
typedef uint8_t commands_t;

#define CD_EN_CAMERA    0
#define CD_DS_CAMERA    1
#define CD_EN_TRANS     2
#define CD_DS_TRANS     3
#define CD_MAX_FREQ     4
#define CD_MIN_FREQ     5
#define CD_EN_LAUNCH    6
#define CD_DS_LAUNCH    7

//flags
typedef uint8_t flags_t;
#define FG_TEST          1
#define FG_GPS_FIX       1 << 1
#define FG_CAMERA        1 << 2 
#define FG_TRANS_FREQ    1 << 3
#define FG_LAUNCH        1 << 4
#define FG_LANDED        1 << 5

typedef struct {
  int32_t latitude;
  int32_t longitude;
  int16_t altitude;
  flags_t flags;
  int16_t gyro_z;
  int8_t acc_z;
  int8_t acc_x;
  int8_t acc_y;
  uint8_t temp;
} Payload;

#endif
