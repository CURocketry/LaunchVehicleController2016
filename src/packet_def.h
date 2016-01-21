#ifndef __PACKET_DEF_H__
#define __PACKET_DEF_H__

#include <stdint.h>

//make sure these match on the receiving end
#define MARKER_LAT 0xFB
#define MARKER_LON 0xFC
#define MARKER_ALT 0xFD
#define MARKER_FLAG 0xFE

//directives received from ground station
//make sure these match on the sending end
#define DIR_TEST 0xAB
#define DIR_BEGIN_LAUNCH 0xAC
#define DIR_PAYLOAD_ABORT 0xAD
#define DIR_PAYLOAD_ABORT_CANCEL 0xAE

//flags
typedef uint8_t flags_t;
#define FG_TEST          1
#define FG_LANDED        1 << 1
#define FG_MAIN_LAUNCH   1 << 2
#define FG_PAYLOAD_ABORT 1 << 3
#define FG_GPS_FIX       1 << 4

typedef struct {
  uint32_t latitude;
  uint32_t longitude;
  int16_t altitude;
  flags_t flags;
} Payload;

#endif
