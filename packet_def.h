#ifndef _PACKET_DEF_H
#define _PACKET_DEF_H

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

typedef struct {
  unsigned int gps_fix : 1;
  unsigned int payload_abort : 1;
  unsigned int main_launch : 1;
  unsigned int landed : 1;
  unsigned int test : 1;
  unsigned int padding : 3;
} Flags;

typedef struct {
  long latitude;
  long longitude;
  int altitude;
  Flags flags;
} Payload;

#endif
