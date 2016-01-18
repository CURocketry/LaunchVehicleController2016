#include "crt_common.h"

int msleep(unsigned int msecs) {
    return usleep(msecs * 1000);
}


