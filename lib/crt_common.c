#include "crt_common.h"

int msleep(unsigned int msecs) {
    return usleep(msecs * 1000);
}

void printh(unsigned char *buf, int size) {
    for(int i=0; i<size; i++) {
        printf("0x%02X ",*buf);
        buf++;
    }
    printf("\n");
}
