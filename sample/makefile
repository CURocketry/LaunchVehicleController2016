LIBDIR:=../lib
BUILDLIBDIR:=../build/lib

all: gps imu_gyro

clean:
	-rm -rf build

gps: gps.c
	mkdir -p build
	gcc $(filter %.c, $^) -o build/$@ -lm -lgps

imu_gyro: imu_gyro.c
	mkdir -p build
	gcc $(filter %c, $^) -o build/$@ -I$(LIBDIR) $(BUILDLIBDIR)/Adafruit_L3GD20_U.o -lwiringPi
