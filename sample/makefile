
all: gps

clean:
	-rm -rf build

gps: gps.c
	mkdir -p build
	gcc $(filter %.c, $^) -o build/$@ -lm -lgps
