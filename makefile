PROG?=main

all: $(PROG)

run: all 
	build/$(PROG)

clean:
	-rm -rf build

$(PROG): src/$(PROG).c
	mkdir -p build
	gcc $(filter %.c, $^) -o build/$@ -lxbee -lpthread -lrt -lm -I.
