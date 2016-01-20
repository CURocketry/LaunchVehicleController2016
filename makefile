PROG?=main
CC:=gcc
CFLAGS:=-Wall -std=gnu99
BUILDDIR:=build
LIBDIR:=lib

.PHONY: all run clean prep lib

all: prep lib gps_conf $(PROG)

run:  all 
	build/$(PROG)

clean:
	-rm -rf build

prep:
	mkdir -p build/lib

test: lib gps_conf

#build everything in the lib directory
lib: $(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard $(LIBDIR)/*.c))) 

#autodetect lib sources
$(BUILDDIR)/$(LIBDIR)/%.o : $(LIBDIR)/%.c
	$(CC) -c  -o $@ $< -lwiringPi -I. $(CFLAGS)

COMPLIBS = $(wildcard $(BUILDDIR)/$(LIBDIR)/*.o)

$(PROG): src/$(PROG).c
	$(CC) $(filter %.c, $^) -o build/$@ -lxbee -lpthread -lrt -lm -lgps -lwiringPi -I. -Ilib $(COMPLIBS) $(CFLAGS)

gps_conf: src/gps_conf.c
	$(CC) $< -o $(BUILDDIR)/$@ -Ilib -lwiringPi $(COMPLIBS) $(CFLAGS)
