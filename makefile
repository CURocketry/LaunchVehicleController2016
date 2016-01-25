PROG?=main
CC:=gcc
CFLAGS:=-Wall -std=gnu99 -O3
BUILDDIR:=build
LIBDIR:=lib

APPFILEDIR:= /var/lib/crtlvc
BINARYNAME:= crtlvc

.PHONY: all run clean prep install uninstall lib

all: prep lib gps_conf $(PROG)

run:  all 
	$(BUILDDIR)/$(PROG)

clean:
	-rm -rf $(BUILDDIR)

prep:
	mkdir -p $(BUILDDIR)/$(LIBDIR)

install:
	cp $(BUILDDIR)/$(PROG) /usr/bin/$(BINARYNAME) 
	cp $(BINARYNAME)_initd /etc/init.d/$(BINARYNAME)
	update-rc.d $(BINARYNAME) defaults
	mkdir -p $(APPFILEDIR)
	cp ./zlog.conf $(APPFILEDIR)

uninstall:
	rm /usr/bin/$(BINARYNAME) 
	rm /etc/init.d/$(BINARYNAME)
	update-rc.d $(BINARYNAME) remove
	rm -rf $(APPFILEDIR)

#build everything in the lib directory
lib: $(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard $(LIBDIR)/*.c))) 

#autodetect lib sources
$(BUILDDIR)/$(LIBDIR)/%.o : $(LIBDIR)/%.c
	$(CC) -c  -o $@ $< -lwiringPi -I. $(CFLAGS)

COMPLIBS = $(wildcard $(BUILDDIR)/$(LIBDIR)/*.o)

$(PROG): src/$(PROG).c
	$(CC) $(filter %.c, $^) -o build/$@ -lxbee -lpthread -lrt -lgps \
		-lwiringPi -lada10dof -lzlog \
		-I. -Ilib -I/usr/lib $(COMPLIBS) $(CFLAGS) -lm
	cp ./zlog.conf build

gps_conf: src/gps_conf.c
	$(CC) $< -o $(BUILDDIR)/$@ -Ilib  $(COMPLIBS) $(CFLAGS)
