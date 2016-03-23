PROG?=main
CC:=gcc
CFLAGS:=-Wall -std=gnu99 -O1
BUILDDIR:=build
LIBDIR:=lib
GPSCONF:=gps_conf

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
	mkdir -p $(APPFILEDIR)/media
	cp $(BUILDDIR)/$(GPSCONF) $(APPFILEDIR)
	cp ./zlog.conf $(APPFILEDIR)
	cp ./camera.py $(APPFILEDIR)
	mkdir -p /var/log/$(BINARYNAME)

uninstall:
	-rm /usr/bin/$(BINARYNAME) 
	-rm /etc/init.d/$(BINARYNAME)
	update-rc.d $(BINARYNAME) remove
	-rm -rf $(APPFILEDIR)

#build everything in the lib directory
lib: $(addprefix $(BUILDDIR)/, $(patsubst %.c,%.o,$(wildcard $(LIBDIR)/*.c))) 

#autodetect lib sources
$(BUILDDIR)/$(LIBDIR)/%.o : $(LIBDIR)/%.c
	$(CC) -c  -o $@ $< -lwiringPi -I. $(CFLAGS)

COMPLIBS = $(wildcard $(BUILDDIR)/$(LIBDIR)/*.o)

$(PROG): src/$(PROG).c
	$(CC) $(filter %.c, $^) -o build/$@ -lxbee -lpthread -lrt -lgps \
		-lwiringPi -lada10dof -lzlog -lpthread -lpython2.7\
		-I. -Ilib -I/usr/lib -Iusr/include/python2.7 $(COMPLIBS) $(CFLAGS) -lm
	cp ./zlog.conf $(BUILDDIR)
	cp ./camera.py $(BUILDDIR)

gps_conf: src/gps_conf.c
	$(CC) $< -o $(BUILDDIR)/$@ -Ilib  $(COMPLIBS) $(CFLAGS)
