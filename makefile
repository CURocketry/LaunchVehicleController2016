PROG?=main

all: $(PROG)

run: all 
	./$(PROG).o

clean:
	-rm $(PROG).o

$(PROG): $(PROG).c
	gcc $(filter %.c, $^) -o $@.o -lxbee -lpthread -lrt -lm -I.
