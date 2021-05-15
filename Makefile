CC=gcc
CFLAGS=-Wall -Wextra
LIBS=-lwiringPi -lm
OBJ=afsk/utils.o afsk/drivers/ina219/ina219.o

all: DEBUG_BEHAVIOR=
all: radioafsk 

debug: DEBUG_BEHAVIOR = -DDEBUG_LOGGING
debug: radioafsk

rebuild: clean
rebuild: all

clean:
	rm -f radioafsk
	rm -f afsk/*.o
	rm -f afsk/drivers/ina219/*.o
	rm -f ina219Test


# docs:
# 	mkdir -p ax5043/doc; cd ax5043; doxygen Doxyfile
# 	cd ax5043/doc/latex && make && cd ../.. && cp doc/latex/refman.pdf doc/TransceiverFramework.pdf

afsk/utils.o: afsk/utils.c afsk/constants.h
afsk/drivers/ina219/ina219.o: afsk/drivers/ina219/ina219.c
	$(CC) $< -c -o $@ $(CFLAGS) $(LIBS)

ina219Test: afsk/drivers/ina219/ina219Test.c afsk/drivers/ina219/ina219.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

radioafsk: afsk/main.c $(OBJ) afsk/constants.h
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

test: test.c afsk/constants.h
	$(CC) -o $@ $^ $(CFLAGS)
