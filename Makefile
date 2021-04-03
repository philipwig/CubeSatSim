CC=gcc
CFLAGS=-Wall -Wextra
LIBS=-lwiringPi -lm
OBJ=afsk/utils.o

all: DEBUG_BEHAVIOR=
all: radioafsk 

debug: DEBUG_BEHAVIOR = -DDEBUG_LOGGING
debug: radioafsk

rebuild: clean
rebuild: all

clean:
	rm -f radioafsk
	rm -f afsk/*.o


# docs:
# 	mkdir -p ax5043/doc; cd ax5043; doxygen Doxyfile
# 	cd ax5043/doc/latex && make && cd ../.. && cp doc/latex/refman.pdf doc/TransceiverFramework.pdf

afsk/utils.o: afsk/utils.c
	$(CC) $< -c -o $@ $(CFLAGS) $(LIBS)

# NEW radioafsk mainfile
radioafsk: afsk/main.c $(OBJ)
	$(CC)  -o $@ $^ $(CFLAGS) $(LIBS)

# TelemEncoding.o: afsk/TelemEncoding.h
# 	gcc $< -c -o $@ $(CFLAGS)
