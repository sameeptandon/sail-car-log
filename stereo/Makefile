# To compile the debug verison need to overwrite CXXFLAGS variable to include -ggdb

CC = g++
INCLUDE = -I/usr/include/flycapture
LIBS    += -ldc1394 -lraw1394 -pthread 
LIBS = -lflycapture -ltriclops -lpnmutils -ldl

all: bumblebee_grab bumblebee_stereo

bumblebee_stereo: bumblebee_stereo.o lodepng.o
	${CC} -o bumblebee_stereo bumblebee_stereo.o lodepng.o ${LIBS}

bumblebee_stereo.o: bumblebee_stereo.cpp triclops_xb3.h
	${CC} ${CFLAGS} ${INCLUDE} -c bumblebee_stereo.cpp

bumblebee_grab: bumblebee_grab.o
	${CC} -o bumblebee_grab bumblebee_grab.o ${LIBS}

bumblebee_grab.o: bumblebee_grab.cpp bumblebee_xb3.h 
	${CC} ${CFLAGS} ${INCLUDE} -c bumblebee_grab.cpp 

load_png.o: lodepng.h lodepng.cpp
	${CC} ${CFLAGS} ${INCLUDE} -c lodepng.cpp

clean_obj:
	rm -f bumblebee_grab.o	bumblebee_stereo.o @echo "all cleaned up!"

clean:
	rm -f bumblebee_grab bumblebee_grab.o bumblebee_stereo bumblebee_stereo.o @echo "all cleaned up!"
