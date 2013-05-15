CC = g++
OUTPUTNAME = CameraLogger 
INCLUDE = -I/usr/include/flycapture
LIBS = -L/usr/lib/ -lflycapture -lboost_thread -lboost_program_options  -lopencv_highgui -lopencv_gpu -lopencv_core
#CFLAGS = -g
SRC = CameraLogger.cpp 
CFLAGS = -O3 -Wno-unused-variable

all: cameralogger

cameralogger:
	${CC} ${CFLAGS} ${INCLUDE} ${SRC} ${LIBS} ${COMMON_LIBS} -o ${OUTPUTNAME}

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE} -Wall -c $*.cpp 
	
clean:
	rm -f ${OUTPUTNAME} 
