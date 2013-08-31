# To compile the debug verison need to overwrite CXXFLAGS variable to include -ggdb

CC = g++
OUTPUTNAME = bumblebee_grab${D}
INCLUDE = -I/usr/include/flycapture
LIBS = -L../../lib -lflycapture${D} -ltriclops

OBJS = bumblebee_grab.o


${OUTPUTNAME}: ${OBJS}
	${CC} -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS}

%.o: %.cpp bumblebee_xb3.h
	${CC} ${CFLAGS} ${INCLUDE} -c $*.cpp

clean_obj:
	rm -f ${OBJS}	@echo "all cleaned up!"

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	@echo "all cleaned up!"
