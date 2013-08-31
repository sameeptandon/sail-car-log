# To compile the debug verison need to overwrite CXXFLAGS variable to include -ggdb

CC = g++
OUTPUTNAME = stereo_grab${D}
INCLUDE = -I/usr/include/flycapture
LIBS = -L../../lib -lflycapture${D} 

OBJS = stereo_grab.o


${OUTPUTNAME}: ${OBJS}
	${CC} -o ${OUTPUTNAME} -g ${OBJS} ${LIBS} ${COMMON_LIBS}

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE} -g -c $*.cpp

clean_obj:
	rm -f ${OBJS}	@echo "all cleaned up!"

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	@echo "all cleaned up!"
