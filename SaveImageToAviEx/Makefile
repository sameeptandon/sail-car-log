# SaveImageToAviEx makefile

CC = g++
OUTPUTNAME = saveimg
INCLUDE = -I/usr/include/flycapture
LIBS = -L/usr/lib/ -lflycapture -lboost_thread 
#CFLAGS = 
CFLAGS = -O3 -Wno-unused-variable 
OBJS = SaveImageToAviEx.o

${OUTPUTNAME}: ${OBJS}
	${CC} ${CFLAGS} -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS} 

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE} -Wall -c $*.cpp
	
clean_obj:
	rm -f ${OBJS}	@echo "all cleaned up!"

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	@echo "all cleaned up!"
