CC = g++
OUTPUTNAME = saveimg
INCLUDE = -I/usr/include/flycapture
LIBS = -L/usr/lib/ -lflycapture -lboost_thread -lboost_program_options  -lopencv_highgui -lopencv_gpu -lopencv_core
#CFLAGS = -g 
CFLAGS = -O3 -Wno-unused-variable
OBJS = SaveImageToAviEx.o
#OBJS = FlyCapTest.o

${OUTPUTNAME}: ${OBJS}
	${CC} ${CFLAGS} -o ${OUTPUTNAME} ${OBJS} ${LIBS} ${COMMON_LIBS} 

%.o: %.cpp
	${CC} ${CFLAGS} ${INCLUDE} -Wall -c $*.cpp 
	
clean_obj:
	rm -f ${OBJS}	

clean:
	rm -f ${OUTDIR}/${OUTPUTNAME} ${OBJS}	
