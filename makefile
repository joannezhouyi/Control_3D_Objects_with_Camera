# Makefile template for shared library




# C compiler
CC = g++


INCLUDE=-I/usr/include/python2.7/ -I/usr/include
PATHLIB=-L/usr/lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib64/
LIBADD=-lpython2.7 -lboost_python -lglut -lGL -lGLU -lm -lGLEW


# C flags
CFLAGS=-c -fPIC -std=c++11
# linking flags
LDFLAGS=-shared
# rm command
RM=rm -f
# target lib
TARGET_LIB=LibShowModel.so


# source files
SRCS=ShowModel.cc
OBJS=ShowModel.o


.PHONY: all
all: ${TARGET_LIB}

${TARGET_LIB}: ${OBJS}
	${CC} ${LDFLAGS} -Wl,-soname,${TARGET_LIB} -o ${TARGET_LIB} ${OBJS} ${PATHLIB} ${LIBADD}

${OBJS}: ${SRCS}
	${CC} ${CFLAGS} -o ${OBJS} ${SRCS} ${INCLUDE}


.PHONY: clean
clean:
	-${RM} ${TARGET_LIB} ${OBJS}