##############################################################################

CC=gcc
CFLAGS=-std=c99 -Wall -pedantic -O3
LDFLAGS=-lm

DEPS=laser.h ransac.h
SOURCES=main.c laser.c ransac.c
OBJ=$(SOURCES:.c=.o)

EXECUTABLE=ransacFPGA

all : $(EXECUTABLE)

$(EXECUTABLE) : $(OBJ)
	${CC} -o $@ $^ ${CFLAGS} ${LDFLAGS}
	@echo

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)
	@echo

clean :
	rm -f ${EXECUTABLE} *.o *~

##############################################################################
