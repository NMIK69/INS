CC=gcc
CFLAGS= -Wall -pedantic -Wextra -Wmissing-prototypes 
OPTIMIZE=-O3
LDFLAGS= -lncurses -lm

SRCS=$(wildcard *.c)
OBJS=$(patsubst %.c, %.o, $(SRCS))

DEPS=$(wildcard *.h)

TARGET=tui

all: $(TARGET)

debug: CFLAGS += -ggdb
debug: OPTIMIZE = -O0
debug: $(TARGET)

$(TARGET) : $(OBJS)
	$(CC) $^ $(CFLAGS) $(OPTIMIZE) $(LDFLAGS) -o $@

%.o : %.c $(DEPS)
	$(CC) $(CFLAGS) $(OPTIMIZE) -c $< -o $@

.PHONY : clean
clean :
	rm -f $(TARGET)
	rm -f *.o
