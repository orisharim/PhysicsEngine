CC = gcc
CFLAGS = -Wall -O2 -I../lib/physics -I../lib/utils
LIBS = -lraylib -lm -ldl -lGL -lX11

SRC = test_balls_pit.c \
      ../lib/physics/world.c \
      ../lib/physics/bodies.c \
      ../lib/physics/collision.c \
      $(wildcard ../lib/utils/*.c)

OUT = main

all:
	$(CC) $(CFLAGS) $(SRC) -o $(OUT) $(LIBS)

clean:
	rm -f $(OUT)
