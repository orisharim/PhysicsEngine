CC = gcc
CFLAGS = -Wall -O2 -I../lib/physics -I../lib/physics/bodies -I../lib/utils 
LIBS = -lraylib -lm -ldl -lGL -lX11

SRC = test_softbody.c \
      $(wildcard ../lib/physics/*.c) \
      $(wildcard ../lib/physics/bodies/*.c) \
      $(wildcard ../lib/utils/*.c)

OUT = main

all:
	$(CC) $(CFLAGS) $(SRC) -o $(OUT) $(LIBS)

clean:
	rm -f $(OUT)
