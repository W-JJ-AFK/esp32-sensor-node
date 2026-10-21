# Host-side unit tests for the pure-C algorithm layer.
# Requires only a C compiler (no ESP-IDF):
#   make          # build and run
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -O2 -I../main -I../main/algo -I../main/net
LDLIBS  ?= -lm

SRCS := test_edge_detect.c ../main/algo/edge_detect.c ../main/algo/imu_ring.c \
        ../main/net/topic.c
BIN  := test_host

.PHONY: all run clean

all: run

$(BIN): $(SRCS) ../main/algo/edge_detect.h ../main/algo/imu_ring.h \
        ../main/net/topic.h
	$(CC) $(CFLAGS) -o $(BIN) $(SRCS) $(LDLIBS)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
