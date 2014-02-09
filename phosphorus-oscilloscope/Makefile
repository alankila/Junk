PKGCONFIG = pkg-config
CFLAGS := -std=c99 -Wall -g -O2 `$(PKGCONFIG) cairo --cflags` `$(PKGCONFIG) libpng --cflags`
LDFLAGS := -lm `$(PKGCONFIG) cairo --libs` `$(PKGCONFIG) libpng --libs`

.phony: all clean

all: wave

clean:
	rm -f wave.o wave test.png

wave.o: wave.c Makefile

wave: wave.o
	$(CC) -o $@ $< $(LDFLAGS)
