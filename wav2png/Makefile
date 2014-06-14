PKGCONFIG = pkg-config
CFLAGS := -std=c99 -Wall -g -O2 `$(PKGCONFIG) sndfile --cflags` `$(PKGCONFIG) libpng --cflags`
LDFLAGS := -lm `$(PKGCONFIG) sndfile --libs` `$(PKGCONFIG) libpng --libs`

.phony: all clean

all: wav2png

clean:
	rm -f wav2png.o wav2png test.png

wav2png.o: wav2png.c Makefile

wav2png: wav2png.o
	$(CC) -o $@ $< $(LDFLAGS)
