PKGCONFIG = pkg-config
CFLAGS := -std=c99 -Wall -g -O2 `$(PKGCONFIG) sndfile --cflags` `$(PKGCONFIG) libpng --cflags`
LDFLAGS := -lm `$(PKGCONFIG) sndfile --libs` `$(PKGCONFIG) libpng --libs`

.phony: all clean

all: wav2png png2wav

clean:
	rm -f png2wav.o png2wav wav2png.o wav2png

png2wav.o: png2wav.c Makefile

wav2png.o: wav2png.c Makefile

png2wav: png2wav.o
	$(CC) -o $@ $< $(LDFLAGS)

wav2png: wav2png.o
	$(CC) -o $@ $< $(LDFLAGS)
