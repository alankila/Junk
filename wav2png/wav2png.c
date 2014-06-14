#include <png.h>
#include <sndfile.h>
#include <stdint.h>
#include <stdlib.h>
#include <zlib.h>

static uint32_t read_file(const char* filename, int16_t **data, uint32_t *length) {
    SF_INFO info = {};
    SNDFILE *sf = sf_open(filename, SFM_READ, &info);
    if (sf == NULL) {
        fprintf(stderr, "NULL result from sf_open()\n");
        return 1;
    }
    if (info.channels != 1) {
        fprintf(stderr, "Too many channels in file, only 1 allowed\n");
        return 1;
    }

    *length = info.frames;
    *data = calloc(sizeof(**data), *length);
    sf_read_short(sf, *data, *length);
    sf_close(sf);
    return 0;
}

static uint32_t autocorrelate(const int16_t *data, uint32_t length) {
    uint32_t min_window = 50;
    uint32_t max_window = 1000;

    int64_t best_correlation = 0;
    uint32_t best_len = min_window;
    for (uint32_t len = min_window; len <= max_window; len += 1) {
        int64_t correlation = 0;
        for (uint32_t i = 0; i < length - len; i += 1) {
            correlation += (int64_t) data[i] * data[i + len];
        }
        //fprintf(stderr, "autocorrelation: %lld for len %d %s\n", correlation, len, (correlation > best_correlation ? "*" : ""));
        if (correlation > best_correlation) {
            best_correlation = correlation;
            best_len = len;
        }
    }

    return best_len;
}

/* perform linear light alpha blending of alpha mask, write result to png */
static void output_image(const int16_t *data, int width, int height)
{
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);

    FILE *file = fopen("test.png", "w");
    png_init_io(png, file);

    png_set_IHDR(png, info, width, height, 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_DEFAULT);
    png_set_compression_level(png, Z_BEST_COMPRESSION);
    png_set_filter(png, 0, PNG_FILTER_PAETH);
    png_bytep row = malloc(2 * width);

    png_write_info(png, info);
    for (int y = 0; y < height; y += 1) {
        for (int x = 0; x < width; x += 1) {
            uint16_t v = data[y * width + x] + 0x8000;
            row[x*2+1] = v & 0xff;
            row[x*2+0] = v >> 8;
        }
        png_write_row(png, row);
    }
    png_write_end(png, NULL);

    free(row);
    fclose(file);
    png_destroy_write_struct(&png, &info);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <wavfile>\n", argv[0]);
        return 1;
    }

    int16_t *data;
    uint32_t length;
    char *filename = argv[1];
    if (read_file(filename, &data, &length)) {
        return 1;
    }

    uint32_t width = autocorrelate(data, length);
    uint32_t height = length / width;

    fprintf(stderr, "File truncated due to me being lame: %d => %d bytes, window=%d\n", length, width * height, width);
    output_image(data, width, height);

    return 0;
}
