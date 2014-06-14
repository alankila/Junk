#include <png.h>
#include <sndfile.h>
#include <stdint.h>
#include <stdlib.h>
#include <zlib.h>

static uint32_t write_file(const char* filename, int16_t *data, uint32_t length) {
    SF_INFO info = { .format = SF_FORMAT_WAV | SF_FORMAT_PCM_16, .channels = 1, .samplerate=44100 };
    SNDFILE *sf = sf_open(filename, SFM_WRITE, &info);
    if (sf == NULL) {
        fprintf(stderr, "NULL result from sf_open()\n");
        return 1;
    }
    sf_write_short(sf, data, length);
    sf_close(sf);
    return 0;
}

/* perform linear light alpha blending of alpha mask, write result to png */
static void read_image(const char *filename, int16_t **data, uint32_t *width, uint32_t *height)
{
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);

    FILE *file = fopen(filename, "r");
    png_init_io(png, file);
    png_read_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);

    *width = png_get_image_width(png, info);
    *height = png_get_image_height(png, info);
    *data = calloc(sizeof(**data), *width * *height);

    png_bytep *rows = png_get_rows(png, info);
    for (int y = 0; y < *height; y += 1) {
        for (int x = 0; x < *width; x += 1) {
            uint16_t v = (rows[y][x * 2 + 0] << 8) | rows[y][x * 2 + 1];
            (*data)[y * *width + x] = v - 0x8000;
        }
    }
    fclose(file);
    png_destroy_read_struct(&png, &info, NULL);
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: %s <pngfile> <wavfile>\n", argv[0]);
        return 1;
    }
    char *filename1 = argv[1];
    char *filename2 = argv[2];

    int16_t *data;
    uint32_t width;
    uint32_t height;
    read_image(filename1, &data, &width, &height);
    write_file(filename2, data, width * height);
    return 0;
}
