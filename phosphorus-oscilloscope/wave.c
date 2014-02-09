#include <cairo.h>
#include <math.h>
#include <png.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static const int oversampling = 16;
static const int points = 32;
static const float pi = 3.141592654f;
static float samples[1000];

/* any finite value of x */
static float sinc(float x)
{
    return x != 0 ? sinf(x) / x : 1.0f;
}

/* -1 <= x <= 1 */
static float window(float x)
{
    return 0.53836f - 0.46164f * cosf(pi * (x + 1));
}

/* return the interpolated sample at point x.
 * signal is assumed to be zero outside the buffer. */
static float get(const float *samples, int samples_length, float x)
{
    float out = 0.0f;
    int whole = (int) x;
    float frac = x - whole;
    for (int i = -points/2+1; i < points/2; i += 1) {
        int idx = whole + i;
        if (idx >= 0 && idx < samples_length) {
            out += samples[idx] * sinc((i - frac) * pi) * window((i - frac) / (points/2));
        }
    }
    return out;
}

/* draw interpolated samples from buffer to surface */
static void draw_samples(cairo_surface_t *surface, const float *samples, const int samples_length)
{
    cairo_t *cr = cairo_create(surface);
    cairo_set_operator(cr, CAIRO_OPERATOR_ADD);
    cairo_set_line_width(cr, 1);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_BUTT);

    int width = cairo_image_surface_get_width(surface);
    int height = cairo_image_surface_get_height(surface);
    
    /*cairo_rectangle(cr, 0, 0, width, height);
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_fill(cr);*/
    
    float x1 = 0;
    float y1 = 0;
    for (int i = 0; i <= samples_length * oversampling; i += 1) {
        float pos = i / (float) oversampling;

        float x2 = pos / samples_length * width;
        float y2 = (get(samples, samples_length, pos) * 0.485f + 0.5f) * height; /* 0.485 => 3 % overshoot allowance */
        if (i != 0) {
            /* Compute intensity approximation for the line segment.
             * First, basic alpha is given by how many line segments in
             * average must share a vertical slice. */
            float a = (float) width / samples_length;
            /* We then modulate it based on the derivative or spread
             * along the y axis -- the faster the signal moves, the
             * fainter the trace must be. y is in pixels here, so if
             * dy = 0, then divide by 1, if dy = 1, divide by 2, etc.
             *
             * The ovesampling factor here corrects the derivative
             * based on the idea that dy halves each time oversampling
             * doubles, so the derivative becomes normalized. */
            a /= 1.0f + fabsf(y2 - y1) * oversampling;

            /* To enhance rendering quality, we store versions of alpha
             * at two resolutions. We also indicate overflow of green
             * in blue channel. */
            cairo_set_source_rgb(cr, a, a * 64, a * 64 <= 1 ? 0 : 1);
            cairo_move_to(cr, x1, y1);
            cairo_line_to(cr, x2, y2);
            cairo_stroke(cr);
        }
        x1 = x2;
        y1 = y2;
    }
    cairo_destroy(cr);
}

/* Approximated sRGB linear blend, source colors in linearized sRGB */
static uint8_t blend(float fg, float bg, float alpha)
{
    float value = fg * alpha + bg * (1.0f - alpha);
    value = powf(value, 1.0f / 2.2f);
    if (value > 1.0f) {
        value = 1.0f;
    }
    return roundf(value * 255.0f);
}

/* perform linear light alpha blending of alpha mask, write result to png */
static void output_image(const uint8_t *data, int width, int height, int stride)
{
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);

    FILE *file = fopen("test.png", "w");
    png_init_io(png, file);

    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
    png_bytep row = malloc(3 * width);

    png_write_info(png, info);
    for (int y = 0; y < height; y += 1) {
        for (int x = 0; x < width; x += 1) {
            uint32_t value = *(int *) &data[y * stride + x * 4];
            uint32_t ra = (value >> 20) & 0x3ff;
            uint32_t ga = (value >> 10) & 0x3ff;
            uint32_t ba = (value      ) & 0x3ff;

            /* Decide on whether to read the green or blue channel */
            float a;
            if (ba == 0) {
                a = ga / 1023.0f / 64.0f;
            } else {
                a = ra / 1023.0f;
            }

            uint8_t r = blend(1.0f, 0.0f, a);
            uint8_t g = blend(4.0f, 0.0f, a);
            uint8_t b = blend(2.0f, 0.0f, a);

            row[x*3+0] = r;
            row[x*3+1] = g;
            row[x*3+2] = b;
        }
        png_write_row(png, row);
    }
    png_write_end(png, NULL);

    free(row);
    fclose(file);
    png_destroy_write_struct(&png, &info);
}

static void draw(const float *samples, int samples_length) {
    cairo_surface_t *surface = cairo_image_surface_create(
        CAIRO_FORMAT_RGB30, 1000, 500
    );

    draw_samples(surface, samples, samples_length);
    cairo_surface_flush(surface);

    //cairo_surface_write_to_png(surface, "test.png");

    output_image(
        cairo_image_surface_get_data(surface),
        cairo_image_surface_get_width(surface),
        cairo_image_surface_get_height(surface),
        cairo_image_surface_get_stride(surface)
    );

    cairo_surface_destroy(surface);
}

int main(int argv, char **argc)
{
    for (int i = 0; i < sizeof(samples) / sizeof(samples[0]); i += 1) {
        samples[i] = sinf(i * pi / 2 * (i * 0.0009f)) * cos(i * pi * 0.002f);
    }
    draw(samples, sizeof(samples) / sizeof(samples[0]));
    return 0;
}
