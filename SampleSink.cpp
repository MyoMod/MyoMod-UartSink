#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"

#include "pico_unicorn.hpp"

using namespace pimoroni;

PicoUnicorn pico_unicorn;

void clearBar(uint32_t barIndex)
{
    for (uint32_t x = 0; x < 16; x++)
    {
        pico_unicorn.set_pixel(x, barIndex, 0, 0, 0);
    }
}

void drawBar(uint32_t barIndex, uint32_t value, uint32_t color)
{
    // map 0-100 to 0-16
    uint32_t barHeight = value * 16 / 100;
    uint8_t r = (color >> 16) & 0xff;
    uint8_t g = (color >> 8) & 0xff;
    uint8_t b = color & 0xff;

    for (uint32_t x = 0; x < barHeight; x++)
    {
        pico_unicorn.set_pixel(x, barIndex, r, g, b);
    }

    // clear the rest of the bar
    for (uint32_t x = barHeight; x < 16; x++)
    {
        pico_unicorn.set_pixel(x, barIndex, 0, 0, 0);
    }
}

int main()
{
    bool a_pressed = false;
    bool b_pressed = false;
    bool x_pressed = false;
    bool y_pressed = false;

    uint32_t i = 0;
    while (true)
    {
        i = i + 1;

        // button test phase
        float pulse = fmod(float(i) / 20.0f, M_PI * 2.0f);
        int v = int((sin(pulse) * 50.0f) + 50.0f);

        if (pico_unicorn.is_pressed(pico_unicorn.A))
        {
            drawBar(0, v, 0xff0000);
            clearBar(1);
            clearBar(2);
            clearBar(3);
        }
        else if (pico_unicorn.is_pressed(pico_unicorn.B))
        {
            drawBar(1, v, 0x00ff00);
            clearBar(0);
            clearBar(2);
            clearBar(3);
        }
        else if (pico_unicorn.is_pressed(pico_unicorn.X))
        {
            drawBar(2, v, 0x0000ff);
            clearBar(0);
            clearBar(1);
            clearBar(3);
        }
        else if (pico_unicorn.is_pressed(pico_unicorn.Y))
        {
            drawBar(3, v, 0xffff00);
            clearBar(0);
            clearBar(1);
            clearBar(2);
        }
        else
        {
            drawBar(0, v, 0xff0000);
            drawBar(1, v, 0x00ff00);
            drawBar(2, v, 0x0000ff);
            drawBar(3, v, 0xffff00);
        }

        sleep_ms(10);
    }

    printf("done\n");

    return 0;
}