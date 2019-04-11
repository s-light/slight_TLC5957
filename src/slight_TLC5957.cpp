
/******************************************************************************

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2018 Stefan Krüger

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
******************************************************************************/


// include Core Arduino functionality
#include <Arduino.h>

// include SPI library
#include <SPI.h>

// include own headerfile
#include "slight_TLC5957.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// helper
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

uint16_t slight_TLC5957::set_bit_with_mask(
    uint16_t value, uint16_t mask, uint16_t value_new
) {
    // """Set bit with help of mask."""
    // clear
    value &= ~mask;
    if (value_new) {
        // set
        value |= mask;
    }
    return value;
}

uint16_t slight_TLC5957::set_bit(
    uint16_t value, uint8_t index, uint16_t value_new
) {
    // """Set bit - return new value.
    //
    // Set the index:th bit of value to 1 if value_new is truthy,
    // else to 0, and return the new value.
    // https://stackoverflow.com/a/12174051/574981
    // """
    // Compute mask, an integer with just bit 'index' set.
    uint16_t mask = 1 << index;
    // Clear the bit indicated by the mask (if x is False)
    value &= ~mask;
    if (value_new) {
        // If x was True, set the bit indicated by the mask.
        value |= mask;
    }
    // Return the result, we're done.
    return value;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


slight_TLC5957::slight_TLC5957(
    uint16_t pixel_count,
    uint8_t latch,
    uint8_t gsclk,
    uint8_t spi_clock,
    uint8_t spi_mosi,
    uint8_t spi_miso
):
    pixel_count(pixel_count),
    channel_count(pixel_count * COLORS_PER_PIXEL),
    chip_count(
        (pixel_count / PIXEL_PER_CHIP) +
        // check if we pixels for a part of an chip
        ((pixel_count % PIXEL_PER_CHIP) > 0 ? 1 : 0)),
    buffer_byte_count(chip_count * CHIP_GS_BUFFER_BYTE_COUNT),
    buffer(reinterpret_cast<uint16_t*>(calloc(buffer_byte_count, 1))),
    buffer_fc_byte_count(chip_count * CHIP_BUFFER_BYTE_COUNT),
    buffer_fc(reinterpret_cast<uint16_t*>(calloc(buffer_fc_byte_count, 1))),
    latch(latch),
    gsclk(gsclk),
    spi_clock(spi_clock),
    spi_mosi(spi_mosi),
    spi_miso(spi_miso
) {
    ready = false;
}

slight_TLC5957::~slight_TLC5957() {
    end();
    free(buffer);
    free(buffer_fc);
}


void slight_TLC5957::begin() {
    // clean up..
    end();
    // start up...
    if (ready == false) {
        // setup
        pinMode(latch, OUTPUT);
        // pinMode(gsclk, OUTPUT);
        pinMode(spi_miso, INPUT);
        pinMode(spi_mosi, OUTPUT);
        pinMode(spi_clock, OUTPUT);
        SPI.begin();
        // SPI.beginTransaction(SPISettings(10 * 1000000, MSBFIRST, SPI_MODE0));
        SPI.beginTransaction(SPISettings(1 * 1000000, MSBFIRST, SPI_MODE0));

        _init_buffer_fc();
        update_fc();

        // write initial 0 values
        show();
        show();
    }
}

void slight_TLC5957::end() {
    if (ready) {
        SPI.endTransaction();
        SPI.end();
    }
}

void slight_TLC5957::show() {
    // """Write out Grayscale Values to chips."""
    _write_buffer_GS();
}

void slight_TLC5957::update_fc() {
    // """Write out Function_Command Values to chips."""
    _write_buffer_FC();
}





void slight_TLC5957::_init_buffer_fc() {
    // TODO(s-light): implement
}

void slight_TLC5957::_write_buffer_GS() {
    // TODO(s-light): implement
}
void slight_TLC5957::_write_buffer_FC() {
    // TODO(s-light): implement
}
void slight_TLC5957::_write_buffer_with_function_command() {
    // TODO(s-light): implement
}


















void slight_TLC5957::update_old() {
    // TODO(s-light): implement.

    uint16_t * buffer_start = buffer;
    size_t write_inc = (CHIP_BUFFER_BYTE_COUNT * chip_count) - 2;

    for (uint8_t i = 0; i < PIXEL_PER_CHIP; i++) {
        SPI.beginTransaction(SPISettings(1 * 1000000, MSBFIRST, SPI_MODE0));
        // write GS data for all chips -1*16bit
        // SPI.transfer(buffer, write_inc);
        SPI.transfer16(buffer_start[0]);
        SPI.transfer16(buffer_start[2]);
        // SPI.transfer(reinterpret_cast<uint8_t*>(buffer_start), write_inc);
        buffer_start += (write_inc / 2);
        SPI.endTransaction();
        // special
        if (i == (PIXEL_PER_CHIP - 1)) {
            write_SPI_with_function_command(_FC__LATGS, *buffer_start);
        } else {
            write_SPI_with_function_command(_FC__WRTGS, *buffer_start);
        }
        buffer_start += 1;
    }
}

void slight_TLC5957::write_SPI_with_function_command(
    slight_TLC5957::function_command_pulse_count function_command,
    uint16_t value
) {
    // faster speeds with direct port access...
    // https://forum.arduino.cc/index.php?topic=4324.0

    pinMode(spi_mosi, OUTPUT);
    pinMode(spi_clock, OUTPUT);

    digitalWrite(spi_clock, LOW);
    digitalWrite(latch, LOW);

    for (uint8_t i = 0; i < 16; i++) {
        if ((16 - function_command) == i) {
            digitalWrite(latch, HIGH);
        }

        // b1000000000000000
        if (value & 0x8000u) {
            digitalWrite(spi_mosi, HIGH);
        } else {
            digitalWrite(spi_mosi, LOW);
        }
        value <<= 1;

        digitalWrite(spi_clock, HIGH);
        delayMicroseconds(10);
        digitalWrite(spi_clock, LOW);
    }

    digitalWrite(latch, LOW);
    // TODO(s-light): check if this works.
}
