
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
// definitions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


slight_TLC5957::slight_TLC5957(
    uint8_t chip_count,
    uint8_t lat_pin,
    uint8_t gclk_pin,
    uint8_t sclk_pin,
    uint8_t sout_pin,
    uint8_t sin_pin
):
    lat_pin(lat_pin),
    gclk_pin(gclk_pin),
    sclk_pin(sclk_pin),
    sout_pin(sout_pin),
    sin_pin(sin_pin),
    buffer(
        reinterpret_cast<uint16_t*>(calloc(chip_count, chip_buffer_byte_count))
) {
    ready = false;
}

slight_TLC5957::~slight_TLC5957() {
    end();
    free(buffer);
}


void slight_TLC5957::begin() {
    // clean up..
    end();
    // start up...
    if (ready == false) {
        // setup
        pinMode(lat_pin, OUTPUT);
        pinMode(gclk_pin, OUTPUT);
        // TODO(s-light): implement.
        SPI.begin();
        // SPI.beginTransaction(SPISettings(10 * 1000000, MSBFIRST, SPI_MODE0));
        SPI.beginTransaction(SPISettings(1 * 1000000, MSBFIRST, SPI_MODE0));
    }
}

void slight_TLC5957::end() {
    if (ready) {
        SPI.endTransaction();
        SPI.end();
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// special non overwritting spi functions
// https://forum.arduino.cc/index.php?topic=421756.msg2904845#msg2904845

void slight_TLC5957::spi_write_buffer(void *buf_write, size_t count) {
    if (count == 0) return;
    uint8_t *p_out = reinterpret_cast<uint8_t *>(buf_write);
    SPDR = *p_out;
    while (--count > 0) {
        // prepare output data
        uint8_t out = *(p_out + 1);
        // wait untill data is send
        while (!(SPSR & _BV(SPIF))) {}
        // send output
        SPDR = out;
        // prepare next loop
        p_out++;
    }
    // wait untill data is send
    while (!(SPSR & _BV(SPIF))) {}
}

void slight_TLC5957::spi_read_buffer(void *buf_read, size_t count) {
    if (count == 0) return;
    uint8_t *p_in = reinterpret_cast<uint8_t *>(buf_read);
    // init transfer
    SPDR = 0;
    while (--count > 0) {
        // wait untill data is transferred
        while (!(SPSR & _BV(SPIF))) {}
        // read input
        uint8_t in = SPDR;
        // init transfer
        SPDR = 0;
        // store input in external buffer
        *p_in++ = in;
    }
    // wait untill data is send
    while (!(SPSR & _BV(SPIF))) {}
    // store input in external buffer
    *p_in = SPDR;
}

void slight_TLC5957::spi_transfer_buffer(
    void *buf_write, void *buf_read, size_t count
) {
    if (count == 0) return;
    uint8_t *p_out = reinterpret_cast<uint8_t *>(buf_write);
    uint8_t *p_in = reinterpret_cast<uint8_t *>(buf_read);
    // send output
    SPDR = *p_out;
    while (--count > 0) {
        // prepare output data
        uint8_t out = *(p_out + 1);
        // wait untill data is transferred
        while (!(SPSR & _BV(SPIF))) {}
        // read input
        uint8_t in = SPDR;
        // send output
        SPDR = out;
        // prepare next loop
        p_out++;
        // store input in external buffer
        *p_in++ = in;
    }
    // wait untill data is send
    while (!(SPSR & _BV(SPIF))) {}
    // store input in external buffer
    *p_in = SPDR;
}

void slight_TLC5957::write() {
    // TODO(s-light): implement.

}

void slight_TLC5957::generate_function_command(
    slight_TLC5957::function_command_pulse_count function_command
) {
    // faster speeds with direct port access...
    // https://forum.arduino.cc/index.php?topic=4324.0
    digitalWrite(sclk_pin, LOW);
    digitalWrite(lat_pin, HIGH);
    for (size_t i = 0; i < function_command; i++) {
        digitalWrite(sclk_pin, HIGH);
        digitalWrite(sclk_pin, LOW);
    }
    digitalWrite(lat_pin, LOW);
    // TODO(s-light): check if this works.
}
