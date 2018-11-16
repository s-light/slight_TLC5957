
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
    uint8_t chip_count, uint8_t lat_pin, uint8_t gclk_pin
): lat_pin(lat_pin), gclk_pin(gclk_pin) buffer(
    reinterpret_cast<uint16_t*> calloc(chip_count, chip_buffer_byte_count)
) {
    ready = false;
}

slight_TLC5957::~slight_TLC5957() {
    end();
    free(buffer);
}


bool slight_TLC5957::begin() {
    // clean up..
    end();
    // start up...
    if (ready == false) {
        // setup
        pinMode(lat_pin, OUTPUT);
        pinMode(gclk_pin, OUTPUT);
        // TODO(s-light): implement.
        SPI.begin();
        SPI.beginTransaction(SPISettings(spiClock, MSBFIRST, SPI_MODE0));
    }
    return ready;
}

void Tlc59711::end() {
  if (ready) {
    SPI.endTransaction();
    SPI.end();
  }
}

void slight_TLC5957::write() {
    // TODO(s-light): implement.
}

void slight_TLC5957::generate_function_command(
    slight_TLC5957::function_commands_SCLK_pulse_count function_command
) {
    // TODO(s-light): implement.
    digitalWrite(lat_pin)
}
