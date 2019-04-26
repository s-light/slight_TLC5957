/******************************************************************************

    spi_test2.ino
        test spi settings on SAMD51
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino with SAMD51
            → tested with Adafruit ItsyBitsy M4
            LED on pin 13
            osciloscope on
                spi_clock = SCK
                spi_mosi = MO

    libraries used:
        ~ slight_DebugMenu

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2019 Stefan Krüger

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

#include <SPI.h>

#include <slight_DebugMenu.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Info
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void sketchinfo_print(Print &out) {
    out.println();
    //             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|                       ^ ^                      |"));
    out.println(F("|                      (0,0)                     |"));
    out.println(F("|                      ( _ )                     |"));
    out.println(F("|                       \" \"                      |"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("| spi_test2.ino"));
    out.println(F("|   test spi settings on SAMD51"));
    out.println(F("|"));
    out.println(F("| This Sketch has a debug-menu:"));
    out.println(F("| send '?'+Return for help"));
    out.println(F("|"));
    out.println(F("| dream on & have fun :-)"));
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|"));
    //out.println(F("| compiled: Nov 11 2013  20:35:04"));
    out.print(F("| compiled: "));
    out.print(F(__DATE__));
    out.print(F("  "));
    out.print(F(__TIME__));
    out.println();
    out.print(F("| last changed: "));
    out.print(F(__TIMESTAMP__));
    out.println();
    //
    // out.println(F("|"));
    // out.print(F("| __FILE__: "));
    // out.print(F(__FILE__));
    // out.println();
    // out.print(F("| __BASE_FILE__"));
    // out.print(F(__BASE_FILE__));
    // out.println();
    // out.println(F("|"));
    //
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    // __DATE__ Nov 11 2013
    // __TIME__ 20:35:04
    // __TIMESTAMP__ Tue Dec 27 14:14:04 2016
    // __FILE__  /home/stefan/mydata/arduino_sketchbook/libraries/slight_TLC5957/examples/spi_test/spi_test2.ino
    // __BASE_FILE__ /tmp/arduino_build_330237/sketch/spi_test2.ino.cpp

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions (global)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Debug Output

boolean infoled_state = 0;
const byte infoled_pin = 13;

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = 0;
boolean debugOut_LiveSign_LED_Enabled = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu

// slight_DebugMenu(Stream &in_ref, Print &out_ref, uint8_t input_length_new);
slight_DebugMenu myDebugMenu(Serial, Serial, 15);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SPI

// 1Hz
// uint32_t spi_baudrate = 1;
// 100Hz
uint32_t spi_baudrate = (0.0001 * 1000 *  1000);
// 0.28MHz
// uint32_t spi_baudrate = (0.28 * 1000 *  1000);

uint8_t _latch = A5;
uint8_t _spi_clock = SCK;
uint8_t _spi_mosi = MOSI;
uint8_t _spi_miso = MISO;

bool animation_run = false;

unsigned long animation_timestamp = 0;
//uint16_t animation_interval = 1000; //ms
uint16_t animation_interval = 10; //ms


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

// Main Menu
void handleMenu_Main(slight_DebugMenu *pInstance) {
    Print &out = pInstance->get_stream_out_ref();
    char *command = pInstance->get_command_current_pointer();
    // out.print("command: '");
    // out.print(command);
    // out.println("'");
    switch (command[0]) {
        case 'h':
        case 'H':
        case '?': {
            // help
            out.println(F("____________________________________________________________"));
            out.println();
            out.println(F("Help for Commands:"));
            out.println();
            out.println(F("\t '?': this help"));
            out.println(F("\t '!': sketch info"));
            out.println(F("\t 'y': toggle DebugOut livesign print"));
            out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            out.println(F("\t 'x': tests"));
            out.println();
            out.println(F("\t 'u': tlc_show() 'u'"));
            out.print(F("\t 'r': toggle animation_run 'r' ("));
            out.print(animation_run);
            out.println(F(")"));
            out.print(F("\t 'a': set animation_interval 'a1000' ("));
            out.print(animation_interval);
            out.println(F("ms)"));
            out.print(F("\t 's': set spi baudrate in MHz 's1.0' ("));
            out.print(spi_baudrate / (1000.0 * 1000), 4);
            out.println(F("MHz)"));
            out.println();
            out.println(F("____________________________________________________________"));
        } break;
        case '!': {
            sketchinfo_print(out);
        } break;
        case 'y': {
            out.println(F("\t toggle DebugOut livesign Serial:"));
            debugOut_LiveSign_Serial_Enabled = !debugOut_LiveSign_Serial_Enabled;
            out.print(F("\t debugOut_LiveSign_Serial_Enabled:"));
            out.println(debugOut_LiveSign_Serial_Enabled);
        } break;
        case 'Y': {
            out.println(F("\t toggle DebugOut livesign LED:"));
            debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
            out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
            out.println(debugOut_LiveSign_LED_Enabled);
        } break;
        case 'x': {
            // get state
            out.println(F("__________"));
            out.println(F("Tests:"));
            out.println(F("nothing to do."));
            out.println(F("__________"));
        } break;
        case 'u': {
            out.println(F("tlc_show()"));
            tlc_show();
        } break;
        case 'r': {
            out.println(F("toggle animation_run"));
            animation_run = !animation_run;
        } break;
        case 'a': {
            out.println(F("set animation interval:"));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            animation_interval = value;
            out.println();
        } break;
        case 's': {
            out.print(F("set spi baudrate in MHz - new value:"));
            float value = atof(&command[1]);
            out.print(value, 4);
            out.println(F("MHz"));
            spi_baudrate = value * 1000 * 1000;
            // out.print(F(" → "));
            // out.print(tlc.spi_baudrate);
            // out.println();
        } break;
        //---------------------------------------------------------------------
        default: {
            if(strlen(command) > 0) {
                out.print(F("command '"));
                out.print(command);
                out.println(F("' not recognized. try again."));
            }
            pInstance->get_command_input_pointer()[0] = '?';
            pInstance->set_flag_EOC(true);
        }
    } // end switch

    // end Command Parser
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TLC5957

void spi_init(Print &out) {
    out.println(F("spi init:")); {
        out.println(F("  SPI.begin()"));
        SPI.begin();
        out.println(F("  setup latch pin"));
        pinMode(_latch, OUTPUT);
        digitalWrite(_latch, LOW);
    }
    out.println(F("  finished."));
}

void animation_init(Print &out) {
    out.println(F("init animation:")); {
        out.println(F("animation_interval: "));
        out.print(animation_interval);
        out.println();
    }
    out.println(F("\t finished."));
}

void animation_update() {
    if ((millis() - animation_timestamp) > animation_interval) {
        animation_timestamp = millis();
        if (animation_run) {
            tlc_show();
        }
    }
}



void tlc_show() {
    // write_SPI_minimal();
    write_SPI_TLC();
}

void write_SPI_minimal() {
    digitalWrite(_latch, HIGH);
    SPI.beginTransaction(
        SPISettings(spi_baudrate, MSBFIRST, SPI_MODE0));
    SPI.transfer(0b01010101);
    SPI.transfer(0b00000000);
    SPI.transfer(0b01010101);
    SPI.transfer(0b11111111);
    SPI.transfer(0b01010101);
    SPI.endTransaction();
    digitalWrite(_latch, LOW);
}

void write_SPI_TLC() {
    const uint8_t PIXEL_PER_CHIP = 16;
    const uint8_t _FC__WRTGS = 1;
    const uint8_t _FC__LATGS = 3;

    uint16_t write_count = 6 - 2;
    for (uint8_t pixel_index = 0; pixel_index < PIXEL_PER_CHIP; pixel_index++) {
        // configure
        SPI.begin();
        SPI.beginTransaction(SPISettings(spi_baudrate, MSBFIRST, SPI_MODE0));
        for (uint16_t byte_index = 0; byte_index < write_count; byte_index++) {
            SPI.transfer(0b00101011);
        }
        SPI.endTransaction();
        SPI.end();
        // special
        // digitalWrite(_latch, HIGH);
        // SPI.beginTransaction(SPISettings(1, MSBFIRST, SPI_MODE0));
        // SPI.transfer(0b01000111);
        // SPI.transfer(0b01000111);
        // SPI.endTransaction();
        // digitalWrite(_latch, LOW);
        if (pixel_index == (PIXEL_PER_CHIP - 1)) {
            _write_buffer_with_function_command(
                _FC__LATGS, 0xffAA);
        } else {
            _write_buffer_with_function_command(
                _FC__WRTGS, 0x55ff);
        }
    }
}


void _write_buffer_with_function_command(
    uint8_t function_command,
    uint16_t value
) {
    // """Bit-Banging SPI write to sync with latch pulse."""

    // faster speeds with direct port access:
    // https://forum.arduino.cc/index.php?topic=4324.0

    pinMode(_spi_mosi, OUTPUT);
    pinMode(_spi_clock, OUTPUT);

    digitalWrite(_spi_clock, LOW);
    digitalWrite(_spi_mosi, LOW);
    digitalWrite(_latch, LOW);

    const uint8_t CHIP_FUNCTION_CMD_BIT_COUNT = 16;

    uint8_t latch_start_index = CHIP_FUNCTION_CMD_BIT_COUNT - function_command;

    for (uint8_t i = 0; i < CHIP_FUNCTION_CMD_BIT_COUNT; i++) {
        if (latch_start_index == i) {
            digitalWrite(_latch, HIGH);
        }

        // b1000000000000000
        if (value & 0x8000u) {
            digitalWrite(_spi_mosi, HIGH);
        } else {
            digitalWrite(_spi_mosi, LOW);
        }
        value <<= 1;

        digitalWrite(_spi_clock, HIGH);
        delayMicroseconds(10);
        digitalWrite(_spi_clock, LOW);
    }

    digitalWrite(_latch, LOW);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs

        //LiveSign
        pinMode(infoled_pin, OUTPUT);
        digitalWrite(infoled_pin, HIGH);

        // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        // wait for arduino IDE to release all serial ports after upload.
        delay(1000);
        // initialise serial
        Serial.begin(115200);

        // Wait for Serial Connection to be Opend from Host or
        // timeout after 6second
        uint32_t timeStamp_Start = millis();
        while( (! Serial) && ( (millis() - timeStamp_Start) < 2000 ) ) {
            // nothing to do
        }

        Serial.println();
        Serial.println();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup TLC5957

        spi_init(Serial);
        animation_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin(true);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("wait 0.5s."));
        delay(500);
        Serial.println(F("loop:"));

} /** setup **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // menu input
        myDebugMenu.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // test things

        animation_update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // debug output

        if (
            (millis() - debugOut_LiveSign_TimeStamp_LastAction) >
            debugOut_LiveSign_UpdateInterval
        ) {
            debugOut_LiveSign_TimeStamp_LastAction = millis();

            if ( debugOut_LiveSign_Serial_Enabled ) {
                Serial.print(millis());
                Serial.print(F("ms;"));
                Serial.println();
            }

            if ( debugOut_LiveSign_LED_Enabled ) {
                infoled_state = ! infoled_state;
                if (infoled_state) {
                    //set LED to HIGH
                    digitalWrite(infoled_pin, HIGH);
                } else {
                    //set LED to LOW
                    digitalWrite(infoled_pin, LOW);
                }
            }

        }

} /** loop **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
