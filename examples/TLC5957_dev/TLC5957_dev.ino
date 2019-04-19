/******************************************************************************

    TLC5957_dev.ino
        some development test for slight_TLC5957 library.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            TLC5957
                lat_pin = 7
                gclk_pin = 9
                sclk_pin = SCK
                sout_pin = MOSI
                sin_pin = MISO


    libraries used:
        ~ slight_DebugMenu
        ~ slight_TLC5957
            written by stefan krueger (s-light),
                github@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

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

#include <slight_DebugMenu.h>

#include <SPI.h>
#include <slight_TLC5957.h>

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
    out.println(F("| TLC5957_dev.ino"));
    out.println(F("|   development things for slight_TLC5957 library."));
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
    // __FILE__  /home/stefan/mydata/arduino_sketchbook/libraries/slight_TLC5957/examples/TLC5957_dev/TLC5957_dev.ino
    // __BASE_FILE__ /tmp/arduino_build_330237/sketch/TLC5957_dev.ino.cpp

}


// Serial.print to Flash: Notepad++ Replace RegEx
//     Find what:        Serial.print(.*)\("(.*)"\);
//     Replace with:    Serial.print\1\(F\("\2"\)\);



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
slight_DebugMenu myDebugMenu(Serial, Serial, 20);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TLC5957

// possible options and defaults:
// slight_TLC5957(
//     uint8_t chip_count,
//     uint8_t lat_pin = 7,
//     uint8_t gclk_pin = 9,
//     uint8_t sclk_pin = SCK,
//     uint8_t sout_pin = MOSI,
//     uint8_t sin_pin = MISO
// );
const uint8_t pixel_count = 1*16;
// use default pins
slight_TLC5957 tlc = slight_TLC5957(pixel_count);


bool animation_run = true;

unsigned long animation_timestamp = 0;
//uint16_t animation_interval = 1000; //ms
uint16_t animation_interval = 5; //ms

uint8_t step = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

void menu__print_help(Print &out) {
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
    // out.println();
    // out.println(F("\t 'f': test fc 'f'"));
    out.println(F("\t 'u': tlc.show() 'u'"));
    out.print(F("\t 'r': toggle animation_run 'r' ("));
    out.print(animation_run);
    out.println(F(")"));
    out.print(F("\t 'e': toggle ESPWM 'e' ("));
    out.print(tlc.get_fc_ESPWM());
    out.println(F(")"));
    out.print(F("\t 'a': set animation_interval 'a1000' ("));
    out.print(animation_interval);
    out.println(F("ms)"));
    out.print(F("\t 'g': set grayscale frequency in MHz 'g1.0' ("));
    out.print(gsclock_get_frequency_MHz(), 4);
    out.println(F("MHz)"));
    out.print(F("\t 's': set spi baudrate in MHz 's1.0' ("));
    out.print(tlc.spi_baudrate / (1000.0 * 1000), 4);
    out.println(F("MHz)"));
    out.println(F("\t 't': set buffer to test values 't'"));
    out.println(F("\t 'p': set pixel 'p0:65535'"));
    out.println(F("\t 'P': set all pixel 'P65535'"));
    out.println(F("\t 'z': set all pixel to 21845 'z'"));
    out.println(F("\t 'b': set all pixel to black 'b'"));
    out.println(F("\t 'B': print Buffer 'B'"));
    out.println(F("\t 'F': print buffer_fc 'F'"));
    out.println();
    out.println(F("____________________________________________________________"));
}

void menu__test_buffer(Print &out) {
    out.println(F("SetBuffer:"));
    out.println(F("--- old"));
    print_tlc_buffer(out);
    // tlc.set_pixel_all_16bit_value(
    //     255, 1, 1);
    //     // 0b11000110, 0b11000110, 0b11000110); // 198
    //     // 0x0055, 0x0055, 0x0055);
    //     // 0b01010101, 0b10101010, 0b10011001);
    //     // 0x0055, 0x00AA, 0x0099);
    //     // 85, 170, 153);
    // out.println(F("--- new"));
    // print_tlc_buffer(out);

    out.println(F("--- red"));
    tlc.set_pixel_all_16bit_value(1, 0, 0);
    print_tlc_buffer(out);
    tlc.show();
    delay(1000);
    out.println(F("--- green"));
    tlc.set_pixel_all_16bit_value(0, 1, 0);
    print_tlc_buffer(out);
    tlc.show();
    delay(1000);
    out.println(F("--- blue"));
    tlc.set_pixel_all_16bit_value(0, 0, 1);
    print_tlc_buffer(out);
    tlc.show();
    delay(1000);
    out.println(F("--- red full"));
    tlc.set_pixel_all_16bit_value(65535, 0, 0);
    print_tlc_buffer(out);
    tlc.show();
    delay(100);
    out.println(F("--- white"));
    tlc.set_pixel_all_16bit_value(1, 1, 1);
    print_tlc_buffer(out);
    tlc.show();

    out.println();
}

void menu__set_pixel(Print &out, char *command) {
    out.print(F("Set pixel "));
    uint8_t command_offset = 1;
    uint8_t index = atoi(&command[command_offset]);
    // a better way than this would be to search for the ':'
    // i have used this a long time ago for MAC address format parsing
    // was something with 'tokenize' or similar..
    command_offset = 3;
    if (index > 9) {
        command_offset = command_offset +1;
    }
    out.print(index);
    out.print(F(" to "));
    uint16_t value = atoi(&command[command_offset]);
    out.print(value);
    tlc.set_pixel_16bit_value(index, value, value, value);
    out.println();
}


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
            menu__print_help(out);
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
            out.println(F("write buffer to chips"));
            tlc.show();
        } break;
        case 'r': {
            out.println(F("toggle animation_run"));
            animation_run = !animation_run;
        } break;
        case 'e': {
            out.println(F("toggle ESPWM"));
            tlc.set_fc_ESPWM_all(!tlc.get_fc_ESPWM());
            tlc.update_fc();
        } break;
        case 'a': {
            out.println(F("set animation interval:"));
            uint16_t value = atoi(&command[1]);
            out.print(value);
            animation_interval = value;
            out.println();
        } break;
        case 'g': {
            out.print(F("set grayscale frequency - new value:"));
            float value = atof(&command[1]);
            out.print(value);
            value = gsclock_set_frequency_MHz(value);
            out.print(F(" → "));
            out.print(value, 4);
            out.println(F("MHz"));
        } break;
        case 's': {
            out.print(F("set spi baudrate in MHz - new value:"));
            float value = atof(&command[1]);
            out.print(value, 4);
            out.println(F("MHz"));
            tlc.spi_baudrate = value * 1000 * 1000;
            // out.print(F(" → "));
            // out.print(tlc.spi_baudrate);
            // out.println();
        } break;
        case 't': {
            menu__test_buffer(out);
        } break;
        case 'p': {
            menu__set_pixel(out, command);
        } break;
        case 'P': {
            out.print(F("Set all pixel to "));
            uint16_t value = atoi(&command[1]);
            tlc.set_pixel_all_16bit_value(value, value, value);
            out.print(value);
            out.println();
        } break;
        case 'z': {
            out.println(F("Set all Pixel to 21845."));
            tlc.set_pixel_all_16bit_value(21845, 21845, 21845);
            out.println();
        } break;
        case 'b': {
            out.println(F("Set all Pixel to black."));
            tlc.set_pixel_all_16bit_value(0, 0, 0);
            out.println();
        } break;
        case 'B': {
            out.println(F("Print Buffer:"));
            print_tlc_buffer(out);
            out.println();
        } break;
        case 'F': {
            out.println(F("Print buffer_fc:"));
            tlc.print_buffer_fc(out);
            out.println();
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

void tlc_init(Print &out) {
    out.println(F("setup tlc:")); {

        tlc.print_buffer_fc(out);

        out.println(F("  tlc.begin()"));
        tlc.begin();

        tlc.print_buffer_fc(out);

        out.println(F("  set spi_baudrate"));
        // 2MHz
        tlc.spi_baudrate = 2.0 * 1000 * 1000;
        // 0.001MHz = 1000kHz
        // tlc.spi_baudrate = 0.001 * 1000 * 1000;

        out.println(F("  set function configurations"));
        tlc.set_fc_CC_all(0x1FF, 0x1FF, 0x0FF);
        tlc.set_fc_BC_all(0x4);
        tlc.set_fc_ESPWM_all(true);

        tlc.print_buffer_fc(out);


        out.print(F("  tlc.pixel_count: "));
        out.print(tlc.pixel_count);
        out.println();
        out.print(F("  tlc.chip_count: "));
        out.print(tlc.chip_count);
        out.println();
        out.print(F("  tlc.buffer_byte_count: "));
        out.print(tlc.buffer_byte_count);
        out.println();
        out.print(F("  tlc.spi_baudrate: "));
        out.print(tlc.spi_baudrate);
        out.println(F("Hz"));
        out.print(F("  tlc.spi_baudrate: "));
        out.print(tlc.spi_baudrate / 1000.0, 4);
        out.println(F("kHz"));
        // out.print(F("  tlc.spi_baudrate: "));
        // out.print(tlc.spi_baudrate / (1000.0 * 1000.0), 4);
        // out.println(F("MHz"));
        out.print(F("  tlc.spi_baudrate: "));
        out.print(tlc.spi_baudrate / float(1000 * 1000), 4);
        out.println(F("MHz"));

        out.print(F("  tlc.get_fc_ESPWM(): "));
        out.print(tlc.get_fc_ESPWM());
        out.println();
    }
    out.println(F("  finished."));
}


void gsclock_init(Print &out) {
    out.println(F("init gsclock:")); {
        out.println(F("  init gsclock timer."));
        setup_D9_10MHz();
        // out.println(F("  set gsclock to 3MHz."));
        // gsclock_set_frequency_MHz(3.0);
        out.println(F("  set gsclock to 30MHz."));
        gsclock_set_frequency_MHz(30.0);
    }
    out.println(F("  finished."));
}


void setup_D9_10MHz() {

    // Activate timer TC3
    // CLK_TC3_APB
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;

    // Set up the generic clock (GCLK7)
    GCLK->GENCTRL[7].reg =
        // Divide clock source by divisor 1
        GCLK_GENCTRL_DIV(1) |
        // Set the duty cycle to 50/50 HIGH/LOW
        GCLK_GENCTRL_IDC |
        // Enable GCLK7
        GCLK_GENCTRL_GENEN |
        // Select 120MHz DPLL clock source
        GCLK_GENCTRL_SRC_DPLL0;
    // Wait for synchronization
    while (GCLK->SYNCBUSY.bit.GENCTRL7);

    // for PCHCTRL numbers have a look at Table 14-9. PCHCTRLm Mapping page168ff
    // http://ww1.microchip.com/downloads/en/DeviceDoc/60001507C.pdf#page=169&zoom=page-width,-8,696
    GCLK->PCHCTRL[26].reg =
        // Enable the TC3 peripheral channel
        GCLK_PCHCTRL_CHEN |
        // Connect generic clock 7 to TC3
        GCLK_PCHCTRL_GEN_GCLK7;

    // Enable the peripheral multiplexer on pin D9
    PORT->Group[g_APinDescription[9].ulPort].
        PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;

    // Set the D9 (PORT_PA19) peripheral multiplexer to
    // peripheral (odd port number) E(6): TC3, Channel 1
    // check if you need even or odd PMUX!!!
    // http://forum.arduino.cc/index.php?topic=589655.msg4064311#msg4064311
    PORT->Group[g_APinDescription[9].ulPort].
        PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);

    TC3->COUNT8.CTRLA.reg =
        // Set prescaler to 2
        // 120MHz/2 = 60MHz
        TC_CTRLA_PRESCALER_DIV2 |
        // Set the reset/reload to trigger on prescaler clock
        TC_CTRLA_PRESCSYNC_PRESC;

    // Set-up TC3 timer for
    // Match Frequency Generation (MFRQ)
    // the period time (T) is controlled by the CC0 register.
    // (instead of PER or MAX)
    // WO[0] toggles on each Update condition.
    TC3->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    // Wait for synchronization
    // while (TC3->COUNT8.SYNCBUSY.bit.WAVE)

    // Set-up the CC (counter compare), channel 0 register
    // this sets the period
    //
    // (clockfreq / 2) / (CC0 + 1)  = outfreq  | * (CC0 + 1)
    // (clockfreq / 2) = outfreq * (CC0 + 1)   | / outfreq
    // (clockfreq / 2) / outfreq  = CC0 + 1    | -1
    // ((clockfreq / 2) / outfreq) -1  = CC0
    //
    // ((60 / 2) / 2) -1  = CC0
    //
    // MAX: (60MHz / 2) / (0 + 1)  = 30MHz
    // MIN: (60MHz / 2) / (255 + 1)  = 0,117MHz = 117kHz
    //
    //       60.0MHz
    //   0 = 30.0MHz
    //   1 = 15.0MHz
    //   2 = 10.0MHz
    //   3 =  7.5MHz
    //   4 =  6.0MHz
    //   5 =  5.0MHz
    //   9 =  3.0MHz
    //  14 =  2.0MHz
    //  29 =  1.0MHz
    //  59 =  0.5MHz
    //  74 =  0.4MHz
    //  99 =  0.3MHz
    // 149 =  0.2MHz
    // 255 =  0.11MHz
    // start with 10MHz
    TC3->COUNT8.CC[0].reg = 2;
    // Wait for synchronization
    while (TC3->COUNT8.SYNCBUSY.bit.CC1);

    // Enable timer TC3
    TC3->COUNT8.CTRLA.bit.ENABLE = 1;
    // Wait for synchronization
    while (TC3->COUNT8.SYNCBUSY.bit.ENABLE);
}


float gsclock_set_frequency_MHz(float frequency_MHz) {
    const float frequency_MHz_min = 0.117 ;
    const float frequency_MHz_max = 30.0;
    if (frequency_MHz < frequency_MHz_min) {
        frequency_MHz = frequency_MHz_min;
    }
    if (frequency_MHz > frequency_MHz_max) {
        frequency_MHz = frequency_MHz_max;
    }
    float frequency_MHz_result = -1;
    // initialise to 1MHz
    uint8_t period_reg = 29;
    float req_raw = ((60 / 2) / frequency_MHz) -1;
    period_reg = int(req_raw);
    set_D9_period_reg(period_reg);
    // calculate actual used frequency
    frequency_MHz_result = (60.0 / 2) / (period_reg + 1);
    return frequency_MHz_result;
}


float gsclock_get_frequency_MHz() {
    uint8_t period_reg = get_D9_period_reg();
    float frequency_MHz_result = (60.0 / 2) / (period_reg + 1);
    return frequency_MHz_result;
}


void set_D9_period_reg(uint8_t period_reg) {
    TC3->COUNT8.CC[0].reg = period_reg;
    while (TC3->COUNT8.SYNCBUSY.bit.CC1);
}


uint8_t get_D9_period_reg() {
    return TC3->COUNT8.CC[0].reg;
}


void print_tlc_buffer(Print &out) {
    uint8_t *buffer = tlc.buffer;
    // uint16_t *buffer16 = reinterpret_cast<uint16_t *>(tlc.buffer);
    char color_names[][6] = {
        "index",
        "blue ",
        "green",
        "red  ",
    };
    // print pixel index
    out.print(color_names[0]);
    out.print(F(" "));
    uint8_t index = 0;
    slight_DebugMenu::print_uint16_align_right(out, index);
    for (index = 1; index < tlc.pixel_count; index++) {
        out.print(F(", "));
        slight_DebugMenu::print_uint16_align_right(out, index);
    }
    out.println();
    // print data
    for (size_t color_i = 0; color_i < tlc.COLORS_PER_PIXEL; color_i++) {
        out.print(color_names[color_i + 1]);
        out.print(F(" "));
        index = color_i * tlc.BUFFER_BYTES_PER_COLOR;
        slight_DebugMenu::print_uint16_align_right(
            // out, buffer16[index]);
            out, ((buffer[index + 1]<<0) | (buffer[index + 0]<<8)) );
            // out, index);
        for (
            index += tlc.BUFFER_BYTES_PER_PIXEL;
            index < tlc.buffer_byte_count;
            index += tlc.BUFFER_BYTES_PER_PIXEL
        ) {
            out.print(F(", "));
            slight_DebugMenu::print_uint16_align_right(
                // out, buffer16[index]);
                out, ((buffer[index + 1]<<0) | (buffer[index + 0]<<8)) );
                // out, index);
        }
        out.println();
    }
    out.println(F("raw: "));
    const uint8_t line_length = 8*2*3;
    out.print(F(" p1 "));
    slight_DebugMenu::print_uint8_array(out, tlc.buffer, line_length);
    out.println();
    out.print(F(" p2 "));
    slight_DebugMenu::print_uint8_array(out, tlc.buffer + line_length, line_length);
    out.println();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// animation

void animation_init(Print &out) {
    out.println(F("init animation:")); {
        out.print(F("  animation_interval: "));
        out.print(animation_interval);
        out.println();

        // out.println(F("  Set all Pixel to 21845."));
        // tlc.set_pixel_all_16bit_value(21845, 21845, 21845);
        out.println(F("  Set all Pixel to red=blue=100."));
        tlc.set_pixel_all_16bit_value(100, 0, 100);
    }
    out.println(F("  finished."));
}


void animation_update() {
    if ((millis() - animation_timestamp) > animation_interval) {
        animation_timestamp = millis();
        // tlc.set_pixel_16bit_value(step, 0, 0, 500);
        // step += 1;
        // // Serial.print("step:");
        // // Serial.println(step);
        // if (step >= tlc.pixel_count) {
        //     step = 0;
        //     Serial.println("step wrap around.");
        //     tlc.set_pixel_all_16bit_value(0, 0, 0);
        // }
        if (animation_run) {
            tlc.show();
        }
    }
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

        tlc_init(Serial);
        gsclock_init(Serial);
        animation_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin(true);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("wait 1s."));
        delay(1000);
        Serial.println(F("Loop:"));

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
