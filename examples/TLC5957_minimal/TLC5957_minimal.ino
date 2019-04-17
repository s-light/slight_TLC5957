/******************************************************************************

    TLC5957_minimal.ino
        minimal usage for slight_TLC5957 library.
        this sketch just lights up all pixels after each other.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            ItsyBitsy M4 (or compatible Port-Pin-Mapping)
            LED on pin 13
            TLC5957


    libraries used:
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
    out.println(F("| TLC5957_minimal.ino"));
    out.println(F("|   minimal usage for slight_TLC5957 library."));
    out.println(F("|   this sketch just lights up all pixels after each other"));
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
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    // __DATE__ Nov 11 2013
    // __TIME__ 20:35:04
    // __TIMESTAMP__ Tue Dec 27 14:14:04 2016
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
// TLC5957

// possible options and there defaults:
// slight_TLC5957(
//     uint8_t chip_count = 16,
//     uint8_t latch = 7,
//     uint8_t gsclk = 9,
//     uint8_t spi_clock = SCK,
//     uint8_t spi_mosi = MOSI,
//     uint8_t spi_miso = MISO
// );
// use default pins
slight_TLC5957 tlc = slight_TLC5957(2*16);


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
    //   5 =  5.0MHz
    //   9 =  3.0MHz → minimum for smooth if not ESPWM
    //  29 =  1.0MHz → if not ESPWM this already leads to visible flickering
    // start with 3MHz
    TC3->COUNT8.CC[0].reg = 2;
    // Wait for synchronization
    while (TC3->COUNT8.SYNCBUSY.bit.CC1);

    // Enable timer TC3
    TC3->COUNT8.CTRLA.bit.ENABLE = 1;
    // Wait for synchronization
    while (TC3->COUNT8.SYNCBUSY.bit.ENABLE);
}

void set_D9_period_reg(uint8_t period_reg) {
    TC3->COUNT8.CC[0].reg = period_reg;
    while (TC3->COUNT8.SYNCBUSY.bit.CC1);
}

unsigned long animation_timestamp = 0;
const uint16_t animation_interval = 1000; //ms

uint8_t step = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TLC5957

void tlc_init(Print &out) {
    out.println(F("setup tlc:")); {
        setup_D9_10MHz();
        tlc.begin();
    }
    out.println(F("\t finished."));
}


void update_animation() {
    if ((millis() - animation_timestamp) > animation_interval) {
        animation_timestamp = millis();
        tlc.set_pixel_16bit_value(step, 0, 0, 500);
        step += 1;
        Serial.print("step:");
        Serial.println(step);
        if (step >= tlc.pixel_count) {
            step = 0;
            Serial.println("step wrap around.");
            tlc.set_pixel_all_16bit_value(0, 0, 0);
        }
        tlc.show();
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
    // initialise PINs
    pinMode(infoled_pin, OUTPUT);
    digitalWrite(infoled_pin, HIGH);

    // wait for arduino IDE to release all serial ports after upload.
    delay(1000);
    // initialise serial
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println();

    // print welcome
    sketchinfo_print(Serial);

    // setup TLC5957
    tlc_init(Serial);

    // go
    Serial.println(F("wait 0.5s."));
    delay(500);
    Serial.println(F("loop:"));
}

void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TLC5957

    update_animation();


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

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
