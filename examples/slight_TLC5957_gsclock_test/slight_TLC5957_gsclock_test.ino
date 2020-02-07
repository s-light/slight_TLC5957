/******************************************************************************

    slight_TLC5957_gsclock_test.ino
        test of custom high speed clock output
        for TLC5957 grayscale clock.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino with SAMD51
            → tested with Adafruit ItsyBitsy M4
            LED on pin 13
            osciloscope on gclk_pin = 9

    libraries used:
        ~ slight_DebugMenu

    written by stefan krueger (s-light),
        git@s-light.eu, http://s-light.eu, https://github.com/s-light/

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

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
    out.println(F("| slight_TLC5957_gsclock_test.ino"));
    out.println(F("|   test high-speed clock output"));
    out.println(F("|   for TLC5957 grayscale clock"));
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
slight_DebugMenu myDebugMenu(Serial, Serial, 15);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// TLC5957

uint8_t gclk_pin = 9;

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
            out.println(F("\t 'f': set D9 & D2 frequency in MHz 'f:1.0'"));
            out.println(F("\t 'r': set D9 period_reg 'r:255'"));
            out.println(F("\t 'R': set D2 period_reg 'r:255'"));
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
        case 'f': {
            out.print(F("set frequency - new value:"));
            float value = atof(&command[1]);
            out.print(value);
            value = gsclock_set_frequency_MHz(value);
            out.print(F(" → "));
            out.print(value);
            out.println();
        } break;
        case 'r': {
            out.print(F("set period_reg - new value:"));
            uint8_t value = atoi(&command[1]);
            out.print(value);
            out.println();
            set_D2_period_reg(value);
            set_D9_period_reg(value);
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

void gsclock_init(Print &out) {
    out.println(F("init gsclock:")); {
        out.println(F("  init gsclock timer."));
        setup_GenericClock7();
        setup_D9_10MHz();
        setup_D2_10MHz();
        out.println(F("  set gsclock to 3MHz."));
        gsclock_set_frequency_MHz(3.0);
        // out.println(F("  set gsclock to 30MHz."));
        // gsclock_set_frequency_MHz(30.0);
    }
    out.println(F("  finished."));
}

// Set-up the CC (counter compare), channel 0 register
// this sets the period
// 750 / (2 * (CC0 + 1))  = outputfrequence
// (750 / 2) / (CC0 + 1)  = outputfrequence
// (750 / 2) / (2 + 1)  = 125
// 750kHz / (2 * (255 + 1))  = 1,4648kHz
// tests
//       750.00kHz =   1,33us
//   0 = 375.00kHz =   2.67us
//   1 = 187.50kHz =   5.35us
//   2 = 125.00kHz =   8.02us
//   3 =  93.75kHz =  10.67us
//   4 =  74.90kHz =  13.40us
//   5 =  61.80kHz =  16.20us
//  10 =  33.60kHz =  29.80us
//  64 =   5.74kHz = 174.00us
// 128 =   2.89kHz = 346.00us
// 255 =   1.46kHz = 687.00us
//
// (clockfreq / 2) / (CC0 + 1)  = outfreq  | * (CC0 + 1)
// (clockfreq / 2) = outfreq * (CC0 + 1)   | / outfreq
// (clockfreq / 2) / outfreq  = CC0 + 1    | -1
// ((clockfreq / 2) / outfreq) -1  = CC0
// (750 / 2) / 93.75  = CC0 + 1
// ((750 / 2) / 93.75) - 1  = CC0
// --------------------------
// ((60 / 2) / 2) -1  = CC0
// (60MHz / 2) / (0 + 1)  = 30MHz
// (60MHz / 2) / (255 + 1)  = 0,117MHz = 117kHz
//
//       60.0MHz
//   0 = 30.0MHz
//   1 = 15.0MHz
//   2 = 10.0MHz
//   3 =  7.5MHz
//   4 =  6.0MHz
//   5 =  5.0MHz
//   9 =  3.0MHz
//  10 =  2.7MHz
//  14 =  2.0MHz
//  29 =  1.0MHz
//  59 =  0.5MHz = 500kHz
//  74 =  0.4MHz
//  99 =  0.3MHz
// 149 =  0.2MHz
// 255 =  0.11MHz


void setup_GenericClock7() {
    // Set up the generic clock (GCLK7)
    GCLK->GENCTRL[7].reg =
        // Divide clock source by divisor 1
        GCLK_GENCTRL_DIV(1) |
        // Set the duty cycle to 50/50 HIGH/LOW
        GCLK_GENCTRL_IDC |
        // Enable GCLK7
        GCLK_GENCTRL_GENEN |
        // // Select 48MHz DFLL clock source
        // GCLK_GENCTRL_SRC_DFLL;
        // // Select 100MHz DPLL clock source
        // GCLK_GENCTRL_SRC_DPLL1;
        // Select 120MHz DPLL clock source
        GCLK_GENCTRL_SRC_DPLL0;
    // Wait for synchronization
    while (GCLK->SYNCBUSY.bit.GENCTRL7) {}
}


void setup_D9_10MHz() {
    // Activate timer TC3
    // check for correct mask at MCLK – Main Clock - 15.7 Register Summary
    // page178f
    // http://ww1.microchip.com/downloads/en/DeviceDoc/60001507C.pdf#page=178&zoom=page-width,-8,539
    // CLK_TC3_APB
    MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;

    // for PCHCTRL numbers have a look at
    // Table 14-9. PCHCTRLm Mapping page168ff
    // http://ww1.microchip.com/downloads/en/DeviceDoc/60001507C.pdf#page=169&zoom=page-width,-8,696
    // GCLK->PCHCTRL[GCLK_TC3].reg =
    GCLK->PCHCTRL[26].reg =
        // Enable the peripheral channel
        GCLK_PCHCTRL_CHEN |
        // Connect generic clock 7
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
    while (TC3->COUNT8.SYNCBUSY.bit.CC1) {}

    // Enable timer TC3
    TC3->COUNT8.CTRLA.bit.ENABLE = 1;
    // Wait for synchronization
    while (TC3->COUNT8.SYNCBUSY.bit.ENABLE) {}
}

void set_D9_period_reg(uint8_t period_reg) {
    TC3->COUNT8.CC[0].reg = period_reg;
    while (TC3->COUNT8.SYNCBUSY.bit.CC1) {}
}

uint8_t get_D9_period_reg() {
    return TC3->COUNT8.CC[0].reg;
}


void setup_D2_10MHz() {
    // Activate timer TC1
    // check for correct mask at MCLK – Main Clock - 15.7 Register Summary
    // page178f
    // http://ww1.microchip.com/downloads/en/DeviceDoc/60001507C.pdf#page=178&zoom=page-width,-8,539
    // CLK_TC1_APB
    MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1;

    // for PCHCTRL numbers have a look at
    // Table 14-9. PCHCTRLm Mapping page168ff
    // http://ww1.microchip.com/downloads/en/DeviceDoc/60001507C.pdf#page=169&zoom=page-width,-8,696
    // GCLK->PCHCTRL[GCLK_TC1].reg =
    GCLK->PCHCTRL[9].reg =
        // Enable the peripheral channel
        GCLK_PCHCTRL_CHEN |
        // Connect generic clock 7
        GCLK_PCHCTRL_GEN_GCLK7;

    // Enable the peripheral multiplexer on pin D2
    PORT->Group[g_APinDescription[2].ulPort].
        PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;

    // Set the D2 (PORT_PA07) peripheral multiplexer to
    // peripheral (odd port number) E(6): TC1, Channel 1
    // check if you need even or odd PMUX!!!
    // http://forum.arduino.cc/index.php?topic=589655.msg4064311#msg4064311
    PORT->Group[g_APinDescription[2].ulPort].
        PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);

    TC1->COUNT8.CTRLA.reg =
        // Set prescaler to 2
        // 120MHz/2 = 60MHz
        TC_CTRLA_PRESCALER_DIV2 |
        // Set the reset/reload to trigger on prescaler clock
        TC_CTRLA_PRESCSYNC_PRESC;

    // Set-up TC1 timer for
    // Match Frequency Generation (MFRQ)
    // the period time (T) is controlled by the CC0 register.
    // (instead of PER or MAX)
    // WO[0] toggles on each Update condition.
    TC1->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    // Wait for synchronization
    // while (TC1->COUNT8.SYNCBUSY.bit.WAVE)

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
    TC1->COUNT8.CC[0].reg = 2;
    // Wait for synchronization
    while (TC1->COUNT8.SYNCBUSY.bit.CC1) {}

    // Enable timer TC1
    TC1->COUNT8.CTRLA.bit.ENABLE = 1;
    // Wait for synchronization
    while (TC1->COUNT8.SYNCBUSY.bit.ENABLE) {}
}

void set_D2_period_reg(uint8_t period_reg) {
    TC1->COUNT8.CC[0].reg = period_reg;
    while (TC1->COUNT8.SYNCBUSY.bit.CC1) {}
}

uint8_t get_D2_period_reg() {
    return TC1->COUNT8.CC[0].reg;
}



float gsclock_set_frequency_MHz(float frequency_MHz) {
    const float frequency_MHz_min = 0.117;
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
    period_reg = static_cast<int>(req_raw);
    set_D9_period_reg(period_reg);
    set_D2_period_reg(period_reg);
    // calculate actual used frequency
    frequency_MHz_result = (60.0 / 2) / (period_reg + 1);
    return frequency_MHz_result;
}

float gsclock_get_frequency_MHz() {
    // uint8_t period_reg = get_D9_period_reg();
    uint8_t period_reg = get_D2_period_reg();
    float frequency_MHz_result = (60.0 / 2) / (period_reg + 1);
    return frequency_MHz_result;
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

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // wait for arduino IDE to release all serial ports after upload.
            delay(1000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 2000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup TLC5957

        gsclock_init(Serial);

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
