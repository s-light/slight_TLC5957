/******************************************************************************

    TLC5957_dev.ino
        minimal usage for slight_TLC5957 library.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            TLC5957
                TODO


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
    out.println(F("|   minimal usage for slight_TLC5957 library"));
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

// slight_TLC5957(uint8_t chipCount, uint8_t lat_pin, uint8_t gclk_pin);
slight_TLC5957 tlc = slight_TLC5957(1, 7, 5);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things

// freeRam found at
// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
// posted by mrburnette
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}




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
            // out.println();
            // out.println(F("\t 's': sensitivity set [1, 2, .., 64, 128] 's128'"));
            // out.println(F("\t 'S': sensitivity get "));
            // out.println(F("\t 'a': auto config - load recommend configuration"));
            // out.println(F("\t 'p': print things "));
            // out.println(F("\t 't': touch status get "));
            // out.println(F("\t 'c': electrode config get "));
            // out.println(F("\t 'e': electrode config electrode enable set 'e12'"));
            // out.println(F("\t 'E': electrode config electrode enable get "));
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

            // uint16_t wTest = 65535;
            // uint16_t wTest = atoi(&command[1]);
            // out.print(F("wTest: "));
            // out.print(wTest);
            // out.println();
            //
            // out.print(F("1: "));
            // out.print((byte)wTest);
            // out.println();
            //
            // out.print(F("2: "));
            // out.print((byte)(wTest>>8));
            // out.println();
            //
            // out.println();

            // out.println(F("    bit test"));
            // out.print(F("      8 (1 << 3) "));
            // slight_DebugMenu::print_Binary_8(
            //     out,
            //     (0b1 << 3)
            // );
            // out.println();
            // out.print(F("      8 (1 << 7) "));
            // slight_DebugMenu::print_Binary_8(
            //     out,
            //     (0b1 << 7)
            // );
            // out.println();
            // out.print(F("     16 (1 << 3) "));
            // slight_DebugMenu::print_Binary_16(
            //     out,
            //     (0b1 << 3)
            // );
            // out.println();
            // out.print(F("     16 (1 << 15) "));
            // slight_DebugMenu::print_Binary_16(
            //     out,
            //     (0b1 << 15)
            // );
            // out.println();
            //
            // uint32_t u32 = 0;
            // out.print(F("     32 ((uint32_t)1 << 3) "));
            // u32 = ((uint32_t)1 << 3);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (1 << 17) "));
            // u32 = ((uint32_t)1 << 17);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (1 << 31) "));
            // u32 = ((uint32_t)1 << 31);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (4,294,967,295) "));
            // u32 = (4294967295);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();

            // out.println(F("test set_register16bit_part: "));
            //
            //
            // uint16_t temp = 0b0001110000001111;
            //
            // out.println(F("original value: "));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // // out.print(F("; "));
            // // out.print(temp, HEX);
            // // out.print(F("; "));
            // // out.print(temp, BIN);
            // // out.print(F("; "));
            // // out.print(temp);
            // out.println();
            //
            // // static uint16_t set_register16bit_part(
            // //     uint16_t reg_value,
            // //     uint16_t reg_mask,
            // //     uint8_t reg_shift,
            // //     uint8_t value
            // // );
            // out.println(F("set part"));
            // temp = slight_FDC1004::set_register16bit_part(
            //     temp,
            //     0b0001110000000000,
            //     10,
            //     0b001
            // );
            // out.println(F("result:"));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // out.println();


            // temp = 0b0000000000111000;
            // out.print(F("0b0000000000111000: "));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // out.print(F("; "));
            // out.print(temp, HEX);
            // out.print(F("; "));
            // out.print(temp, BIN);
            // out.print(F("; "));
            // out.print(temp);
            // out.println();



            out.println(F("__________"));
        } break;
        //---------------------------------------------------------------------
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
        tlc.begin();
    }
    out.println(F("\t finished."));
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
            delay(2000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup TLC5957

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        tlc_init(Serial);

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
                Serial.print(F("  free RAM = "));
                Serial.println(freeRam());
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
