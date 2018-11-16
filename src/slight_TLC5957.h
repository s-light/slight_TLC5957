/******************************************************************************

    arduino library for TI TLC5957 48-channel 16bit LED-Driver.
    tested with LEDBoard_4x4_HD and Pololu A-Star (32U4 - Leonardo compatible)
        (https://github.com/s-light/magic_amulet_pcbs)

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



#ifndef slight_TLC5957_H_
#define slight_TLC5957_H_

// include Core Arduino functionality
#include <Arduino.h>


class slight_TLC5957 {
public:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // constructor

    slight_TLC5957();
    // slight_TLC5957(uint8_t twi_address, uint8_t interrupt_pin);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // basic library api
    bool begin();
    void write();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // gloal helpers


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // individual registers


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // configurations

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes
    bool ready;

    uint8_t chipCount;

    uint8_t latPin;
    uint8_t gclkPin;

};  // class slight_TLC5957

#endif  // slight_TLC5957_H_
