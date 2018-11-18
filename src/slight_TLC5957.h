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
    // info
    //
    // most of the chapter mentionings in the code target the
    // AppNote SLVUAF0
    // Build a High-Density, High-Refresh Rate, Multiplexing Panel With the TLC5957
    // http://www.ti.com/lit/ug/slvuaf0/slvuaf0.pdf


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // constructor

    slight_TLC5957(
        uint8_t chip_count,
        uint8_t lat_pin = 7,
        uint8_t gclk_pin = 5,
        uint8_t sclk_pin = SCK,
        uint8_t sout_pin = MOSI,
        uint8_t sin_pin = MISO
    );
    ~slight_TLC5957();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types

    // 3.10 Function Commands Summary (page 30)
    // http://www.ti.com/lit/ug/slvuaf0/slvuaf0.pdf#page=30&zoom=auto,-110,464
    // WRTGS
    //     48-bit GS data write
    //     copy common 48bit to GS-data-latch[GS-counter]
    //     GS-counter -1
    // LATGS
    //     latch grayscale
    //     (768-bit GS data latch)
    //     copy common 48bit to GS-data-latch[0]
    //     if XREFRESH = 0
    //         GS-data-latch copy to GS-data-latch 2
    //     if XREFRESH = 1
    //         GS-data-latch copy to GS-data-latch 2
    // WRTFC
    //     write FC data
    //     copy common 48bit to FC-data
    //     if used after FCWRTEN
    // LINERESET
    //     Line Counter register clear.
    //     copy common 48bit to GS-data-latch[0]
    //     data-latch-counter reset
    //     if XREFRESH = 0
    //         Autorefresh enabled
    //         wehn GS-counter == 65535: GS-data-latch copy to GS-data-latch 2
    //     if XREFRESH = 1
    //         Autorefresh disabled
    //         GS-data-latch copy to GS-data-latch 2
    //         GS-counter reset
    //         OUTx forced off
    //     change group pattern when received
    // READFC
    //     read FC data
    //     copy FC-data to common 48bit
    //     (can be read at SOUT)
    // TMGRST
    //     reset line-counter
    //     GS-counter = 0
    //     OUTx forced off
    // FCWRTEN
    //     enable writes to FC
    //     this must send before WRTFC
    //
    enum function_command_pulse_count {
        fc_WRTGS = 1,
        fc_LATGS = 3,
        fc_WRTFC = 5,
        fc_LINERESET = 7,
        fc_READFC = 11,
        fc_TMGRST = 13,
        fc_FCWRTEN = 15
    };

    // 3.3.3 Function Control (FC) Register
    // BIT     NAME            default     description
    // 0-1     LODVTH          01          LED Open Detection Voltage
    // 2-3     SEL_TD0         01          TD0 select. SOUT hold time.
    // 4       SEL_GDLY        1           Group Delay. 0 = No Delay
    // 5       XREFRESH        0           auto data refresh mode.
    //                                     on LATGS/LINERESET → data copied from GS1 to GS2
    //                                     0 = enabled → GS-counter continues
    //                                     1 = disabled → GS-counter reset; OUTx forced off
    // 6       SEL_GCK_EDGE    0           GCLK edge select.
    //                                     0 = OUTx toggle only on rising edge of GLCK
    //                                     1 = OUTx toggle on rising & falling edge of GLCK
    // 7       SEL_PCHG        0           Pre-charge working mode select
    // 8       ESPWM           0           ESPWM mode enable bit. (0 = enabled, 1 = disabled)
    // 9       LGSE3           0           Compensation for Blue LED. (0 = disabled, 1 = enabled)
    // 10      SEL_SCK_EDGE    0           SCLK edge select (0 = only rising edge, 1 = both edges)
    // 11-13   LGSE1           000         Low Gray Scale Enhancement for Red/Green/Blue color
    // 14-22   CCB             100000000   Color brightness contro data Blue (000h-1FFh)
    // 23-31   CCG             100000000   Color brightness contro data Blue (000h-1FFh)
    // 32-40   CCR             100000000   Color brightness contro data Blue (000h-1FFh)
    // 41-43   BC              100         Global brightness control data (0h-7h)
    // 44      PokerTransMode  0           Poker trans mode enable bit. (0 = disabled, 1 = enabled)
    // 45-47   LGSE2           000         first line performance improvment

    // enum function_control_mask {
    //     //                      __444444443333333333222222222211111111110000000000
    //     //                      __765432109876543210987654321098765432109876543210
    //     // fc_mask_XXX =        0b000000000000000000000000000000000000000000000000,
    //     fc_mask_LODVTH =        0b000000000000000000000000000000000000000000000000,
    //     fc_mask_SEL_TD0 =       0b000000000000000000000000000000000000000000000000,
    //     fc_mask_SEL_SCK_EDGE =  0b000000000000000000000000000000000000000000000000,
    // };

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // basic library api
    void begin();
    void end();
    void write();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // gloal helpers

    const uint8_t chip_buffer_bit_count = 48;
    const uint8_t chip_buffer_byte_count = chip_buffer_bit_count / 8;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // individual registers


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // configurations

    void generate_function_command(
        function_command_pulse_count function_command
    );

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    inline static void spi_write_buffer(void *buf_write, size_t count);
    inline static void spi_read_buffer(void *buf_read, size_t count);
    inline static void spi_transfer_buffer(void *buf_write, void *buf_read, size_t count);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes
    bool ready;

    uint8_t chip_count;

    const uint8_t lat_pin;
    const uint8_t gclk_pin;
    const uint8_t sclk_pin;
    const uint8_t sout_pin;
    const uint8_t sin_pin;

    uint16_t buffer_size;
    uint16_t *buffer;

};  // class slight_TLC5957

#endif  // slight_TLC5957_H_
