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
        uint16_t pixel_count = 16,
        uint8_t latch = 7,
        uint8_t gsclk = 9,
        uint8_t spi_clock = SCK,
        uint8_t spi_mosi = MOSI,
        uint8_t spi_miso = MISO
    );
    ~slight_TLC5957();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types

    // ##########################################
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
        _FC__WRTGS = 1,
        _FC__LATGS = 3,
        _FC__WRTFC = 5,
        _FC__LINERESET = 7,
        _FC__READFC = 11,
        _FC__TMGRST = 13,
        _FC__FCWRTEN = 15
    };

    // ##########################################
    // 3.3.3 Function Control (FC) Register
    // BIT     NAME            default     description
    // 0-1     LODVTH          01          LED Open Detection Voltage
    // 2-3     SEL_TD0         01          TD0 select. SOUT hold time.
    // 4       SEL_GDLY        1           Group Delay. 0 = No Delay
    // 5       XREFRESH        0           auto data refresh mode.
    //                                     on LATGS/LINERESET → data copied
    //                                       from GS1 to GS2
    //                                     0 = enabled → GS-counter continues
    //                                     1 = disabled → GS-counter reset;
    //                                       OUTx forced off
    // 6       SEL_GCK_EDGE    0           GCLK edge select.
    //                                     0 = OUTx toggle only on
    //                                       rising edge of GLCK
    //                                     1 = OUTx toggle on
    //                                       rising & falling edge of GLCK
    // 7       SEL_PCHG        0           Pre-charge working mode select
    // 8       ESPWM           0           ESPWM mode enable bit.
    //                                       (0 = enabled, 1 = disabled)
    // 9       LGSE3           0           Compensation for Blue LED.
    //                                       (0 = disabled, 1 = enabled)
    // 10      SEL_SCK_EDGE    0           SCLK edge select
    //                                       (0 = rising edge, 1 = both edges)
    // 11-13   LGSE1           000         Low Gray Scale Enhancement for
    //                                       Red/Green/Blue color
    // 14-22   CCB             100000000   Color brightness control data Blue
    //                                       (000h-1FFh)
    // 23-31   CCG             100000000   Color brightness control data Green
    //                                       (000h-1FFh)
    // 32-40   CCR             100000000   Color brightness control data Red
    //                                       (000h-1FFh)
    // 41-43   BC              100         Global brightness control data
    //                                       (0h-7h)
    // 44      PokerTransMode  0           Poker trans mode enable bit.
    //                                       (0 = disabled, 1 = enabled)
    // 45-47   LGSE2           000         first line performance improvment

    const uint8_t _FC_BIT_COUNT = CHIP_BUFFER_BIT_COUNT;

    struct function_control_t {
      const uint8_t offset;
      const uint8_t length;
      const uint16_t mask;
      const uint16_t defaultv;
    };

    static const struct _FC_FIELDS_t {
        const function_control_t LODVTH = {
            .offset = 0,
            .length = 2,
            .mask = 0b11,
            .defaultv = 0b01,
        };
        const function_control_t SEL_TD0 = {
            .offset = 2,
            .length = 2,
            .mask = 0b11,
            .defaultv = 0b01,
        };
        const function_control_t SEL_GDLY = {
            .offset = 4,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b1,
        };
        const function_control_t XREFRESH = {
            .offset = 5,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t SEL_GCK_EDGE = {
            .offset = 6,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t SEL_PCHG = {
            .offset = 7,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t ESPWM = {
            .offset = 8,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t LGSE3 = {
            .offset = 9,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t LGSE1 = {
            .offset = 11,
            .length = 3,
            .mask = 0b111,
            .defaultv = 0b000,
        };
        const function_control_t CCB = {
            .offset = 14,
            .length = 9,
            .mask = 0b111111111,
            .defaultv = 0b100000000,
        };
        const function_control_t CCG = {
            .offset = 23,
            .length = 9,
            .mask = 0b111111111,
            .defaultv = 0b100000000,
        };
        const function_control_t CCR = {
            .offset = 32,
            .length = 9,
            .mask = 0b111111111,
            .defaultv = 0b100000000,
        };
        const function_control_t BC = {
            .offset = 41,
            .length = 3,
            .mask = 0b111,
            .defaultv = 0b100,
        };
        const function_control_t PokerTransMode = {
            .offset = 44,
            .length = 1,
            .mask = 0b1,
            .defaultv = 0b0,
        };
        const function_control_t LGSE2 = {
            .offset = 45,
            .length = 3,
            .mask = 0b111,
            .defaultv = 0b000,
        };
    } _FC_FIELDS;



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // basic library api
    void begin();
    void end();
    void update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // helper

    const uint8_t COLORS_PER_PIXEL = 3;
    const uint8_t PIXEL_PER_CHIP = 16;
    const uint8_t CHANNEL_PER_CHIP = COLORS_PER_PIXEL * PIXEL_PER_CHIP;

    const uint8_t BUFFER_BYTES_PER_COLOR = 2;
    const uint8_t BUFFER_BYTES_PER_PIXEL = BUFFER_BYTES_PER_COLOR * COLORS_PER_PIXEL;

    const uint8_t CHIP_BUFFER_BIT_COUNT = 48;
    const uint8_t CHIP_BUFFER_BYTE_COUNT = CHIP_BUFFER_BIT_COUNT / 8;
    const uint8_t CHIP_GS_BUFFER_BYTE_COUNT = CHIP_BUFFER_BYTE_COUNT * PIXEL_PER_CHIP;
    const uint8_t CHIP_FUNCTION_CMD_BIT_COUNT = 16;
    const uint8_t CHIP_FUNCTION_CMD_BYTE_COUNT = CHIP_FUNCTION_CMD_BIT_COUNT / 8;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // individual registers


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // configurations

    void write_SPI_with_function_command(
        function_command_pulse_count function_command,
        uint16_t value
    );

    const uint8_t pixel_count;
    const uint8_t chip_count;

    const uint16_t buffer_byte_count;
    uint16_t *buffer;

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes
    bool ready;

    const uint8_t latch;
    const uint8_t gsclk;
    const uint8_t spi_clock;
    const uint8_t spi_mosi;
    const uint8_t spi_miso;



};  // class slight_TLC5957

#endif  // slight_TLC5957_H_
