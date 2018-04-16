/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65
        Device            :  PIC16F1619
        Driver Version    :  2.00
 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

#include "mcc_generated_files/mcc.h"
#include <stdlib.h>

/*
                         Main application
 */

uint8_t writebuffer[17];
uint16_t pixelbuffer[8];

uint8_t walls[8];
uint8_t createWall = 0;
uint8_t elevation = 0;
uint16_t score = 0;
uint16_t hiscore = 0;
bool fall = true;
bool start = false;
bool stopPrinting = false;
uint8_t gravityCounter = 0;
uint16_t shiftCounter = 0;

uint8_t i = 0;
uint8_t wallColor = 1;

uint16_t hiscore_addr = 0x1F05;
uint16_t zerobuf[ERASE_FLASH_BLOCKSIZE];

#define OFF 0
#define GREEN 1
#define RED 2
#define YELLOW 3

#define NEW_HISCORE 10
#define OLD_HISCORE 11
#define PLAYERSUCKS 12

uint8_t letter_space[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
};

uint8_t letter_n[8] = {
    0b00000000,
    0b01000010,
    0b01100010,
    0b01010010,
    0b01001010,
    0b01000110,
    0b01000010,
    0b00000000
};

uint8_t letter_e[8] = {
    0b00000000,
    0b01111100,
    0b01000000,
    0b01110000,
    0b01110000,
    0b01000000,
    0b01111100,
    0b00000000
};

uint8_t letter_w[8] = {
    0b00000000,
    0b01000100,
    0b01000100,
    0b01010100,
    0b01010100,
    0b01010100,
    0b01111100,
    0b00000000
};

uint8_t letter_hi[8] = {
    0b00000000,
    0b01001010,
    0b01001000,
    0b01111010,
    0b01111010,
    0b01001010,
    0b01001010,
    0b00000000
};

uint8_t letter_s[8] = {
    0b00000000,
    0b01111110,
    0b01000000,
    0b01000000,
    0b01111110,
    0b00000010,
    0b01111110,
    0b00000000
};

uint8_t letter_c[8] = {
    0b00000000,
    0b01111110,
    0b01100000,
    0b01100000,
    0b01100000,
    0b01100000,
    0b01111110,
    0b00000000
};

uint8_t letter_o[8] = {
    0b00000000,
    0b01111110,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01111110,
    0b00000000
};

uint8_t letter_r[8] = {
    0b00000000,
    0b01111100,
    0b01000100,
    0b01111100,
    0b01110000,
    0b01011000,
    0b01001100,
    0b00000000
};

uint8_t number_0[8] = {
    0b00000000,
    0b01111110,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01000010,
    0b01111110,
    0b00000000
};

uint8_t number_1[8] = {
    0b00000000,
    0b01111000,
    0b00011000,
    0b00011000,
    0b00011000,
    0b00011000,
    0b01111110,
    0b00000000
};

uint8_t number_2[8] = {
    0b00000000,
    0b01111110,
    0b00000010,
    0b00000010,
    0b01111110,
    0b01000000,
    0b01111110,
    0b00000000

};

uint8_t number_3[8] = {
    0b00000000,
    0b01111110,
    0b00000010,
    0b00000010,
    0b01111110,
    0b00000010,
    0b01111110,
    0b00000000
};

uint8_t number_4[8] = {
    0b00000000,
    0b01000100,
    0b01000100,
    0b01000100,
    0b01111110,
    0b00000100,
    0b00000100,
    0b00000000
};

uint8_t number_5[8] = {
    0b00000000,
    0b01111110,
    0b01000000,
    0b01000000,
    0b01111110,
    0b00000010,
    0b01111110,
    0b00000000
};

uint8_t number_6[8] = {
    0b00000000,
    0b01111110,
    0b01000000,
    0b01111110,
    0b01000010,
    0b01000010,
    0b01111110,
    0b00000000
};

uint8_t number_7[8] = {
    0b00000000,
    0b01111110,
    0b00000010,
    0b00000010,
    0b00000010,
    0b00000010,
    0b00000010,
    0b00000000
};

uint8_t number_8[8] = {
    0b00000000,
    0b01111110,
    0b01000010,
    0b01000010,
    0b01111110,
    0b01000010,
    0b01111110,
    0b00000000
};

uint8_t number_9[8] = {
    0b00000000,
    0b01111110,
    0b01000010,
    0b01111110,
    0b00000010,
    0b00000010,
    0b00000010,
    0b00000000
};

uint8_t * numberTable[10] = {
    number_0,
    number_1,
    number_2,
    number_3,
    number_4,
    number_5,
    number_6,
    number_7,
    number_8,
    number_9
};

void writeDisplay() {
    writebuffer[0] = 0;
    for (i = 1; i < 17; i += 2) {
        writebuffer[i] = pixelbuffer[i / 2] & 0xFF;
        writebuffer[i + 1] = pixelbuffer[i / 2] >> 8;
    }
    i2c_writeNBytes(0x70, writebuffer, 17);
}

void drawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (color == GREEN) {
        pixelbuffer[y] |= 1 << x;
        pixelbuffer[y] &= ~(1 << (x + 8));
    } else if (color == RED) {
        pixelbuffer[y] |= 1 << (x + 8);
        pixelbuffer[y] &= ~(1 << x);
    } else if (color == YELLOW) {
        pixelbuffer[y] |= (1 << (x + 8)) | (1 << x);
    } else if (color == OFF) {
        pixelbuffer[y] &= ~(1 << x) & ~(1 << (x + 8));
    }
}

void clearPixels() {
    for (i = 0; i < 8; i++) {
        pixelbuffer[i] = 0;
    }
}

void initializeDisplay() {
    for (i = 0; i < 8; i++) {
        pixelbuffer[i] = 0;
    }
    for (i = 0; i < 8; i++) {
        writebuffer[i] = 0;
    }
    for (i = 0; i < 8; i++) {
        walls[i] = 0;
    }
    // connect to display
    writebuffer[0] = 0x21;
    i2c_writeNBytes(0x70, writebuffer, 1);

    // set blink rate to not blink
    writebuffer[0] = 0x80 | 0x01;
    i2c_writeNBytes(0x70, writebuffer, 1);
}

uint8_t random(uint8_t lower, uint8_t upper) {
    return rand() % (upper - lower) + lower;
}

uint8_t reverse(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

void drawWalls() {
    for (i = 0; i < 8; i++) {
        if (wallColor == GREEN) {
            pixelbuffer[i] |= reverse(walls[i]);
        } else if (wallColor == RED) {
            pixelbuffer[i] |= reverse(walls[i]) << 8;
        } else {
            pixelbuffer[i] |= reverse(walls[i]) << 8 | reverse(walls[i]);
        }
    }
}

uint8_t wallHeight = 0;
uint8_t topHalf = 0;

void shiftWalls() {
    for (i = 0; i < 8; i++) {
        walls[i] = walls[i] << 1;
    }
    if (createWall == 3) {
        wallHeight = random(4, 7);
        topHalf = random(1, wallHeight);
        for (i = 0; i < topHalf; i++) {
            walls[i] |= 1;
        }
        for (i = 7; i >= 8 - (wallHeight - topHalf); i--) {
            walls[i] |= 1;
        }
        createWall = 0;
        score++;
    } else {
        createWall++;
    }
}

void buttonInterrupt() {
    start = true;
    stopPrinting = true;
    fall = false;
    if (gravityCounter < 255) {
        gravityCounter = 255;
    }
}

void writeWord(uint8_t ** text, uint8_t len) {
    for (i = 0; i < 8; i++) {
        walls[i] = 0;
    }
    for (uint8_t idx = 0; idx < len; idx++) {
        if (stopPrinting) return;
        uint8_t shiftValue = idx % 8;
        for (uint8_t k = 0; k < 8; k++) {
            walls[k] = ((text[idx / 8][k]) << shiftValue) |
                    ((text[idx / 8 + 1][k]) >> 8 - shiftValue);
        }
        clearPixels();
        drawWalls();
        writeDisplay();
        __delay_ms(60);
    }
}

void write(uint8_t selection) {
    stopPrinting = false;
    uint16_t number = 0;
    if (selection == NEW_HISCORE && !stopPrinting) {
        wallColor = YELLOW;
        uint8_t * toWrite[5];
        toWrite[0] = letter_space;
        toWrite[1] = letter_n;
        toWrite[2] = letter_e;
        toWrite[3] = letter_w;
        toWrite[4] = letter_space;
        writeWord(toWrite, 32);
    }
    if ((selection == OLD_HISCORE || selection == NEW_HISCORE) && !stopPrinting) {
        uint8_t * toWrite[8];
        toWrite[0] = letter_space;
        toWrite[1] = letter_hi;
        toWrite[2] = letter_s;
        toWrite[3] = letter_c;
        toWrite[4] = letter_o;
        toWrite[5] = letter_r;
        toWrite[6] = letter_e;
        toWrite[7] = letter_space;
        writeWord(toWrite, 56);
    }
    if (selection == PLAYERSUCKS && !stopPrinting) {
        wallColor = RED;
        uint8_t * toWrite[7];
        toWrite[0] = letter_space;
        toWrite[1] = letter_s;
        toWrite[2] = letter_c;
        toWrite[3] = letter_o;
        toWrite[4] = letter_r;
        toWrite[5] = letter_e;
        toWrite[6] = letter_space;
        writeWord(toWrite, 48);
        number = score;
    } else {
        number = hiscore;
    }
    if (!stopPrinting) {
        uint8_t digits[5];
        digits[0] = (number / 10000) % 10;
        digits[1] = (number / 1000) % 10;
        digits[2] = (number / 100) % 10;
        digits[3] = (number / 10) % 10;
        digits[4] = number % 10;
        uint8_t * toWrite[7];
        // i have no idea why, but we need to redefine space here
        // otherwise it screws up the space display permanently
        uint8_t letter_space[8] = {
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000
        };
        toWrite[0] = letter_space;
        uint8_t writeCounter = 1;
        uint8_t x = 0;
        while (digits[x] == 0 && x < 5) x++;
        for (; x < 5; x++) {
            toWrite[writeCounter] = numberTable[digits[x]];
            writeCounter++;
        }
        if (writeCounter == 1) {
            toWrite[writeCounter] = numberTable[0];
            writeCounter++;
        }
        toWrite[writeCounter] = letter_space;
        writeWord(toWrite, writeCounter*8);
    }
}

/* Performs program reset for startup/endgame */
void endGame(bool played) {
    if (played) {
        for (i = 0; i < 8; i++) {
            pixelbuffer[i] = 0xFF00;
        }
        writeDisplay();
        __delay_ms(200);
        clearPixels();
        if (score > hiscore) {
            hiscore = score;
            FLASH_WriteWord(hiscore_addr, zerobuf, hiscore);
            write(NEW_HISCORE);
        } else {
            write(PLAYERSUCKS);
        }
    } else {
        write(OLD_HISCORE);
    }
    clearPixels();
    writeDisplay();
    wallColor = GREEN;
    for (i = 0; i < 8; i++) {
        walls[i] = 0;
    }
    writeDisplay();
    start = false;
    while (!start);
    elevation = 3;
    gravityCounter = 0;
    shiftCounter = 0;
    score = 0;
}

void main(void) {
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();

    initializeDisplay();

    IOCCF4_SetInterruptHandler(buttonInterrupt);

    hiscore = FLASH_ReadWord(hiscore_addr);
    // if hiscore is 16383, either we're dealing with a really good player
    // or we just reprogrammed the board. either way, not interested in
    // keeping the score.
    if (hiscore == 16383) {
        hiscore = 0;
        FLASH_WriteWord(hiscore_addr, zerobuf, hiscore);
    }

    endGame(false);

    while (1) {
        drawPixel(1, elevation, YELLOW);
        drawWalls();
        writeDisplay();
        clearPixels();
        if (elevation < 0 || elevation > 7 || (walls[elevation] & 0b01000000)) {
            endGame(true);
        }
        if (gravityCounter >= 70) {
            if (fall) {
                elevation += 1;
            } else {
                elevation -= 1;
                fall = true;
            }
            gravityCounter = 0;
        } else {
            gravityCounter++;
        }
        if (shiftCounter >= 80) {
            shiftWalls();
            shiftCounter = 0;
        } else {
            shiftCounter++;
        }
    }
}
/**
 End of File
 */