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
uint8_t gravityCounter = 0;
uint16_t shiftCounter = 0;

uint8_t i = 0;

#define OFF 0
#define GREEN 1
#define RED 2
#define YELLOW 3

void writeDisplay() {
    writebuffer[0] = 0;
    for (i = 1; i < 17; i += 2) {
        writebuffer[i] = pixelbuffer[i/2] & 0xFF;
        writebuffer[i + 1] = pixelbuffer[i/2] >> 8;
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
    return rand() % (upper-lower) + lower;
}

uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void drawWalls() {
    for (i = 0; i < 8; i++) {
        pixelbuffer[i] |= reverse(walls[i]);
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
  fall = false;
  if (gravityCounter < 255) {
    gravityCounter = 255;
  }
}

void endGame(bool flash) {
    if (flash) {
        for (i = 0; i < 8; i++) {
            pixelbuffer[i] = 0xFF00;
        }
        writeDisplay();
        __delay_ms(200);
    }
    clearPixels();
    writeDisplay();
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

void main(void)
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    initializeDisplay();
    
    IOCCF4_SetInterruptHandler(buttonInterrupt);
    
    endGame(false);
    
    while (!start);
    
    while (1)
    {
        drawPixel(1, elevation, YELLOW);
        drawWalls();
        writeDisplay();
        clearPixels();
        if (elevation < 0 || elevation > 7 || (walls[elevation] & 0b01000000)) {
            endGame(true);
        }
        if (gravityCounter >= 80) {
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
        if (shiftCounter >= 100) {
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