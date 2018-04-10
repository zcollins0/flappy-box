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

/*
                         Main application
 */

uint8_t writebuffer[17];
uint16_t displaybuffer[8];

#define OFF 0
#define GREEN 1
#define RED 2
#define YELLOW 3

void writeDisplay() {
    writebuffer[0] = 0;
    for (uint8_t i = 1; i < 17; i += 2) {
        writebuffer[i] = displaybuffer[i/2] & 0xFF;
        writebuffer[i + 1] = displaybuffer[i/2] >> 8;
    }
    i2c_writeNBytes(0x70, writebuffer, 17);
}

void drawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (color == GREEN) {
        displaybuffer[y] |= 1 << x;
        displaybuffer[y] &= ~(1 << (x + 8));
    } else if (color == RED) {
        displaybuffer[y] |= 1 << (x + 8);
        displaybuffer[y] &= ~(1 << x);
    } else if (color == YELLOW) {
        displaybuffer[y] |= (1 << (x + 8)) | (1 << x);
    } else if (color == OFF) {
        displaybuffer[y] &= ~(1 << x) & ~(1 << (x + 8));
    }
}

void clearDisplayBuffer() {
    for (int i = 0; i < 8; i++) {
        displaybuffer[i] = 0;
    }
}

void initializeDisplay() {
    for (uint8_t i = 0; i < 8; i++) {
        displaybuffer[i] = 0;
    }
    for (uint8_t i = 0; i < 8; i++) {
        writebuffer[i] = 0;
    }
    // connect to display
    writebuffer[0] = 0x21;
    i2c_writeNBytes(0x70, writebuffer, 1);
    
    // set blink rate to 0
    writebuffer[0] = 0x80 | 0x01;
    i2c_writeNBytes(0x70, writebuffer, 1);
}

void main(void)
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    initializeDisplay();
    
    uint16_t x = 0;
    uint16_t y = 0;
    while (1)
    {
        drawPixel(x, y, (x + y)%4);
        x += 1;
        if (x == 8) {
            x = 0;
            y += 1;
        }
        writeDisplay();
        __delay_ms(300);
    }
}
/**
 End of File
*/