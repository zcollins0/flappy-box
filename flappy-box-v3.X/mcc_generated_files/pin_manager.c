/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.35
        Device            :  PIC16F1619
        Driver Version    :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/

#include <xc.h>
#include "pin_manager.h"
#include "stdbool.h"


void (*IOCCF4_InterruptHandler)(void);


void PIN_MANAGER_Initialize(void)
{
    /**
    LATx registers
    */   
    LATA = 0x00;    
    LATB = 0x00;    
    LATC = 0x00;    

    /**
    TRISx registers
    */    
    TRISA = 0x37;
    TRISB = 0xF0;
    TRISC = 0xFF;

    /**
    ANSELx registers
    */   
    ANSELC = 0xCF;
    ANSELB = 0xA0;
    ANSELA = 0x17;

    /**
    WPUx registers
    */ 
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x10;
    OPTION_REGbits.nWPUEN = 0;

    /**
    ODx registers
    */   
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;
    

    /**
    IOCx registers
    */
    // interrupt on change for group IOCCF - flag
    IOCCFbits.IOCCF4 = 0;
    // interrupt on change for group IOCCN - negative
    IOCCNbits.IOCCN4 = 1;
    // interrupt on change for group IOCCP - positive
    IOCCPbits.IOCCP4 = 1;

    // register default IOC callback functions at runtime; use these methods to register a custom function
    IOCCF4_SetInterruptHandler(IOCCF4_DefaultInterruptHandler);
   
    // Enable IOCI interrupt 
    INTCONbits.IOCIE = 1; 
    
    
    SSPCLKPPS = 0x0E;   //RB6->MSSP:SCL;
    SSPDATPPS = 0x0C;   //RB4->MSSP:SDA;
    RB6PPS = 0x10;   //RB6->MSSP:SCL;
    RB4PPS = 0x11;   //RB4->MSSP:SDA;

}       

void PIN_MANAGER_IOC(void)
{   
    // interrupt on change for pin IOCCF4
    if(IOCCFbits.IOCCF4 == 1)
    {
        IOCCF4_ISR();  
    }                          


}

/**
   IOCCF4 Interrupt Service Routine
*/
void IOCCF4_ISR(void) {

    // Add custom IOCCF4 code

    // Call the interrupt handler for the callback registered at runtime
    if(IOCCF4_InterruptHandler)
    {
        IOCCF4_InterruptHandler();
    }
    IOCCFbits.IOCCF4 = 0;
}

/**
  Allows selecting an interrupt handler for IOCCF4 at application runtime
*/
void IOCCF4_SetInterruptHandler(void (* InterruptHandler)(void)){
    IOCCF4_InterruptHandler = InterruptHandler;
}

/**
  Default interrupt handler for IOCCF4
*/
void IOCCF4_DefaultInterruptHandler(void){
    // add your IOCCF4 interrupt custom code
    // or set custom function using IOCCF4_SetInterruptHandler()
}

/**
 End of File
*/