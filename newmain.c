#include <xc.h>   // include header file for pic microcontroller

#define _XTAL_FREQ 20000000   // define crystal frequency as 20mhz for delay functions

#pragma config FOSC = HS      // use high speed oscillator
#pragma config WDTE = OFF     // watchdog timer disabled
#pragma config PWRTE = ON     // power-up timer enabled
#pragma config LVP = OFF      // low voltage programming disabled
#pragma config BOREN = OFF    // brown-out reset disabled

// 7 segment configuration
unsigned char digit[10] = {
    0xc0, 0xf9, 0xa4, 0xb0, 0x99,
    0x92, 0x82, 0xf8, 0x80, 0x90
};

unsigned char countc = 0;    // counter for sensor connected to rc7
unsigned char countd = 0;    // counter for sensor connected to rd7

void main(void) {
    ADCON1 = 0x06;           

    TRISD = 0x80;             // rd7 as input (sensor)
    TRISC = 0x80;             // rc7 as input(sensor)

    PORTD = digit[0];        
    PORTC = digit[0];         

    while (1) {
        
        if (RD7 == 1) {
            __delay_ms(30);

            countd++;         // increment count for rd7 sensor
            if (countd > 9) countd = 0;   // reset after reaching 9

            PORTD = digit[countd];   // update 7-segment 

            while (RD7 == 1); // wait until sensor signal goes low
            __delay_ms(30);   
        }

        // check if sensor connected to rc7 is activated
        if (RC7 == 1) {
            __delay_ms(30);   

            countc++;        
            if (countc > 9) countc = 0;

            PORTC = digit[countc];   // update 7-segment 

            while (RC7 == 1); // wait until sensor signal goes low
            __delay_ms(30);   
        }
    }
}