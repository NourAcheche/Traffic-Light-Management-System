#include <xc.h>          // compiler XC8

#pragma config FOSC = HS     // high speed crystal oscillator((20MHZ)
#pragma config WDTE = OFF    // watchdog timer disabled
#pragma config PWRTE = OFF   // power up timer disabled
#pragma config BOREN = ON    // brown out reset enabled
#pragma config LVP = OFF     // low voltage programming disabled
#pragma config CPD = OFF     // data memory code protection off(CPD,WRT,CP)
#pragma config WRT = OFF     
#pragma config CP = OFF      

#define _XTAL_FREQ 20000000  // frequency pic 20MHZ


// 7 segment configuration
unsigned char digit[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,
    0x92, 0x82, 0xF8, 0x80, 0x90
};


// function to display two digits using multiplexing
void show(unsigned char left, unsigned char right)
{
    PORTE = 0x00;            
    PORTB = digit[left];   
    PORTE = 0x02;           // enable left display (RE1)
    __delay_ms(5);        

    PORTE = 0x00;            
    PORTB = digit[right];   
    PORTE = 0x04;           // enable right display (RE2)
    __delay_ms(5);          
}


// function to count down seconds on display
void countdownseconds(unsigned char sec)
{
    signed char t;           
    unsigned char left, right, r;

    for(t = sec; t > 0; t--)    // loop from sec down to 1
    {
        left  = t / 10;         // calculate tens digit
        right = t % 10;         // calculate ones digit

        for(r = 0; r < 100; r++) // refresh display ~1 second
            show(left, right);
    }
}


// Red light function
void redon(void)
{
    PORTA = 0;      // clear PORTA
    RA0 = 1;        // RED car on
    RA5 = 1;        // GREEN ped on
    countdownseconds(15); 
}


// Orange light function
void orangeon(void)
{
    PORTA = 0;      
    RA1 = 1;        // ORANGE car on
    RA3 = 1;        // RED ped on
    countdownseconds(5);  
}


// Green light function
unsigned char greenon(void)
{
    signed char t;
    unsigned char left, right, r;

    PORTA = 0;     
    RA2 = 1;        // GREEN car on
    RA3 = 1;        // RED ped on

    for(t = 15; t > 0; t--)   // green light countdown
    {
        left  = t / 10;       // tens digit
        right = t % 10;       // ones digit

        for(r = 0; r < 100; r++)
        {
            show(left, right);    // display time

            if(RA4 == 0)          // check push button
            {
                __delay_ms(50);   // debounce delay
                if(RA4 == 0)
                    return 1;     // button pressed
            }
        }
    }
    return 0;   // no button press
}


// main function
void main(void)
{
    ADCON1 = 0x06;     // set all PORTA pins as digital

    TRISA = 0b00010000; // RA4 input
    TRISB = 0x00;       // PORTB as output
    TRISE = 0x00;       // PORTE as output

    PORTA = 0;          // clear PORTA
    PORTB = 0;          // clear PORTB
    PORTE = 0;          // clear PORTE

    while(1)            
    {
        redon();        
        orangeon();     

        if(greenon())   
        {
            orangeon();     
            while(RA4 == 0); // wait until button released
            continue;       
        }

        orangeon();    
    }
}