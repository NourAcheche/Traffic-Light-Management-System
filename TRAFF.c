#include <xc.h>
#include <stdint.h>

#pragma config FOSC = HS, WDTE = OFF, PWRTE = ON, BOREN = ON, LVP = OFF
#pragma config CPD = OFF, WRT = OFF, CP = OFF

#define _XTAL_FREQ 20000000

#include "I2C_LCD.h"

// ================== HC-SR04 PINS ==================
#define TRIG_TRIS   TRISDbits.TRISD0
#define TRIG_LAT    PORTDbits.RD0

#define ECHO_TRIS   TRISDbits.TRISD1
#define ECHO_PORT   PORTDbits.RD1

// ================== SETTINGS ==================
#define LCD_ADDR    0x4E
#define LIMIT_CM    20

// ================== BUTTON (RA4, active-low) ==================
volatile unsigned char buttonPressed = 0;
static volatile unsigned char db_cnt = 0;
static volatile unsigned char latched = 0;

// ================== 7-SEG CODES (common anode) ==================
unsigned char digit[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99,
    0x92, 0x82, 0xF8, 0x80, 0x90
};

// Digits currently shown (updated by main, displayed by ISR)
volatile uint8_t seg_left = 0;
volatile uint8_t seg_right = 0;

// ================== TIME BASE FROM TIMER0 ==================
// Timer0 overflow ~1.638ms with prescaler 1:32 at 20MHz.
// We'll accumulate microseconds to build a ms counter.
volatile uint32_t g_ms = 0;
static volatile uint16_t us_acc = 0;

// ================== TIMER0 INIT ==================
void timer0_init(void)
{
    OPTION_REGbits.T0CS = 0;   // internal clock (Fosc/4)
    OPTION_REGbits.PSA  = 0;   // prescaler -> TMR0
    OPTION_REGbits.PS2  = 1;   // 1:32
    OPTION_REGbits.PS1  = 0;
    OPTION_REGbits.PS0  = 0;

    TMR0 = 0;
    T0IF = 0;
    T0IE = 1;
}

// ================== TIMER1 FOR HC-SR04 ==================
static void Timer1_Init(void)
{
    T1CON = 0x00;          // prescaler 1:1, internal clock, Timer1 OFF
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;
}

// ================== LCD HELPERS ==================
static void LCD_ShowWarning(void)
{
    LCD_Set_Cursor(1, 1);
    LCD_Write_String(" warning        ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String(" limit passed   ");
}

static void LCD_ShowGreatJob(void)
{
    LCD_Set_Cursor(1, 1);
    LCD_Write_String(" Great job      ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("                ");
}

static void LCD_Clear2Lines(void)
{
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("                ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("                ");
}

// ================== HC-SR04 MEASUREMENT ==================
static uint16_t HCSR04_ReadEcho_us(void)
{
    uint32_t timeout;

    // 10us trigger
    TRIG_LAT = 0;
    __delay_us(2);
    TRIG_LAT = 1;
    __delay_us(10);
    TRIG_LAT = 0;

    // Wait echo high (timeout ~30ms)
    timeout = 0;
    while (!ECHO_PORT)
    {
        __delay_us(1);
        if (++timeout > 30000) return 0;
    }

    // Start Timer1
    TMR1H = 0; TMR1L = 0;
    PIR1bits.TMR1IF = 0;
    T1CONbits.TMR1ON = 1;

    // Wait echo low or overflow
    while (ECHO_PORT)
    {
        if (PIR1bits.TMR1IF)
        {
            T1CONbits.TMR1ON = 0;
            return 0;
        }
    }

    T1CONbits.TMR1ON = 0;
    uint16_t ticks = ((uint16_t)TMR1H << 8) | TMR1L;

    // 20MHz => Fosc/4=5MHz => 0.2us/tick => us = ticks/5
    return (uint16_t)(ticks / 5);
}

static uint16_t HCSR04_ReadDistance_cm(void)
{
    uint16_t echo_us = HCSR04_ReadEcho_us();
    if (echo_us == 0) return 0;
    return (uint16_t)(echo_us / 58);
}

// ================== TRAFFIC LIGHT OUTPUTS ==================
static void set_red(void)
{
    PORTA = 0;
    RA0 = 1;

    // NOTE: PIC16F877A RA5 is INPUT ONLY -> LED on RA5 will NOT work.
    // If your chip is not 16F877A or your RA5 is actually output-capable, keep it.
    RA5 = 1;
}

static void set_orange(void)
{
    PORTA = 0;
    RA1 = 1;
    RA3 = 1;
}

static void set_green(void)
{
    PORTA = 0;
    RA2 = 1;
    RA3 = 1;
}

// ================== TRAFFIC STATE MACHINE ==================
typedef enum {
    TL_RED,
    TL_ORANGE1,  // after red
    TL_GREEN,
    TL_ORANGE2   // after green
} TL_State;

static TL_State tl = TL_RED;
static uint8_t  tl_seconds = 15;

#define RED_TIME     15
#define ORANGE_TIME   5
#define GREEN_TIME   15

static void traffic_enter(TL_State s)
{
    tl = s;

    switch (s)
    {
        case TL_RED:     tl_seconds = RED_TIME;    set_red();    break;
        case TL_ORANGE1: tl_seconds = ORANGE_TIME; set_orange(); break;
        case TL_GREEN:   tl_seconds = GREEN_TIME;  set_green();  break;
        case TL_ORANGE2: tl_seconds = ORANGE_TIME; set_orange(); break;
    }
}

static void traffic_tick_1s(void)
{
    // Button behavior: ONLY during green -> force to ORANGE2 (then red)
    if (tl == TL_GREEN && buttonPressed)
    {
        buttonPressed = 0;
        traffic_enter(TL_ORANGE2);
        return;
    }

    if (tl_seconds > 0) tl_seconds--;

    if (tl_seconds == 0)
    {
        switch (tl)
        {
            case TL_RED:     traffic_enter(TL_ORANGE1); break;
            case TL_ORANGE1: traffic_enter(TL_GREEN);   break;
            case TL_GREEN:   traffic_enter(TL_ORANGE2); break;
            case TL_ORANGE2: traffic_enter(TL_RED);     break;
        }
    }
}

// ================== 7-SEG DISPLAY UPDATE ==================
static void seg_set_number(uint8_t value)
{
    if (value > 99) value = 99;
    seg_left  = value / 10;
    seg_right = value % 10;
}

// ================== ISR ==================
void __interrupt() isr(void)
{
    if (T0IF)
    {
        T0IF = 0;

        // ---- Debounce RA4 (active-low) ----
        if (RA4 == 0)
        {
            if (!latched)
            {
                if (db_cnt < 20) db_cnt++;
                if (db_cnt >= 20)
                {
                    buttonPressed = 1;
                    latched = 1;
                }
            }
        }
        else
        {
            db_cnt = 0;
            latched = 0;
        }

        // ---- Build millisecond counter from ~1.638ms ticks ----
        // 1 overflow ≈ 1638us
        us_acc += 1638;
        while (us_acc >= 1000)
        {
            us_acc -= 1000;
            g_ms++;
        }

        // ---- 7-seg multiplex (NO delays, runs always) ----
        // Refresh about every ~5ms: 3 overflows * 1.638ms ≈ 4.9ms
        static uint8_t mux_div = 0;
        static uint8_t which = 0;

        mux_div++;
        if (mux_div >= 3)
        {
            mux_div = 0;

            PORTE = 0x00; // disable both

            if (which == 0)
            {
                PORTB = digit[seg_left];
                PORTE = 0x02; // RE1 left
                which = 1;
            }
            else
            {
                PORTB = digit[seg_right];
                PORTE = 0x04; // RE2 right
                which = 0;
            }
        }
    }
}

// ================== MAIN ==================
void main(void)
{
    ADCON1 = 0x07; // make RA/RE digital on PIC16F877A-like parts

    // RA4 input, others output
    TRISA = 0b00010000;

    // 7-seg segments and enables
    TRISB = 0x00;
    TRISE = 0x00;

    // Ultrasonic pins
    TRIG_TRIS = 0;
    ECHO_TRIS = 1;
    TRIG_LAT = 0;

    // I2C pins (safe set as inputs)
    TRISC3 = 1;
    TRISC4 = 1;

    PORTA = 0;
    PORTB = 0;
    PORTE = 0;

    timer0_init();
    Timer1_Init();

    I2C_Master_Init();
    LCD_Init(LCD_ADDR);

    LCD_Set_Cursor(1, 1);
    LCD_Write_String(" Traffic       ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String(" Regulatotions ");
    __delay_ms(800);
    LCD_Clear2Lines();

    GIE = 1;
    PEIE = 1;

    // Start traffic
    traffic_enter(TL_RED);

    // Start display with initial seconds
    seg_set_number(tl_seconds);

    // LCD/ultra scheduling
    uint32_t last1s = g_ms;
    uint32_t lastUltra = g_ms;
    uint32_t lastFlash = g_ms;
    uint8_t  flash_state = 0;

    while (1)
    {
        uint32_t now = g_ms;

        // ----- 1-second traffic tick -----
        if ((now - last1s) >= 1000)
        {
            last1s += 1000;

            traffic_tick_1s();
            seg_set_number(tl_seconds);
        }

        // ----- Ultrasonic read every ~200ms -----
        if ((now - lastUltra) >= 200)
        {
            lastUltra = now;

            uint16_t d = HCSR04_ReadDistance_cm();

            if (d > 0 && d <= LIMIT_CM)
            {
                // flash message every 250ms
                if ((now - lastFlash) >= 250)
                {
                    lastFlash = now;
                    flash_state ^= 1;

                    if (flash_state) LCD_ShowWarning();
                    else            LCD_Clear2Lines();
                }
            }
            else
            {
                flash_state = 0;
                lastFlash = now;
                LCD_ShowGreatJob();
            }
        }

        // nothing blocking here => everything runs smoothly
    }
}
