#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define _XTAL_FREQ = 1000000;

/*********************Definition of Ports********************************/

#define RS LATB0  /*PIN 0 of PORTB is assigned for register select Pin of LCD*/
#define EN LATB1  /*PIN 1 of PORTB is assigned for enable Pin of LCD */
#define ldata LATB  /*PORTB(PB4-PB7) is assigned for LCD Data Output*/ 
#define LCD_Port TRISB  /*define macros for PORTB Direction Register*/


/*********************Proto-Type Declaration*****************************/

void MSdelay(unsigned int); /*Generate delay in ms*/
void LCD_Init(); /*Initialize LCD*/
void LCD_Command(unsigned char); /*Send command to LCD*/
void LCD_Char(unsigned char x); /*Send data to LCD*/
void LCD_String(const char *); /*Display data string on LCD*/
void LCD_String_xy(char, char, const char *);
void LCD_Clear(); /*Clear LCD Screen*/
int ADC_Read(int channel);


/*************** PID DEFINITION  *************/
char output[6];
char debounce = false;

float KP = 0.00;
float KD = 0.00;
float KI = 0.00;
float SP = 0.00;

int main(void) {
    // Use internal oscillator and set frequency to 8 MHz
    // OSCCON = 0x72;

    TRISA = 0xFF; //PortA A0-A7 = Input
    ADCON1bits.VCFG = 0x0; // INTERNAL REFERENCE
    ADCON1bits.PCFG = 0b1011; // only A0, A1, A2, A3
    ADCON2bits.ADFM = 1; // Right justified
    ADCON2bits.ADCS = 0b110; // FOSC/64
    ADCON2bits.ADCS = 0b001; // 8 TOSC

    ADRESH = 0; /* Flush ADC output Register */
    ADRESL = 0;

    // Initialize LCD to 5*8 matrix in 4-bit mode
    LCD_Init();

    while (1) {
        KP = (float) ADC_Read(0) * 10 / 1023;
        // KP = roundf(KP * 100) / 100;

        KD = (float) ADC_Read(1) * 10 / 1023;
        // KD = roundf(KD * 100) / 100;

        KI = (float) ADC_Read(2) * 10 / 1023;
        // KI = roundf(KI * 100) / 100;

        SP = (float) ADC_Read(3) * 359 / 1023;
        SP = (int) SP;

        /** 
         * Display PID constants
         */
        LCD_String_xy(1, 0, "Kp:");
        sprintf(output, "%.3f", KP);
        LCD_String_xy(1, 3, output);

        LCD_String_xy(1, 8, "Kd:");
        sprintf(output, "%.3f", KD);
        LCD_String_xy(1, 11, output);

        LCD_String_xy(2, 0, "Ki:");
        sprintf(output, "%.3f", KI);
        LCD_String_xy(2, 3, output);

        LCD_String_xy(2, 8, "SP:");
        sprintf(output, "%.3f", SP);
        LCD_String_xy(2, 11, output);
    }
}

int ADC_Read(int channel) {
    int digital;

    /* Channel 0 is selected i.e.(CHS3CHS2CHS1CHS0=0000) & ADC is disabled */
    ADCON0 = (ADCON0 & 0b11000011) | ((channel << 2) & 0b00111100);

    ADCON0 |= ((1 << ADON) | (1 << GO)); /*Enable ADC and start conversion*/

    /* Wait for End of conversion i.e. Go/done'=0 conversion completed */
    while (ADCON0bits.GO_nDONE == 1);

    digital = (ADRESH * 256) | (ADRESL); /*Combine 8-bit LSB and 2-bit MSB*/
    return (digital);
}

/****************************Functions********************************/

void LCD_Init() {
    LCD_Port = 0; /*PORT as Output Port*/
    MSdelay(15); /*15ms,16x2 LCD Power on delay*/
    LCD_Command(0x02); /*send for initialization of LCD 
                          for nibble (4-bit) mode */
    LCD_Command(0x28); /*use 2 line and 
                          initialize 5*8 matrix in (4-bit mode)*/
    LCD_Command(0x01); /*clear display screen*/
    LCD_Command(0x0c); /*display on cursor off*/
    LCD_Command(0x06); /*increment cursor (shift cursor to right)*/
}

void LCD_Command(unsigned char cmd) {
    ldata = (ldata & 0x0f) | (0xF0 & cmd); /*Send higher nibble of command first to PORT*/
    RS = 0; /*Command Register is selected i.e.RS=0*/
    EN = 1; /*High-to-low pulse on Enable pin to latch data*/
    NOP();
    EN = 0;
    MSdelay(1);
    ldata = (ldata & 0x0f) | (cmd << 4); /*Send lower nibble of command to PORT */
    EN = 1;
    NOP();
    EN = 0;
    MSdelay(3);
}

void LCD_Char(unsigned char dat) {
    ldata = (ldata & 0x0f) | (0xF0 & dat); /*Send higher nibble of data first to PORT*/
    RS = 1; /*Data Register is selected*/
    EN = 1; /*High-to-low pulse on Enable pin to latch data*/
    NOP();
    EN = 0;
    MSdelay(1);
    ldata = (ldata & 0x0f) | (dat << 4); /*Send lower nibble of data to PORT*/
    EN = 1; /*High-to-low pulse on Enable pin to latch data*/
    NOP();
    EN = 0;
    MSdelay(3);
}

void LCD_String(const char *msg) {
    while ((*msg) != 0) {
        LCD_Char(*msg);
        msg++;
    }
}

void LCD_String_xy(char row, char pos, const char *msg) {
    char location = 0;
    if (row <= 1) {
        location = (0x80) | ((pos) & 0x0f); /*Print message on 1st row and desired location*/
        LCD_Command(location);
    } else {
        location = (0xC0) | ((pos) & 0x0f); /*Print message on 2nd row and desired location*/
        LCD_Command(location);
    }

    LCD_String(msg);
}

void LCD_Clear() {
    LCD_Command(0x01); /*clear display screen*/
    MSdelay(3);
}

void MSdelay(unsigned int val) {
    unsigned int i, j;
    for (i = 0; i < val; i++)
        for (j = 0; j < 165; j++); /*This count Provide delay of 1 ms for 8MHz Frequency */
}