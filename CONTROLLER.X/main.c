#include "header.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "b4x_switch.h"

/*********************Definition of Ports********************************/

#define RS LATB0  /*PIN 0 of PORTB is assigned for register select Pin of LCD*/
#define EN LATB1  /*PIN 1 of PORTB is assigned for enable Pin of LCD */
#define ldata LATB  /*PORTB(PB4-PB7) is assigned for LCD Data Output*/ 
#define LCD_Port TRISB  /*define macros for PORTB Direction Register*/
#define F_CPU 8000000/64
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)

#define SW_PREV PORTDbits.RD0
#define SW_NEXT PORTDbits.RD1

#define SW_DECREMENT PORTDbits.RD3
#define SW_INCREMENT PORTDbits.RD2

/*********************Proto-Type Declaration*****************************/

void LCD_Init(); /*Initialize LCD*/
void LCD_Command(unsigned char); /*Send command to LCD*/
void LCD_Char(unsigned char x); /*Send data to LCD*/
void LCD_String(const char *); /*Display data string on LCD*/
void LCD_String_xy(char, char, const char *);
void LCD_Clear(); /*Clear LCD Screen*/
int ADC_Read(int channel);
void USART_Init(long);
char USART_RxChar();
void InitPWM();


/*************** PARAMS  *************/

// FOR RECEIVE DATA
volatile char* buffer = "";

typedef struct {
    char* title;
    float value;
    float max_increment;
    float min_increment;
    float max;
    float min;
    bool continuousModeActivated;
    bool increment;
} CONTROLLER;


char bufferLCD[10];
CONTROLLER controller[4];
bool continous = false;
float ER = 0.00;

int page = 0;
int previousPage = 0;

void __interrupt() ISR(void) {
    if (PIE1bits.RCIE && RCIF) {
        char receive = RCREG;

        if (receive != ';') {
            strcat(buffer, receive);
        } else {
            buffer = "";
        }
    }
}

int main(void) {
    // Use internal oscillator and set frequency to 8 MHz
    OSCCON = 0x72;

    // ADC settings
    TRISA = 0xFF; //PortA A0-A7 = Input
    ADCON1bits.VCFG = 0x0; // INTERNAL REFERENCE
    ADCON1bits.PCFG = 0b1110; // only A0
    ADCON2bits.ADFM = 1; // Right justified
    ADCON2bits.ADCS = 0b110; // FOSC/64
    ADCON2bits.ADCS = 0b001; // 8 TOSC
    ADRESH = 0; /* Flush ADC output Register */
    ADRESL = 0;


    InitPWM();
    //CCPR1L = 50;    /* load 25% duty cycle value */
    //CCPR2L = 100;	/* load 50% duty cycle value */

    // Initialize LCD to 5*8 matrix in 4-bit mode
    LCD_Init();

    TRISDbits.RD0 = 1; // PREV
    TRISDbits.RD1 = 1; // NEXT

    TRISDbits.RD2 = 1; // INCREMENT
    TRISDbits.RD3 = 1; // DECREMENT

    // Initialize USART
    USART_Init(9600);


    // Define PID
    // SP
    controller[0].title = "SP:";
    controller[0].value = 0;
    controller[0].max_increment = 5;
    controller[0].min_increment = 1;
    controller[0].max = 359;
    controller[0].min = 0;
    controller[0].continuousModeActivated = false;
    controller[0].increment = false;
    // KP
    controller[1].title = "KP:";
    controller[1].value = 0;
    controller[1].max_increment = 0.1;
    controller[1].min_increment = 0.01;
    controller[1].max = 10;
    controller[1].min = 0;
    controller[1].continuousModeActivated = false;
    controller[1].increment = false;
    // KI
    controller[2].title = "KI:";
    controller[2].value = 0;
    controller[2].max_increment = 0.1;
    controller[2].min_increment = 0.01;
    controller[2].max = 10;
    controller[2].min = 0;
    controller[2].continuousModeActivated = false;
    controller[2].increment = false;
    // KD
    controller[3].title = "KD:";
    controller[3].value = 0;
    controller[3].max_increment = 0.1;
    controller[3].min_increment = 0.01;
    controller[3].max = 10;
    controller[3].min = 0;
    controller[3].continuousModeActivated = false;
    controller[3].increment = false;


    while (1) {

        // PREV PAGE

        B4X_SW_nCLICK(SW_PREV) {
            B4X_SW_DEBOUNCE(SW_PREV);
            page = page == 0 ? 3 : page--;
        }

        // NEXT PAGE

        B4X_SW_nCLICK(SW_NEXT) {
            B4X_SW_DEBOUNCE(SW_NEXT);
            page = page == 3 ? 0 : page++;
        }


        if (page != previousPage) {
            LCD_Clear();
            previousPage = page;
        }

        // INCREMENT

        B4X_SW_nCLICK(SW_INCREMENT) {
            B4X_SW_DEBOUNCE(SW_INCREMENT);

            if (controller[page].value < controller[page].max) {
                controller[page].value += controller[page].min_increment;
            }

            // INCREMENT CONTINUOS

            B4X_SW_LONG_nCLICK(SW_INCREMENT) {
                B4X_SW_DEBOUNCE(SW_INCREMENT);
                controller[page].continuousModeActivated = true;
                controller[page].increment = true;
            }

        }

        if (SW_INCREMENT && controller[page].increment) {
            controller[page].continuousModeActivated = false;
        }


        // DECREMENT

        B4X_SW_nCLICK(SW_DECREMENT) {
            B4X_SW_DEBOUNCE(SW_DECREMENT);

            if (controller[page].value > controller[page].min) {
                controller[page].value -= controller[page].min_increment;
            }

            // DECREMENT CONTINOUS

            B4X_SW_LONG_nCLICK(SW_DECREMENT) {
                B4X_SW_DEBOUNCE(SW_DECREMENT);
                controller[page].continuousModeActivated = true;
                controller[page].increment = false;
            }

        }

        if (SW_DECREMENT && !controller[page].increment) {
            controller[page].continuousModeActivated = false;
        }

        if (controller[page].continuousModeActivated) {
            float sum;
            if (controller[page].increment) {
                if (controller[page].value < controller[page].max) {
                    sum = controller[page].value + controller[page].max_increment;

                    if (sum > controller[page].max) {
                        controller[page].value = controller[page].max;
                    } else {
                        controller[page].value = sum;
                    }

                }
            } else {
                if (controller[page].value > controller[page].min) {
                    sum = controller[page].value - controller[page].max_increment;
                    if (sum < 0) {
                        controller[page].value = controller[page].min;
                    } else {
                        controller[page].value = sum;
                    }

                }
            }

            __delay_ms(100);
        }

        LCD_String_xy(1, 0, controller[page].title);
        sprintf(bufferLCD, "%.3f", controller[page].value);
        LCD_String_xy(1, 3, bufferLCD);
    }
}

void InitPWM() {
    TRISC1 = 0; /* Set CCP2 pin as output for PWM out */
    TRISC2 = 0; /* Set CCP1 pin as output for PWM out */
    PR2 = 199; /* Load period value */

    /**** generate PWM on CCP1 ****/
    CCP1CON = 0x0C; /* Set PWM mode and no decimal for PWM */

    /**** generate PWM on CCP2 ****/
    CCP2CON = 0x0C; /* Set PWM mode and no decimal for PWM */

    /*configure Timer 2 for PWM*/
    T2CON = 0; /* No pre-scalar, timer2 is off */
    TMR2 = 0; /* Clear Timer2 initially */
    TMR2ON = 1; /* Timer ON for start counting*/
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
    __delay_ms(15); /*15ms,16x2 LCD Power on delay*/
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
    __delay_ms(1);
    ldata = (ldata & 0x0f) | (cmd << 4); /*Send lower nibble of command to PORT */
    EN = 1;
    NOP();
    EN = 0;
    __delay_ms(3);
}

void LCD_Char(unsigned char dat) {
    ldata = (ldata & 0x0f) | (0xF0 & dat); /*Send higher nibble of data first to PORT*/
    RS = 1; /*Data Register is selected*/
    EN = 1; /*High-to-low pulse on Enable pin to latch data*/
    NOP();
    EN = 0;
    __delay_ms(1);
    ldata = (ldata & 0x0f) | (dat << 4); /*Send lower nibble of data to PORT*/
    EN = 1; /*High-to-low pulse on Enable pin to latch data*/
    NOP();
    EN = 0;
    __delay_ms(3);
}

void LCD_String(const char *msg) {
    while ((*msg) != 0) {
        LCD_Char(*msg);
        msg++;
    }
}

void LCD_String_xy(char row, char pos, const char *msg) {
    char location = 0;

    switch (row) {
        case 1:
            location = (0x80) | ((pos) & 0x0f);
            break;

        case 2:
            location = (0xC0) | ((pos) & 0x0f);
            break;

        case 3:
            location = (0x94) | ((pos) & 0x0f);
            break;

        case 4:
            location = (0xD4) | ((pos) & 0x0f);
            break;
    }

    LCD_Command(location);
    LCD_String(msg);
}

void LCD_Clear() {
    LCD_Command(0x01); /*clear display screen*/
    __delay_ms(3);
}

void USART_Init(long baud_rate) {
    float temp;
    TRISC6 = 0; /* Make Tx pin as output*/
    TRISC7 = 1; /* Make Rx pin as input*/
    temp = Baud_value;
    SPBRG = (int) temp; /* Baud rate=9600 SPBRG=(F_CPU /(64*9600))-1*/
    //TXSTA = 0x20; /* TX enable; */
    //RCSTA = 0x90; /* RX enable and serial port enable*/

    TXSTAbits.SYNC = 0; //Setting Asynchronous Mode, ie UART
    RCSTAbits.SPEN = 1; //Enables Serial Port
    RCSTAbits.CREN = 1; //Enables Continuous Reception
    TXSTAbits.TXEN = 0; //Disable Transmission

    INTCONbits.GIE = 1; /* Enable Global Interrupt */
    INTCONbits.PEIE = 1; /* Enable Peripheral Interrupt */
    PIE1bits.RCIE = 1; /* Enable Receive Interrupt*/
    PIE1bits.TXIE = 0; /* Disable Transmit Interrupt (not necessary)*/
}