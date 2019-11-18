#include "header.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define F_CPU _XTAL_FREQ/64
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)
#define ENCODER_RESOLUTION 910

void USART_Init(long);
unsigned char USART_RxChar();
void USART_TxChar(unsigned char);
void USART_TxText(char*);

char output[10];
volatile int pulses = 0;
volatile bool connectionEstablished = false;

void __interrupt() ISR(void) {
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        //sprintf(output, "%d", pulses);
        //strcat(output, ";");
        //USART_TxText(output);
        int dataToSend = pulses;
        USART_TxChar(dataToSend & 0xFF);
        USART_TxChar((dataToSend >> 8) & 0xFF);
        //PORTCbits.RC0 ^= 1;

        TMR0 = 0;
        INTCONbits.TMR0IF = 0;
    }

    if (PIE1bits.RCIE && RCIF) {
        unsigned char recieve = USART_RxChar();

        if (recieve == 98) {
            // init transmission
            connectionEstablished = true;
            INTCONbits.TMR0IE = 1;

        }
    }


    if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        if ((!INTCON2bits.INTEDG0 && PORTBbits.RB1) || (INTCON2bits.INTEDG0 && !PORTBbits.RB1)) {
            pulses = (pulses == ENCODER_RESOLUTION) ? 1 : pulses++;
        } else {
            pulses = (pulses == 0) ? ENCODER_RESOLUTION : pulses--;
        }

        INTCON2bits.INTEDG0 ^= 1;
        INTCONbits.INT0IF = 0;
    }

    if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) {
        if ((!INTCON2bits.INTEDG1 && PORTBbits.RB0) || (INTCON2bits.INTEDG1 && !PORTBbits.RB0)) {
            pulses = (pulses == 0) ? ENCODER_RESOLUTION : pulses--;
        } else {
            pulses = (pulses == ENCODER_RESOLUTION) ? 1 : pulses++;
        }

        INTCON2bits.INTEDG1 ^= 1;
        INTCON3bits.INT1IF = 0;
    }
    return;
}

void main() {
    OSCCON = 0x72; // Internal oscillator 8MHZ

    USART_Init(9600); // Init USART 9600 bps

    /**
     * FOR EXTERNAL INTERRUPR
     */
    TRISBbits.TRISB0 = 1; // Output
    TRISBbits.TRISB1 = 1; // Output

    ADCON0bits.ADON = 0; // disable ADC
    ADCON1bits.PCFG = 0b0101; //ADC 12 11 10 DIGITAL **FOR SOLVE BUG**

    INTCONbits.GIE = 1; // Global interruption

    INTCONbits.INT0IE = 1; // Enable external interruption
    INTCONbits.INT0IF = 0; // reset flag

    INTCON3bits.INT1IE = 1; // Enable external interruption
    INTCON3bits.INT1IF = 0; // reset flag

    INTCON2bits.INTEDG0 = 1; // Rasing edge
    INTCON2bits.INTEDG1 = 1; // Rasing edge

    /**
     *  FOR TIMER 0
     *  Timer0 Registers Prescaler = 256 - TMR0 Preset = 0 - Freq = 30.52 Hz - Period = 0.032768 seconds
     */
    INTCONbits.PEIE = 1;
    INTCONbits.TMR0IE = 0; // Disable timer 0
    INTCONbits.TMR0IF = 0; // Reset flag timer 0
    INTCON2bits.TMR0IP = 1; //TMR0 Overflow Interrupt Priority bit (High priority)     


    T0CONbits.TMR0ON = 1; // Set on timer 0
    T0CONbits.T08BIT = 1; // Set 8 Bits
    T0CONbits.T0CS = 0; // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    T0CONbits.T0SE = 0; // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    T0CONbits.PSA = 0; // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    T0CONbits.T0PS = 0b111; // 1:256 Prescaler
    TMR0 = 0; // preset for timer register 15hz (sample time)


    //TRISCbits.RC0 = 0;

    while (1) {
        if (!connectionEstablished) {
            USART_TxChar('a');
            __delay_ms(100);
        }
    };
}

/*
void USART_Init(long baud_rate) {
    float temp;
    TRISC6 = 0; // Make Tx pin as output
    TRISC7 = 1; // Make Rx pin as input
    temp = Baud_value;
    SPBRG = (int) temp; // baud rate=9600, SPBRG = (F_CPU /(64*9600))-1
    TXSTA = 0x20; // Transmit Enable(TX) enable
    RCSTA = 0x90; // Receive Enable(RX) enable and serial port enable 
}
 */

void USART_Init(long baud_rate) {
    float temp;
    TRISC6 = 0; /* Make Tx pin as output*/
    TRISC7 = 1; /* Make Rx pin as input*/
    temp = Baud_value;
    SPBRG = (int) temp; /* Baud rate=9600 SPBRG=(F_CPU /(64*9600))-1*/
    TXSTA = 0x20; /* TX enable; */
    RCSTA = 0x90; /* RX enable and serial port enable*/
    INTCONbits.GIE = 1; /* Enable Global Interrupt */
    INTCONbits.PEIE = 1; /* Enable Peripheral Interrupt */
    PIE1bits.RCIE = 1; /* Enable Receive Interrupt*/
    PIE1bits.TXIE = 0; /* Enable Transmit Interrupt*/
}

unsigned char USART_RxChar() {
    while (!RCIF); //Wait till RCREG is full
    return RCREG; //Return value in received data
}

/******************TRANSMIT FUNCTION*****************************************/
void USART_TxChar(unsigned char out) {
    TXREG = out; /*transmit data via TXREG register*/
    while (!TRMT); //Wait till transmission is complete
}

void USART_TxText(char *text) {
    for (int i = 0; text[i] != '\0'; i++) {
        USART_TxChar(text[i]);
    }
}