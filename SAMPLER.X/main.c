#include <xc.h>
#include <stdio.h>

#define F_CPU 8000000/64
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)

void USART_Init(long);
void USART_TxChar(char);
void USART_TxText(char*);

char output[3];
volatile int pulses = 0;

void __interrupt() ISR(void) {
    if (INTCONbits.INT0IE && INTCONbits.INT0IF) {
        if ((!INTCON2bits.INTEDG0 && PORTBbits.RB1) || (INTCON2bits.INTEDG0 && !PORTBbits.RB1)) {
            pulses = (pulses == 360) ? 1 : pulses++;
        } else {
            pulses = (pulses == 0) ? 359 : pulses--;
        }

        sprintf(output, "%d", pulses);
        USART_TxText(output);

        INTCON2bits.INTEDG0 ^= 1;
        INTCONbits.INT0IF = 0;
    }

    if (INTCON3bits.INT1IE && INTCON3bits.INT1IF) {
        if ((!INTCON2bits.INTEDG1 && PORTBbits.RB0) || (INTCON2bits.INTEDG1 && !PORTBbits.RB0)) {
            pulses = (pulses == 0) ? 359 : pulses--;
        } else {
            pulses = (pulses == 360) ? 1 : pulses++;
        }

        sprintf(output, "%d", pulses);
        USART_TxText(output);

        INTCON2bits.INTEDG1 ^= 1;
        INTCON3bits.INT1IF = 0;
    }
    return;
}

void main() {
    OSCCON = 0x72; // Internal oscillator 8MHZ

    USART_Init(9600); // Init USART 9600 bps

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

    while (1);
}

void USART_Init(long baud_rate) {
    float temp;
    TRISC6 = 0; /*Make Tx pin as output*/
    TRISC7 = 1; /*Make Rx pin as input*/
    temp = Baud_value;
    SPBRG = (int) temp; /*baud rate=9600, SPBRG = (F_CPU /(64*9600))-1*/
    TXSTA = 0x20; /*Transmit Enable(TX) enable*/
    RCSTA = 0x90; /*Receive Enable(RX) enable and serial port enable */
}

/******************TRANSMIT FUNCTION*****************************************/
void USART_TxChar(char out) {
    while (TXIF == 0); /*wait for transmit interrupt flag*/
    TXREG = out; /*transmit data via TXREG register*/
}

void USART_TxText(char *text) {
    int i;
    for (i = 0; text[i] != '\0'; i++) {
        USART_TxChar(text[i]);
    }
}