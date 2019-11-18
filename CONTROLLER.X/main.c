#include "header.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "b4x_switch.h"


/*********************Definition of Ports********************************/

#define RS LATB0  /*PIN 0 of PORTB is assigned for register select Pin of LCD*/
#define EN LATB1  /*PIN 1 of PORTB is assigned for enable Pin of LCD */
#define ldata LATB  /*PORTB(PB4-PB7) is assigned for LCD Data Output*/ 
#define LCD_Port TRISB  /*define macros for PORTB Direction Register*/
#define F_CPU _XTAL_FREQ/64
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)

#define SW_PREV PORTDbits.RD0
#define SW_NEXT PORTDbits.RD1

#define SW_DECREMENT PORTDbits.RD3
#define SW_INCREMENT PORTDbits.RD2

#define ENCODER_RESOLUTION 910

/*********************Proto-Type Declaration*****************************/

void LCD_Init(); /*Initialize LCD*/
void LCD_Command(unsigned char); /*Send command to LCD*/
void LCD_Char(unsigned char x); /*Send data to LCD*/
void LCD_String(const char *); /*Display data string on LCD*/
void LCD_String_xy(char, char, const char *);
void LCD_Clear(); /*Clear LCD Screen*/
void USART_Init(long);
unsigned char USART_RxChar();
void USART_TxChar(unsigned char);
void InitPWM();
void actions(int page);

float PID(float error);
void setPID(bool value);

/*************** PARAMS  *************/

// FOR RECEIVE DATA USART
volatile bool connectionEstablished = false;
volatile bool toggle = false;
volatile unsigned char buffer;


/**
 * PID
 */
volatile int encoderValue = 0;
volatile bool enablePID = false;
float SP = 50.00;
float KP = 0.80;
float KI = 0.00;
float KD = 0.00;
float ER = 0.00;

float integrate = 0.00;
float lastError = 0.00;
float dt = 0.06;
int minCCPR1L = 115;
int inertial = 19;
int inertialReverse = 19;
char direction = 'd';

bool stoppedMotor = false;

// STRUCT OF CONTROLLER (FOR PID AND MANAGER PAGE)
#define numberOfPages 10

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
CONTROLLER controller[numberOfPages];


// BUFFER LCD
char bufferLCD[16];
char bufferFloat[10];
// MANAGER CONTINOUS MODE IN BUTTON
bool continous = false;

// MANAGER PAGE
int page = 0;
int previousPage = 0;

void __interrupt() ISR(void) {
    if (PIE1bits.RCIE && RCIF) {
        unsigned char receive = USART_RxChar();

        if (!connectionEstablished) {
            if (receive == 97) {
                connectionEstablished = true;
                USART_TxChar('b');
            }
        } else {
            if (!toggle) {
                buffer = receive;
                toggle = true;
            } else {
                int encoderValueReceive = (receive << 8) + (buffer << 0);

                if (encoderValueReceive > encoderValue) {
                    direction = 'd';
                } else {
                    direction = 'i';
                }

                if (encoderValueReceive == encoderValue) {
                    stoppedMotor = true;
                } else {
                    stoppedMotor = false;
                }

                encoderValue = encoderValueReceive;

                toggle = false;

                if (enablePID) {
                    float error = (float) SP * ENCODER_RESOLUTION / 360 - encoderValue;
                    /*
                    if (abs(error) <= ENCODER_RESOLUTION / 360) {
                        error = 0;
                    }
                     */
                    CCPR1L = (int) PID(error);
                }
            }
        }
    }
}

void setPID(bool value) {
    enablePID = value;
    CCPR1L = 0;
    integrate = 0.00;
    lastError = 0.00;
    stoppedMotor = false;
}

float PID(float error) {
    integrate = integrate + KI * error * dt;
    float derivate = KD * (error - lastError) / dt;
    lastError = error;

    float action = (KP * error) + integrate + derivate;

    char directionOfController = '\0';

    if (action < 0) {
        // anticlockwise
        PORTCbits.RC0 = 1;
        PORTCbits.RC1 = 0;
        directionOfController = 'i';
    } else {
        // clockwise
        PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 1;
        directionOfController = 'd';
    }

    action = (200 - minCCPR1L) * abs(action) / ENCODER_RESOLUTION + minCCPR1L;

    if (stoppedMotor) {
        action += inertial;
    } else if (directionOfController != direction) {
        action += inertialReverse;
    }

    return action;
}

int main(void) {

    /**
     * MOTOR CONTROL
     */
    InitPWM();
    TRISCbits.RC0 = 0;
    TRISCbits.RC1 = 0;

    // Initialize LCD to 5*8 matrix in 4-bit mode
    LCD_Init();

    TRISDbits.RD0 = 1; // PREV
    TRISDbits.RD1 = 1; // NEXT

    TRISDbits.RD2 = 1; // INCREMENT
    TRISDbits.RD3 = 1; // DECREMENT

    // Define PID
    // SP
    controller[0].title = "SP:";
    controller[0].value = SP;
    controller[0].max_increment = 5;
    controller[0].min_increment = 1;
    controller[0].max = 359;
    controller[0].min = 0;
    controller[0].continuousModeActivated = false;
    controller[0].increment = false;
    // KP
    controller[1].title = "KP:";
    controller[1].value = KP;
    controller[1].max_increment = 0.1;
    controller[1].min_increment = 0.1;
    controller[1].max = 10;
    controller[1].min = 0;
    controller[1].continuousModeActivated = false;
    controller[1].increment = false;
    // KI
    controller[2].title = "KI:";
    controller[2].value = KI;
    controller[2].max_increment = 0.1;
    controller[2].min_increment = 0.1;
    controller[2].max = 10;
    controller[2].min = 0;
    controller[2].continuousModeActivated = false;
    controller[2].increment = false;
    // KD
    controller[3].title = "KD:";
    controller[3].value = KD;
    controller[3].max_increment = 0.1;
    controller[3].min_increment = 0.1;
    controller[3].max = 10;
    controller[3].min = 0;
    controller[3].continuousModeActivated = false;
    controller[3].increment = false;
    // DT
    controller[4].title = "CCPR1L:";
    controller[4].value = CCPR1L;
    controller[4].max_increment = 5;
    controller[4].min_increment = 1;
    controller[4].max = 200;
    controller[4].min = 0;
    controller[4].continuousModeActivated = false;
    controller[4].increment = false;
    // DT
    controller[5].title = "Direction:";
    controller[5].value = 0;
    controller[5].max_increment = 1;
    controller[5].min_increment = 1;
    controller[5].max = 1;
    controller[5].min = 0;
    controller[5].continuousModeActivated = false;
    controller[5].increment = false;
    // PID ENABLE
    controller[6].title = "Enable PID:";
    controller[6].value = 0;
    controller[6].max_increment = 1;
    controller[6].min_increment = 1;
    controller[6].max = 1;
    controller[6].min = 0;
    controller[6].continuousModeActivated = false;
    controller[6].increment = false;
    // Minimum CCPR1L
    controller[7].title = "CCPR1Lmin:";
    controller[7].value = minCCPR1L;
    controller[7].max_increment = 1;
    controller[7].min_increment = 1;
    controller[7].max = 200;
    controller[7].min = 0;
    controller[7].continuousModeActivated = false;
    controller[7].increment = false;
    // GAIN Inertial
    controller[8].title = "Inertial:";
    controller[8].value = inertial;
    controller[8].max_increment = 1;
    controller[8].min_increment = 1;
    controller[8].max = 200;
    controller[8].min = 0;
    controller[8].continuousModeActivated = false;
    controller[8].increment = false;
    // GAIN Inertial reverse 
    controller[9].title = "Inertial rev:";
    controller[9].value = inertialReverse;
    controller[9].max_increment = 1;
    controller[9].min_increment = 1;
    controller[9].max = 200;
    controller[9].min = 0;
    controller[9].continuousModeActivated = false;
    controller[9].increment = false;


    // Initialize USART
    USART_Init(9600);

    while (1) {

        // PREV PAGE

        B4X_SW_nCLICK(SW_PREV) {
            B4X_SW_DEBOUNCE(SW_PREV);
            page = page == 0 ? (numberOfPages - 1) : page--;
        }

        // NEXT PAGE

        B4X_SW_nCLICK(SW_NEXT) {
            B4X_SW_DEBOUNCE(SW_NEXT);
            page = page == (numberOfPages - 1) ? 0 : page++;
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

                actions(page);
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

                actions(page);
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

        strcpy(bufferLCD, "");
        strcat(bufferLCD, controller[page].title);
        sprintf(bufferFloat, "%.3f", controller[page].value);
        strcat(bufferLCD, bufferFloat);
        LCD_String_xy(1, 0, bufferLCD);

        sprintf(bufferLCD, "%.1f", (float) encoderValue);
        LCD_String_xy(2, 0, bufferLCD);

        sprintf(bufferLCD, "%.1f", ((float) encoderValue * 360 / ENCODER_RESOLUTION));
        LCD_String_xy(2, 6, bufferLCD);

    }
}

void actions(int page) {
    switch (page) {
        case 0:
            SP = controller[0].value;
            break;
        case 1:
            KP = controller[1].value;
            break;
        case 2:
            KI = controller[2].value;
            break;
        case 3:
            KD = controller[3].value;
            break;
        case 4:
            CCPR1L = controller[4].value;
            break;
        case 5:
        {
            if (controller[5].value == 1) {
                PORTCbits.RC0 = 1;
                PORTCbits.RC1 = 0;
            } else {
                PORTCbits.RC0 = 0;
                PORTCbits.RC1 = 1;
            }
        }
            break;
        case 6:
            setPID(controller[6].value);
            break;
        case 7:
            minCCPR1L = controller[7].value;
            break;
        case 8:
            inertial = controller[8].value;
            break;
        case 9:
            inertialReverse = controller[8].value;
            break;
    };


}

void InitPWM() {
    TRISCbits.TRISC2 = 0; /* Set CCP1 pin as output for PWM out */

    // x=(16000000/(5000*4*4) - 1)
    PR2 = 199; /* Load period value */

    /**** generate PWM on CCP1 ****/
    CCP1CON = 0x0C; /* Set PWM mode and no decimal for PWM */
    //(199 + 1) x (duty_cicle/100)
    CCPR1L = 0;

    T2CON = 0; /* No pre-scalar, timer2 is off */
    T2CONbits.T2CKPS = 0b01; // 4 pre-scalar
    TMR2 = 0; /* Clear Timer2 initially */
    TMR2ON = 1; /* Timer ON for start counting*/
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
    while (!TRMT);
}
