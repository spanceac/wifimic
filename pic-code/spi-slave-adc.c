#define _XTAL_FREQ 64000000

#include <p18f24k22.h>
#include <xc.h>
#include <string.h>

#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
#pragma config WDTEN = OFF
#pragma config IESO = OFF

#define BUF_SIZ 512
char adc_read_buf[BUF_SIZ];
char adc_send_buf[BUF_SIZ];
char adc_result = 0;
char adc_triggered = 0;
char buffer_full = 0;
int cnt = 0;
#define NOTIFY_DATA_READY (LATBbits.LATB4 = 0)
#define NOTIFY_DATA_NOT_READY (LATBbits.LATB4 = 1)

void interrupt adc_interrupt(void)
{
    PORTBbits.RB5 = 1;
    adc_read_buf[cnt] = ADRESH;
    cnt++;
    if(cnt == BUF_SIZ)
    {
        buffer_full = 1;
        cnt = 0;
    }
    PIR1bits.ADIF = 0;
    //ADCON0bits.GODONE = 1;
    PORTBbits.RB5 = 0;
}

void spi_slave_init(void)
{
    SSP1CON1 = 4; // slave mode using /SS pin
    SSP1STATbits.SMP = 0;
    SSP1STATbits.CKE = 1;
    SSP1CON1bits.CKP = 0;
    SSP1CON1bits.SSPEN1 = 1;
    PIR1bits.SSPIF = 0;
}

void spi_send_buf(char *buf, int size)
{
    int i = 0;
    PORTAbits.RA5 = 0; /*slave select */
    for(i = 0; i < size; i++)
    {
        SSP1BUF = buf[i];
        while(SSP1STATbits.BF == 0);
    }
    PORTAbits.RA5 = 1;
}

void adc_init(void)
{
    ADCON0 = 1; // start ADC module, sample on AN0
    ADCON1 = 0; /* TRIGSEL is CCP5 */
    
    ADCON2bits.ADCS = 6; /* 1T_AD = (1/64)*Fosc = 1us, 11us necesary for full ADC read */
    ADCON2bits.ACQT = 7; /* add 20*T_AD time before starting coversion */
    /* this result in 11us + 20us = 31 us acquisition time*/
    
    CCPTMRS1bits.C5TSEL = 0; /* TMR 1 used for CCP */
    CCP5CONbits.CCP5M = 11; /* special event trigger */
    T1CONbits.TMR1CS = 0; /* Fosc/4 clock */
    T1CONbits.T1CKPS = 3; /* 1:8 prescaler -> 2 MHz */
    TMR1L = 0;
    TMR1H = 0;
    CCPR5H = 0;
    CCPR5L = 200; /* 70 us / 0,5 us = 140 */
    T1CONbits.TMR1ON = 1;
}

char adc_read(void)
{
    ADCON0bits.GODONE = 1;
    while(ADCON0bits.GODONE);
    return ADRESH;
}

void main(void)
{
    OSCCONbits.IRCF = 7; /* 16MHz internal OSC */
    OSCTUNEbits.PLLEN = 1; /* for int OSC, PLL need to be manually enabled */
    ANSELCbits.ANSC3 = 0;
    ANSELCbits.ANSC4 = 0;
    ANSELCbits.ANSC5 = 0;
    ANSELAbits.ANSA5 = 0;
    TRISCbits.TRISC3 = 1; // clock input
    TRISCbits.TRISC4 = 1; //data input
    TRISCbits.TRISC5 = 0; // data out
    TRISAbits.TRISA5 = 1; // slave select
    TRISAbits.TRISA0 = 1; ANSELAbits.ANSA0 = 1;
    TRISBbits.TRISB4 = 0; // data ready
    ANSELBbits.ANSB4 = 0;
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSB5 = 0;
    NOTIFY_DATA_NOT_READY;
    
    PIE1bits.ADIE = 1; /* activate ADC interrupt */
    INTCON = 0xC0; /* activate global and peripheral interrupts */
    adc_init();
    spi_slave_init();
    ADCON0bits.GODONE = 1;
    while(1)
    {
        int i;
        while(buffer_full == 0);

        buffer_full = 0;

        memcpy(adc_send_buf, adc_read_buf, BUF_SIZ);

        NOTIFY_DATA_READY;
        /* now wait to trasnfer all the bytes */
        for(i = 0; i < BUF_SIZ; i++)
        {
            char x;
            SSP1BUF = adc_send_buf[i];
            while(SSP1STATbits.BF == 0);
            x = SSP1BUF;
        }
        NOTIFY_DATA_NOT_READY;
    }
}

