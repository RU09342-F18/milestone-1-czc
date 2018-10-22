#include <msp430.h>

#define MCLK_FREQ_MHZ 8                     // MCLK = 8MHz

/**
 * main.c
 */
char total = 0; // Global
char count = 0; // Counting variable to transition between RGB LEDs and to transmission


UART_setup()  // Configures UART to Baud rate of 9600
{
    __bis_SR_register(SCG0);                 // disable FLL
    CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
    CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                // enable FLL
    Software_Trim();                        // Software Trim to get the best DCOFTRIM value

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                             // default DCODIV as MCLK and SMCLK source

    // Configure UART pins
    P1SEL0 |= BIT6 | BIT7;                    // set 2-UART pin as second function

    // Configure UART
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 17-4: UCBRSx = 0x49
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BR0 = 52;                             // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

}

void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}




#pragma vector=EUSCI_A0_VECTOR  // Interrupt conditional code for UART
__interrupt void UART_A(void)
{
    switch(UCA0IV) {
            case 0:
                break;
            case 2: // Receive buffer full  (UCRXIFG)
                  if(total == 0){
                      total = UCA0RXBUF; // Register used to keep track of total byte num. in code
                      if(total){
                          count = 1;     // Register used to set RGB values for the first three bytes
                      }
                  }
                  else if(count == total){ // Once the message ends, clear the globals
                      total = 0;           // and send the end of message character
                      count = 0;
                      UCA0TXBUF = 0x0D;
                  }

                  else {
                      switch(count)
                      {
                        case 1:  // First byte of message sets Red LED
                            TB0CCR1 = UCA0RXBUF;
                            count += 1;
                            break;
                        case 2:  // Second byte of message sets Green LED
                            TB1CCR1 = UCA0RXBUF;
                            count += 1;
                            break;
                        case 3:  // Third byte of message sets Blue LED
                            TB1CCR2 = UCA0RXBUF;
                            count += 1;
                            UCA0TXBUF = total - 3; // Send new byte total for message to next RGB node
                            break;
                        default:  // Send remainder of message to linked RGB node
                            count += 1;
                            UCA0TXBUF = UCA0RXBUF;
                            break;
                      }
   }
 }
}


/* RGB PWM signals are set up in a hybrid fashion.
 * Two LEDs are set using hardware for PWM
 * One LED is set using software for PWM*/
RGB_setup()
{
    // RGB Ports

        P1OUT = 0;   // Initially clears anything connected to the output of Port 1
        P2OUT = 0;   // Initially clears anything connected to the output of Port 2

        P2SEL0 &= ~BIT3;  // Set P2.3 as GPIO
        P2SEL1 &= ~BIT3;

        P2SEL0 |= BIT0 | BIT1;   // Connects P2.0 to PWM associated with TB1CCR1
        P2SEL1 &= ~BIT0 & ~BIT1; // as well as connects P2.1 to PWM associated with TB1CCR2

        P1DIR |= BIT7;           // Directional registers are set as outputs for connected LEDs
        P2DIR |= BIT0 | BIT1 | BIT3;


    // RGB Timer setup ///////////////////////////////////////////////////////////////////////

        TB1CCR0 = 255; // Sets frequency for Timer_B1 PWM

        TB0CCR1 = 0; // Duty Cycle initially set to zero for Red LED
        TB1CCR1 = 0; // Duty Cycle initially set to zero for Green LED
        TB1CCR2 = 0; // Duty Cycle initially set to zero for Blue LED


        TB0CCTL1 = CCIE; // Interrupt for Red LED
        TB1CCTL1 = OUTMOD_3; // (Set / Reset) Configures PWM for green LED
        TB1CCTL2 = OUTMOD_3; // (Set / Reset) Configures PWM for blue LED


        TB0CTL = TBCLR;
        TB1CTL = TBCLR;

        // Might have to stop timer until all RGB values are gathered
        TB0CTL = TBSSEL_2 + MC_2 + ID_3 + CNTL_3 + TBCLGRP0 + TBIE;  // Timer is set in continuous mode mode (SMCLK)
                                                                     // Counter goes up to 256 (CNTL)
                                                                     // Enables overflow interrupt

        TB1CTL = TBSSEL_2 + MC_1 + ID_3 + CNTL_3 + TBCLGRP0;  // Timer is set in up mode mode (SMCLK)
                                                              // Counter goes up to 256 (CNTL)
    }

#pragma vector=TIMER0_B1_VECTOR   // Interrupt sequence for CCR1 register (TimerB1)
__interrupt void Timer_B1(void)
{
    if(TB0CCR1)           // Shuts off overflow interrupt if CCR equals zero
          TB0CTL |= TBIE; // Ensures LED is off
    else
          TB0CTL &= ~TBIE;

    switch(TB0IV) {
        case 0:
            break;
        case 2:
            P2OUT |= BIT3; // Toggles Red LED
            TB0CCTL1 &= ~CCIFG;
            break;
        case 14:
            P2OUT &= ~BIT3; // Clear Red LED
            break;
        }
}


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;       // Disable the GPIO power-on default high-impedance mode

    UART_setup();
    RGB_setup();
    UCA0TXBUF = 0xAA;   // Test byte

    __bis_SR_register(GIE); // Enables global interrupts

    while(1) // Keeps processor in continuous loop
    {}

    return 0;
}

// RECOMMENDED SETTINGS FOR UART

// Baud Rate: 9600

//       BRCLK: 1000000  (SMCLK)
//       UCOS16: 1
//       UCBRx: 6
//       UCBRFx: 8
//       UCBRSx: 0x20
