#include "msp.h"
#include <string.h>
#include <stdint.h>
#define queueSize 128
char buffer[queueSize];
int bluetoothIndex = 0;
void sendPutty(char *message);
void sendBluetooth(char *message);
void puttySetup();
void bluetoothSetup();
void set3Mhz();
void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    set3Mhz();

    P2->DIR |= BIT2;
    P2->OUT &= ~BIT2;
    puttySetup();
    bluetoothSetup();

    __enable_irq();

    while(1){
    }
}
void EUSCIA2_IRQHandler(void)
{
    if(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){
        buffer[bluetoothIndex++] = EUSCI_A2->RXBUF;
        if(buffer[bluetoothIndex] == '\0'){
            sendPutty(buffer);
            memset(&buffer,0,sizeof(buffer));
            bluetoothIndex = 0;
        }
    }
}
void set3Mhz(){
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_1;           // Set DCO to 3MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 |CS_CTL1_SELM_3; // Select ACLK = REFO// MCLK = DCO // SMCLK = DCO
    CS->KEY = 0; //lock CS module
}
void puttySetup(){
    // Configure UART pins
      P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
      P1->SEL1 &= ~(BIT2+BIT3);
      // Configure UART
      EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
      EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                        EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
      EUSCI_A0->BRW = 19;                     // 12000000/16/9600
      EUSCI_A0->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16;
      EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
      EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
      EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
      // Enable eUSCIA0 interrupt in NVIC module
      NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}
void bluetoothSetup(){
    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function
    P3->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A2->BRW = 19;
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}
void sendPutty(char *message){
    int i;
         for(i = 0; i < strlen(message); i++){
             while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                    EUSCI_A0->TXBUF = message[i];
         }
         memset(&buffer,0,sizeof(buffer));
         return;
}
void sendBluetooth(char *message){
    int i;
         for(i = 0; i < strlen(message); i++){
             while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
                    EUSCI_A2->TXBUF = message[i];
         }
         return;
}
