/*
 *
 * Date: Oct. 28,2020
 * JESUS MINJARES
 * BSEE with Computer Concentration
 *
 * App:
 *      Set the HC-05 Module as a slave using UART. Set the HC-05 in
 *      AT Mode to configure. On AT Mode the baud rate must be at 38400 bps.
 *      Print HC-05 transmit data via puTTY Serial Monitor.
 *
 *      S: State
 *      RX: Receiver
 *      TX: Transmitter
 *      VCC: 3.3-5v
 *      GND: 0v
 *      EN: Set on high, for AT Mode
 *                                                -----------------
 *               MSP432P401x                     |      HC-05      |
 *            -----------------                   S RX TX VCC GND EN
 *           |          0V(GND)|-->HC-05 GND
 *           |                 |
 *           |RST         VCC  |-->HC-05 VCC
 *           |                 |
 *           |         P3.2(RX)|-->HC-05 TX
 *           |         P3.3(TX)|-- 1K-.<--HC-05 RX
 *HC-05 EN<--|P2.3             |      |
 *           |_________________|      2k
 *                                    |
 *                                    GND
 *
 *
*/
#include "msp.h"
#include <string.h>
#include <stdint.h>
#define queueSize 256 // set buffer size
char buffer[queueSize];  // create a buffer of size queueSize
int bluetoothIndex = 0; //set the index of the buffer to 0
void sendPutty(char *message); //send message via serial
void sendBluetooth(char *message); // send message via bluetooth
void puttySetup(); //setup for serial
void bluetoothSetup(); //setup for the HC-05 @ 38400
void set3Mhz(); // set clock at 3 Mhz
void enablePinSetup(uint16_t bit); //set the Enable pin as output
#define EN BIT3 //define EN as BIT3
char *slaveCommand[] ={
                       "AT\r\n",
                       "AT+UART?\r\n",
                       "AT+ROLE?\r\n",
                       "AT+ADDR?\r\n"
};
int s_i = 0;
void main(void)
 {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    set3Mhz(); //set 3Mhz
    enablePinSetup(EN); //set BIT3 as output

    puttySetup(); //enable UART0 for serial
    bluetoothSetup(); //enable UART2 for bluetooth

    __enable_irq(); //enable interrupts
    sendPutty("Setting Slave Module!!\r\n"); // sent message

    P2->OUT |= EN; //set pin as HIGH

    while(s_i < 4){ //iterate over the list
        sendBluetooth(slaveCommand[s_i++]); //send command, and increment
        __delay_cycles(15000000); //delay
        sendPutty(buffer); //send buffer
        __delay_cycles(10000000); //delay
    }

    P2->OUT &= ~(EN); //set EN as low
    P2->DIR &= ~(EN); //turn off output
    __delay_cycles(1000000); //delay

    while(1){
        sendPutty("Slave has been configured\r\n");
        __delay_cycles(12000000);
    } //infinite loop
}
void EUSCIA2_IRQHandler(void){
    if(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG){ //check flag
        buffer[bluetoothIndex++] = EUSCI_A2->RXBUF; //save char, increment buffer
        if(bluetoothIndex == queueSize){ //index > buffer size
            memset(&buffer,0,sizeof(buffer)); //delete buffer
            bluetoothIndex = 0; //reset index
        }
    }
}
void enablePinSetup(uint16_t bit){
    P2->DIR |= bit; //enable bit as output
    P2->OUT &= ~bit; //set output bit as low
    P2->SEL0 &=~(bit); //disable sel0
    P2->SEL1 &= ~(bit); //disable sel1
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
      P1->SEL0 |= BIT2 | BIT3;                // set UART pin as secondary function
      P1->SEL1 &= ~(BIT2+BIT3);
      // Configure UART
      EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
      EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
      EUSCI_A0->BRW = 19;                     // 3MHz/16/9600 = 19.53125, 19
      EUSCI_A0->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS) |EUSCI_A_MCTLW_OS16; // 19.53125 - 19 = .53125*16 = 8.5, round up
      EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
      EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
      EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt
      // Enable eUSCIA0 interrupt in NVIC module
      NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}
void bluetoothSetup(){
    // Configure UART pins
    P3->SEL0 |= BIT2 | BIT3;                // set UART pin as secondary function
    P3->SEL1 &= ~(BIT2+BIT3);
    // Configure UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    EUSCI_A2->BRW = 4; // 3Mhz/(38400) = 78.125 / 16 = 4.8828125
    EUSCI_A2->MCTLW = (15 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16; //(4.8828125-4)*16 = 14.125 -> 15

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A2 RX interrupt
    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA2_IRQn) & 31);
}
void sendPutty(char *message){
    int i; //create variable
    for(i = 0; i < strlen(message); i++){ //iterate over length of the string
         while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); //check TX flag
                EUSCI_A0->TXBUF = message[i]; //set char to TXBUF
    }
   //  memset(&buffer,0,sizeof(buffer));//clear buffer
     return;
}
void sendBluetooth(char *message){
    int i; //create variable
    for(i = 0; i < strlen(message); i++){ //iterate over length of the string
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); //check TX flag
            EUSCI_A2->TXBUF = message[i]; //set char to TXBUF
    }
    return;
}
