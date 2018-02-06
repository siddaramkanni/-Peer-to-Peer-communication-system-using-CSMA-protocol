// Serial Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#define DATA_EN        (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
// #define RED_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
// #define GREEN_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
// #define BLUE_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED         PWM1_2_CMPB_R
#define BLUE_LED        PWM1_3_CMPA_R
#define GREEN_LED       PWM1_3_CMPB_R
#define RED_LED_B      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_B    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define MAX_CHARS 80
#define MAX_FIELDS 10
char str[MAX_CHARS+1];
#define MAX_MESSAGE 30
#define MAX_DATA 50
#define MAX_RETRY 5
uint8_t SRC_ADDR= 1;
#define BROADCAST_ADDR 255
#define PACKET_SIZE 7
#define RX_BUFFER_SIZE 50
#define TX_BUFFER_SIZE 50

#define SET_size 1
#define ACK_size 1
#define RGB_size 3
#define Poll_Res_size 1
#define Data_Report_size 1
#define Set_Addr_size 1

// Defining the command values
#define CmdSET 0
#define CmdRGB 0x48
#define CmdACK 0x70
#define CmdPOLLREQ 0x78
#define CmdPOLLRES 0x79
#define CmdSETADDR 0x7A
#define CmdRESET 0x7F
#define CmdDATAREQ 0x20
#define CmdDATAREP 0x21
#define CmdPULSE 0x02
#define CmdSQUARE 0x03
#define CmdSAWT 0x04
#define CmdTRI 0x05


// Defining the table for data
uint8_t DST_ADDRESS[MAX_MESSAGE];
uint8_t SEQ_ID[MAX_MESSAGE];
uint8_t COMMAND[MAX_MESSAGE];
uint8_t CHANNEL[MAX_MESSAGE];
uint8_t SIZE[MAX_MESSAGE];
uint8_t CHECKSUM[MAX_MESSAGE];
uint8_t DATA[MAX_MESSAGE][MAX_DATA];
bool valid[MAX_MESSAGE];
bool ackreq[MAX_MESSAGE];
uint8_t retranscount[MAX_MESSAGE];
uint16_t timeout[MAX_MESSAGE];


uint8_t field=0;
uint8_t type[MAX_FIELDS];
uint8_t pos[MAX_FIELDS];
char* str2;
char str4[100];
char str5[50];
uint8_t i,j,addr, channel, value[50];
uint8_t sequence=0;
uint8_t valid1=0, val=0;
uint16_t rx_test_data;
bool ackon;
bool random;
bool cs=false;
bool csEnable;
bool end=false;
uint8_t  currentIndex, txPhase=0, rxPhase=0;
uint16_t rxData[RX_BUFFER_SIZE],d=0;
uint8_t txData[TX_BUFFER_SIZE];
bool inProgress=false;
uint16_t led_timeout;
bool blink=false;
bool time=false;
uint8_t check_data=0;
uint8_t check_data_1=0,n;
uint16_t N=0,M, T0=100, T=500;
char* uart[50];
uint16_t old_rxPhase,dead_rxtimeout,dead_limit=300,old_txPhase,dead_txtimeout;
uint8_t rx_dest_addr,rx_src_addr,rx_seq_id,rx_cmd,rx_channel,rx_size,rx_data_value[20],rx_checksum;
uint16_t pulse_timeout;
uint8_t pulse_amplitude=0;
uint16_t high,low;
uint8_t square_high_amp=0,square_low_amp=0;
uint16_t pulse_timeout_on,pulse_timeout_off,cy,cycles;
uint8_t saw_high_amp,saw_low_amp,delta;
uint16_t dwell;
uint8_t tri_low_amp, tri_high_amp ,delta1;
bool pul=false,squ=false,saw=false,tri=false;
uint8_t ran();
void processmsg();
void square();
void pulse();
void sawtooth();
void triangle();
void command();
void transmitdata();
void receivedata();
void deadlocks();
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------



// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)| SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,F and C peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTA_DIR_R = 0x0C;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0x0C; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons(red, blue, green)
    GPIO_PORTA_DEN_R = 0x0C;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    GPIO_PORTF_DIR_R = 0x0E;  // bits 1,2 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_AFSEL_R |= 0x0E;
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6|GPIO_PCTL_PF3_M1PWM7;


    // Configure GPIO pins for UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    // Configure UART1 pins(GPIO)
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;  // turn-on UART1, leave other UART's in same status
    GPIO_PORTC_DIR_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x70;                           //
    GPIO_PORTC_AFSEL_R |= 0x30;                         //
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
       UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
       UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
       UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx38.4kHz), set floor(r)=21, where N=16
       UART1_FBRD_R = 7;                               // round(fract(r)*64)=45
       UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
       UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

     // Configure Timer 1 as the time base
         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
         TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
         TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
         TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
         TIMER1_TAILR_R = 0x9C40;                       // set load value to 40e3 for 1 kHz interrupt rate 0x9C40;
         TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
         NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
         TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


     //Configure PWM
         SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
             SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
             __asm(" NOP");
             __asm(" NOP");
             __asm(" NOP");
             SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
             SYSCTL_SRPWM_R = 0;
             PWM1_2_CTL_R = 0;
             PWM1_3_CTL_R = 0;
             PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
             PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
             PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

             PWM1_2_LOAD_R = 256;
             PWM1_3_LOAD_R = 256;
             PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

             PWM1_2_CMPB_R = 0;
             PWM1_3_CMPB_R = 0;
             PWM1_3_CMPA_R = 0;

             PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
             PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
             PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;



}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Function that returns string that is compatible with backspace, space and carriage return
void getsUart0()
{
    uint8_t count=0;
    char c;
    start:  c= getcUart0() ;
    if (c==8)
     {
         if(count>0)
         {
             count--;
           goto start ;
         }
         goto start ;
     }

     else if (c==13)
     {
         x: str[count++]=0;
          return;
     }
      if(c>=' ')
      {
          str[count++]=c;
      }
      else
      {
          goto start;
      }
      if(count> MAX_CHARS)
      {
          goto x;
      }
      else
      {
      goto start ;
      }

}

    //Function that defines the differentiates delimiters and alphanumeric characters and defines fields, position and type of string
void isstring(char* str)
{
    field=0;

    uint8_t i, flag=0;
    uint8_t l=strlen(str);
    for(i=0; i<l; i++)
    {
        // Condition for Alphanumeric characters
        if((str[i]>='0' && str[i]<= '9') || (str[i]>='A' && str[i]<='Z')||(str[i]>='a' && str[i]<='z'))
            {
            if((str[i]>='a' && str[i]<='z'))        //defining case insensitivity
            {
                    str[i]=str[i]-32;
            }
            flag++;
            if(flag==1)
                {
                field++;                                                        // Setting Field value
                pos[field-1]=i;                                                 // Setting position value

                // Defining the type of string
                if ((str[i]>='A' && str[i]<='Z')||(str[i]>='a' && str[i]<='z'))
                    type[field-1]='a';
                else
                    type[field-1]='n';
                }
            }
        // Condition for delimiters and converting them to zero
        else
            {
                    flag=0;
                    str[i]=0;

            }
    }

}

// Function for checking the entered data
bool iscommand(char strcmd[], uint8_t min_args)
{
    if((strcmp(&str[pos[0]], strcmd)==0) && (field>= min_args))
        return true;
    else
        return false;
}

// Function to get the entered string
char* getstring(uint8_t field)
{
        if (type[field]=='a')
            return &str[pos[field]];
        else
            valid1 =1;
        return 0;
}

// Function to get the entered number
int16_t getnumber(uint8_t field)
{
    uint8_t x;
    if (type[field]=='n')
    {
        x= atoi(&str[pos[field]]);
        return x;
    }
    else
        valid1 =1;
    return valid1;
}

// Function to get the entered number(16 bits)
int16_t getnumber2(uint8_t field)
{
    uint16_t x;
    if (type[field]=='n')
    {
        x= atoi(&str[pos[field]]);
        return x;
    }
    else
        valid1 =1;
    return valid1;
}

//Function to store the data in the table

void sendPacket(uint8_t destAddr, uint8_t command, uint8_t channel, uint8_t size, uint8_t data[])
{

            if(valid[i]== false)
            {
                uint8_t sum[50] ,sum1[50],l;
                DST_ADDRESS[i]=destAddr;
                COMMAND[i]=command;
                ackreq[i]=ackon ;
                if(ackreq[i]==1 && COMMAND[i]!=0x70 )
                    COMMAND[i]|= 0x80;
                CHANNEL[i]=channel;
                SIZE[i]=size;
                for(l=0;l<size;l++)
                {
                        DATA[i][l]= value[i+l];
                }

                SEQ_ID[i]=sequence;
                sequence++;
                sum[i]=0;
                for( l=0;l<size; l++)
                {
                    sum[i]+= DATA[i][l];

                }
                sum1[i]= SRC_ADDR+DST_ADDRESS[i]+COMMAND[i]+CHANNEL[i]+SIZE[i]+sum[i]+SEQ_ID[i];

                CHECKSUM[i]=~sum1[i];

                retranscount[i]=0;
                timeout[i]=0;
                valid[i]= true;
            }
            else
                putsUart0("ERROR \n");
}



//Timer function that runs every one milisecond

void Timer1Isr()
{
    uint8_t x;

    if(!inProgress)                                                    //Routine for searching the valid message
    {
        for(x=0;x<MAX_MESSAGE;x++)
        {
            if((valid[x]==true)&&(timeout[x]==0))
            {
                inProgress= true;
                currentIndex=x;
                txPhase=0;
                old_txPhase=0;
            }
        }
    }
    transmitdata();                                                 // Function to transmit the data

    if(time)                                                        // Timeout Routine
    {
        if(timeout[currentIndex]>0)
        {
            timeout[currentIndex]--;
        }
        if(timeout[currentIndex]==0)
        {
            sprintf(str5,"Transmitting Msg %u, Attempt %u \r\n",SEQ_ID[currentIndex],retranscount[currentIndex]);
            putsUart0(str5);
        }
    }


    if(!(UART1_FR_R & UART_FR_BUSY))
        DATA_EN=0;                                                 //Turn off transmit pin after transmitting


    receivedata();                                                 // Function to receive the data


    if(blink)                                                      //Routine for led timeout
    {

        if(led_timeout>0)
        {
            led_timeout--;
        }
        if(led_timeout==0)
        {
            GREEN_LED_B=0;
            RED_LED_B=0;
        }
    }



    if(squ)
        square();                                           //calling square function
    if(pul)
        pulse();                                             // calling pulse function
    if(saw)
        sawtooth();                                         // calling sawtooth function
    if(tri)
        triangle();                                         // calling triangle function

    deadlocks();                                            //calling deadlock function

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

//Function for transmitting data.................................................................................................................................................................

void transmitdata()
{
    uint8_t x,l;
    if(inProgress)
    {
        DATA_EN=1;                                                             //Turning on transmit data pin
        if (csEnable)                                                          // Checking for carrier sense
        {
            if(!(UART1_FR_R & UART_FR_RXFE))
            {
                putsUart0("Carrier is busy \n\r");
            }
        }
        switch(txPhase)
        {
        case 0:                                                                 //case 0: Transmitting destination address
            if((UART1_FR_R & UART_FR_TXFE)^(csEnable==false|rxPhase==0))
            {
               UART1_LCRH_R &= ~UART_LCRH_EPS ;
               UART1_DR_R = DST_ADDRESS[currentIndex];
               txData[txPhase]=DST_ADDRESS[currentIndex];
                txPhase++;
                time=false;
                break;
            }

        case 1:                                                                  //case 1: Transmitting source address
            if(!(UART1_FR_R & UART_FR_BUSY))
            {
                UART1_LCRH_R |= UART_LCRH_SPS|UART_LCRH_PEN|UART_LCRH_EPS ;
                UART1_DR_R = SRC_ADDR;
                txData[txPhase]= SRC_ADDR;
                txPhase++;
                time=false;

            }
        case 2:                                                                   //case 2: Transmitting sequence ID address
            UART1_DR_R = SEQ_ID[currentIndex];
            txData[txPhase]=SEQ_ID[currentIndex];
            txPhase++;
            time=false;

        case 3:                                                                  //case 3: Transmitting command address
            UART1_DR_R = COMMAND[currentIndex];
            txData[txPhase]=COMMAND[currentIndex];
            txPhase++;
            time=false;

        case 4:                                                                   //case 4: Transmitting channel address
            UART1_DR_R = CHANNEL[currentIndex];
            txData[txPhase]=CHANNEL[currentIndex];
            txPhase++;
            time=false;

        case 5:                                                                      //case 5: Transmitting size address
            UART1_DR_R = SIZE[currentIndex];
            txData[txPhase]=SIZE[currentIndex];
            txPhase++;
            time=false;

        case 6:                                                                     //case 6: Transmitting data address
            for( l=0;l<SIZE[currentIndex];l++)
            {
                UART1_DR_R =  DATA[currentIndex][l];
                txData[txPhase]= DATA[currentIndex][l];
                time=false;
                txPhase++;
            }
        default:                                                                     //case 7: Transmitting checksum address
            UART1_DR_R =  CHECKSUM[currentIndex];
            txData[txPhase]=CHECKSUM[currentIndex];
            time=false;
           txPhase++;
           end=true;
           break;
        }
        if(SIZE[currentIndex]==0)
            x=PACKET_SIZE+1;
        else
            x=PACKET_SIZE;

        if(end)
        {
            if(txPhase==x + SIZE[currentIndex])                                     //End of message
            {
                inProgress=false;
                RED_LED_B=1;
                led_timeout=500;
                blink=true;
                txPhase=0;
                sprintf(str5,"Queuing Msg %u \r\n",SEQ_ID[currentIndex]);
                putsUart0(str5);
                if(!ackon || COMMAND[currentIndex]==0x70)
                {
                    valid[currentIndex]=false;
                    time=false;
                    if(COMMAND[currentIndex]==0x70)
                    {
                        sprintf(str5,"Acknowledgement sent \r\n");
                        putsUart0(str5);

                    }
                    else
                    {
                        sprintf(str5,"Transmitting Msg %u, Attempt %u \r\n",SEQ_ID[currentIndex],retranscount[currentIndex]);
                        putsUart0(str5);
                    }
                }
                else
                {
                    retranscount[currentIndex]++;
                    if(retranscount[currentIndex]> MAX_RETRY)
                    {
                        valid[currentIndex]=false;
                        RED_LED_B=1;
                        blink=false;
                        time=false;
                        sprintf(str5,"Error sending message %u \r\n",SEQ_ID[currentIndex]);
                        putsUart0(str5);
                    }
                    else
                    {
                        if(random)                                                  //random retransmission
                        {
                            timeout[currentIndex]= T0 + ran()*T;
                            time=true;
                        }
                        else
                        {
                            timeout[currentIndex]= T0 + pow(2,N)*T;
                            N++;
                            if(N>4)
                                N=0;
                            time=true;
                        }
                    }

                }
            }
            end=false;
        }
    }
}

//...Function to receive the data..................................................................................................................................................................

void receivedata()
{

    UART1_LCRH_R |=UART_LCRH_SPS|UART_LCRH_PEN|UART_LCRH_EPS ;

    if((!(UART1_FR_R & UART_FR_RXFE))&&(txPhase==0))
    {
        d = UART1_DR_R ;

        if((d & 0x200)&& txPhase==0)
        {
            rxPhase=0;
            old_rxPhase=0;
            rxData[rxPhase]=d & 0xFF;
            if((rxData[rxPhase]==SRC_ADDR)|| (rxData[rxPhase]== BROADCAST_ADDR))
                rxPhase++;
        }
        else if(rxPhase!=0)
        {
            rxData[rxPhase++]=d;
        }

        if(rxPhase==PACKET_SIZE +rxData[5] )
        {
            processmsg();

            rxPhase=0;
        }
    }
}

// Function to avoid deadlocks

void deadlocks()
{
     if(old_rxPhase != rxPhase)
         dead_rxtimeout++;
     if(dead_rxtimeout>dead_limit)
     {
         rxPhase=0;
         putsUart0("RECEIVE DEADLOCK\n\r");

         dead_rxtimeout=0;
     }


     if(old_txPhase != txPhase)
         dead_txtimeout++;
     if(dead_txtimeout>dead_limit)
     {
         txPhase=0;
         putsUart0("TRANSMIT DEADLOCK\n\r");

         dead_rxtimeout=0;
     }
}

//Random sequence function

uint8_t ran()
{
    uint8_t k;
    uint8_t ran[20]={1,3,4,6,4,2,5,4,1,9,1,5,9,3,1,2,2,3,1,6};
    k=ran[i];
    i++;
    if(i>19)
        i=0;
    return k;
}

//..Function to process the received message packet..................................................................................................................................................

void processmsg()
{
    check_data=0;
    check_data_1=0;

    uint8_t x;
    rx_dest_addr=rxData[0];
    rx_src_addr=rxData[1];
    rx_seq_id=rxData[2];
    rx_cmd=rxData[3];
    rx_channel=rxData[4];
    rx_size=rxData[5];
    for(x=0;x<rx_size;x++)
        rx_data_value[x]=rxData[6+x];
    rx_checksum=rxData[6+rx_size];

    for(n=0; n<PACKET_SIZE-1+rx_size;n++)
        {
            check_data+= rxData[n];
        }
        check_data_1=~check_data;


    if(check_data_1==rx_checksum)               // checking for checksum error
    {
        GREEN_LED_B=1;
        led_timeout=500;
        blink=true;


        if((rx_cmd & 0x7F)==CmdSET)               //set routine
        {
            if(rx_channel==0x01)
                RED_LED=rx_data_value[0];
            if(rx_channel==0x02)
                BLUE_LED=rx_data_value[0];
            if(rx_channel==0x03)
                GREEN_LED=rx_data_value[0];

            if(!ackon)
            {
                sprintf(str4,"DestAddr=%u,SrcAddr=%u,SeqID=%u,Command=%u,channel=%u,size=%u,data=%u,checksum=%u \n\r",rx_dest_addr,rx_src_addr,rx_seq_id,rx_cmd,rx_channel,rx_size,rx_data_value[0],rx_checksum);
                putsUart0(str4);
            }

        }

        if((rx_cmd & 0x7F) ==CmdRGB)         //RGB receive routine
        {
            if(rx_channel==0x04)
            {
                    RED_LED=rx_data_value[0];
                    BLUE_LED=rx_data_value[1];
                    GREEN_LED= rx_data_value[2];
            }
        }
        if((rx_cmd & 0x80) ==0x80)         // Sending ACK
        {
            addr=rx_src_addr;
            channel=rx_channel;
            value[i]=rx_seq_id;
            sprintf(str4,"ACK:Received Address=%u, channel=%u, data=%u, SEQ_ID=%u \n\r",rx_src_addr,rx_channel,rx_data_value[0],rx_seq_id);
            putsUart0(str4);
            sendPacket(addr,CmdACK,channel,ACK_size,&value[i]);
        }
        if((rx_cmd & 0x7F) ==CmdACK)         // Receiving ACK
        {
            if(SEQ_ID[currentIndex]==rx_data_value[0])
            {
                sprintf(str4,"Acknowledgment  received with SEQ_ID= %u \n\r",rx_data_value[0]);

                putsUart0(str4);
                valid[currentIndex]=false;
                time=false;

            }
        }
        if ((rx_cmd & 0x7F) ==CmdPOLLREQ)            //poll request
        {
            addr=rx_src_addr;
            channel=rx_channel;
            value[i]=SRC_ADDR;
            valid[currentIndex]=false;
            sendPacket(addr,CmdPOLLRES,channel,Poll_Res_size,&value[i]);
            putsUart0("Poll response sent \n\r");
        }
        if ((rx_cmd & 0x7F) ==CmdPOLLRES)            // poll response
        {
            sprintf(str4,"POLL RESPONSE RECEIVED FROM ADDRESS %u \n\r",rx_data_value[0]);
            putsUart0(str4);
        }
        if ((rx_cmd & 0x7F) == CmdDATAREQ)                 //data request
        {
            addr=rx_src_addr;
            channel=rx_channel;
            if(rx_channel==40)
                value[i]=PUSH_BUTTON;
            if(rx_channel==1)
                value[i]=RED_LED;
            if(rx_channel==2)
                value[i]=BLUE_LED;
            if(rx_channel==3)
                value[i]=GREEN_LED;
            valid[currentIndex]=false;
            sendPacket(addr,CmdDATAREP,channel,Data_Report_size,&value[i]);
        }
        if ((rx_cmd & 0x7F) == CmdDATAREP)            //data report
        {
            sprintf(str4,"DATA REPORT: CHANNEL=%u, VALUE=%u \n\r",rx_channel,rx_data_value[0]);
            putsUart0(str4);
        }
        if((rx_cmd & 0x7F) == CmdRESET)             //reset
        {
            NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }
        if((rx_cmd & 0x7F) == CmdSETADDR)             //set source address
        {
            SRC_ADDR=rx_data_value[0];
            sprintf(str4,"NEW ADDRESS= %u \n\r",SRC_ADDR);
            putsUart0(str4);
        }
        if((rx_cmd & 0x7F) == CmdPULSE)             //pulse signal
        {
            if(rx_channel==10)
            {
                pulse_amplitude=rx_data_value[0];
                high=rx_data_value[1]*256;
                pulse_timeout= (high|rx_data_value[2])*10;
                RED_LED=pulse_amplitude;
                pul=true;
            }



        }
        if((rx_cmd & 0x7F) == CmdSQUARE)             //square signal
        {
            if(rx_channel==11)
            {
                square_high_amp=rx_data_value[0];
                square_low_amp=rx_data_value[1];
                high=rx_data_value[2]*256;
                pulse_timeout_on=(high|rx_data_value[3]);
                GREEN_LED=square_high_amp;
                low=rx_data_value[4]*256;
                pulse_timeout_off=(high|rx_data_value[5]);
                cy=rx_data_value[6]*256;
                cycles=(cy|rx_data_value[7]);
                squ=true;
            }
        }
        if((rx_cmd & 0x7F) == CmdSAWT)             //sawtooth signal
        {
            if(rx_channel==12)
            {
                saw_low_amp=rx_data_value[0];
                saw_high_amp=rx_data_value[1];
                delta=rx_data_value[2];
                high=rx_data_value[3]*256;
                dwell=(high|rx_data_value[4])*10;
                cy=rx_data_value[5]*256;
                cycles=(cy|rx_data_value[6]);
                saw=true;
                BLUE_LED=saw_low_amp;
            }
        }


        if((rx_cmd & 0x7F) == CmdTRI)             //triangle signal
        {
            if(rx_channel==13)
            {
                tri_low_amp=rx_data_value[0];
                tri_high_amp=rx_data_value[1];
                delta=rx_data_value[2];
                delta1=rx_data_value[3];
                high=rx_data_value[4]*256;
                dwell=(high|rx_data_value[5])*10;
                cy=rx_data_value[6]*256;
                cycles=(cy|rx_data_value[7]);
                tri=true;
                RED_LED=tri_low_amp;
            }
        }

    }
    else
    {
        GREEN_LED_B=1;
        blink=false;
    }
}

// Function to process the received the triangle command

void triangle()
{
    if(cycles!=0)
    {
        if(RED_LED==tri_low_amp)
        {
            if(dwell>0)
                dwell--;
            if(dwell==0)
            {
                tri_low_amp=tri_low_amp+delta;
                dwell=(high|rx_data_value[5])*10;
                RED_LED=tri_low_amp;
            }
        }
        if(RED_LED>=tri_high_amp-20)
        {

           // tri_high_amp=tri_high_amp-delta1;
            RED_LED=tri_high_amp;
            if(dwell>0)
                dwell--;
            if(dwell==0)
            {
                tri_high_amp=tri_high_amp-delta1;
                dwell=(high|rx_data_value[5])*10;
                RED_LED=tri_high_amp;
            }
            if(RED_LED<=rx_data_value[0]+20)
            {
                RED_LED=rx_data_value[0];
                tri_low_amp=rx_data_value[0];
                tri_high_amp=rx_data_value[1];
                if(cycles>0)
                    cycles--;
            }
        }

    }
    else
    {
        RED_LED=0;
        tri=false;
    }


}

// Function to process the received the sawtooth command

void sawtooth()
{
    if(cycles!=0)
    {
        if(BLUE_LED==saw_low_amp)
        {
            if(dwell>0)
                dwell--;
            if(dwell==0)
            {
                saw_low_amp=saw_low_amp+delta;
                dwell=(high|rx_data_value[4])*10;
                BLUE_LED=saw_low_amp;
            }
        }
        if(BLUE_LED>=saw_high_amp-10)
        {
            saw_low_amp=rx_data_value[0];
            BLUE_LED=saw_low_amp;
            if(cycles>0)
            {
                cycles--;
            }
        }
    }
    else
    {
        BLUE_LED=0;
        saw=false;
    }
}

// Function to process the received the pulse command

void pulse()
{
    if(pulse_timeout>0)
        pulse_timeout--;
    if(pulse_timeout==0)
    {
        RED_LED=0;
        pul=false;
    }
}

// Function to process the received the square command

void square()
{

    if(cycles!=0)
    {
        if(GREEN_LED==square_high_amp)
        {
            if(pulse_timeout_on>0)
                pulse_timeout_on--;
            if(pulse_timeout_on==0)
            {
                GREEN_LED=square_low_amp;
                pulse_timeout_on=(high|rx_data_value[3]);
            }

        }
        if(GREEN_LED==square_low_amp)
        {
            if(pulse_timeout_off>0)
                pulse_timeout_off--;
            if(pulse_timeout_off==0)
            {
                GREEN_LED=square_high_amp;
                pulse_timeout_off=(high|rx_data_value[5]);
                if(cycles>0)
                        cycles--;
            }
        }
    }
    else
    {
        GREEN_LED=0;
        squ=false;
    }
}

// Function to find and process the entered command

void command()
{
    if (iscommand("SET", 3))
    {
        str2= getstring(0);
        addr= getnumber(1);
        channel= getnumber(2);
        value[i]= getnumber(3);
        sendPacket(addr,CmdSET,channel,SET_size,&value[i]);
        val=1;

    }

    if(iscommand("RGB",4))
    {
        addr= getnumber(1);
        channel= getnumber(2);
        value[i]= getnumber(3);
        value[i+1]= getnumber(4);
        value[i+2]= getnumber(5);
        sendPacket(addr,CmdRGB,channel,RGB_size,&value[i]);
        val=1;
    }
    if (iscommand ("CS",1))
    {
        if(strcmp(getstring(1),"ON")==0)
        {
            csEnable=true;
            putsUart0("carrier sense detection is enabled \n\r");
        }
        else if(strcmp(getstring(1),"OFF")==0)
        {
            csEnable=false;
            putsUart0("carrier sense detection is disabled \n\r");
        }
        else
            putsUart0("Error in 'cs' command \n\r");
        val=1;
    }
    if (iscommand ("ACK",1))
    {
        if(strcmp(getstring(1),"ON")==0)
        {
            ackon=true;
            putsUart0("data acknowledgment is enabled \n\r");
        }
        else if(strcmp(getstring(1),"OFF")==0)
        {
            ackon=false;
            putsUart0("data acknowledgment is disabled \n\r");
        }
        else
            putsUart0("Error in 'ack' command \n\r");
        val=1;
    }

    if (iscommand ("RANDOM",1))
    {
        if(strcmp(getstring(1),"ON")==0)
        {
            random=true;
            putsUart0("random retransmissions is enabled \n\r");
        }
        else if(strcmp(getstring(1),"OFF")==0)
        {
            random=false;
            putsUart0("random retransmissions is disabled \n\r");
        }
        else
            putsUart0("Error in 'random' command \n\r");
            val=1;
        }

    if (iscommand("POLL", 0))
    {
        addr=BROADCAST_ADDR;
        channel=0;
        value[i]=0;
        sendPacket(addr,CmdPOLLREQ,channel,0,&value[i]);
        val=1;
    }

    if (iscommand("RESET", 1))
    {
        addr=getnumber(1);
        channel=0;
        value[i]=0;
        sendPacket(addr,CmdRESET,channel,0,&value[i]);
        val=1;
    }

    if (iscommand("GET", 1))
    {
        addr=getnumber(1);
        channel=getnumber(2);
        value[i]=0;
        sendPacket(addr,CmdDATAREQ,channel,0,&value[i]);
        val=1;
    }

    if (iscommand("SA", 1))
    {
        addr=getnumber(1);
        channel=0;
        value[i]=getnumber(2);
        sendPacket(addr,CmdSETADDR,channel,1,&value[i]);
        val=1;
    }


    if (iscommand("PULSE", 3))
    {
        addr=getnumber(1);
        channel=getnumber(2);
        value[i]= getnumber(3);
        value[i+1]=getnumber2(4)/0xFF;
        value[i+2]=getnumber2(4)&0xFF;
        sendPacket(addr,CmdPULSE,channel,3,&value[i]);
        val=1;
    }

    if (iscommand("SQUARE", 7))
    {
        addr=getnumber(1);
        channel=getnumber(2);
        value[i]= getnumber(3);
        value[i+1]=getnumber(4);
        value[i+2]=getnumber2(5)/0xFF;
        value[i+3]=getnumber2(5)&0xFF;
        value[i+4]=getnumber2(6)/0xFF;
        value[i+5]=getnumber2(6)&0xFF;
        value[i+6]=getnumber2(7)/0xFF;
        value[i+7]=getnumber2(7)&0xFF;
        sendPacket(addr,CmdSQUARE,channel,8,&value[i]);
        val=1;
    }
    if (iscommand("SAWTOOTH", 6))
    {
        addr=getnumber(1);
        channel=getnumber(2);
        value[i]= getnumber(3);
        value[i+1]=getnumber(4);
        value[i+2]=getnumber(5);
        value[i+3]=getnumber2(6)/0xFF;
        value[i+4]=getnumber2(6)&0xFF;
        value[i+5]=getnumber2(7)/0xFF;
        value[i+6]=getnumber2(7)&0xFF;
        sendPacket(addr,CmdSAWT,channel,7,&value[i]);
        val=1;
    }

    if (iscommand("TRIANGLE", 7))
    {
        addr=getnumber(1);
        channel=getnumber(2);
        value[i]= getnumber(3);
        value[i+1]=getnumber(4);
        value[i+2]=getnumber(5);
        value[i+3]=getnumber(6);
        value[i+4]=getnumber2(7)/0xFF;
        value[i+5]=getnumber2(7)&0xFF;
        value[i+6]=getnumber2(8)/0xFF;
        value[i+7]=getnumber2(8)&0xFF;
        sendPacket(addr,CmdTRI,channel,8,&value[i]);
        val=1;
    }

}



//--    ---------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------


int main(void)
{
    // Initialize hardware
    initHw();
    // Blinking on board green LED on for 500 ms
    GREEN_LED_B = 1;
    waitMicrosecond(500000);
    GREEN_LED_B = 0;
    waitMicrosecond(500000);

    putsUart0("READY \n\r");            // Display status

    sprintf(str4,"DEVICE ADDRESS: %u \n\r",SRC_ADDR);
    putsUart0(str4);


     for(i=0;i<MAX_MESSAGE;i++)
     {
         valid[i]= false;
         val=0;
         valid1=0;



     // Receive. display and configure string

         getsUart0();
         putsUart0(str);
         putcUart0('\n');
         putcUart0('\r');
         isstring(str);
         command();

         if(valid1==1)
         {
             putsUart0("Error \n\r");
         }
         if(!val)
         {
             putsUart0("Error \n\r");
         }

     }
    while(1);
}



