//HK decoder 8 ch abd PPM output
//Author - dollop
//date - 3.07.2015
//Based on modded MSV LRS system
//Please choose the MC in Project->Configure

//This string is valid for all 48/88/168 MCUs 
#include <mega168_bits.h>

#include <delay.h>
#include <string.h>
#include <stdio.h>

#define _VERSION_ 15

//#define useUART
//#define DEBUG
#define BITBUFFER

// defined I/O ports
#define LED_WHITE PORTD.7 //For debug
#define LED_RED PORTC.1
#define IN_KEY PIND.0
#define pinIRQ PIND.2
#define portIRQ PORTD.2
#define ddrIRQ DDRD.2
#define SatPower PORTD.3
#define PPM_OUT_PORT PORTC
#define PPM_OUT_PIN 0

unsigned int gDataPWM[8];
unsigned char Servo_Number=0;

#define _BV(x)   (1 << x) 
#define RC_CHANNEL_COUNT 8              /* number of control channels */
#define PWM_OUT_NUM 8                   /* maximum PWM channels count */
#define MAX_PPM_OUT 8                   /* maximum PPM channels count (should be less than PWM_OUT_NUM) */
#define ppmPwmCycleTime 40000           // frame length 20ms*2000
#define ppmSync 500                     // syncro length 0,25ms*2000
#define Serial_PPM_OUT_HIGH PPM_OUT_PORT |= _BV(PPM_OUT_PIN) //Serial PPM out on Servo 8
#define Serial_PPM_OUT_LOW PPM_OUT_PORT &= ~_BV(PPM_OUT_PIN) //Serial PPM out on Servo 8
#define gPeriodRx 200

//This helped me a lot to create a bit array!
//http://stackoverflow.com/questions/47981/how-do-you-set-clear-and-toggle-a-single-bit-in-c-c
//http://www.microchip.com/forums/m701812.aspx
#ifdef BITBUFFER
    #define BOOL(x) (!(!(x)))
    #define clrbit(x,n)  ((x[(n)>>3]) &= ~(1UL<<((n)&7)))
    #define setbit(x,n)  ((x[(n)>>3]) |=  (1UL<<((n)&7)))
    #define bitValue(x,n)  BOOL(((x[(n)>>3])&(1UL<<((n)&7))))
#endif

unsigned char offOutsMask[2] = { 0b10011111, 0b11000000 };      // port masks for setting all PWM outputs to 0

    char *portAddr[] = {                   // port addresses for each channel
        &PORTD, &PORTD, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB
    };
      
    unsigned char portMask[] = {                    // port masks for each channel
        _BV(5), _BV(6), _BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5)
    };

// defined types
#define u_char unsigned char
#define s_char signed char
#define u_int unsigned int
#define s_int signed int
#define u_long unsigned long
#define s_long signed long

// uart protocol
#define SIZE_UART_BUFF 32
#define SYN_UART1 0xaf
#define SYN_UART2 0x50
enum { CMD_READ_CNF=0x1, CMD_WRITE_CNF=0x2, CMD_MAIN_DATA=0x3};
typedef struct
{
u_char flash_ver;     // used to check eeprom settings validity
u_char id_sys;        // syn word. not used here
u_char FSmode;        // bit shows how to behave while FS mode. Not used now.
}CNF_DATA;

bit f_timer;
bit f_FailSave;
bit f_OutPWM;
//bit f_PWM_en;
bit f_yet_no_data;
//bit f_PPM_en;
bit f_satmode;         // sattelite mode: bind/receive
bit f_was_FS=0;        // when we recover from FailSave mode this flag is UP

u_char bindbit;

#ifdef BITBUFFER
    u_char databits[21]; // channel data and bind-data holder (startbit+16bits+stopbit)x9=162
#else
    u_char databits[162];
#endif

u_char chancount;
u_char chanbit;

u_char guid[10];

u_char gTCNT2_ex; 
u_char f_flash;
u_char gTimeoutRx;
eeprom CNF_DATA EEP_CnfData;
CNF_DATA gCnfData;

#ifdef useUART
typedef struct
{
u_char ChVal[8];
u_char RSSI;
u_char DropCnt;
u_char Valid;
}MAIN_DATA;
MAIN_DATA gMainData;
u_char gLastRSSI;
u_char gRxErrCnt;
#endif

eeprom unsigned int EEP_FailSave[8];
eeprom u_char EEP_Id[10];
unsigned int ChannelVal[8];
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//--- UART -------------------------------------------------------------
//----------------------------------------------------------------------
#ifdef useUART
u_char gSendUartCnt;
u_char gSendUartInd;
u_char gSendUartBuff[2+1+1+SIZE_UART_BUFF+1];
u_char gRecUartCnt;
u_char gRecUartType;
u_char gRecUartInd;
u_char gRecUartState;
u_char gRecUartCRC;
u_char gRecUartBuff[SIZE_UART_BUFF];
//----------------------------------------------------------------------
void sending_UART(void)
{
if(gSendUartCnt==0) return;
if((UCSR0A & 0x20)==0) return;
UDR0=gSendUartBuff[gSendUartInd];
gSendUartInd++;
if(gSendUartInd==gSendUartCnt) gSendUartCnt=0;
}
//----------------------------------------------------------------------
void send_UART(u_char type, u_char *buff, u_char cnt)
{
u_char crc, i;
u_char *p;

while(gSendUartCnt!=0) sending_UART();
gSendUartCnt=0;
gSendUartInd=0;
p=gSendUartBuff;
*p++=SYN_UART1;
*p++=SYN_UART2;
*p++=cnt;
crc=cnt;
*p++=type;
crc^=type;
for(i=0; i<cnt; i++)
  {
  crc^=buff[i];
  *p++=buff[i];
  }
*p=crc;  
gSendUartCnt=2+1+1+cnt+1;
}
//----------------------------------------------------------------------
void send_to_host(u_char valid)
{
gMainData.Valid=valid;
gMainData.RSSI=gLastRSSI;
gMainData.DropCnt=gRxErrCnt;
send_UART(CMD_MAIN_DATA, (u_char *)&gMainData, sizeof(MAIN_DATA));
OCR0A=gLastRSSI;  
}
//----------------------------------------------------------------------
void parsing_UART();
void received_UART(void)
{
u_char val;

if((UCSR0A & 0x80)==0) return;
val=UDR0;
switch(gRecUartState)
  {
case 0: 
  if(val==SYN_UART1) gRecUartState++; 
break;
case 1: 
  if(val==SYN_UART2) gRecUartState++;
  else gRecUartState=0;
break;
case 2:
  if(val>SIZE_UART_BUFF) { gRecUartState=0; break; }
  gRecUartCRC=val;
  gRecUartCnt=val;
  gRecUartState++;
break;
case 3:
  gRecUartCRC^=val;
  gRecUartType=val;
  gRecUartInd=0;
  gRecUartState++;
  if(gRecUartCnt==0) gRecUartState++;
break;  
case 4:
  gRecUartCRC^=val; 
  gRecUartBuff[gRecUartInd]=val;
  gRecUartInd++;
  if(gRecUartInd==gRecUartCnt) gRecUartState++;
break;    
case 5:
  gRecUartState=0;  
  if(val==gRecUartCRC) parsing_UART();
break;
  }
}
//----------------------------------------------------------------------
void parsing_UART()
{
switch(gRecUartType)
  {
  case CMD_READ_CNF:
    if(gRecUartCnt!=0) break;
    send_UART(CMD_READ_CNF, (u_char *)&gCnfData, sizeof(CNF_DATA));
  break;
  case CMD_WRITE_CNF:
    if(gRecUartCnt!=sizeof(CNF_DATA)) break;
    memcpy(&gCnfData, gRecUartBuff, sizeof(CNF_DATA));
    gCnfData.flash_ver=_VERSION_;
    EEP_CnfData=gCnfData;
    send_UART(CMD_WRITE_CNF, 0, 0);
  break;
  }
}
//------------------------------------------------------------------
#endif
//-----------------------------------------------------
interrupt [EXT_INT0] void ext_int0_isr(void)
  { // data pin from satellite
    EIMSK=(0<<INT1) | (0<<INT0);
    TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
    TCNT0=0xD3;//24us time constant
    gTimeoutRx=0;
    if (f_FailSave==1) f_was_FS=1;
    f_FailSave=0;
    //f_PWM_en=1;
  }
//-----------------------------------------------------
//-----------------------------------------------------
// Timer2 output compare interrupt service routine
interrupt [TIM2_COMPA] void timer2_compa_isr(void)
{// 2000Hz=0.5ms
u_char i;

if(++gTCNT2_ex>=40) { //20ms
    gTCNT2_ex=0; 
    f_timer=1; 
    /*f_OutPWM=1; send_to_host(1);*/
    } //20ms

if(gTimeoutRx>gPeriodRx)                
  { //timeout RX  
  if(f_FailSave==0)
    {
    f_FailSave=1;
        for(i=0; i<8; i++) gDataPWM[i]=EEP_FailSave[i];
        //f_PWM_en=1;
        #ifdef DEBUG
        printf("FS data loaded");
        #endif
    }
  //f_OutPWM=1;
  //send_to_host(0);
  }
else if(gTimeoutRx<255) gTimeoutRx++;
    
}
//------------------------------------------------------
//------------------------------------------------------
#define PPM_MIN_CH    1600   // 0.8ms*2000
#define PPM_MAX_CH    4400   // 2.2ms*2000
//------------------------------------------------------
//------------------------------------------------------
// Timer1 output compare B interrupt service routine
interrupt [TIM1_COMPB] void timer1_compb_isr(void)
{
//Sync time of PPM is over (ppmSync). Put the PPM pin to 0
PPM_OUT_PORT.PPM_OUT_PIN=0;//PPM=LOW;
}
//-----------------------------------------------------

//-----------------------------------------------------
// Timer1 overflow interrupt service routine
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
  
  static unsigned int total_ppm_time=0;
 
  unsigned int us; // this value is not real microseconds
//if (f_PWM_en){

     if (Servo_Number>RC_CHANNEL_COUNT) // back to the first servo 
     {
          total_ppm_time = 0; // clear the total servo ppm time
          Servo_Number=0;
     }
   

     if(Servo_Number <= RC_CHANNEL_COUNT) 
     { // Beginning of PPM pulse
       PORTD &= offOutsMask[0]; // All PPM outs to 0
       PORTB &= offOutsMask[1];
       *portAddr[Servo_Number] |= portMask[Servo_Number]; //Put 1 to a channel        
       //return;  //и вылетаем из прерывани€. ¬ следующий раз прерываемс€ дл€ засекани€ времени на импульс дл€ канала
     }

     if (Servo_Number == RC_CHANNEL_COUNT)  // Check the servo number. 
     {
          //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
          us = ppmPwmCycleTime - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times 
     }
     else           
     {
         us =gDataPWM[Servo_Number];//-ppmSync;  // read the servo timing from buffer  и отнимаем врем€ синхроимпульса
     }
     
     total_ppm_time += us; // calculate total servo signal times.
        
     TCCR1A   =   0;  
     TCCR1B   =   0; //stop timer
      
     TCNT1H= (ppmPwmCycleTime - us)>>8;  
     TCNT1L= ppmPwmCycleTime - us; // configure the timer interrupt for X micro seconds 
             
     OCR1BH=(ppmPwmCycleTime - us+ppmSync)>>8; //«аписываем значени€ дл€ определени€ окончани€ ppmSync
     OCR1BL=ppmPwmCycleTime - us+ppmSync; 

     PPM_OUT_PORT.PPM_OUT_PIN=1;           // ставим 1-ку на PPM выходе 

     Servo_Number++; // jump to next servo

     TCCR1A   =   0x02;  
     TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
//    }
}
//-----------------------------------------------------

//-----------------------------------------------------
//Here we save GUID to EEPROM
void SaveGuid()
{
  u_char tval;
  u_char b=89;     // bit index 
  int i,j;
#ifdef DEBUG
printf("GUID received:");
#endif
  for (i=9;i>=0;i--)
  {
    tval=0;        // reset the channel accum
    b--;           // skip stop bit
    for (j=0;j<8;j++)
    {     
      tval <<= 1;
      #ifdef BITBUFFER
      tval |= bitValue(databits,b);
      b--;
      #else
      tval |= databits[b--];
      #endif
    }
    //b--;          // skip start bit
    EEP_Id[i]=tval;
  }  
//Print out the GUID
#ifdef DEBUG
for (i=0;i<10;i++) {printf("%d ", EEP_Id[i]);}
printf("GUID saved to EEPROM.");
#endif
while(1) {LED_WHITE=1; LED_RED=0;} //Swith on the LED forewer   
}
//-----------------------------------------------------

//-----------------------------------------------------
void radiodecodemode()
{
  ddrIRQ=0;                         //set data pin to input
  EIMSK=(0<<INT1) | (1<<INT0);      //wait for next bit of next byte
}
//------------------------------------------------------

//-----------------------------------------------------
void decodechan()
{
  static u_char b;        // bit index
  u_int tempval;
  int j,i;
  b=160;  
  for (i=8;i>=0;i--)      // 
  {
    tempval=0;            // reset the channel accum
    b--;                  // skip stop bit
    for (j=0;j<16;j++)
    {
      tempval <<= 1;
      #ifdef BITBUFFER
      tempval |= bitValue(databits,b);
      b--;
      #else
      tempval |= databits[b--];
      #endif
    }
    b--;                   // skip start bit
    ChannelVal[i]=tempval;
  }
  //Channel data is decoded
  #ifdef DEBUG
  //for (i=0;i<8;i++) printf("%d,", ChannelVal[i]);
  //printf("\n\r");
  #endif
  chancount=0;
  chanbit=0;      
}
//-----------------------------------------------------

  
//------------------------------------------------------
// Timer 0 overflow interrupt service routine   //each 24us
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
static u_char bindbitcounter, bitcount;
// Reinitialize Timer 0 value
TCNT0=0xD3; //24us time constant
if (f_satmode)                                         //binding mode!
   {
      TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);

      #ifdef BITBUFFER
      if (pinIRQ) setbit(databits,bindbit);
      else clrbit(databits,bindbit);                  //read datapin and store it value in the array
      bindbit++;
      #else
      databits[bindbit++]=pinIRQ;
      #endif
      bindbitcounter++; 
       if (bindbit == 90)                             //if bindbit = 90 then guid complete
         {
            TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);           
            SaveGuid();                               //we need to save guid in eeprom
         }
                                      
       if (bindbitcounter == 9)                       // if 9 bits received then one data is received
         {                                            //and it is time to
            TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);  //enable start bit interrupt
            bindbitcounter = 0;
            EIMSK=(0<<INT1) | (1<<INT0);              //wait for next start bit of next byte
         }
    }
else                                                  //Normal mode
    {
        LED_WHITE=1;
        LED_RED=0;
        f_OutPWM=0;
        TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
        //may need to skip start bit
        #ifdef BITBUFFER
        if (pinIRQ) setbit(databits, chanbit);
        else clrbit(databits, chanbit);
        chanbit++;
        #else
        databits[chanbit++]=pinIRQ;
        #endif
        bitcount++;
        if (bitcount == 18)                           // channel data complete, skip stop bit
            {
            TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00); // stop timer
            bitcount = 0;
            chancount++;
            if (chancount > 8)
              {
                 TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00); // stop timer 0
                 decodechan();
                 f_OutPWM=1;
                 f_yet_no_data=0; //there is data from receiver. So we can generate PWM and PPM
                 if (f_was_FS) TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1); //Stop PPM/PWM timer if we recover from FS state 
              }                                                                                                          

            EIMSK=(0<<INT1) | (1<<INT0);              //wait for next start bit of the next byte 
            }
    }
}
//------------------------------------------------------

//------------------------------------------------------
void sendguidbit() //bitbang the GUID to sattelite
{
  u_char sguidbit=0;
  int byt, b;
delay_ms(500);
  for (byt=0;byt<10;byt++)
  {
    delay_us(150);  //100-600us works
    for (b=0;b<10;b++)
    {
      #ifdef BITBUFFER
      portIRQ=bitValue(databits, sguidbit);
      sguidbit++;
      delay_us(17);//17us, but the actual time according to scope is 24us
      #else
      portIRQ=databits[sguidbit++];
      delay_us(23);// the actual time according to scope is 24us
      #endif

    }
  }
}
//------------------------------------------------------

//------------------------------------------------------
void getguidbits()
{
  int i, x;
  u_char d;
  bindbit=0;

  for (i = 0; i < 10; i++)
  {
    guid[i]= EEP_Id[i];
  }
  
  for (i = 0; i < 10; i++)  
  {
    #ifdef BITBUFFER
    clrbit(databits,bindbit);  // start bit=0
    bindbit++;
    #else
    databits[bindbit++]=0;
    #endif
    d=guid[i];
    for(x=8; x; x--) 
    {
      #ifdef BITBUFFER
      if(d&0x01) setbit(databits,bindbit);
      else clrbit(databits,bindbit);
      bindbit++;
      #else
      if(d&0x01) databits[bindbit++]=1;
      else databits[bindbit++]=0;     
      #endif
      d >>= 1;
    }
    #ifdef BITBUFFER
    setbit(databits,bindbit);  // stop bit = 1
    bindbit++;
    #else
    databits[bindbit++]=1;
    #endif     
  }
}
//------------------------------------------------------

//------------------------------------------------------
void sendguid()
{
  ddrIRQ=1;   //set data pin to output 
  delay_ms(10);
  getguidbits();
  sendguidbit();
  delay_ms(1);
  ddrIRQ=0;   //set data pin to input
  #ifdef DEBUG
  printf("GUID sent to satellite\n\r");
  #endif  
}
//------------------------------------------------------


//------------------------------------------------------
void checkforbindplug()
{    
  if (!IN_KEY)
  { 
      f_satmode=1;
      TCCR1B=0x00;      //stop and disable all the timers but TIMER0
      TCCR2B=0x00;
      TIMSK1=0;
      TIMSK2=0;
      // put satellite into bind mode 
      ddrIRQ=1;         //pinIRQ as OUTPUT;
      portIRQ=0;        //pinIRQ = OFF;
      SatPower=0;       //Switch off satellite power
      delay_ms(500);
      SatPower=1;       //Supply power to satellite
      portIRQ=0;        //pinIRQ = LOW  
      delay_ms(200);
      portIRQ=1;        //pinIRQ = HIGH; 
      ddrIRQ=0;         //pinIRQ as INPUT
      pinIRQ=1;         //enable pullUP on pin IRQ   
      
      delay_ms(500);
      TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
      TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00); //Timer0.stop();
      EIMSK=(0<<INT1) | (1<<INT0); // start the start bit interrupt - wait for first start bit
      #ifdef DEBUG
       printf("Binding mode. Waiting for bind info...\n\r");
      #endif

      #asm("sei")
      while(1){ //blink untill binding informetion is found
           LED_WHITE=1;
           LED_RED=0;
           delay_ms(500);
           LED_WHITE=0;
           LED_RED=1;
           delay_ms(500);   
      };
  }
 else
  {
     #ifdef DEBUG
     printf("Receive mode.n\r");
     #endif
  }
}

//------------------------------------------------------

//------------------------------------------------------
void in_key(void)
{ // 50Hz
u_char key, i;
static u_char old_key=1, debounce;

key=IN_KEY;
if(old_key==key) 
  {
  debounce=0;
  if(!key)
    {
// 
    }
  return; 
  }

if(++debounce<5) return;
old_key=key;
if(key) return;
// Press key
if(f_FailSave==0 && gTimeoutRx<40)//gTimeoutRx<40//each 0.5ms gTimeoutRx inctements. So after 40 it means that 20ms there were no response from satellite
    { 
        EEP_CnfData.FSmode=!EEP_CnfData.FSmode;
        for(i=0; i<8; i++) EEP_FailSave[i]=gDataPWM[i];
        #ifdef DEBUG
        printf("FailSave data stored. FSmode=%d\n\r", EEP_CnfData.FSmode);
        printf("FailSave data: ");
        for (i=0;i<8;i++) {printf("%d ", EEP_FailSave[i]/2);}
        printf("\n\r");
        #endif
    }
}
//-----------------------------------------------------

//-----------------------------------------------------
void OutPWM(void) //actualy it is only scaling subroutine
{
u_char i;
for(i=0; i<8; i++) 
  { // scale
  gDataPWM[i]=ChannelVal[i]<<1; //multiply by 2
  }
//Strt PPM/PWM timer here only if we to recover from FS state 
if (f_was_FS) {f_was_FS=0; Servo_Number=9; TCNT1H= (2000)>>8; TCNT1L=2000; TIMSK1=(0<<ICIE1) | (1<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);} 
}
//-----------------------------------------------------

//-----------------------------------------------------
void saveguidmanual() //manual write TX ID to EEPROM. For testing only
{
  u_char i;
  u_char b[10]={ 238,0,0,0,85,85,85,85,85,151	}; //my turborix
                //224,110,0,0,85,85,85,85,85,247  };
  for (i=0;i<10;i++) EEP_Id[i]=b[i];
  #ifdef DEBUG
      printf("GUID manualy updated\n\r");
      printf("GUID: ");
      for (i=0;i<10;i++) {printf("%d ", EEP_Id[i]);}
      printf("\n\r");
  #endif
}
//-----------------------------------------------------

//-----------------------------------------------------
void main(void)
{
u_char i, div;
// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=0x80;
CLKPR=0x00;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=In Bit6=In Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=T Bit6=T Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=0 Bit0=T 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=In Bit6=Out Bit5=Out Bit4=In Bit3=Out Bit2=In Bit1=In Bit0=In 
DDRD=(0<<DDD7) | (1<<DDD6) | (1<<DDD5) | (0<<DDD4) | (1<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
// State: Bit7=T Bit6=0 Bit5=0 Bit4=T Bit3=0 Bit2=P Bit1=T Bit0=P 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (1<<PORTD0);


// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 2000.000 kHz
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
// Timer Period: 0.024 ms
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0xD0;
OCR0A=0x00;
OCR0B=0x00;


// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1875,000 kHz
// Mode: Fast PWM top=ICR1
// OC1A output: Non-Inverted PWM
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 0,53333 us
// Output Pulse(s):
// OC1A Period: 0,53333 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: On
// Compare B Match Interrupt: Off   
//   TCCR1A   =   0x02; //it is commented, because we need to start the timer only after the first sucessful packet.   
//   TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
 
   ICR1H=(unsigned char)(ppmPwmCycleTime>>8);
   ICR1L=(unsigned char)ppmPwmCycleTime;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 250.000 kHz
// Mode: CTC top=OCR2A
// OC2A output: Disconnected
// OC2B output: Disconnected
// Timer Period: 0.5 ms
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (1<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2A=0x7C;
OCR2B=0x00;



// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EIMSK=(0<<INT1) | (0<<INT0);
EIFR=(0<<INTF1) | (1<<INTF0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (1<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1); 

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);

#ifdef DEBUG
// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART0 Mode: Asynchronous
// USART Baud Rate: 19200
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
//UBRR0L=0x33;  //19200 For MSV AP
UBRR0L=0x08;   //115200 For testing
#else
// USART initialization
// USART disabled
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
#endif

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
ADCSRB=0x00;

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

f_yet_no_data=1; //it is the "power on" condition. This flag prevents PPM and PWM generation before the first packet received

//---------------
if(EEP_CnfData.flash_ver!=_VERSION_)
  { // reset to default
  EEP_CnfData.flash_ver=_VERSION_;
  EEP_CnfData.id_sys=0x56;    // 86
  EEP_CnfData.FSmode=0;
  for(i=0; i<8; i++) EEP_FailSave[i]=3000;
#ifdef DEBUG
  printf("The first time turn on. Defaults loaded");
#endif
  }
gCnfData=EEP_CnfData;
div=0;
//--------------------
for(i=0; i<8; i++) gDataPWM[i]=EEP_FailSave[i];
#ifdef DEBUG
  printf("FS data loaded");
#endif
//--------------------

checkforbindplug();
//saveguidmanual();

SatPower=1;  
delay_ms(200);
LED_WHITE=0;
LED_RED=1;

sendguid();
#asm("sei")
f_satmode=0;
radiodecodemode();

while (1)
  {
  if(!f_yet_no_data) {TCCR1A   =   0x02; TCCR1B   =   0x1A; } //We start PPM/PWM timer after the first decoced packet received

  if(f_FailSave&&!f_yet_no_data)
    {
    LED_WHITE=f_flash;
    LED_RED=!f_flash;            
    }
  if(f_timer)
    {
    f_timer=0;
    if(++div>=5) { div=0; f_flash=!f_flash; }
    in_key();
    }
  if(f_OutPWM) { f_OutPWM=0; OutPWM(); } //its time to convert data 
  //received_UART();
  //sending_UART();
  }
}
