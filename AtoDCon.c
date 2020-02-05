
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "header.h"
#include "LCD_Board.h"

/* 
   For Explorer 16 board, here are the data and control signal definitions
   RS -> RB15  
   E  -> RD4    
   RW -> RD5   
   DATA -> RE0 - RE7   
*/

// Control signal data pins
#define RW LATDbits.LATD5  // LCD R/W signal
#define RS LATBbits.LATB15 // LCD RS signal
#define E LATDbits.LATD4   // LCD E signal

// Control signal pin direction
#define RW_TRIS TRISDbits.TRISD5
#define RS_TRIS TRISBbits.TRISB15
#define E_TRIS TRISDbits.TRISD4

// Data signals and pin direction
#define DATA LATE // Port for LCD data
#define DATAPORT PORTE
#define TRISDATA TRISE // I/O setup for data Port

#define M_PI 3.141592653589793

#define FCY 40000000

// Define Message Buffer Length for ECAN1/ECAN2
#define MAX_CHNUM 4       // Highest Analog input number in Channel Scan
#define SAMP_BUFF_SIZE 64 // Size of the input buffer per analog input
#define NUM_CHS2SCAN 2    // Number of (input pins) channels enabled for channel scan

// Number of locations for ADC buffer = 14 (AN0 to AN13) x 8 = 112 words
// Align the buffer to 128 words or 256 bytes. This is needed for peripheral indirect mode
int BufferA[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__((space(dma), aligned(256)));

// Bluetooth mac address D8:80:39:F9:17:92

struct Impedance
{
  float real;
  float imag;
};

void processADCSamples(int *);
struct Impedance calcImpedance(float *, float *, int);
float convertSample(int);
void initADC();
void readADC();
void initDMA();

void ms_delay(int N)
{
  int delay;
  delay = N * 62.5; // N ms delay
  TMR1 = 0;         // reset TMR1
  while (TMR1 < delay)
    ; // wait for delay time
}

void initClockPLL()
{
  // via the header.h file, the system is set to Primary Oscillator (XT) w/ PLL
  // the crystal has a speed of 8 MHz and the system has a max speed of 40 MHz
  // we will run the board at 40 MHz using PLL

  CLKDIVbits.PLLPOST = 0; //Set the postscaler N2 = 2
  CLKDIVbits.PLLPRE = 0;  //Set the prescaler N1 = 2
  PLLFBDbits.PLLDIV = 40; //Set the multipler M = 40;

  //This gives us Fosc = Fin*M/(N1*N2) = 8*40/(2*2) = 80 MHz
  //Then Fcy = Fosc/2 = 40 Mhz
}

void initADC()
{
  AD1CON1bits.ADDMABM = 0; // DMA buffers are built in scatter/gather mode
  AD1CON1bits.FORM = 0;    // Data Output Format: unsigned integer (0-1023)
  AD1CON1bits.SSRC = 2;    // Sample Clock Source: GP Timer starts conversion
  AD1CON1bits.ASAM = 1;    // ADC Sample Control: Sampling begins immediately after conversion
  AD1CON1bits.AD12B = 0;   // 10-bit ADC operation

  AD1CON2bits.CSCNA = 1;               // Scan Input Selections for CH0+ during Sample A bit
  AD1CON2bits.CHPS = 0;                // Converts CH0
  AD1CON2bits.SMPI = NUM_CHS2SCAN - 1; // 4 ADC Channel is scanned

  AD1CON3bits.ADRC = 0;  // ADC Clock is derived from Systems Clock
  AD1CON3bits.ADCS = 63; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
                         // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us

  AD1CON4bits.DMABL = 6; // Each buffer contains 64 words

  //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
  AD1CSSH = 0x0000;
  AD1CSSLbits.CSS0 = 1; // Enable AN0 for channel scan
  AD1CSSLbits.CSS3 = 1; // Enable AN3 for channel scan

  // AD1CHS123bits.CH123SA = 0; //CH1 is AN0
  // AD1CHS123bits.CH123NA = 0;
  // AD1CHS0bits.CH0SA = 3; //CH0 is AN3

  //AD1PCFGH/AD1PCFGL: Port Configuration Register
  AD1PCFGH = 0xFFFF;
  AD1PCFGL = 0xFFFF;
  AD1PCFGLbits.PCFG0 = 0; //AN0 is analog
  AD1PCFGLbits.PCFG3 = 0; //AN3 is analog

  IFS0bits.AD1IF = 0;   // Clear the A/D interrupt flag bit
  IEC0bits.AD1IE = 0;   // Do Not Enable A/D interrupt
  AD1CON1bits.ADON = 1; //Turn on AD
}

void startSamp()
{
  AD1CON1bits.SAMP = 1; // start sampling
}

void initDMA()
{
  DMA1C1Nbits.AMODE = 2; // Configure DMA for Peripheral indirect mode
  DMA1C1Nbits.MODE = 0;  // continuous ping pong disabled
  DMA1PAD = (int)&ADC1BUF0;
  DMA1CNT = (SAMP_BUFF_SIZE * NUM_CHS2SCAN) - 1;
  DMA1REQ = 13; // Select ADC1 as DMA Request source

  DMA0STA = __builtin_dmaoffset(BufferA);

  IFS0bits.DMA1IF = 0; //Clear the DMA interrupt flag bit
  IEC0bits.DMA1IE = 1; //Set the DMA interrupt enable bit

  DMA1CONbits.CHEN = 1; // Enable DMA
}

void initTimer3()
{
  TMR3 = 0;
  PR3 = 4999;
  IFS0bits.T3IF = 0; // Clear Timer 3 interrupt
  IEC0bits.T3IE = 0; // Disable Timer 3 interrupt
  T3CONbits.TON = 1; // Turn Timer 3 on
}

void setFrequency(float frequency)
{
  float scale = 64.0;
  PR3 = (int)(1 / frequency) / (scale * (1 / FCY));
}

float vn[SAMP_BUFF_SIZE];
float in[SAMP_BUFF_SIZE];

struct Impedance currImpedance;

float R = 987.0;

void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
{
  int i;
  for (i = 0; i < SAMP_BUFF_SIZE; ++i)
  {
    vn[i] = convertSample(BufferA[0][i]);
    in[i] = convertSample(BufferA[3][i]) / R;
  }

  currImpedance = calcImpedance(&vn, &in, SAMP_BUFF_SIZE);

  IFS0bits.DMA1IF = 0; // Clear the DMA0 Interrupt Flag
}

float convertSample(int samp)
{
  return samp * (3.3 / 1024.0);
}

struct Impedance calcImpedance(float *vArr, float *iArr, int N)
{
  float vReal = 0;
  float vImag = 0;
  float iReal = 0;
  float iImag = 0;
  float k = 1.0;
  int n;

  for (n = 0; n < N; ++n)
  {
    float t = 2 * M_PI * k * ((float)n / N);

    vReal += *(vArr + n) * cos(t);
    vImag += *(vArr + n) * -1 * sin(t);

    iReal += *(iArr + n) * cos(t);
    iImag += *(iArr + n) * -1 * sin(t);
  }

  struct Impedance imp;

  float mag = sqrt(pow(vReal, 2.0) + pow(vImag, 2.0)) / sqrt(pow(iReal, 2.0) + pow(iImag, 2.0));
  float phase = atan(vImag / vReal) - atan(iImag / iReal);

  imp.real = mag * cos(phase);
  imp.imag = mag * sin(phase);

  return imp;
}

void Init_LCD(void) // initialize LCD display
{
  // 15mS delay after Vdd reaches nnVdc before proceeding with LCD initialization
  // not always required and is based on system Vdd rise rate
  ms_delay(15); // 15ms delay

  /* set initial states for the data and control pins */
  LATE &= 0xFF00;
  RW = 0; // R/W state set low
  RS = 0; // RS state set low
  E = 0;  // E state set low

  /* set data and control pins to outputs */
  TRISE &= 0xFF00;
  RW_TRIS = 0; // RW pin set as output
  RS_TRIS = 0; // RS pin set as output
  E_TRIS = 0;  // E pin set as output

  /* 1st LCD initialization sequence */
  DATA &= 0xFF00;
  DATA |= 0x0038;
  E = 1;
  ms_delay(50);
  //NOP
  E = 0;       // toggle E signal
  ms_delay(5); // 5ms delay

  /* 2nd LCD initialization sequence */
  DATA &= 0xFF00;
  DATA |= 0x0038;
  E = 1;
  ms_delay(50);
  //NOP
  E = 0;       // toggle E signal
  ms_delay(1); // 1m delay

  /* 3rd LCD initialization sequence */
  DATA &= 0xFF00;
  DATA |= 0x0038;
  E = 1;
  ms_delay(50);
  //NOP
  E = 0;       // toggle E signal
  ms_delay(1); // 00uS delay

  lcd_cmd(0x38); // function set
  lcd_cmd(0x0C); // Display on/off control, cursor blink off (0x0C)
  lcd_cmd(0x06); // entry mode set (0x06)
}

void lcd_cmd(char cmd) // subroutiune for lcd commands
{
  DATA &= 0xFF00; // prepare RD0 - RD7
  DATA |= cmd;    // command byte to lcd
  RW = 0;         // ensure RW is 0
  RS = 0;
  E = 1;
  ms_delay(50); // toggle E line
  //NOP
  E = 0;
  ms_delay(5); // 5ms delay
}

void lcd_data(char data) // subroutine for lcd data
{
  RW = 0;         // ensure RW is 0
  RS = 1;         // assert register select to 1
  DATA &= 0xFF00; // prepare RD0 - RD7
  DATA |= data;   // data byte to lcd
  E = 1;
  ms_delay(50);
  //NOP
  E = 0;  // toggle E signal
  RS = 0; // negate register select to 0
}

void puts_lcd(unsigned char *data, unsigned char count)
{
  while (count)
  {
    lcd_data(*data++);
    count--;
  }
}

void clearDisplay()
{
  lcd_cmd(0x0001);
}

void cursorHome()
{
  lcd_cmd(0x0002);
}

int main(void)
{
  initClockPLL();
  T1CON = 0x8030;
  initADC();
  initDMA();
  initTimer3();
  // Init_LCD();

  setFrequency(5000.0);
  startSamp();

  while (1)
  {
    // cursorHome();
    // ms_delay(1000);
    // puts_lcd("Imped = 10", 10);
    // ms_delay(1000);
    // clearDisplay();
    // ms_delay(1000);
  }

  return 0;
}
