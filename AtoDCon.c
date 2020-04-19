
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

#define FCY 42016000.0

// Define Message Buffer Length for ECAN1/ECAN2
#define MAX_CHNUM 3       // Highest Analog input number in Channel Scan
#define SAMP_BUFF_SIZE 128 // Size of the input buffer per analog input
#define NUM_CHS2SCAN 2    // Number of (input pins) channels enabled for channel scan

#define SAMP_FREQUENCY 500000
#define MAX_SAMPLES_PER_PERIOD SAMP_FREQUENCY / 50

// Align the buffer to 512 bytes. This is needed for peripheral indirect mode
int BufferA[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__((space(dma), aligned(1024)));
int BufferB[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__((space(dma), aligned(1024)));

// Bluetooth mac address D8:80:39:F9:17:92

struct Impedance
{
  double real;
  double imag;
};

void processADCSamples(int *);
struct Impedance calcImpedance(int);
double convertSample(int);
void initADC();
void readADC();
void initDMA();

void ms_delay(int N)
{
  unsigned int delay = N; // N ms delay
  int i;
  int numLoops = N / 399;

  for (i = 0; i <= numLoops; ++i) {
    TMR1 = 0;         // reset TMR1
    int tmr1Delay = delay % 399;
    if (i < numLoops) {
      tmr1Delay = 399;
    }
    
    tmr1Delay *= 164;

    while (TMR1 < tmr1Delay); // wait for delay time
  }
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
  AD1CON1bits.SIMSAM = 1; // sample simultaneously

  AD1CON2bits.CSCNA = 1; // Scan Input Selections for CH0+ during Sample A bit
  AD1CON2bits.CHPS = 1; // Converts CH0 and CH1
  AD1CON2bits.SMPI = 0; // 2 ADC Channel is scanned

  AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
  AD1CON3bits.ADCS = 1; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*2
                        // ADC Conversion Time for 10-bit Tc=12*Tab
  AD1CON3bits.SAMC = 1;

  AD1CON4bits.DMABL = 7; // Each buffer contains 128 words

  //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
  AD1CSSH = 0x0000;
  AD1CSSLbits.CSS0 = 1; // Enable AN0 for channel scan
  AD1CSSLbits.CSS3 = 1; // Enable AN3 for channel scan

  // AD1CHS123bits.CH123SA = 0; //CH1 is AN0
  // AD1CHS123bits.CH123NA = 0;
  // AD1CHS0bits.CH0SA = 3; //CH0 is AN3

  //AD1CHS0: A/D Input Select Register
  AD1CHS0bits.CH0SA = 0; // MUXA +ve input selection (AIN0) for CH0
  AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (VREF-) for CH0
  //AD1CHS123: A/D Input Select Register
  AD1CHS123bits.CH123SA = 1; // MUXA +ve input selection (AIN3) for CH1
  AD1CHS123bits.CH123NA = 0; // MUXA -ve input selection (VREF-) for CH1

  //AD1PCFGH/AD1PCFGL: Port Configuration Register
  AD1PCFGH = 0xFFFF;
  AD1PCFGL = 0xFFFF;
  AD1PCFGLbits.PCFG0 = 0; // AN0 is analog
  AD1PCFGLbits.PCFG3 = 0; // AN3 is analog

  IFS0bits.AD1IF = 0;   // Clear the A/D interrupt flag bit
  IEC0bits.AD1IE = 0;   // Do Not Enable A/D interrupt
  AD1CON1bits.ADON = 1; // Turn on AD
}

void startSamp()
{
  AD1CON1bits.SAMP = 1; // start sampling
  DMA0CONbits.CHEN = 1; // Enable DMA
}

void stopSamp()
{
  AD1CON1bits.SAMP = 0; // stop sampling
  DMA0CONbits.CHEN = 0; // disable DMA
}

void initDMA()
{
  DMA0CONbits.AMODE = 2; // Configure DMA for Peripheral indirect mode
  DMA0CONbits.MODE = 2;  // continuous ping pong
  DMA0PAD = (int)&ADC1BUF0;
  DMA0CNT = (SAMP_BUFF_SIZE * NUM_CHS2SCAN) - 1;
  DMA0REQ = 13; // Select ADC1 as DMA Request source

  DMA0STA = __builtin_dmaoffset(BufferA);
  DMA0STB = __builtin_dmaoffset(BufferB);

  IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit
  IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit
}

void initTimer3(double frequency)
{
  TMR3 = 0x0000;

  double scale = 2.0;
  PR3 = (1.0 / frequency) / (scale * (1.0 / FCY));

  IFS0bits.T3IF = 0; // Clear Timer 3 interrupt
  IEC0bits.T3IE = 0; // Disable Timer 3 interrupt
  T3CONbits.TON = 1; // Turn Timer 3 on
}

void initPWM(double frequency)
{
  T2CON = 0x0000;
  PR2 = ((FCY / frequency) - 1) / 2; // divide by 2 since we need 2 toggles for a full period

  OC1CONbits.OCSIDL = 0;
  OC1CONbits.OCFLT = 0; // PWM fault detection, not used unless OCM=0b111
  OC1CONbits.OCTSEL = 0; // use timer 2
  OC1CONbits.OCM = 3; // toggle mode, i.e. square wave

  OC1R = 0;

  T2CONbits.TON = 1; // turn timer 2 on
}

void stopPWM()
{
  T2CONbits.TON = 0; // turn timer 2 off
}

struct Impedance currImpedance;

double R = 9820.0;

struct SampleBuffer
{
  double vReal;
  double vImag;
  double iReal;
  double iImag;
};

unsigned short int vn[2200];
unsigned short int in[2200];

int dmaInterruptCount = 0;
int sampAvg = 0;
unsigned int pingPongState = 0;

double avgArray(int *a, int N)
{
  int sum = 0;
  int i = 0;

  for (i = 0; i < N; ++i)
  {
    sum += *(a + i);
  }

  return (double)sum / N;
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
  int bufferOffset = dmaInterruptCount * SAMP_BUFF_SIZE;
  int i;
  if (pingPongState == 0)
  {
    for (i = 0; i < SAMP_BUFF_SIZE; ++i)
    {
      vn[bufferOffset + i] = BufferA[0][i];
      in[bufferOffset + i] = BufferA[3][i];
    }
  }
  else
  {
    for (i = 0; i < SAMP_BUFF_SIZE; ++i)
    {
      vn[bufferOffset + i] = BufferB[0][i];
      in[bufferOffset + i] = BufferB[3][i];
    }
  }  

  pingPongState ^= 1;

  dmaInterruptCount++; // increment DMA interrupt counter

  IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
}

double convertSample(int samp)
{
  return samp * (3.3 / 1024.0);
}

struct Impedance calcImpedance(int N)
{
  double vReal = 0.0;
  double vImag = 0.0;
  double iReal = 0.0;
  double iImag = 0.0;
  double k = 1.0;
  int n;

  for (n = 0; n < N; ++n)
  {
    double t = 2 * M_PI * k * ((double)n / N);
    double v = convertSample(vn[n]) * 3.3;
    double i = convertSample(in[n]) / 1000.0;

    vReal += v * cos(t);
    vImag += v * -1 * sin(t);

    iReal += i * cos(t);
    iImag += i * -1 * sin(t);
  }

  struct Impedance imp;

  double mag = sqrt(pow(vReal, 2.0) + pow(vImag, 2.0)) / sqrt(pow(iReal, 2.0) + pow(iImag, 2.0));
  double phase = atan(vImag / vReal) - atan(iImag / iReal);

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

void bottomLine()
{
  lcd_cmd(0xC0);
  ms_delay(10);
}

void lcd_data(char data) // subroutine for lcd data
{
  RW = 0;         // ensure RW is 0
  RS = 1;         // assert register select to 1
  DATA &= 0xFF00; // prepare RD0 - RD7
  DATA |= data;   // data byte to lcd
  E = 1;
  ms_delay(10);
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

void measureImpedance()
{
  unsigned int freqs[3] = {50, 500, 5000, 50000};
  int i;
  struct Impedance impResults[4];

  unsigned char impedanceStr[19];

  for (i = 0; i < sizeof(freqs) / sizeof(freqs[0]); ++i)
  {
    clearDisplay();
    ms_delay(100);
    cursorHome();
    ms_delay(100);
    sprintf(impedanceStr, "Measure %uHz", freqs[i]);
    puts_lcd(impedanceStr, 13 + i);
    ms_delay(3000);

    initPWM(freqs[i]);
    ms_delay(500); // wait 0.5s to reach steady state

    int numSamples = SAMP_FREQUENCY / freqs[i];
    int numInterrupts = numSamples / SAMP_BUFF_SIZE;

    // account for integer division truncation
    if (numInterrupts == 0 || numSamples % SAMP_BUFF_SIZE != 0)
    {
      numInterrupts++;
    }

    startSamp();

    dmaInterruptCount = 0;
    while (dmaInterruptCount < numInterrupts);

    stopSamp();
    stopPWM();

    impResults[i] = calcImpedance(numSamples);

    clearDisplay();
    cursorHome();
    ms_delay(100);
    sprintf(impedanceStr, "Re. %+.2e", impResults[i].real);
    puts_lcd(impedanceStr, 13);
    bottomLine();
    sprintf(impedanceStr, "Im. %+.2e",impResults[i].imag);
    puts_lcd(impedanceStr, 13);
    ms_delay(5000);
  }

  // send final results over bluetooth
}

int main(void)
{
  initClockPLL();
  T1CON = 0x8030;
  T1CONbits.TCKPS = 3; // prescale period of 256
  T1CONbits.TON = 1; // Turn Timer 1 on

  initADC();
  initDMA();
  initTimer3(SAMP_FREQUENCY);
  Init_LCD();

  measureImpedance();

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
