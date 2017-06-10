
/************************************
 * RPM measurement tool
 * rev 1.0 - shabaz - June 2017
 * 
 * Connections:
 * P1.0 - pin2  - LED (present on MSP-EXP430G2 dev board)
 * P1.6 - pin14 - SCL (needs a pull-up resistor!)
 * P1.7 - pin15 - SDA (needs a pull-up resistor!)
 * P2.0 - pin8  - optotransistor input
 * P2.1 - pin9  - LCD reset
 * P2.5 - pin13 - optional button
 * ***********************************/

// #includes
#include <msp430.h>

// #defines
#define LED1_ON P1OUT |= BIT0
#define LED1_OFF P1OUT &= ~BIT0
#define LCD_RES_HIGH P2OUT |= BIT1
#define LCD_RES_LOW P2OUT &= ~BIT1
#define MS50 50000
#define LCD_ADDR 0x3e
#define ROW_TOP 0
#define ROW_BOTTOM 1
#define USE_XTAL
#ifdef USE_XTAL
#define ITER 1
#define CCRINC 4095
#else
#define ITER 16
#define CCRINC 62500
#endif

// global variables
const char lcd_init_arr[]={ 0x38, 0x39, 0x14, 0x7f, 0x51, 0x6c, 0x0c, 0x01 };
unsigned char *PTxData;                     // Pointer to TX data
unsigned char TXByteCtr=0;
unsigned char led_toggle=0;
unsigned char clk_iter=0;
unsigned char lcd_refresh=0;
unsigned int edge_cnt=0;
unsigned int edge_sum=0;
char tstring[14];

// function prototypes
void lcd_init(void);
void lcd_clear(void);
void lcd_setpos(char ln, char idx);
void lcd_print(char* str);
void uint2str(unsigned int number, char* ta);
void i2c_write(char* data, char len);

/************************************
 * interrupt functions
 ************************************/
// we use timer A as a gate to see how many pulses were captured within a certain time
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void)
{
 CCR0 =+ CCRINC;    // increment the compare register for the next interrupt to occur
 clk_iter++;
 if (clk_iter>=ITER)
 {
     clk_iter=0;
     edge_sum=edge_cnt; // store the summed number of edges captured
     lcd_refresh=1;     // we are ready to update the LCD display
     edge_cnt=0;        // reset the dynamic count value
 }
 
 // toggle an LED (present on the development board) as a heartbeat indication
 if (led_toggle)
 {
    LED1_OFF;
    led_toggle=0;
 }
 else
 {
    LED1_ON;
    led_toggle=1;
 }
}

// interrupt to load the serial comms register to stream out bytes of I2C
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  if (TXByteCtr)                            // Check TX byte counter
  {
    UCB0TXBUF = *PTxData++;                 // Load TX buffer
    TXByteCtr--;                            // Decrement TX byte counter
  }
  else
  {
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
    __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
  }
}

// edge interrupt
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{    
    edge_cnt++;
    P2IFG &= ~BIT0; // clear the interrupt flag
} 



/************************************
 * main function
 ************************************/
int
main(void)
{
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    
    // clock configuration
#ifdef USE_XTAL
    // Set up 32768Hz crystal
    BCSCTL3 |= XCAP_3;    // select 12pF caps
     _delay_cycles(10000); // allow xtal time to start up
#else
    //
#endif
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // I/O pin configuration
	P1OUT = 0;
	P2OUT = 0;
	P3OUT = 0;
    P1DIR |= BIT0; // set P1 pin 0 (LED) output
    P2DIR |= BIT1; // set P2 pin 1 (LCD Reset) output
    P3DIR = 0;

    // I2C configuration
    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = LCD_ADDR;                       // Slave Address
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    IE2 |= UCB0TXIE;                          // Enable TX interrupt
    
    // configure timer
    TACTL = TACLR; // reset the timer
#ifdef USE_XTAL
    TACTL = TASSEL_1 + MC_1 + ID_3; // ACLK/8
#else
    TACTL = TASSEL_2 + MC_1 + 0x0c; // SMCLK/8
#endif
    CCTL0 |= CCIE;  
    CCR0 = CCRINC; 
    
    // configure external interrupt
    P2IE |= BIT0;
    P2IES |= BIT0; // set interrupt to be falling edge
    P2IFG &= ~BIT0; // clear the interrupt flag
    
    __bis_SR_register(GIE); // enable interrupts

    // initialize the LCD and print hello!
    lcd_init();
    lcd_setpos(ROW_TOP,0);
    lcd_print("Hello");
	
	// main program loop
	while(1)
	{
	    if (lcd_refresh)
	    {
	        lcd_clear();
	        if (edge_sum==0) // no pulses counted
	        {
	            lcd_setpos(ROW_TOP, 0);
	            lcd_print("Point at Target"); // print helpful suggestion
	        }
	        
	        // print result in rpm
	        lcd_setpos(ROW_BOTTOM, 0);
	        if (edge_sum<1093) // we only print up to 65520 RPM (1092 Hz)
	        {
	            uint2str(edge_sum*60, tstring);
	            lcd_print(tstring);
	        }
	        else
	        {
	            lcd_print("*OVL*"); // print overload message
	        }
	        lcd_setpos(ROW_BOTTOM, 5);
	        lcd_print("RPM");
	        
	        // print result in Hz
	        lcd_setpos(ROW_BOTTOM, 9);
	        uint2str(edge_sum, tstring);
	        lcd_print(tstring);
	        lcd_setpos(ROW_BOTTOM, 14);
	        lcd_print("Hz");
	        lcd_refresh=0;
	    }
	}
	
	return 0; // warning on this line is ok
}

/************************************
 * Other functions
 ************************************/
// LCD handling functions
void
lcd_init(void)
{
    unsigned int i;
    LCD_RES_HIGH;
    _delay_cycles(MS50);
    LCD_RES_LOW;  // Put LCD into reset
    _delay_cycles(MS50);
    LCD_RES_HIGH; // Bring LCD out of reset
    _delay_cycles(MS50);
    char data[2];
    data[0]=0;
    // This loop is used to configure the LCD
    for (i=0; i<8; i++)
    {
        data[1]=lcd_init_arr[i];
        i2c_write(data, 2);
    }
    _delay_cycles(MS50);
}

void
lcd_clear(void)
{
    char data[2];
    data[0]=0;
    data[1]=1;
    i2c_write(data, 2);
    _delay_cycles(MS50);
}

void
lcd_setpos(char ln, char idx)
{
    char data [2];
    data[0]=0;
    if (ln==0)
        data[1]=0x80+idx;   // 0x00 | 0x80
    else
        data[1]=0xc0+idx;   // 0x40 | 0x80
    i2c_write(data, 2);
}

void
lcd_print(char* str)
{
    char data[2];
    data[0]=0x40;
    while(*str != '\0')
    {
        data[1]=*str++;
        i2c_write(data, 2);
    }
}

// convert an unsigned integer (0-65535) into an ASCII string
// this is used in place of sprintf
void
uint2str(unsigned int number, char* ta)
{
    char digit;
    char idx=5;
    char i;
    while (number > 0)
    {
        digit = (char)(number%10);
        number /= 10;
        *(ta+idx-1)=0x30+digit;
        idx--;
        if (idx==0)
          break;
    }
    if (idx==5)
    {
        *(ta+4)='0';
        idx=4;
    }
    if (idx>0)
    {
        for (i=0; i<idx; i++)
        {
            *(ta+i)=' ';
        }
    }
    *(ta+5)='\0';
}

void
i2c_write(char* data, char len)
{
    PTxData = (unsigned char *)data;                      // TX array start address
    TXByteCtr = len;                  // Load TX byte counter
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts until all data TX'd
}

