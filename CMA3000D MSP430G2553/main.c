//******************************************************************************
// MSP430G2553 Demo - USCI_B0, SPI Interface to CMA3000 Acceleration Sensor
//             Adapted from http://tinyurl.com/bzpfgrn
//
// Uses Texas Instruments MSP-EXP430G2 Launchpad with MSP430G2553
// Tested on Code Composer Studio Version 5.2.1.00018
// Version 0.1 Adam Cohen
//
//******************************************************************************
// CONSTANTS
#define XTAL 16000000L
#define TICKSPERMS (XTAL / 1000 / 5 - 1)
#define TICKSPERUS (TICKSPERMS / 1000)
// LIBRARIES
#include "msp430g2553.h"
#include <stdint.h>

// PORT DEFINITIONS
#define PORT_INT_IN  P2IN
#define PORT_INT_OUT P2OUT

#define PORT_INT_DIR P2DIR
#define PORT_INT_IE  P2IE
#define PORT_INT_IES P2IES
#define PORT_INT_IFG P2IFG
#define PORT_INT_VECTOR PORT2_VECTOR

#define PORT_CSB_OUT P1OUT
#define PORT_CSB_DIR P1DIR

#define PORT_SPI_DIR P1DIR
//#define PORT_SPI_SEL P1SEL

// REGISTER AND FLAG DEFINITIONS
#define TX_BUFFER UCB0TXBUF
#define RX_BUFFER UCB0RXBUF
#define IRQ_REG IFG2
#define RX_IFG UCB0RXIFG
#define TX_IFG UCB0TXIFG
#define SPI_CTL0 UCB0CTL0
#define SPI_CTL1 UCB0CTL1
#define SPI_BR0 UCB0BR0
#define SPI_BR1 UCB0BR1
// CMA3000 Registers
#define WHO_AM_I 0x00
#define REVID 0x01
#define CTRL 0x02
#define STATUS 0x03
#define RSTR 0x04
#define INT_STATUS 0x05
#define DOUTX 0x06
#define DOUTY 0x07
#define DOUTZ 0x08
#define MDTHR 0x09
#define MDFFTMR 0x0A
#define FFTHR 0x0B

// Control Register setup
#define G_RANGE_2 0x80      // 2g range
#define INT_LEVEL_LOW 0x40  // INT active high
#define MDET_NO_EXIT 0x20   // Remain in motion detection mode
#define I2C_DIS 0x10        // I2C disabled
#define MODE_PD 0x00        // Power Down
#define MODE_100 0x02       // Measurement mode 100 Hz ODR
#define MODE_400 0x04       // Measurement mode 400 Hz ODR
#define MODE_40 0x06        // Measurement mode 40 Hz ODR
#define MODE_MD_10 0x08     // Motion detection mode 10 Hz ODR
#define MODE_FF_100 0x0A    // Free fall detection mode 100 Hz ODR
#define MODE_FF_400 0x0C    // Free fall detection mode 400 Hz ODR
#define INT_DIS 0x01        // Interrupts disabled
#define MDET_MASK 0x3       // Interrupts disabled


// PIN DEFINITIONS
#define PIN_INT  BIT0        // Interrupt on P2.0
#define PIN_CSB  BIT4        // CSB/STE (slave transmit enable)/SS (slave select)/Serial Port Enable active low on P1.4
#define PIN_MOSI BIT7        // MOSI/SIMO on P1.7
#define PIN_MISO BIT6        // MISO/SOMI on P1.6
#define PIN_SCK  BIT5        // SCK/CLK  on P1.5

#define LED1  BIT0          //P1.0
#define LED2  BIT1          //P1.1
#define LED3  BIT2          //P1.2
#define LED4  BIT3          //P1.3
#define LED5  BIT1          //P2.1
#define LED6  BIT2          //P2.2

// FUNCTION PROTOTYPES
unsigned char ReadRegister(unsigned char Address);
unsigned char WriteRegister(unsigned char Address, unsigned char Data);

int16_t convert_raw_data_to_acceleration_value(uint8_t acc_data);
void wait_ms(unsigned short ms);
void wait_us(unsigned short us);
void wait_1_3us(uint16_t t);
void disable_leds();

uint8_t Data = 0xff;
uint8_t RevID = 0xff;
unsigned char Xdata;
unsigned char Ydata;
unsigned char Zdata;
unsigned char IntStatus;


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop WDT.


    BCSCTL1  = CALBC1_16MHZ;      // Set DCO to calibrated 16MHz
    DCOCTL   = CALDCO_16MHZ;
    BCSCTL2 |= DIVS_3;            // SMCLK to 2 MHz. Divide 16MHz by 8 to give us 2MHz

    PORT_INT_DIR &= ~PIN_INT;    // Set INT pin as input (0 for input, 1 for output)
    PORT_INT_IE |= PIN_INT;      // INT pin interrupt enabled
    PORT_INT_IES = 0x00;         // Generate interrupt on Lo to Hi edge

    PORT_INT_IFG &= ~PIN_INT;    // Clear interrupt flag

    PORT_CSB_DIR |= PIN_CSB;     // Set CSB pin as output
    PORT_CSB_OUT |= PIN_CSB;     // Unselect acceleration sensor.  CSB is active low, so
                                 // by setting the pin to high, we deselect the sensor

    P1SEL  |= PIN_MOSI | PIN_MISO | PIN_SCK; // P1.7,P1.6,P1.5 USCI_B0 option select
    P1SEL2 |= PIN_MOSI | PIN_MISO | PIN_SCK; //(MSP430G2x53 guide page 49).
    //P1DIR |= BIT0;        // P1.0 output direction, no idea why
    //P1OUT |= BIT0;

    disable_leds();

    // configure all leds for output
    P1DIR |= LED1 | LED2 | LED3 | LED4;
    P2DIR |= LED5 | LED6;

    // Initialize SPI interface to acceleration sensor
    // 3 pin SPI, synchronous mode, SPI master, 8 data bits, MSB first,
    // clock idle (inactive state) low (UCCKPL = 0), data output on falling edge
    // (data is captured on the first UCLK edge and changed on the following edge).
    // note: we're actually using 4 pins, but the CS pin isn't considered a dedicated
    // wire. If we were using true 4 pin SPI, we'd have to use UCMODE_2 (4-pin SPI with
    // UCB0STE active low on port 1.4), but this isn't necessary, since we can just
    // enable/disable the CS bit on 1.4 as needed
    //SPI_CTL0 |= UCSYNC | UCMST | UCMSB;

    //negative clock polarity means UCCKPL = 0 (clock idle low/inactive state is low).
    SPI_CTL0 |= UCSYNC | UCMST | UCMSB | UCCKPH;


    SPI_CTL1 |= UCSSEL_2 | UCSWRST;        // SMCLK as clock source (USCI 0 Clock Source: 2).
    SPI_BR0 = 0x08;              // Low byte of division factor for baud rate (250 kHz)
    SPI_BR1 = 0x00;              // High byte of division factor for baud rate

    // set baud rate division factor to 0x0004 (since 2mhz/4 = 500 kHz)
    //SPI_BR0 = 0x04;
    //SPI_BR1 = 0x00;

    SPI_CTL1 &= ~UCSWRST;        // Start SPI hardware

    wait_us(44);
    ReadRegister(STATUS);

    wait_us(44);
    RevID = ReadRegister(REVID); // Read REVID register

    wait_us(44); // 11 * tsck
    Data = WriteRegister(MDTHR, BIT3); // set motion detection threshold to 71 mg
    //Data = WriteRegister(MDTHR, BIT2); // set motion detection threshold to 571 mg

    wait_us(44);
    Data = WriteRegister(MDFFTMR, BIT4); // set motion detection time to 100ms

    // Activate measurement mode: 2g/400Hz
    //Data = WriteRegister(CTRL, G_RANGE_2 | I2C_DIS | MODE_400);

    // Activate measurement mode: 8g/40Hz
    //Data = WriteRegister(CTRL, I2C_DIS | MODE_40);
    wait_us(44);
    // motion detection mode measurement mode: 8g/10Hz
    //Data = WriteRegister(CTRL, G_RANGE_2 | I2C_DIS | MODE_MD_10 | MDET_NO_EXIT);

    wait_us(44);

    // motion detection mode: 8g/10Hz. Enter measurement mode
    // 400Hz after motion is detected
    Data = WriteRegister(CTRL,  G_RANGE_2 | I2C_DIS | MODE_MD_10);

    //wait_us(44);
    //Xdata = ReadRegister(DOUTX); // Dummy read to generate first INT pin Lo to Hi

    // transition in all situations, also while debugging
    __bis_SR_register(LPM4_bits + GIE); // Enter LPM3 w/interrupt
}

// Port 2 interrupt service routine, INT pin
#pragma vector=PORT_INT_VECTOR
__interrupt void Port_INT_ISR(void)
{
  int16_t x_acceleration_data = 0;
  int16_t y_acceleration_data = 0;
  int16_t z_acceleration_data = 0;

  uint16_t x_acceleration_data_abs = 0;
  uint16_t y_acceleration_data_abs = 0;
  uint16_t z_acceleration_data_abs = 0;

    if (PORT_INT_IN & PIN_INT)
    {
        wait_us(44);
        Xdata = ReadRegister(DOUTX); // Read DOUTX register
        wait_us(44); // 11 * tsck
        Ydata = ReadRegister(DOUTY); // Read DOUTY register
        wait_us(44);
        Zdata = ReadRegister(DOUTZ); // Read DOUTZ register

        x_acceleration_data = convert_raw_data_to_acceleration_value(Xdata);
        y_acceleration_data = convert_raw_data_to_acceleration_value(Ydata);
        z_acceleration_data = convert_raw_data_to_acceleration_value(Zdata);

        x_acceleration_data_abs = abs(x_acceleration_data);
        y_acceleration_data_abs = abs(y_acceleration_data);
        z_acceleration_data_abs = abs(z_acceleration_data);

        disable_leds();

        if ((x_acceleration_data_abs > y_acceleration_data_abs) &&
            (x_acceleration_data_abs > z_acceleration_data_abs))
        {
          if (x_acceleration_data < 0)
          {
            P1OUT |= LED4;
          }
          else
          {
            P1OUT |= LED1;
          }
        }
        else if ((y_acceleration_data_abs > x_acceleration_data_abs) &&
            (y_acceleration_data_abs > z_acceleration_data_abs))
        {
          if (y_acceleration_data < 0)
          {
            P2OUT |= LED5;
          }
          else
          {
            P1OUT |= LED2;
          }
        }
        else if ((z_acceleration_data_abs > x_acceleration_data_abs) &&
            (z_acceleration_data_abs > y_acceleration_data_abs))
        {
          if (z_acceleration_data < 0)
          {
            P2OUT |= LED6;
          }
          else
          {
            P1OUT |= LED3;
          }
        }
        else if ((x_acceleration_data_abs == y_acceleration_data_abs) &&
            (y_acceleration_data_abs == z_acceleration_data_abs))
        {
           P1OUT |= LED1 | LED2 | LED3 | LED4;
           P2OUT |= LED5 | LED6;
        }




        // go back into motion detection mode, will take some time before
        // this actually takes effect.
        //Data = WriteRegister(CTRL,  G_RANGE_2 | I2C_DIS | MODE_MD_10);
        //wait_us(44);

        // Read INT_STATUS register to acknowledge and clear the interrupt
        wait_us(44);
        ReadRegister(INT_STATUS);

        PORT_INT_IFG &= ~PIN_INT; // Clear interrupt flag
    }
}

void disable_leds()
{
  P1OUT &= ~(LED1 | LED2 | LED3 | LED4);
  P2OUT &= ~(LED5 | LED6);
}

int16_t convert_raw_data_to_acceleration_value(uint8_t raw_acceleration_data)
{
  uint16_t acceleration_values[] = {1142,571,286,143,71,36,18};
  int16_t acceleration_data = 0;
  uint8_t negative = 0, i = 0;


  // sign bit is store in most significant bit.
  // If enabled, we've got a negative number
  if (raw_acceleration_data & BIT7)
  {
    negative = 1;
    //convert using two's complement. Could also flip all
    //the bits of the integer and add one, then make the
    //number negative
    raw_acceleration_data = 256 - raw_acceleration_data;
  }

  for (i = 0; i < 7; i++)
  {
    if (raw_acceleration_data & (BIT6 >> i))
      acceleration_data += acceleration_values[i];
  }

  return negative ? acceleration_data*-1 : acceleration_data;
}

/*
PORT_CSB_OUT &= ~PIN_CSB;
IFG2 &= ~UCB0TXIFG;
while ((UCB0STAT & UCBUSY));
UCB0TXBUF = byte; // Adress to be read
while ((UCB0STAT & UCBUSY));
UCB0TXBUF = 0xff; // Dummy write
while ((UCB0STAT & UCBUSY));
return UCB0RXBUF;
PORT_CSB_OUT |= PIN_CSB;
*/

// Read a byte from the acceleration sensor
unsigned char ReadRegister(unsigned char Address)
{
    unsigned char Result;
    Address <<= 2; // Address to be shifted left by 2 and RW bit to be reset
    PORT_CSB_OUT &= ~PIN_CSB; // Select acceleration sensor

    Result = UCB0RXBUF; // Read RX buffer just to clear interrupt flag

    UCB0TXBUF = Address; // Write address to TX buffer

    while (!(IFG2 & UCB0RXIFG)); // Wait until new data was written into RX buffer

    Result = UCB0RXBUF; // Read RX buffer just to clear interrupt flag

    UCB0TXBUF = 0xFF; // Write dummy data to TX buffer

    while (!(IFG2 & UCB0RXIFG)); // Wait until new data was written into RX buffer

    Result = UCB0RXBUF; // Read RX buffer

    PORT_CSB_OUT |= PIN_CSB; // Deselect acceleration sensor
    // Return new data from RX buffer
    return Result;
}

// Write a byte to the acceleration sensor
unsigned char WriteRegister(unsigned char Address, unsigned char Data)
{
    unsigned char Result;
    Address <<= 2; // Address to be shifted left by 2
    Address |= 2; // RW bit to be set
    PORT_CSB_OUT &= ~PIN_CSB; // Select acceleration sensor

    Result = RX_BUFFER; // Read RX buffer just to clear interrupt flag
    TX_BUFFER = Address; // Write address to TX buffer
    while (!(IRQ_REG & RX_IFG)); // Wait until new data was written into RX buffer
    Result = RX_BUFFER; // Read RX buffer just to clear interrupt flag
    TX_BUFFER = Data; // Write data to TX buffer
    while (!(IRQ_REG & RX_IFG)); // Wait until new data was written into RX buffer
    Result = RX_BUFFER; // Read RX buffer
    PORT_CSB_OUT |= PIN_CSB; // Deselect acceleration sensor
    return Result;
}

void accReadReg(uint8_t reg, uint8_t *pVal)
{
    PORT_CSB_OUT &= ~PIN_CSB; // Select acceleration sensor
    wait_1_3us(2);
    spiWriteByte(reg);
    spiReadByte(pVal, 0xFF);
    PORT_CSB_OUT |= PIN_CSB; // Deselect acceleration sensor
}

// wait ms
void wait_ms(unsigned short ms)
{
    unsigned short a, b;
    for (a = ms; a > 0; a--) // outer loop takes 5 ck per round
    for (b = TICKSPERMS; b > 0; b--) // inner loop takes 5 ck per round
    asm(" nop");
}
// wait us
void wait_us(unsigned short us)
{
    unsigned short a;
    us *= TICKSPERUS;
    for (a = us; a > 0; a--) // loop takes 5 ck per round
    asm(" nop");
}

// Wait 1+1/3*t [us]
void wait_1_3us(uint16_t t)
{
  uint16_t i = 0;
  do{
      for (i = 0; i<t; i++)
          asm(" nop");
  }while(0);
}
