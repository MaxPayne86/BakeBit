// BakeBit Firmware

//#include <Wire.h>
//#include <msp430.h>

#define SLAVE_ADDRESS       0x04
#define READ_ADDRESS        ((SLAVE_ADDRESS << 1) | BIT0)     //Address plus bit R/_W = 1
#define WRITE_ADDRESS        ((SLAVE_ADDRESS << 1) & ~BIT0)    //Address plus bit R/_W = 0
#define N_CMD               5
#define N_TRANSMIT          4
 
//#define LED_VERDE         BIT3                                // P1.3 Led meaning: all systems are working!
#define LED_ROSSO           BIT0                                // P1.0 Error: I2C bus error / encoder error
 
// Macro definitions
#define PIN_ON_P1(P) (P1OUT |= P)
#define PIN_OFF_P1(Q) (P1OUT &= ~Q)
#define PIN_ON_P2(R) (P2OUT |= R)
#define PIN_OFF_P2(T) (P2OUT &= ~T)

void receiveData(int);
void sendData();
void sendAck(void);

byte val = 0, b[21];
int aRead = 0;
int run_once;

// Global variables
enum {        // Error states
    address_err,
    txdata_err,
    none
}i2c_error;
enum {        // State machine's states
    Idle = 0,
    PrepareReceiveAddress,
    ReceiveAddress,
    PrepareReceiveData,
    ReceiveData,
    PrepareTransmitData,
    TransmitData,
    PrepareReceiveAck,
    ReceiveAck,
    Finish
}I2C_State = Idle;
 
volatile uint8_t TxCount = 0;
uint8_t TxQueue = 0;
uint8_t TxData[N_TRANSMIT];           // Trasmission data vector
volatile uint8_t RxCount = 0;
volatile uint8_t RxQueue = 0;
volatile uint8_t RxData[N_CMD];       // Trasmission data vector
volatile uint8_t lock = 0;

void setup()
{
  //Serial.begin(115200);         // start serial for output
  //Wire.begin(SLAVE_ADDRESS);
  //Wire.onReceive(receiveData);
  //Wire.onRequest(sendData);
  //Serial.print("setup\n");

  USICTL0 = USIPE6 + USIPE7 + USISWRST;   // Port & USI mode setup
  USICTL1 = USII2C + USIIE + USISTTIE;    // Enable I2C mode & enable USI start interrupts
  USICKCTL = USICKPL;                     // Setup clock polarity attivo basso
  USICNT = USISCLREL;                     // release SCL we are not active
  USICTL0 &= ~USISWRST;                   // Release USI from reset
  USICTL1 &= ~USIIFG;                     // Clear pending flag

  i2c_error = none;
  _EINT();
}

void loop()
{
  if (RxQueue == 4) // If new data available
  {
    lock = 1; // Acquire lock

    TxData[0] = RxData[0]; // TODO: verify expected sent first byte

    //Digital Read
    if (RxData[0] == 1)
      val = digitalRead(RxData[1]);

    //Digital Write
    else if (RxData[0] == 2)
      digitalWrite(RxData[1], RxData[2]);

    //Analog Read
    else if (RxData[0] == 3)
    {
      aRead = analogRead(RxData[1]);
      TxData[1] = aRead / 256; // msB
      TxData[2] = aRead % 256; // lsB
      TxQueue = 3;
    }

    //Set up Analog Write
    else if (RxData[0] == 4)
      analogWrite(RxData[1], RxData[2]);

    //Set up pinMode
    else if (RxData[0] == 5)
      pinMode(RxData[1], RxData[2]);

    //Firmware version
    else if (RxData[0] == 8)
    {
      TxData[1] = 1;
      TxData[2] = 2;
      TxData[3] = 6 + 1;
      TxQueue = 4;
    }

    RxQueue = 0; // Queue has been processed
    lock = 0; // Clear lock
  }
}

//void receiveData(int byteCount)
//{
  //  while (Wire.available())
  //  {
  //    if (Wire.available() == 4)
  //    {
  //      flag = 0;
  //      index = 0;
  //      run_once = 1;
  //    }
  //    cmd[index++] = Wire.read();
  //  }
//}

// callback for sending data
void sendData()
{
  //  if (cmd[0] == 1)
  //    Wire.write(val);
  //  else if (cmd[0] == 3)
  //    Wire.write(b, 3);
  //  else if (cmd[0] == 8)
  //    Wire.write(b, 4);
}

void sendAck(void)
{
  // Send acknowledgment bit
  USISRL = 0;                     // Clear bit to be sent (msb)
  USICTL0 |= USIOE;               // Enable output
  USICNT |= 1;                    // Send 1 bit
}

//******************************************************
// USI interrupt service routine
//******************************************************
#pragma vector = USI_VECTOR
__interrupt void USI_TXRX (void)
{
  P1OUT |= BIT3;                      // Debug purpose - lenght of I2C interrupt
  if (USICTL1 & USISTTIFG)            // I2C start condition detected
  {
    P1OUT &= ~LED_ROSSO;              // LED off: Sequence start
    I2C_State = PrepareReceiveAddress;       // Start state machine
  }
  switch (I2C_State)
  {
    case Idle: //Idle, should not get here
      _NOP();
      break;

    case PrepareReceiveAddress: //RX Address
      USICNT = (USIIFGCC | 8);            // Stop auto clr USIIFG, read 8 bits
      USICTL1 &= ~USISTTIFG;              // Clear start flag
      I2C_State = ReceiveAddress;           // Go to next state: check address
      break;

    case ReceiveAddress:                    // Process Address and send (N)Ack
      if (USISRL == READ_ADDRESS)
      {
        sendAck();
        I2C_State = PrepareTransmitData;      // Go to next state: TX data
        P1OUT &= ~LED_ROSSO;                  // LED rosso off
      }
      else if(USISRL == WRITE_ADDRESS)
      {
        sendAck();
        RxCount = 0;
        I2C_State = PrepareReceiveData;
      }
      else
      {
        // Not my address!
        USICNT |= USISCLREL;            // No: release hold on SCL
        I2C_State = Idle;
      }
      break;
    case PrepareReceiveData:
      USICTL0 &= ~USIOE;                // SDA = input
      USICNT = (USIIFGCC | 8);          // Stop auto clr USIIFG, read 8 bits
      I2C_State = ReceiveData;
      break;
    case ReceiveData:
      // TODO: verify lock before writing Data into buffer
      RxData[RxCount++] = USISRL;       // Save data into rx buffer
      sendAck();
      if(RxCount == N_CMD)
      {
        RxCount = 0;
        RxQueue = N_CMD;
        I2C_State = Finish;
      }
      else
        I2C_State = PrepareReceiveData;
      break;
    case PrepareTransmitData:
      USISRL = TxData[TxCount++];       // Load first byte
      USICNT |=  8;         // Bit counter = 8, TX data
      I2C_State = PrepareReceiveAck;    // Go to next state
      break;
    case PrepareReceiveAck:// Receive Data (N)Ack
      USICTL0 &= ~USIOE;    // SDA = input
      USICNT |= 1;          // Bit counter = 1, receive (N)Ack
      I2C_State = ReceiveAck;  // Check acknowledgment next
      break;
    case ReceiveAck:// Process Data Ack/NAck
      if ((USISRL & BIT0) != 0)         // Received Nack or Ack?
      {
        USICNT |= USISCLREL;            // Nack: Release hold on SCL
        I2C_State = Idle;               // Return to idle state
        P1OUT |= LED_ROSSO;             // LED on: error
        i2c_error = txdata_err;
      }
      else                              // Ack received
      {
        if (TxCount == TxQueue) // Has last byte been sent?
        {
          USICNT |= USISCLREL;      // Release hold on SCL
          I2C_State = Idle;
          TxQueue = 0;
        }
        else
        {
          USISRL = TxData[TxCount++];     // Load next byte of data
          USICTL0 |= USIOE;               // Enable output
          USICNT |= 8;                    // Send 8 bits
          I2C_State = PrepareReceiveAck;  // Receive acknowledgment next
        }
      }
      break;
    case Finish:
      USICTL0 &= ~USIOE;        // SDA = input, release SDA
      USICNT |= USISCLREL;      // Release hold on SCL
      I2C_State = Idle;         // Return to idle state
      break;
  } // End Switch

  P1OUT &= ~BIT3;               // Debug purpose - lenght of I2C interrupt
  USICTL1 &= ~USIIFG;           // Clear flag, release SCL if held
}
