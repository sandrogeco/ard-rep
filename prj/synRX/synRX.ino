// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0

// who am i? (server address)
#define MY_ADDRESS     1


#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  // 
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

/* Teensy 3.x w/wing
  #define RFM69_RST     9   // "A"
  #define RFM69_CS      10   // "B"
  #define RFM69_IRQ     4    // "C"
  #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/

/* WICED Feather w/wing
  #define RFM69_RST     PA4     // "A"
  #define RFM69_CS      PB4     // "B"
  #define RFM69_IRQ     PA15    // "C"
  #define RFM69_IRQN    RFM69_IRQ
*/

#define RFM69_INT     3  // 
#define RFM69_CS      40  //
#define RFM69_RST     2  // "A"
#define LED           13

#define PPSpin 21

#define ACQ_RATE 46 //True acquisition rate less than ADC rate
#define PHASE 100//sync distance from last falling timer3 edge
#define MAX_NODE_NUMBER 3

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int16_t packetnum = 0;  // packet counter, we increment per xmission

// Dont put this on the stack
uint8_t data[] = "And hello back to you";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
long m, mold;
int nb;
byte bufOld;
long j = 0;
uint8_t b[2];
bool txNow;
long h, hh, calcPeriod, tOld;
uint8_t node;

void PPSisr() {

  txNow = true;
  rf69_manager.seconds++;
  sei();
  rf69_manager.sendtoWaitSync(node);
 // Serial.println(rf69_manager.seconds);



}

ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  hh++;
}



void setup()
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);

  rf69_manager.setRetries(3);
  rf69_manager.setTimeout(50);
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;

  OCR3A = 41000;            // compare match register 16MHz/256/2Hz
  TCCR3B |= (1 << WGM12);   // CTC mode
  TCCR3B |= (1 << CS11);    // 8 prescaler
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  interrupts();

  attachInterrupt( digitalPinToInterrupt(PPSpin), PPSisr, RISING );



}


bool rxNow;

uint8_t seconds;
uint8_t len = sizeof(buf);
uint8_t from;
uint8_t nPacket, packetNum;
void loop() {
 
  if (txNow) {
     node++;
  if (node>MAX_NODE_NUMBER )
{
    node=2;
  }
    //rf69_manager.sendtoWaitSync(node);
    Serial.print("REQUEST packet from #"); Serial.println(node);
    txNow = false;
    rxNow = true;
    nPacket = 0;
  }


  if (rxNow) {
    if (rf69_manager.available()) {

      if (rf69_manager.recvfromAck(buf, &len, &from)) {
        Serial.print("Got packet from #"); Serial.println(from);
        if (buf[0] == 0) {
          packetNum = buf[1];
        }
      }
      if (packetNum == nPacket) {
        rxNow = false;
      }
      nPacket++;
    }

  }

  /*if (rf69_manager.available())
    {



    /* if (rf69_manager.recvfromAck(buf, &len, &from)) {
    Serial.println("g");

      Serial.print("Got packet from #"); Serial.println(from);
      /*  Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.print("] : ");
        Serial.println(buf[0]);
        Serial.println(buf[1]);
        Serial.println(buf[2]);*/
  //Serial.println(micros()-m);




  //}
  // }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}
