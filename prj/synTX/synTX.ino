


#include <SPI.h>
#include <ADS1256.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>



#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 868.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     3

#define RFM69_CS      40  //
#define RFM69_INT     3  // 
#define RFM69_RST     2  // "A"
#define LED           13
// Singleton instance of the radio driver

#define DDRYpin     20
#define ADC_SYN     21

#define BUFFER_LENGTH   250



RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission




/************ADC SETUP*************/
float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.5; // voltage reference

ADS1256 adc(clockMHZ, vRef, false);


volatile uint8_t pack[4 * BUFFER_LENGTH];
volatile int i;
volatile float anValue;

int iOld;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t from;




ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  digitalWrite(ADC_SYN, LOW);
  __builtin_avr_delay_cycles(6);//6
  digitalWrite(ADC_SYN, HIGH);
  DDRYisr(); 
}


bool sendLargeSignal(uint8_t* s, uint16_t l, uint8_t type = 1, uint8_t packetSize = RH_RF69_MAX_MESSAGE_LEN) {
  int ii = 0;
  uint8_t packetNum = 0;
  int offset = 2;
  int _packetSize;
  int totPacketNum;
  long m;
  uint8_t packet[RH_RF69_MAX_MESSAGE_LEN];

  while (ii < l) {
    if (ii == 0) {
      totPacketNum = uint16_t((l + 8) / packetSize) + 1;
      packet[0] = packetNum;
      packet[1] = totPacketNum;
      packet[2] = 0;//statusByte;
      packet[4] = 0;//nSample;
      packet[5] = 0;//sps;
      packet[6] = 0; // auxByte1;
      packet[7] = 0;//auxByte2;
      _packetSize = packetSize - 8;
    } else {
      _packetSize = min(l - ii, packetSize);
    }
    memcpy (&packet[8], &s[ii], _packetSize );
    rf69_manager.sendtoWait(packet, sizeof(packet), DEST_ADDRESS);
    packetNum++;
    ii = ii + _packetSize;
  }
  Serial.print("transmit");
    Serial.println(totPacketNum);
  return true;
}



void DDRYisr() {
  rf69.ti++;
  i = (i + 1);
  if ( i > BUFFER_LENGTH) {
    i = 0;
  }
  anValue = adc.readChannel();
  byte * b = (byte *) & anValue;
  pack[i] = b[0];
  pack[i + 1] = b[1];
  pack[i + 2] = b[2];
  pack[i + 3] = b[3];
}

void setup()
{
  float v;
  Serial.begin(115200);
  while (!Serial)
    ;

  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);

  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(50);
  digitalWrite(21, HIGH);

  adc.begin(ADS1256_DRATE_50SPS, ADS1256_GAIN_1, false);

  Serial.println("ADC Started");
  adc.waitDRDY();
  adc.setChannel(2, 3);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  digitalWrite(RFM69_RST, HIGH);
  delay(50);
  digitalWrite(RFM69_RST, LOW);
  delay(50);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);


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


  power_adc_disable();
  power_timer5_disable();
  power_timer4_disable();

  power_usart3_disable();
  power_usart2_disable();
  power_usart1_disable();

  power_twi_disable();


  //rf69_manager.setRetries(1);
  rf69_manager.setTimeout(50);
  rf69._PHASE = 5000;
  rf69._ACQ_RATE = 41;
  //rf69.ACQ_DRYpin=DDRYpin;



  //attachInterrupt( digitalPinToInterrupt(DDRYpin), DDRYisr, FALLING );



}








void debug(int i, float anValue) {
  Serial.print(i);
  Serial.print(" ");
  Serial.print(" intTim ");
  Serial.print(rf69.intTim);
    Serial.print(" OCR3A ");
  Serial.print(OCR3A);
  Serial.print(" ");
  Serial.println(anValue, 6);
}



void loop() {



  if (i != iOld) {
    debug( i, anValue);
  }
  iOld = i;
  if (rf69_manager.available())// & (sendSgn == false))
  {
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      Serial.println(from);
     // if (from == DEST_ADDRESS) {
        i = 0;
        sendLargeSignal(pack, sizeof(pack));
      //}
    }
  }

}
