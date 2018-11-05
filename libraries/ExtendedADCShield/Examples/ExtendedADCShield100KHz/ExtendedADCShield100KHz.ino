//This example shows how to achieve 100kHz sampling rate on a single input on the Extended ADC Shield

#include <SPI.h>

#define CONVST 8
#define RD 10
#define BUSY 9

byte x=0; byte y=0;  //Storage for ADC result high and low byte
word data[600]={0};  //Storage for approx 6us of data
float voltage = 0;


void setup() {
  Serial.begin(115200);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); //Set for 8Mhz SPI (can be faster if using Due)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  pinMode(CONVST, OUTPUT);
  pinMode(RD, OUTPUT);
  pinMode(BUSY, INPUT);
  digitalWrite(CONVST, LOW);  //Set CONVST low by default
  digitalWrite(RD, LOW);  //Set and keep RD (SS) low
}


void loop() {
  //record 600 ADC samples
  for (int i=0;i<600;i++){
    x=SPI.transfer(B10001000);      // Single ended input at channel 0, 0V to 5V range (See LTC185x datasheet p15-16)
    y=SPI.transfer(B00000000);      // Filler
  
    delayMicroseconds(2);  //This delay is necessary to meet Tacq time (see datasheet p5). Change to 3us if voltage reads low
    
    //Trigger a conversion with a fast pulse
    noInterrupts();
    #if defined (__SAM3X8E__) // Arduino Due compatible
      REG_PIOC_SODR |= (0x01 << 22);
      REG_PIOC_CODR |= (0x01 << 22);
    #else
      PORTB |= 0x01;
      PORTB &= ~0x01;
    #endif
    interrupts();
    
    //Wait for conversion to be finished
    delayMicroseconds(4);
    
    //store ADC result as a word
    data[i]=word(x,y);
  }
  
  //Print results of 600 samples
  Serial.print("start data dump");
  Serial.print("\n"); 
  
  for (int i=0;i<600;i++){
    //convert to voltage (will need changed for bipolar or 0 to 10V)
    voltage = (float)data[i];
    voltage = voltage/65534*5;
    
    Serial.print(voltage,5);
    Serial.print("\n"); 
  }
      
  Serial.print("end data dump");
  Serial.print("\n"); 
}  
