//This example shows how to use the Extended ADC Shield for a variety of analog inputs

#include <ExtendedADCShield.h>
#include <SPI.h>

//Initialize the ADC Shield with default pins and 16-bit ADC (LTC1859)
ExtendedADCShield extendedADCShield(16);

float ch0, ch1, ch2m3, ch4, ch5, ch7m6;


void setup() {
  Serial.begin(115200);
  //SPI.begin must be called here on Due only
  SPI.begin();
  //Throw out first read (junk data) and configure input 0 as single ended bipolar -5 to +5V
  extendedADCShield.analogReadConfigNext(0, SINGLE_ENDED, BIPOLAR, RANGE10V);
}


void loop() {
  //Read input 0, set up input 1 as single ended unipoloar 0 to 5V
  ch0 =  extendedADCShield.analogReadConfigNext(1, SINGLE_ENDED, UNIPOLAR, RANGE5V);
  
  //Read input 1, set up input 2 and 3 as differential (input 2 minus input 3) unipolar 0 to 5V
  ch1 =  extendedADCShield.analogReadConfigNext(2, DIFFERENTIAL, UNIPOLAR, RANGE5V);
  
  //Read input 2 minus 3, set up input 4 as single ended bipolar 0 to 10V
  ch2m3 =  extendedADCShield.analogReadConfigNext(4, SINGLE_ENDED, BIPOLAR, RANGE10V);
  
  //Read input 4, set up input 5 as single ended bipolar 0 to 5V
  ch4 =  extendedADCShield.analogReadConfigNext(5, SINGLE_ENDED, BIPOLAR, RANGE5V);
  
  //Read input 5, set up input 6 and 7 as differential (input 7 minus input 6) bipolar 0 to 10V
  ch5 =  extendedADCShield.analogReadConfigNext(7, DIFFERENTIAL, BIPOLAR, RANGE10V);
  
  //Read input 7 minus 6, set up input 0 as single ended bipolar -5 to +5V
  ch7m6 =  extendedADCShield.analogReadConfigNext(0, SINGLE_ENDED, BIPOLAR, RANGE10V);
  
  //Print results with 5 decimal places of precision
  Serial.print(ch0,5);  
  Serial.print("\t"); 
  Serial.print(ch1,5);  
  Serial.print("\t"); 
  Serial.print(ch2m3,5);  
  Serial.print("\t"); 
  Serial.print(ch4,5);  
  Serial.print("\t"); 
  Serial.print(ch5,5);  
  Serial.print("\t"); 
  Serial.print(ch7m6,5);  
  Serial.print("\n"); 
  delay(1000);
}  
  
