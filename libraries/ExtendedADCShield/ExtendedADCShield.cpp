/*
 ExtendedADCShield.cpp - Library for using Mayhew Labs' Extended ADC Shield.
 Created by Mark Mayhew, June 8, 2015.
 Released into the public domain.
 */
 
#include "Arduino.h"
#include "ExtendedADCShield.h"
#include <SPI.h>

//TODO change this to 
ExtendedADCShield::ExtendedADCShield(byte CONVST, byte RD, byte BUSY, byte number_bits)
{
	_BUSY = BUSY;
	_CONVST = CONVST;
	_RD = RD;
	_NUMBER_BITS = number_bits;
	
	pinMode(_BUSY, INPUT);
	pinMode(_CONVST, OUTPUT);
	pinMode(_RD, OUTPUT);
	
	//calling SPI.begin at this point doesn't work on Due. Instead, call it inside sketch's setup()
	#if not defined (__arm__) && not defined (__SAM3X8E__) // Arduino Due compatible
	SPI.begin();
	#endif
	
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	
	digitalWrite(_CONVST,LOW);
	digitalWrite(_RD, HIGH);
	
	_LAST_UNI_BIPOLAR = 0;
	_LAST_RANGE = 0;
}

ExtendedADCShield::ExtendedADCShield(byte number_bits)
{
	_BUSY = 9;
	_CONVST = 8;
	_RD = 10;
	_NUMBER_BITS = number_bits;
	
	pinMode(_BUSY, INPUT);
	pinMode(_CONVST, OUTPUT);
	pinMode(_RD, OUTPUT);
	
	//calling SPI.begin at this point doesn't work on Due. Instead, call it inside sketch's setup()
	#if not defined (__arm__) && not defined (__SAM3X8E__) // Arduino Due compatible
	SPI.begin();
	#endif

	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	
	digitalWrite(_CONVST,LOW);
	digitalWrite(_RD, HIGH);
	
	_LAST_UNI_BIPOLAR = 0;
	_LAST_RANGE = 0;
	
}

float ExtendedADCShield::analogReadConfigNext(byte channel, byte sgl_diff, byte uni_bipolar, byte range)
{
	byte command = 0;
	word adc_code = 0;
	float voltage = 0;
	float sign = 1;
	
	command = buildCommand(channel,sgl_diff,uni_bipolar,range);
	adc_code = sendSetupGetData(command);
	//TODO deal with adding 1 in the right place
	if(_LAST_UNI_BIPOLAR == BIPOLAR) {
		if ((adc_code & 0x8000) == 0x8000) {	//adc code is < 0
			adc_code = (adc_code ^ 0xFFFF)+(1<<(16-_NUMBER_BITS));   //Convert ADC code from two's complement to binary
			sign = -1;
		}
		adc_code = adc_code>>(16-_NUMBER_BITS);	//shift out zero bits (2 for 14-bit, 4 for 12-bit)
		voltage = sign*(float)adc_code;
		voltage = voltage / (pow(2,_NUMBER_BITS-1)-1);
	}
	else {
		adc_code = adc_code>>(16-_NUMBER_BITS);
		voltage = (float)adc_code;
		voltage = voltage / (pow(2,_NUMBER_BITS)-1);
	}
		
	switch (_LAST_RANGE) {
		case RANGE5V:
			voltage = voltage*5;
			break;
		case RANGE10V:
			voltage = voltage*10;
			break;
		default:
			break;
	}		
	
	_LAST_UNI_BIPOLAR = uni_bipolar;
	_LAST_RANGE = range;

	return voltage;
}
	
byte ExtendedADCShield::buildCommand(byte channel, byte sgl_diff, byte uni_bipolar, byte range)
{	
	byte command=0;
	
	//Build Command
	switch (channel) {
		case 1:
			command = command | B01000000;
			break;
		case 2:
			command = command | B00010000;
			break;
		case 3:
			command = command | B01010000;
			break;
		case 4:
			command = command | B00100000;
			break;
		case 5:
			command = command | B01100000;
			break;
		case 6:
			command = command | B00110000;
			break;
		case 7:
			command = command | B01110000;
			break;
		default:
			break;
	}
	
	if (sgl_diff==SINGLE_ENDED) {
		command = command | B10000000;
	}
		
	if (uni_bipolar==UNIPOLAR) {
		command = command | B00001000;
	}
	
	if (range == RANGE10V) {
		command = command | B00000100;
	}
	
	return command;
}	
word ExtendedADCShield::sendSetupGetData(byte command)	
{
	word conv_result = 0;
	
	//Trigger a conversion
	digitalWrite(_CONVST,HIGH);
	digitalWrite(_CONVST,LOW);
	
	//Wait for BUSY to go high 
	while(digitalRead(_BUSY)==LOW);
	
	//Set RD low
	digitalWrite(_RD, LOW);
	
	//Send command, get high byte of last conversion
	conv_result = SPI.transfer(command)<<8;
	
	//send filler, get low byte of last conversion
	conv_result = conv_result | SPI.transfer(B00000000);
	
	//Set RD high
	digitalWrite(_RD, HIGH);
	
	//return last conversion in raw word form
	return conv_result;
}