/*
 ExtendedADCShield.h - Library for using Mayhew Labs' Extended ADC Shield.
 Created by Mark Mayhew, June 8, 2015.
 Released into the public domain.
 */
#ifndef ExtendedADCShield_h
#define ExtendedADCShield_h

#include "Arduino.h"

#define SINGLE_ENDED 0
#define DIFFERENTIAL 1 
#define UNIPOLAR 0 
#define BIPOLAR 1 
#define RANGE5V 0 
#define RANGE10V 1 


class ExtendedADCShield
{
public:
    ExtendedADCShield(byte number_bits);
    ExtendedADCShield(byte CONVST, byte RD, byte BUSY, byte number_bits);
	float analogReadConfigNext(byte channel, byte sgl_diff, byte uni_bipolar, byte range);
   
private:
    byte buildCommand(byte channel, byte sgl_diff, byte uni_bipolar, byte range);
	word sendSetupGetData(byte command);
	byte _BUSY, _CONVST, _RD, _NUMBER_BITS;
	byte _LAST_UNI_BIPOLAR, _LAST_RANGE;  
};

#endif
