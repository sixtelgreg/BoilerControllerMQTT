#ifndef PeakControl_h
#define PeakControl_h

#if ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

class PeakControl
{
private:
	byte InpState;
	byte OutState;

	byte PinFlowInp; 
	byte PinFlowOut;

	uint16_t InpPeaks;
	uint16_t OutPeaks;

public:
	PeakControl(byte pinFlowInp, byte pinFlowOut);
	void Run();
	void Rst();
	bool Compare(byte inaccuracyPercents);

};
#endif
