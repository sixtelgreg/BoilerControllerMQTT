#ifndef PEAK_CONTROL_H
#define PEAK_CONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
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
