#include "PeakControl.h"


PeakControl::PeakControl(byte pinFlowInp, byte pinFlowOut)
{
	pinMode(pinFlowInp, INPUT);
	digitalWrite(pinFlowInp, HIGH);

	pinMode(pinFlowOut, INPUT);
	digitalWrite(pinFlowOut, HIGH);

	PinFlowInp = pinFlowInp;
	PinFlowOut = pinFlowOut;
	Rst();
}

void PeakControl::Rst()
{
	InpPeaks = 0u;
	OutPeaks = 0u;

	InpState = HIGH;
	OutState = HIGH;
}

void PeakControl::Run()
{
	byte inpCurr = digitalRead(PinFlowInp);
	byte outCurr = digitalRead(PinFlowOut);

	if (inpCurr != InpState)
	{
		InpState = inpCurr;
		// Configured to increase on a FALLING state change 
		// (transition from HIGH state to LOW state)
		if (LOW == InpState)
			{ ++InpPeaks; }
	}

	if (outCurr != OutState)
	{
		OutState = outCurr;
		// Configured to increase on a FALLING state change 
		// (transition from HIGH state to LOW state)
		if (LOW == OutState)
			{ ++OutPeaks; }
	}
}

// Returns true if peaks counts are the same or not more than 
// inaccuracyPercents Delta
bool PeakControl::Compare(byte inaccuracyPercents)
{
	auto maxPeaks = max(InpPeaks, OutPeaks);
	auto minPeaks = min(InpPeaks, OutPeaks);
	auto delta = ((maxPeaks * inaccuracyPercents) / 100u);
	return ((maxPeaks - minPeaks) < delta);
}

