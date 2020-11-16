#ifndef _HIST_DATE_TIME_H_
#define _HIST_DATE_TIME_H_

#include "DateTime.h"
#include "DataTypes.h"

#pragma pack(push, 1)
class HistDateTime
{
public:
	HistDateTime() :
		DtOn(),
		DtOff(),
		ReasonOn(HeatingTrigger::UnknownReason),
		ReasonOff(HeatingTrigger::UnknownReason)
	{ }

	bool IsValid()
	{
		return (HeatingTrigger::UnknownReason != ReasonOn);
	}

	String& GetReason(bool on)
	{
		static String val("                             ");
		auto reason = on ? ReasonOn : ReasonOff;

		switch (reason)
		{
		case OnAutoTime:
			val = F("OnAutoTime");
			break;
		case OffAutoTime:
			val = F("OffAutoTime");
			break;
		case OffAutoTemp:
			val = F("OffAutoTemp");
			break;
		case OffAutoMode:
			val = F("OffAutoMode");
			break;
		case OnManual:
			val = F("OnManual");
			break;
		case OffManual:
			val = F("OffManual");
			break;
		case OffManualTemp:
			val = F("OffManualTemp");
			break;
		case OffManualLimitTimer:
			val = F("OffManualTimer");
			break;
		case OnPause:
			val = F("OnPause");
			break;
		case OffPause:
			val = F("OffPause");
			break;
		case OffOverHeating:
			val = F("OffOverHeating");
			break;
		default:
			val = F("UnknownReason");
			break;
		}

		return val;
	}

	//HistDateTime(bool on, DateTime& dt, HeatingTrigger reason = HeatingTrigger::UnknownReason)
	//{ 
	//	Update(on, dt, reason);
	//}

	HistDateTime(bool on, DateTime& dt, uint16_t flagCurr, uint16_t flagPrev)
	{
		Update(on, dt, flagCurr, flagPrev);
	}

	//void Update(bool on, DateTime& dt, HeatingTrigger reason = HeatingTrigger::UnknownReason)
	//{ 
	//	if (on)
	//	{
	//		DtOn = dt;
	//		ReasonOn = reason;
	//	}
	//	else
	//	{
	//		DtOff = dt;
	//		ReasonOff = reason;
	//	}
	//}

	void Update(bool on, DateTime& dt, uint16_t flagCurr, uint16_t flagPrev)
	{
		auto reason = HeatingTrigger::UnknownReason;

		// TODO: Here will implemented set reason by the flags states:
		//
		auto am1 = IS_FLG_AUTOMATIC_MODE_ON(flagCurr);
		auto it1 = IS_FLG_IN_TIME_RANGE_IN(flagCurr);
		auto ah1 = IS_FLG_AUTO_HEATING_ON(flagCurr);
		auto mh1 = IS_FLG_MANUAL_HEATING_ON(flagCurr);
		auto ps1 = IS_FLG_HEATING_PAUSED_YES(flagCurr);
		auto oh1 = IS_FLG_HEATING_OVER_YES(flagCurr);

		auto am0 = IS_FLG_AUTOMATIC_MODE_ON(flagPrev);
		auto it0 = IS_FLG_IN_TIME_RANGE_IN(flagPrev);
		auto ah0 = IS_FLG_AUTO_HEATING_ON(flagPrev);
		auto mh0 = IS_FLG_MANUAL_HEATING_ON(flagPrev);
		auto ps0 = IS_FLG_HEATING_PAUSED_YES(flagPrev);
		auto oh0 = IS_FLG_HEATING_OVER_YES(flagPrev);
		//
		//
		// ======================================================

		if (on)
		{
			if (mh1)
			{
				reason = HeatingTrigger::OnManual;
			}
			else if (it1)
			{
				reason = HeatingTrigger::OnAutoTime;
			}
			//else if (!ps)
			//{
			//	reason = HeatingTrigger::OffPause;
			//}

			DtOn = dt;
			ReasonOn = reason;
		}
		else
		{
			if (ps1)
			{
				reason = HeatingTrigger::OnPause;
			}
			else
			{
				if (HeatingTrigger::OnManual == ReasonOn)
				{
					if (oh1)
					{
						reason = HeatingTrigger::OffOverHeating;
					}
					else if (!mh1)
					{
						reason = HeatingTrigger::OffManual;
					}
					else
					{
						reason = HeatingTrigger::OffManualTemp;
					}
				}
				else if (HeatingTrigger::OnAutoTime == ReasonOn)
				{
					if (!it1)
					{
						reason = HeatingTrigger::OffAutoTime;
					}
					else if (!am1)
					{
						reason = HeatingTrigger::OffAutoMode;
					}
					else
					{
						reason = HeatingTrigger::OffAutoTemp;
					}
				}
			}

			DtOff = dt;
			ReasonOff = reason;
		}
	}

	HistDateTime(const HistDateTime& copy) :
		DtOn(copy.DtOn),
		DtOff(copy.DtOff),
		ReasonOn(copy.ReasonOn),
		ReasonOff(copy.ReasonOff)
	{ }

	DateTime DtOn;
	HeatingTrigger ReasonOn = HeatingTrigger::UnknownReason;

	DateTime DtOff;
	HeatingTrigger ReasonOff = HeatingTrigger::UnknownReason;
};
#pragma pack(pop)
#endif
