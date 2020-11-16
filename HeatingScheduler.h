#ifndef HeatingSchedulerClass
#define HeatingSchedulerClass

#if ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

//#define SCHEDULES_IN_ROM 233456789U
#define SCHEDULER_CODE 233456789U
#define NUM_SCHEDULES  10
#define HYSTERESIS_HI  2
#define HYSTERESIS_LOW 0

#define DOW_NO 0x00 // 0b00000000  // Empty
#define DOW_SU 0x01 // 0b00000001
#define DOW_MO 0x02 // 0b00000010
#define DOW_TU 0x04 // 0b00000100
#define DOW_WE 0x08 // 0b00001000
#define DOW_TH 0x10 // 0b00010000
#define DOW_FR 0x20 // 0b00100000
#define DOW_SA 0x40 // 0b01000000   
#define DOW_WW 0x1F // 0b00011111  // Work week mask
#define DOW_WK 0x60 // 0b01100000  // Weekend mask
#define DOW_FW 0x7F // 0b01111111  // Full Week mask
#define DOW_ON 0x80 // 0b10000000  // Activate/Deactivate flag

enum eSchFlag : uint8_t
{
	Heating = 0x02,
	OnTime  = 0x04,
	Paused  = 0x08,
};

//#define HYSTERESIS(temp,perc) ((uint8_t)((((float)temp/100.0F) * ((float)perc)) + 0.5F))
#pragma pack(push, 1)
struct ScheduleElm
{
	// Temperature threshold parameter.
	// Average value between 2 highest sensor values.
	uint8_t TemperatureAvg = 0;

	// Day of week (DOW) is the bit masked 1 - 7 day of week. Allows few day collection by the OR
	uint8_t DOW = DOW_NO;
	uint8_t TimeFr = 0U;
	uint8_t TimeTo = 0U;

	bool IsFree()
	{
		return (0 == (DOW & DOW_FW));
	}
	
	bool IsDow(uint8_t dow)
	{
		return (0 != (dow & DOW));
	}

	bool IsEqual(ScheduleElm& other)
	{
		return
			other.TemperatureAvg == TemperatureAvg &&
			other.DOW == DOW &&
			other.TimeFr == TimeFr &&
			other.TimeTo == TimeTo;
	}
	
	bool IsDowRelevant(uint8_t dow)
	{
		return (IsActive() && IsDow(dow));
	}

	void SetElm(ScheduleElm& elm)
	{
		TemperatureAvg = elm.TemperatureAvg;
		DOW = elm.DOW;
		TimeFr = elm.TimeFr;
		TimeTo = elm.TimeTo;
	}

#pragma region Active
	bool IsActive()
	{
		return (!IsFree() && 0 != (DOW & DOW_ON));
	}

	void SetActive(bool set)
	{
		if (set)
		{
			if (!IsFree())
				{ DOW |= DOW_ON; }
		}
		else
			{ DOW &= ~DOW_ON; }
	}

	void SetActive()
	{
		if (IsFree())
			{ DOW = DOW_NO;  } // Full clear
		else
			{ DOW |= DOW_ON; } // Sure is active
		 
	}
#pragma endregion

#pragma region Relevants
	bool IsRelevantTime(uint8_t dow, uint8_t time)
	{
		return (
			IsDowRelevant(dow) &&
			TimeFr <= time &&
			time < TimeTo);
	}

	bool IsExactly(uint8_t timeFr, uint8_t timeTo, uint8_t tempAver)
	{
		return (
			TimeFr == timeFr &&
			TimeTo == timeTo &&
			TemperatureAvg == tempAver);
	}
#pragma endregion

	void AddDow(String& val);

};
#pragma pack(pop)

struct HeatJob
{
	ScheduleElm Schedule;
	uint8_t Flags = 0U;

	void ResetFlags()
	{
		Flags = 0;
	}

#pragma region Heating
	bool IsHeating()
	{
		return (0 != bitRead(Flags, eSchFlag::Heating));
	}

	void SetHeating(bool set)
	{
		bitWrite(Flags, eSchFlag::Heating, set);
	}
#pragma endregion

#pragma region OnTime
	bool IsOnTime()
	{
		return (0 != bitRead(Flags, eSchFlag::OnTime));
	}

	void SetOntime(bool set)
	{
		bitWrite(Flags, eSchFlag::OnTime, set);
	}
#pragma endregion

#pragma region Paused
	bool IsPaused()
	{
		return (0 != bitRead(Flags, eSchFlag::Paused));
	}

	void SetPaused(bool set)
	{
		bitWrite(Flags, eSchFlag::Paused, set);
	}
#pragma endregion
};


class HeatingScheduler
{
public:
	HeatingScheduler();

	bool Init(uint32_t schedulesCode/* = SCHEDULES_IN_ROM*/);
	bool IsEqual(uint8_t index, ScheduleElm& elm);
	uint8_t HeatingStatus(uint8_t wDay, uint8_t hour, uint8_t tempAvg);
	void CopySchedule(uint8_t *dst, uint8_t* scheduleSize);
	ScheduleElm& GetSchedule(uint8_t index);
	void SetSchedule(uint8_t index, ScheduleElm& elm);
	// Returns ActiveJob temperature if ActiveJob is null, returns 0,
	uint8_t GetActiveTemperature();
	void Pause(uint8_t pauses) { Pauses = pauses; } // 0- Stop pausing
	uint8_t LeftPauses() { return Pauses; }
	const HeatJob* GetJobs() const { return Schedules; } 
	uint8_t GetDow(String& dowStr);

private:
	HeatJob Schedules[NUM_SCHEDULES];
	HeatJob *ActiveJob = nullptr;
	uint8_t Pauses = 0;

private:
	// Returns found Heating Job if In Range, otherwise nullptr
	HeatJob* GetJobIfInRange(uint8_t dow, uint8_t hour);
	// Returns found nearest, but NOT InRange Heating Job if presents in this DOW, otherwise nullptr
	HeatJob* GetNearestJob(uint8_t dow, uint8_t hour);
	HeatJob* FindEmptySlot();

	bool __LoadEeprom(uint32_t schedulesCode);
	uint8_t __UpdateSchedule(uint8_t id, uint8_t wDay, uint8_t tmFr, uint8_t tmTo, uint8_t avgTemp);
	uint8_t __ActivateSchedule(uint8_t id, uint8_t activate = 1U);
	uint8_t __RemoveSchedule(uint8_t id);
	void __SetDefaultSchedules(uint32_t schedulesCode);
};

#endif // !HeatingSchedulerClass

