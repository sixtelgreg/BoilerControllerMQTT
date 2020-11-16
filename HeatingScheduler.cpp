#include "HeatingScheduler.h"
#include "DataTypes.h"
#include <EEPROM.h>

HeatingScheduler::HeatingScheduler()
{
	memset(Schedules, 0, sizeof(Schedules));
	ActiveJob = nullptr;
	Pauses = 0;
}

uint8_t HeatingScheduler::GetActiveTemperature()
{
	return ((nullptr == ActiveJob) ? 0 : ActiveJob->Schedule.TemperatureAvg);
}

bool HeatingScheduler::Init(uint32_t schedulesCode)
{
	//==return LoadEeprom(schedulesCode);
	return true;
}

HeatJob* HeatingScheduler::FindEmptySlot()
{
	HeatJob* foundJob = nullptr;

	for (auto& job : Schedules)
	{
		// New Job is found
		if (job.Schedule.IsFree())
		{
			job.SetHeating(false);
			job.Schedule.SetActive(false);
			foundJob = &job;
			break;
		}
	}

	return foundJob;
}

void HeatingScheduler::CopySchedule(uint8_t *dst, uint8_t* scheduleSize)
{
	*scheduleSize = NUM_SCHEDULES * sizeof(ScheduleElm);
	auto dst32 = (uint32_t*)dst;

	for (int id = 0; id < NUM_SCHEDULES; ++id)
	{
		auto jbPtr = (uint32_t*)((ScheduleElm*)(Schedules + id));
		dst32[id] = *jbPtr;
	}
}

ScheduleElm& HeatingScheduler::GetSchedule(uint8_t index)
{
	auto i = MIN(NUM_SCHEDULES - 1, index);
	return Schedules[i].Schedule;
}

bool HeatingScheduler::IsEqual(uint8_t index, ScheduleElm& elm)
{
	auto ret = false;
	if (index < NUM_SCHEDULES) {
		ret = Schedules[index].Schedule.IsEqual(elm);
	}
	return ret;
}

void HeatingScheduler::SetSchedule(uint8_t index, ScheduleElm& elm)
{
	if (index < NUM_SCHEDULES) {
		Schedules[index].Schedule.SetElm(elm);
	}
}

// Returns found Heating Job if In Range, otherwise nullptr
HeatJob* HeatingScheduler::GetJobIfInRange(uint8_t dow, uint8_t hour)
{
	HeatJob* foundJob = nullptr;

	for (auto& job : Schedules)
	{
		// Relevant Job is found
		if (job.Schedule.IsRelevantTime(dow, hour))
		{
			foundJob = &job;
			break;
		}
	}

	return foundJob;
}

// Returns found nearest, but NOT InRange Heating Job if presents in this DOW, otherwise nullptr
// Restriction - Looking for the job inside a current day only :-(
HeatJob* HeatingScheduler::GetNearestJob(uint8_t dow, uint8_t hour)
{
	HeatJob* foundJob = nullptr;
	uint8_t foundDelta = 0xFF; // Initialize any big number

	for (auto& job : Schedules)
	{
		// Relevant Job is found
		if (job.Schedule.IsDowRelevant(dow))
		{
			// Checking if time Now not after TimeFrom of the job
			if (job.Schedule.TimeFr > hour)
			{
				// If current delta less than the once found delta, choose current Job
				uint8_t currDelta = job.Schedule.TimeFr - hour;
				if (currDelta < foundDelta)
				{
					foundDelta = currDelta;
					foundJob = &job;
				}
			}
		}
	}

	return foundJob;
}

// Return status is 2 bit flags HeatingTime, HeatingOnOffStt
// TIME- 0x2, HEAT- 0x1
uint8_t HeatingScheduler::HeatingStatus(uint8_t wDay, uint8_t hour, uint8_t temperature)
{
	uint8_t stt = 0;
	uint8_t dow = DOW_NO; // Clean dow bits.
	uint8_t dowBit = wDay - 1; // Set dow [0-6] as relevant bit
	bitSet(dow, dowBit);
#ifdef DBG_SCHEDULER
	Dbg.println();
	Dbg.print(F("HeatingStatus: Inp Wday: "));
	Dbg.print(wDay);
	Dbg.print(F(" Calc DOW: "));
	Dbg.println(dow);
#endif // DBG

	// Scheduler found an job
	if (nullptr != ActiveJob)
	{
		if (!ActiveJob->Schedule.IsRelevantTime(dow, hour))
		{
			// Reset parameters before cancelling job.
			ActiveJob->ResetFlags();
			
			if (Pauses > 0) // decrease Pauses counter if relevants
				{ --Pauses; }
			ActiveJob = nullptr;
		}
	}
	else
	{
		// Try to find job if not presents.
		ActiveJob = GetJobIfInRange(dow, hour);
	}

	// if Job is found, check temperature status
	if (nullptr != ActiveJob)
	{
#ifdef DBG_SCHEDULER
		Dbg.print(F("HeatingStatus: ActiveJob DOW: "));
		Dbg.println((DOW_FW & ActiveJob->DOW));
#endif // DBG

		// Heating is working, temperature is raising
		if (ActiveJob->IsHeating())
		{
			auto highHystThr = ActiveJob->Schedule.TemperatureAvg + HYSTERESIS_HI;
			// If current temperature is bigger than Schedule temperature 
			// plus HYSTERESIS_HI, set heating flag to OFF (RST)
			ActiveJob->SetHeating(temperature < highHystThr);
		}
		else
		{
			auto lowHystThr = ActiveJob->Schedule.TemperatureAvg - HYSTERESIS_LOW;
			// If current temperature is lower than Schedule temperature
			// minus HYSTERESIS_LOW, set heating flag to ON (SET)
			ActiveJob->SetHeating(temperature < lowHystThr);
		}
	}
#ifdef DBG_SCHEDULER
	else
	{
		Dbg.print(F("HeatingStatus: Relewant Job DOW: "));
		Dbg.print(dow);
		Dbg.println (F(" NOT FOUND"));
	}
#endif // DBG

	uint8_t retFlg = ((nullptr != ActiveJob) ? eSchFlag::OnTime : 0);
	retFlg |= ((nullptr != ActiveJob && ActiveJob->IsHeating()) ? eSchFlag::Heating : 0);
	retFlg |= ((0 != Pauses) ? eSchFlag::Paused : 0);
	
	return retFlg;
}

void HeatingScheduler::__SetDefaultSchedules(uint32_t schedulesCode)
{
	// WW Before Morning
	Schedules[0].Schedule.DOW = (DOW_WW | DOW_ON); // 0b10011111
	Schedules[0].Schedule.TimeFr = 3U;
	Schedules[0].Schedule.TimeTo = 4U;
	Schedules[0].Schedule.TemperatureAvg = 33U;
	Schedules[0].SetHeating(false);

	// WW Morning
	Schedules[1].Schedule.DOW = (DOW_WW | DOW_ON); // 0b10011111
	Schedules[1].Schedule.TimeFr = 5U;
	Schedules[1].Schedule.TimeTo = 8U;
	Schedules[1].Schedule.TemperatureAvg = 39U;
	Schedules[1].SetHeating(false);

	// WW Evening
	Schedules[2].Schedule.DOW = (DOW_WW | DOW_ON); // 0b10011111
	Schedules[2].Schedule.TimeFr = 19U;
	Schedules[2].Schedule.TimeTo = 23U;
	Schedules[2].Schedule.TemperatureAvg = 42U;
	Schedules[2].SetHeating(false);

	// Fr and Sa
	Schedules[3].Schedule.DOW = (DOW_WK | DOW_ON); // 0b11100000;
	Schedules[3].Schedule.TimeFr = 9U;
	Schedules[3].Schedule.TimeTo = 19U;
	Schedules[3].Schedule.TemperatureAvg = 39U;
	Schedules[3].SetHeating(false);

	// Fr and Sa
	Schedules[4].Schedule.DOW = (DOW_WK | DOW_ON); // 0b11100000;
	Schedules[4].Schedule.TimeFr = 19U;
	Schedules[4].Schedule.TimeTo = 23U;
	Schedules[4].Schedule.TemperatureAvg = 43U;
	Schedules[4].SetHeating(false);

	// User1
	Schedules[5].Schedule.DOW = DOW_NO;
	Schedules[5].Schedule.TimeFr = 6U;
	Schedules[5].Schedule.TimeTo = 23U;
	Schedules[5].Schedule.TemperatureAvg = 40U;
	Schedules[5].SetHeating(false);

	// User2
	Schedules[6].Schedule.DOW = DOW_NO;
	Schedules[6].Schedule.TimeFr = 6U;
	Schedules[6].Schedule.TimeTo = 23U;
	Schedules[6].Schedule.TemperatureAvg = 40U;
	Schedules[6].SetHeating(false);

	// User3
	Schedules[7].Schedule.DOW = DOW_NO;
	Schedules[7].Schedule.TimeFr = 6U;
	Schedules[7].Schedule.TimeTo = 23U;
	Schedules[7].Schedule.TemperatureAvg = 40U;
	Schedules[7].SetHeating(false);

	// User4
	Schedules[8].Schedule.DOW = DOW_NO;
	Schedules[8].Schedule.TimeFr = 6U;
	Schedules[8].Schedule.TimeTo = 23U;
	Schedules[8].Schedule.TemperatureAvg = 40U;
	Schedules[8].SetHeating(false);

	// User5
	Schedules[9].Schedule.DOW = DOW_NO;
	Schedules[9].Schedule.TimeFr = 6U;
	Schedules[9].Schedule.TimeTo = 23U;
	Schedules[9].Schedule.TemperatureAvg = 40U;
	Schedules[9].SetHeating(false);

	// Save Schedules code first
	//--eeprom_update_dword((uint32_t*)(0), schedulesCode);
	EEPROM.writeUInt(0, schedulesCode);
	for (int id = 0; id < NUM_SCHEDULES; ++id)
	{
		auto addr = 4 + id * 4;
		auto jbPtr = (uint32_t*)((ScheduleElm*)(Schedules + id));
		//--eeprom_update_dword((uint32_t*)(4 + id * 4), *jbPtr);
		EEPROM.writeUInt(addr, *jbPtr);
	}
}

bool HeatingScheduler::__LoadEeprom(uint32_t schedulesCode)
{
	auto loadedExisting = true;
	//auto schedulesCodeFound = eeprom_read_dword((uint32_t*)(0));
	auto schedulesCodeFound = EEPROM.readUInt(0);
	// Checking the sheduler code. If does not present, may be it first time run
	// Need to initialize the default shedule values.
	if (schedulesCode != schedulesCodeFound)
	{
		__SetDefaultSchedules(schedulesCode); 
		loadedExisting = false;
	}
	else
	{
		for (int id = 0; id < NUM_SCHEDULES; ++id)
		{
			auto jbPtr = (uint32_t*)((ScheduleElm*)(Schedules + id));
			//*jbPtr = eeprom_read_dword((uint32_t*)(4 + id * 4));
			auto addr = 4 + id * 4;
			*jbPtr = EEPROM.readUInt(addr);
			Schedules[id].Schedule.SetActive();
		}
	}
	return loadedExisting;
}

// Returns Found Relevant Job ID
uint8_t HeatingScheduler::__UpdateSchedule(
	uint8_t id,     // ONE based ID
	uint8_t wDay,   // DOW is 1 - 7, WW-8, WK-9, FW-10
	uint8_t tmFr, 
	uint8_t tmTo, 
	uint8_t avgTemp)
{
	if (0 == id || 0 == wDay || wDay > 10)
		{ return 0; }

	HeatJob* foundJob = nullptr;
	uint8_t relevantID = MIN((id - 1), NUM_SCHEDULES - 1);
	uint8_t foundId = 0;
	for (auto& job : Schedules)
	{
		// New Job is found
		if (job.Schedule.IsExactly(tmFr, tmTo, avgTemp))
		{
			foundJob = &job;
			relevantID = foundId;
			break;
		}
		++foundId;
	}

	if (nullptr == foundJob)
	{
		foundJob = &Schedules[relevantID];
		foundJob->Schedule.TimeFr = tmFr;
		foundJob->Schedule.TimeTo = tmTo;
		foundJob->Schedule.TemperatureAvg = avgTemp;
	}

	foundJob->Schedule.DOW = DOW_NO;    // Clean previous dow bit if exist.
	if (wDay > 0 && wDay < 8)
	{
		uint8_t dowBit = wDay - 1; // Set dow [0-6] as relevant bit
		bitSet(foundJob->Schedule.DOW, dowBit);
	}
	else if (8 == wDay) // Work week
	{
		foundJob->Schedule.DOW |= DOW_WW;
	}
	else if (9 == wDay) // Weekend
	{
		foundJob->Schedule.DOW |= DOW_WK;
	}
	else if (10 == wDay) // Full week
	{
		foundJob->Schedule.DOW |= DOW_FW;
	}
	else
	{ return 0; }

	foundJob->Schedule.DOW |= DOW_ON; // Set job as active
	auto jbPtr = (uint32_t*)((ScheduleElm*)foundJob);
	auto addr = 4 + relevantID * 4;
	//eeprom_update_dword((uint32_t*)(4 + relevantID * 4), *jbPtr);
	EEPROM.writeUInt(addr, *jbPtr);
	return relevantID;
}

// DOW is 1 - 7 range
// Returns Found Relevant Job ID
// ONE based ID
uint8_t HeatingScheduler::__ActivateSchedule(uint8_t id, uint8_t activate)
{
	if (0 == id)
		{ return 0; }

	uint8_t relevantID = MIN((id - 1), NUM_SCHEDULES - 1);
	auto hj = (Schedules + relevantID);
	hj->Schedule.SetActive(0 != activate);
	return relevantID;
}

// Returns Found Relevant Job ID
// ONE based ID
uint8_t HeatingScheduler::__RemoveSchedule(uint8_t id)
{
	if (0 == id)
		{ return 0; }

	uint8_t relevantID = MIN((id - 1), NUM_SCHEDULES - 1);
	auto hj = (Schedules + relevantID);

	Schedules[relevantID].Schedule.DOW = DOW_NO;
	Schedules[relevantID].Schedule.TimeFr = 6U;
	Schedules[relevantID].Schedule.TimeTo = 23U;
	Schedules[relevantID].Schedule.TemperatureAvg = 40U;
	Schedules[relevantID].SetHeating(false);

	auto jbPtr = (uint32_t*)((ScheduleElm*)hj);
	//eeprom_update_dword((uint32_t*)(4 + relevantID * 4), *jbPtr);
	auto addr = 4 + relevantID * 4;
	EEPROM.writeUInt(addr, *jbPtr);

	return relevantID;
}

void ScheduleElm::AddDow(String& val)
{
	auto dow = (DOW & DOW_FW);
	switch (dow)
	{
	case DOW_NO: val += S_Ds; break;
	case DOW_SU: val += S_Su; break;
	case DOW_MO: val += S_Mo; break;
	case DOW_TU: val += S_Tu; break;
	case DOW_WE: val += S_We; break;
	case DOW_TH: val += S_Th; break;
	case DOW_FR: val += S_Fr; break;
	case DOW_SA: val += S_Sa; break;
	case DOW_WW: val += S_WW; break;
	case DOW_WK: val += S_WE; break;
	case DOW_FW: val += S_FW; break;
	default: break;
	}
}

uint8_t HeatingScheduler::GetDow(String& dowStr)
{
	uint8_t ret = DOW_NO;
	if (dowStr == "SU") {
		ret = DOW_SU;
	}
	else if (dowStr == "MO") {
		ret = DOW_MO;
	}
	else if (dowStr == "TU") {
		ret = DOW_TU;
	}
	else if (dowStr == "WE") {
		ret = DOW_WE;
	}
	else if (dowStr == "TH") {
		ret = DOW_TH;
	}
	else if (dowStr == "FR") {
		ret = DOW_FR;
	}
	else if (dowStr == "SA") {
		ret = DOW_SA;
	}
	else if (dowStr == "WW") {
		ret = DOW_WW;
	}
	else if (dowStr == "WK") {
		ret = DOW_WK;
	}
	else if (dowStr == "FW") {
		ret = DOW_FW;
	}

	return ret;
}