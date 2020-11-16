#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "DataTypes.h"
#include "HeatingScheduler.h"

#define HEADER_SIZE 10

#pragma pack(push, 1)
struct BHeader
{
	uint8_t  DataSize = 0;				// [DATA_SIZE]    [*]    1 byte  => UserData size in bytes
	uint32_t Id = _ID_;					// [ID]           [****] 4 bytes => Unique 32 bits ID
	uint8_t  User = 0;					// [USERS]		  [*]    1 byte  => enum Users
	uint16_t Route = 0;					// [ROUTE]        [**]   2 byte  => enum Route
	Opcode   OpCode = Opcode::CMD_NONE;	// [OPCODE]		  [*]    1 byte  => enum MCMD opcode
	uint8_t  FullSize = 0;				// [FULL_SIZE]	  [*]    1 byte  => Full size in bytes

	BHeader(
		Opcode opcode, 
		uint8_t dataSize) :
		OpCode(opcode),
		DataSize(dataSize),
		FullSize(dataSize + sizeof(BHeader))
	{ }
	
	//BHeader(
	//	Opcode opcode) :
	//	OpCode(opcode),
	//	DataSize(0),
	//	FullSize(sizeof(BHeader))
	//{ }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BCommand
{
	BHeader Header;
	uint8_t Command = 0;

	BCommand(
		uint8_t command, 
		Opcode opcode) :
		Header(opcode, 1),
		Command(command)
	{ }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BTelemetry
{
	BHeader Header;
	uint16_t Flags = 0;
	uint8_t HeatDestTemp = 0;
	uint16_t LeftPauseMinutes = 0;
	uint8_t Left = 0;
	uint32_t TemperatureMeasurement = 0;
	DateTime Time;

	BTelemetry(
		uint16_t  flags,
		uint8_t   heatDestTemp,
		uint16_t  leftPauseMinutes,
		uint8_t   left,
		uint32_t  temperatureMeasurement,
		DateTime& time) : 
		Header(Opcode::CMD_RT, sizeof(DateTime) + 10),
		Flags(flags),
		HeatDestTemp(heatDestTemp),
		LeftPauseMinutes(leftPauseMinutes),
		Left(left),
		TemperatureMeasurement(temperatureMeasurement),
		Time(time)
	{ }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BHistTemp
{
	BHeader Header;
	uint32_t TemperatureHistory[TEMP_HIST_DEEP];

	BHistTemp(
		const uint32_t *temperatureHistory, 
		uint8_t length) :
		Header(Opcode::CMD_HT, length * sizeof(uint32_t))
	{
		memset(TemperatureHistory, 0, TEMP_HIST_DEEP * sizeof(uint32_t));
		memcpy(TemperatureHistory, temperatureHistory, Header.DataSize);
	}
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BHistHeat
{
	BHeader Header;
	HistDateTime HeatHistory[HEAT_HIST_DEEP];

	BHistHeat(
		const HistDateTime *heatHistory, 
		uint8_t length) :
		Header(Opcode::CMD_HH, length * sizeof(HistDateTime))
	{
		memset(HeatHistory, 0, HEAT_HIST_DEEP * sizeof(HistDateTime));
		memcpy(HeatHistory, heatHistory, Header.DataSize);
	}
};
#pragma pack(pop)

#pragma pack(push, 1)
struct BSchedule
{
	BHeader Header;
	ScheduleElm Schedulers[NUM_SCHEDULES];

	// Approve Schedules are received
	BSchedule() :
		Header(Opcode::CMD_AS, 0)
	{ }

	// Get All Schedules
	BSchedule(const HeatJob* schedulers) :
		Header(Opcode::CMD_GS, NUM_SCHEDULES * sizeof(ScheduleElm))
	{
		for (size_t i = 0; i < NUM_SCHEDULES; ++i) {
			memcpy(&Schedulers[i], &(schedulers + i)->Schedule, sizeof(ScheduleElm));
		}		
	}

};
#pragma pack(pop)

