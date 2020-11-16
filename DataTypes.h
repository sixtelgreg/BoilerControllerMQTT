#ifndef _DataTypes_h
#define _DataTypes_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifdef DBG
#ifndef Dbg 
#define Dbg Serial
#endif
#endif // DBG

#ifndef MAKEINT64 // Pointer to X go to UINT64
#define MAKEINT64(pV) (*((uint64_t*)((void*)(pV))))
#endif // !MAKEINT64

#ifndef AVERAGE
#define AVERAGE(a, b) ((uint8_t)(((((float)(a)) + ((float)(b))) / 2.0f) + 0.5f))
#endif // !AVERAGE

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif // !MIN

#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif // !MAX

#define _ID_ 0x799E07EDU
#define _DATA_SIZE_INDEX 0U		// 1 bytes
#define _ID_INDEX 1U			// 4 bytes
#define _USERS_INDEX 5U			// 1 byte
#define _ROUTE_INDEX 6U			// 2 bytes
#define _OPCODE_INDEX 8U		// 1 bytes
#define _SPARE_INDEX 9U			// 1 bytes
#define _PROTOCOL_HEADER_SZ 10U // Communication Protocol header size
#define _DATA_START_INDEX _PROTOCOL_HEADER_SZ

#define _DATA_FLAGS_SZ 2U
#define _DATA_SENSORS_SZ 4U
#define _DATA_TIMES_SEC_SZ 4U

#define SENSORS 4U
#define UNKNOWN_TEMPERATURE 4U  //
#define UNKNOWN_TEMPERATURE_THRSH 10U  //
#define TEMP_HI_LEVEL_DEGREE 68U // Overheating Celsium
#define MAX_AUTOMATIC_TEMP 60U // Celsium
#define MAX_MANUAL_TEMP 55U // Celsium
#define MAX_MANUAL_HEATING_TIME 120 // minuts
#define TEMP_HIST_DEEP 48U // Temperature history pool deep (4 hours, 5min resolution)
#define HEAT_HIST_DEEP 15U // Heating history pool deep
#define HEAT_MAX_PAUSE 25U // Heating MAX pause interval (hours)
#define BROADCAST_MAX_ARRAY_SIZE 64
#define COMMUNICATION_ARR_SIZE 250 //
#define COMMUNICATION_ARR_MAX_DATA_SIZE (COMMUNICATION_ARR_SIZE - _PROTOCOL_HEADER_SZ)

#define RTC_HEAT_ON  0x80000000U // Time tick bit says that Heating status is ON.
#define RTC_HEAT_MH  0x40000000U // Time tick bit says that Heating status is ON.

// If Flag is ON
#define IS_FG_ON(X, FG) (0 != ((FG) & (X)))
// If Flag is OFF
#define IS_FG_OFF(X, FG) (0 == ((FG) & (X)))
// SET flag
#define FG_SET(X, FG) ((FG) |= (X))
// RESET flag
#define FG_RST(X, FG) ((FG) &= ~(X))
// If condition C is TRUE, SET flag, otherwise RESET
#define RS_FG_IF(C, X, FG) ((C) ? FG_SET(X, FG) : FG_RST(X, FG))

#pragma region COMMAND
#define S_APP F("APP")
#define S_F1  F("??")
#define S_RT  F("RT")
#define S_BR  F("BR")
#define S_ST  F("ST")
#define S_GT  F("GT")
#define S_HT  F("HT")
#define S_HH  F("HH")
#define S_AH  F("AH")
#define S_MH  F("MH")
#define S_GS  F("GS")
#define S_US  F("US")
#define S_AS  F("AS")
#define S_RS  F("RS")
#define S_PH  F("PH")
#define S_PS  F("PS")
#define S_DS  F("DS")
#define S_UKN F("UNKN")
#pragma endregion

#pragma region STATE
#define S_OK  F("OK")
#define S_ERR F("ERR")
#define S_NO  F("NO")
#define S_YES F("YES")
#define S_IN  F("IN")
#define S_OUT F("OUT")
#define S_ON  F("ON")
#define S_OFF F("OFF")
#define S_NEW F("NEW")
#define S_MIN F("Min")
#pragma endregion
  
#pragma region WEEK
#define S_Ds F("--")
#define S_Su F("Su")
#define S_Mo F("Mo")
#define S_Tu F("Tu")
#define S_We F("We")
#define S_Th F("Th")
#define S_Fr F("Fr")
#define S_Sa F("Sa")
#define S_WW F("WW")
#define S_WE F("WE")
#define S_FW F("FW")
#pragma endregion

#pragma region YEAR
#define S_Jan F("Jan")
#define S_Feb F("Feb")
#define S_Mar F("Mar")
#define S_Apr F("Apr")
#define S_May F("May")
#define S_Jun F("Jun")
#define S_Jul F("Jul")
#define S_Aug F("Aug")
#define S_Sep F("Sep")
#define S_Oct F("Oct")
#define S_Nov F("Nov")
#define S_Dec F("Dec")  
#pragma endregion

#pragma region SYMBOL
#define S_POINT		F(".")
#define S_COMMA		F(",")
#define S_COLON		F(":")
#define S_MINUS		F("-")
#define S_MNMN		F("--")
#define S_POINT_SPC	F(". ")
#define S_COLON_SPC	F(": ")
#define S_MNSPC		F("- ")
#define S_SPC1		F(" ")
#define S_SPC2		F("  ")
#define S_SPC3		F("   ")
#define S_SPC4		F("    ")
#define S_SPC5		F("     ")
#define S_SPC6		F("      ")
#define S_ENDL		F("\n")
#define S_PAUSE		F("::")
#define S_0			F("0")
#define S_1			F("1")
#define S_O			F("O")
#define S_S			F("S")
#define S_H			F("H")
#define S_A			F("A")
#define S_T			F("T")
#define S_C			F("C")
#define S_P			F("P")
#define S_M			F("M")  
#define S_Y			F("Y") 
#define S_N			F("N")  
#define S_T_COLON	F("T:") 
#define S_H_COLON	F("H:")  
#define S_A_COLON	F("A:") 
#define S_M_COLON	F("M:") 
#define S_C_COLON	F("C:") 
#define S_P_COLON	F("P:") 
#pragma endregion


#pragma region Communication Protocol
// System Communication Protocol
//
//					 [------------------ H E A D E R -----------------]
// Protocol Fields=> [DATA_SIZE][ID][USERS][ROUTE][MESSAGE_NAME][SPARE] [...DATA...]
//
// [DATA_SIZE]    [*]    1 byte  => Data size in bytes
// [ID]           [****] 4 bytes => Unique 32 bits ID
// [USERS]		  [*]    1 byte  => enum Users
// [ROUTE]        [**]   2 byte  => enum Route
// [OPCODE]		  [*]    1 byte  => enum MCMD opcode
// [SPARE]		  [*]    1 byte  => Spare

enum Opcode : uint8_t
{
	CMD_NONE = 0,
	// OBSOLETE == CMD_F1 = 1, // Get Help.	 	Key: [??], Value: ------
	CMD_RT = 2, // Read Telemetry.	 			Key: [RT], Value: ------
	// OBSOLETE == CMD_BR = 3, // Broadcasting.	Key: [BR], Value: [TEMP][FLAGS]
	// OBSOLETE == CMD_ST = 4, // Set Time.		Key: [ST], Value: [Y,M,D,H,Min,WD]
	// OBSOLETE == CMD_GT = 5, // Get Time.		Key: [GT], Value: ------
	CMD_HT = 6, // Get Temp History.			Key: [HT], Value: ------
	CMD_HH = 7, // Get Heat History.			Key: [HH], Value: ------
	CMD_AH = 8, // Automatic Mode.				Key: [AH], Value: [ON,OFF]
	CMD_MH = 9, // Manual Heating.				Key: [MH], Value: [ON,OFF]
	CMD_GS = 10, // Get All Schedules.			Key: [GS], Value: ------
	CMD_US = 11, // Update Schedule.			Key: [US], Value: [DOW,Fr,To,Tmp] x 10
	CMD_AS = 12, // Approve Schedules Update.	Key: [AS], Value: ------		
	// CMD_RS = 13, // Remove Schedule.			Key: [RS], Value: ------		
	CMD_PH = 14, // Pause Heating.				Key: [PH], Value: [Interval(Hours)]
	// CMD_PS = 15, // Pause Schedule.			Key: [PS], Value: ------			
	CMD_DS = 16, // Set Default Schedules.		Key: [DS], Value: ------			
	CMD_GA = 17, // Get All Statuses.			Key: [GA], Value: ------			
};

//enum Route : uint16_t
//{
//	_TX_  = 0x0001, // Message is Response
//	UsrRx = 0x0002, // Request sent by User Terminal or Application
//	TRxTx = 0x0004, // Terminal turn RX -> TX
//	CRxTx = 0x0008, // Controll turn RX -> TX
//
//	TLgRx = 0x0010, // Use Terminal RxLogic
//	TLgTx = 0x0020, // Use Terminal TxLogic
//	CLgRx = 0x0040, // Use Controll RxLogic
//	CLgTx = 0x0080, // Use Controll TxLogic
//
//	TRxBt = 0x0100, // Terminal RX Communication using BT
//	TTxBt = 0x0200,	// Terminal TX Communication using BT
//	CRxBt = 0x0400,	// Controll RX Communication using BT
//	CTxBt = 0x0800,	// Controll TX Communication using BT
//
//	TRxRf = 0x1000, // Terminal RX Communication using RF
//	TTxRf = 0x2000, // Terminal TX Communication using RF
//	CRxRf = 0x4000, // Controll RX Communication using RF
//	CTxRf = 0x8000, // Controll TX Communication using RF
//
//	C_Broadcast = CRxTx | CTxRf | TLgTx,
//	//C_RQST_BT = UsrRx | CRxTx | CLgRx | CTxBt,
//	T_REQUEST = TLgRx | TRxRf | CLgRx | CRxTx | CTxRf | TLgTx,
//	T_RQST_MQTT = UsrRx | T_REQUEST,
//	//T_RQST_BT = UsrRx | T_REQUEST | TTxBt,
//};

//enum Users : uint8_t
//{
//	USER_1 = 0x01,
//	USER_2 = 0x02,
//	USER_3 = 0x04,
//	USER_All = 0x07,
//	TO_CONTROLLER = 0x80,
//};

enum HeatingTrigger : uint8_t
{
	UnknownReason,
	OnAutoTime,
	OffAutoTime,
	OffAutoMode,
	OffAutoTemp,
	OnManual,
	OffManual,
	OffManualTemp,
	OffManualLimitTimer,
	OnPause,
	OffPause,
	OffOverHeating,
};
#pragma endregion

class BinArray;
typedef void (*msg_callback)(BinArray *pArr);

enum Flags : uint16_t
{
	Empty          = 0x000, // Empty
	AutomaticMode  = 0x001, // Automatic Heating Mode is  ON(1)/OFF(0)
	ManualHeating  = 0x002, // Manual Heating status is ON(1)/OFF(0)
	InTimeRange    = 0x004, // In the Automatic Mode - In Time Range
	AutoHeating    = 0x008, // Auto Heating is ON(1)/OFF(0).
	HeatingPaused  = 0x010, // Heating is PAUSED. Manual or Automatic
	OverHeating    = 0x020, // Over Heating is detected
	SchedulePaused = 0x040, // Scheduler is PAUSED.
	SensorsFault   = 0x080, // Sensors reading problem
	DstTempReached = 0x100, // Destination temperature is reached. Use with flags ManualHeating or AutoHeating
	ProtectTimerOn = 0x200, // Manual heating Protection timer is IN Range
};

#define IS_FLG_AUTOMATIC_MODE_ON(FG)  IS_FG_ON(Flags::AutomaticMode, FG)
#define IS_FLG_AUTOMATIC_MODE_OFF(FG) IS_FG_OFF(Flags::AutomaticMode, FG)
#define SET_FLG_AUTOMATIC_MODE(FG)    FG_SET(Flags::AutomaticMode, FG)
#define RST_FLG_AUTOMATIC_MODE(FG)    FG_RST(Flags::AutomaticMode, FG)

#define IS_FLG_MANUAL_HEATING_ON(FG)  IS_FG_ON(Flags::ManualHeating, FG)
#define IS_FLG_MANUAL_HEATING_OFF(FG) IS_FG_OFF(Flags::ManualHeating, FG)
#define SET_FLG_MANUAL_HEATING(FG)    FG_SET(Flags::ManualHeating, FG)
#define RST_FLG_MANUAL_HEATING(FG)    FG_RST(Flags::ManualHeating, FG)

#define IS_FLG_IN_TIME_RANGE_IN(FG) ((IS_FG_ON(Flags::AutomaticMode, FG)) && (IS_FG_ON(Flags::InTimeRange, FG)))
#define SET_FLG_IN_TIME_RANGE(FG)   FG_SET(Flags::InTimeRange, FG)
#define RST_FLG_IN_TIME_RANGE(FG)   FG_RST(Flags::InTimeRange, FG)

#define IS_FLG_AUTO_HEATING_ON(FG)  IS_FG_ON(Flags::AutoHeating, FG)
#define IS_FLG_AUTO_HEATING_OFF(FG) IS_FG_OFF(Flags::AutoHeating, FG)
#define SET_FLG_AUTO_HEATING(FG)    FG_SET(Flags::AutoHeating, FG)
#define RST_FLG_AUTO_HEATING(FG)    FG_RST(Flags::AutoHeating, FG)

#define IS_FLG_HEATING_PAUSED_YES(FG) IS_FG_ON(Flags::HeatingPaused, FG) 
#define IS_FLG_HEATING_PAUSED_NO(FG)  IS_FG_OFF(Flags::HeatingPaused, FG)
#define SET_FLG_HEATING_PAUSE(FG)     FG_SET(Flags::HeatingPaused, FG)   
#define RST_FLG_HEATING_PAUSE(FG)     FG_RST(Flags::HeatingPaused, FG)   

#define IS_FLG_SCHEDULER_PAUSED_YES(FG) IS_FG_ON(Flags::SchedulePaused, FG) 
#define IS_FLG_SCHEDULER_PAUSED_NO(FG)  IS_FG_OFF(Flags::SchedulePaused, FG)
#define SET_FLG_SCHEDULER_PAUSE(FG)     FG_SET(Flags::SchedulePaused, FG)   
#define RST_FLG_SCHEDULER_PAUSE(FG)     FG_RST(Flags::SchedulePaused, FG)  

#define IS_FLG_HEATING_OVER_YES(FG) IS_FG_ON(Flags::OverHeating, FG) 
#define IS_FLG_HEATING_OVER_NO(FG)  IS_FG_OFF(Flags::OverHeating, FG)
#define SET_FLG_HEATING_OVER(FG)    FG_SET(Flags::OverHeating, FG)   
#define RST_FLG_HEATING_OVER(FG)    FG_RST(Flags::OverHeating, FG)   

#define IS_FLG_SENSORS_FAULT_YES(FG) IS_FG_ON(Flags::SensorsFault, FG) 
#define IS_FLG_SENSORS_FAULT_NO(FG)  IS_FG_OFF(Flags::SensorsFault, FG)
#define SET_FLG_SENSORS_FAULT(FG)    FG_SET(Flags::SensorsFault, FG)   
#define RST_FLG_SENSORS_FAULT(FG)    FG_RST(Flags::SensorsFault, FG)   

#define IS_FLG_DST_TEMP_REACHED_YES(FG)	IS_FG_ON(Flags::DstTempReached, FG) 
#define IS_FLG_DST_TEMP_REACHED_NO(FG)	IS_FG_OFF(Flags::DstTempReached, FG)
#define SET_FLG_DST_TEMP_REACHED(FG)	FG_SET(Flags::DstTempReached, FG)   
#define RST_FLG_DST_TEMP_REACHED(FG)	FG_RST(Flags::DstTempReached, FG)   
#define SR_FLG_DST_TEMP_REACHED(C, FG)	RS_FG_IF(C, Flags::DstTempReached, FG)   

#define IS_FLG_PROTECT_TIMER_YES(FG)	IS_FG_ON(Flags::ProtectTimerOn, FG) 
#define IS_FLG_PROTECT_TIMER_NO(FG)     IS_FG_OFF(Flags::ProtectTimerOn, FG)
#define SET_FLG_PROTECT_TIMER(FG)		FG_SET(Flags::ProtectTimerOn, FG)   
#define RST_FLG_PROTECT_TIMER(FG)		FG_RST(Flags::ProtectTimerOn, FG)   

//#define IS_FLG_BT_CONNECTED(FG) IS_FG_ON(Flags::BTConnected, FG) 
//#define SET_FLG_BT_CONNECTED_STT(C, FG) RS_FG_IF(C, Flags::BTConnected, FG)   

// Heating Status is ON(1)/OFF(0). SSR(SolidStateRelay) opened or closed. Manual or Automatic
#define IS_HEATING(FG) (IS_FG_ON((Flags::AutoHeating | Flags::ManualHeating), FG) && \
						IS_FLG_HEATING_PAUSED_NO(FG) && \
						IS_FLG_HEATING_OVER_NO(FG) && \
						IS_FLG_SENSORS_FAULT_NO(FG))


enum MMSG : uint8_t
{
	STR_NO = 20,
	STR_YES,
	STR_IN,
	STR_OUT,
	STR_ON,
	STR_OFF,
	STR_NEW,
	STR_COMMA,
	STR_POINT,
	STR_COLON,
	STR_MINUS,
	STR_MNMN,
	STR_MNSPC,
	STR_ENDL,
	STR_SPC1,
	STR_SPC2,
	STR_SPC3,
	STR_SPC4,
	STR_SPC5,
	STR_SPC6,

	STR_OK,
	STR_ERR,
	STR_MIN,
	STR_0,
	STR_S,
	STR_H,
	STR_A,
	STR_T,
	STR_C,
	STR_P,
	STR_M,
	STR_Y,
	STR_N,

#pragma region Week Days
	STR_WEEK,
	STR_Ds = STR_WEEK,
	STR_Su,
	STR_Mo,
	STR_Tu,
	STR_We,
	STR_Th,
	STR_Fr,
	STR_Sa,
	STR_WW, // WorkWeek
	STR_WE, // WeekEnd
	STR_FW, // FullWeek
#pragma endregion

#pragma region Calendar Month
	STR_MONTH,
	STR_Jan,
	STR_Feb,
	STR_Mar,
	STR_Apr,
	STR_May,
	STR_Jun,
	STR_Jul,
	STR_Aug,
	STR_Sep,
	STR_Oct,
	STR_Nov,
	STR_Dec,
#pragma endregion
};

enum BtSender : uint8_t
{
	BT_Empty       = 0, // Empty
	BT_Terminal    = 1, // Sent by BT Terminal
	BT_Application = 2, // Sent by BT Application
};

enum CommStt : uint8_t
{
	AvailEmpty     = 0x00, // 1000 0000
	AvailNotEnough = 0x01, // 0000 0001
	IdErr          = 0x02, // 0000 0010
	HeaderSizeErr  = 0x04, // 0000 0100
	DataSizeErr    = 0x08, // 0000 1000
	InvalidUser    = 0x0C, // 0000 1100
	UnknownData    = 0x10, // 0001 0000
	TooManyRdItr   = 0x14, // 0001 0100
	NoWholeMsgRead = 0x18, // 0001 1000
	Dummy2		   = 0x1C, // 0001 1100
	ReadFull       = 0x80, // 1000 0000
	ErrorMask      = 0x7C, // 0111 1100
};

// Request Mask
//#define IS_ROUTE_RX(HDR) (0 != (HDR[_ROUTE_INDEX] & (ROUTE::ROUTE_RX)))
//#define SET_ROUTE_RX(HDR) (HDR[_ROUTE_INDEX] |= (ROUTE::ROUTE_RX))
//#define CLR_ROUTE_RX(HDR) (HDR[_ROUTE_INDEX] &= ~(ROUTE::ROUTE_RX))
// Response Mask
//#define IS_ROUTE_TX(HDR) (0 != (HDR[_ROUTE_INDEX] & (ROUTE::ROUTE_TX)))
//#define SET_ROUTE_TX(HDR) (HDR[_ROUTE_INDEX] |= (ROUTE::ROUTE_TX))
//#define CLR_ROUTE_TX(HDR) (HDR[_ROUTE_INDEX] &= ~(ROUTE::ROUTE_TX))
// BT App Mask
//#define IS_ROUTE_BT_APP(HDR) (0 != (HDR[_ROUTE_INDEX] & (ROUTE::ROUTE_BT_APP)))
//#define SET_ROUTE_BT_APP(HDR) (HDR[_ROUTE_INDEX] |= (ROUTE::ROUTE_BT_APP))
//#define CLR_ROUTE_BT_APP(HDR) (HDR[_ROUTE_INDEX] &= ~(ROUTE::ROUTE_BT_APP))
// BT Terminal Mask
//#define IS_ROUTE_BT_TRM(HDR) (0 != (HDR[_ROUTE_INDEX] & (ROUTE::ROUTE_BT_TRM)))
//#define SET_ROUTE_BT_TRM(HDR) (HDR[_ROUTE_INDEX] |= (ROUTE::ROUTE_BT_TRM))
//#define CLR_ROUTE_BT_TRM(HDR) (HDR[_ROUTE_INDEX] &= ~(ROUTE::ROUTE_BT_TRM))
//== Need to response this request (someone is waiting for the requested data)
//== #define IS_ROUTE_NEED_TX(HDR) (0 != (HDR[_ROUTE_INDEX] & (ROUTE::ROUTE_NEED_TX)))
//== #define SET_ROUTE_NEED_TX(HDR) (HDR[_ROUTE_INDEX] |= (ROUTE::ROUTE_NEED_TX))
//== #define CLR_ROUTE_NEED_TX(HDR) (HDR[_ROUTE_INDEX] &= ~(ROUTE::ROUTE_NEED_TX))
// Message
//#define GET_MESSAGE(HDR) ((MCMD)(HDR[_MESSAGE_INDEX]))
//#define SET_MESSAGE(HDR, MSG) (HDR[_MESSAGE_INDEX] = (uint8_t)MSG)
// Data
//#define GET_DATA(MSG) (MSG + _DATA_START_INDEX)
//#define GET_DATA_SIZE(HDR) (min(HDR[_DATA_SIZE_INDEX], COMMUNICATION_ARR_MAX_DATA_SIZE))
//#define SET_DATA_SIZE(HDR, DSZ) (HDR[_DATA_SIZE_INDEX] = min(DSZ, COMMUNICATION_ARR_MAX_DATA_SIZE))

//#define SET_DATA_ANY(DT, IND, VAL, SZ) (memcpy(DT + IND, VAL, min(SZ, COMMUNICATION_ARR_MAX_DATA_SIZE - IND)))

//#define GET_DATA_ANY_BYTE(DT, IND) (DT[min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - 1)])
//#define SET_DATA_ANY_BYTE(DT, IND, VAL) (DT[min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - 1)] = VAL)

//#define GET_DATA_LONG(DT, IND) (*((uint32_t*)(DT + min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - 5))))
//#define SET_DATA_LONG(DT, IND, VAL) (*((uint32_t*)(DT + min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - 5))) = (uint32_t)VAL)

//#define GET_DATA_DATETIME(DT, IND) (*((DateTime*)(DT + min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - (1 + sizeof(DateTime))))))
//#define SET_DATA_DATETIME(DT, IND, VAL) (*((DateTime*)(DT + min(IND, COMMUNICATION_ARR_MAX_DATA_SIZE - (1 + sizeof(DateTime))))) = (DateTime)VAL)
// Users
//#define GET_USERS(HDR) ((USERS)(HDR[_USERS_INDEX]))
//#define SET_USERS(HDR, U) (HDR[_USERS_INDEX] = (uint8_t)U)
//#define CLR_USERS(HDR, U) (HDR[_USERS_INDEX] &= ~((uint8_t)U))
//#define IS_USER(HDR, U) (0 != (HDR[_USERS_INDEX] & (uint8_t)U))
//#define SET_TO_CONTROLLER(HDR) (HDR[_USERS_INDEX] = (uint8_t)(USERS::TO_CONTROLLER))
//#define CLR_TO_CONTROLLER(HDR) (HDR[_USERS_INDEX] &= ~(USERS::TO_CONTROLLER))
//#define IS_TO_CONTROLLER(HDR) (0 != (HDR[_USERS_INDEX] & (uint8_t)(USERS::TO_CONTROLLER)))
// ID
//#define SET_ID(HDR) (*((uint32_t*)HDR) = _ID_)
//#define IS_ID(HDR) (_ID_ == *((uint32_t*)HDR))
// Clear
//#define MSG_CLEAR_ALL(MSG) (memset(MSG, 0, COMMUNICATION_ARR_SIZE))
//#define MSG_CLEAR_HEADER(MSG) (memset(MSG, 0, _PROTOCOL_HEADER_SZ))
//#define MSG_CLEAR_DATA(MSG) (memset(MSG + _DATA_START_INDEX, 0, COMMUNICATION_ARR_MAX_DATA_SIZE))

//#pragma endregion

#endif