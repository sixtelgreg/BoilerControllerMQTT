#include "ToolStrings.h"
#include "DateTime.h"

const char* ToolStr::MessageReceivedOK(Opcode opcode)
{
	// Reserved capacity of 10 chars
	//"\nRT: OK\n\0"
	static String val("          "); 
	val += F("\n");
	val += ToolStr::CmdGet(opcode);
	val += F(": OK\n");
	return val.c_str();
}

const char* ToolStr::CmdGet(Opcode opcode)
{
	// Reserved capacity of 3 chars
	//"RT\0"
	static String val("   ");
	CmdGet(opcode, val);
	return val.c_str();
}

void ToolStr::CmdGet(Opcode opcode, String& val)
{
	// Reserved capacity of 3 chars
	//"RT\0"
	switch (opcode)
	{
	default:
		val = "";
		break;

	// OBSOLETE == case Opcode::CMD_F1: val = S_F1; break;
	case Opcode::CMD_RT: val = S_RT; break;
	// OBSOLETE == case Opcode::CMD_BR: val = S_BR; break;
	// OBSOLETE == case Opcode::CMD_ST: val = S_ST; break;
	// OBSOLETE == case Opcode::CMD_GT: val = S_GT; break;
	case Opcode::CMD_HT: val = S_HT; break;
	case Opcode::CMD_HH: val = S_HH; break;
	case Opcode::CMD_AH: val = S_AH; break;
	case Opcode::CMD_MH: val = S_MH; break;
	case Opcode::CMD_GS: val = S_GS; break;
	case Opcode::CMD_US: val = S_US; break;
	case Opcode::CMD_AS: val = S_AS; break;
	// case Opcode::CMD_RS: val = S_RS; break;
	case Opcode::CMD_PH: val = S_PH; break;
	case Opcode::CMD_DS: val = S_DS; break;
	}
}

const char* ToolStr::MsgGet(MMSG name)
{
	// Reserved capacity of 10 chars
	static String val("          ");
	MsgGet(name, val);
	return val.c_str();
}

void ToolStr::MsgGet(MMSG name, String& val)
{
	switch (name)
	{
#pragma region Status
	//case MMSG::WAIT_RESULT:	val = F("ACPT, wait...");	break;
	case MMSG::STR_OK:	val = S_OK;  break;
	case MMSG::STR_ERR:	val = S_ERR; break;
	case MMSG::STR_NO:	val = S_NO;  break;
	case MMSG::STR_YES:	val = S_YES; break;
	case MMSG::STR_IN:	val = S_IN;  break;
	case MMSG::STR_OUT:	val = S_OUT; break;
	case MMSG::STR_ON:	val = S_ON;  break;
	case MMSG::STR_OFF:	val = S_OFF; break;
	case MMSG::STR_NEW:	val = S_NEW; break;
	case MMSG::STR_MIN:	val = S_MIN; break;
#pragma endregion

#pragma region Symbols
	case MMSG::STR_POINT: S_POINT; break;
	case MMSG::STR_COMMA: S_COMMA; break;
	case MMSG::STR_COLON: S_COLON; break;
	case MMSG::STR_MINUS: S_MINUS; break;
	case MMSG::STR_MNMN:  S_MNMN;  break;
	case MMSG::STR_MNSPC: S_MNSPC; break;
	case MMSG::STR_SPC1:  S_SPC1;  break;
	case MMSG::STR_SPC2:  S_SPC2;  break;
	case MMSG::STR_SPC3:  S_SPC3;  break;
	case MMSG::STR_SPC4:  S_SPC4;  break;
	case MMSG::STR_SPC5:  S_SPC5;  break;
	case MMSG::STR_SPC6:  S_SPC6;  break;
	case MMSG::STR_ENDL:  S_ENDL;  break;
	case MMSG::STR_0:	  S_0;     break;
	case MMSG::STR_S:	  S_S;     break;
	case MMSG::STR_H:	  S_H;     break;
	case MMSG::STR_A:	  S_A;     break;
	case MMSG::STR_T:	  S_T;     break;
	case MMSG::STR_C:	  S_C;     break;
	case MMSG::STR_P:	  S_P;     break;
	case MMSG::STR_M:	  S_M;     break;
	case MMSG::STR_Y:	  S_Y;     break;
	case MMSG::STR_N:	  S_N;     break;
#pragma endregion

#pragma region Week
	case MMSG::STR_Ds: val = S_Ds; break;
	case MMSG::STR_Su: val = S_Su; break;
	case MMSG::STR_Mo: val = S_Mo; break;
	case MMSG::STR_Tu: val = S_Tu; break;
	case MMSG::STR_We: val = S_We; break;
	case MMSG::STR_Th: val = S_Th; break;
	case MMSG::STR_Fr: val = S_Fr; break;
	case MMSG::STR_Sa: val = S_Sa; break;
	case MMSG::STR_WW: val = S_WW; break;
	case MMSG::STR_WE: val = S_WE; break;
	case MMSG::STR_FW: val = S_FW; break;
#pragma endregion

#pragma region Year
	case MMSG::STR_Jan:	val = S_Jan; break;
	case MMSG::STR_Feb:	val = S_Feb; break;
	case MMSG::STR_Mar:	val = S_Mar; break;
	case MMSG::STR_Apr:	val = S_Apr; break;
	case MMSG::STR_May:	val = S_May; break;
	case MMSG::STR_Jun:	val = S_Jun; break;
	case MMSG::STR_Jul:	val = S_Jul; break;
	case MMSG::STR_Aug:	val = S_Aug; break;
	case MMSG::STR_Sep:	val = S_Sep; break;
	case MMSG::STR_Oct:	val = S_Oct; break;
	case MMSG::STR_Nov:	val = S_Nov; break;
	case MMSG::STR_Dec:	val = S_Dec; break;
	default:
		val = "";
		break;
#pragma endregion
	}
}

void ToolStr::MsgCatDec(int num, uint8_t numSign, char* dst)
{
	if (nullptr == dst)
		{ return; }

	bool minus = (num < 0);
	num = abs(num);

	uint8_t sign =
		(0 <= num && num < 10) ? 1 :
		(9 < num && num < 100) ? 2 :
		(99 < num && num < 1000) ? 3 :
		(999 < num && num < 10000) ? 4 :
		(9999 < num && num < 100000) ? 5 :
		(99999 < num && num < 1000000) ? 6 : 0;

	if (minus)
		{ MsgCat(MMSG::STR_MINUS, dst); }

	uint8_t addZeros = min(6, max(0, numSign - sign));

	for (uint8_t i = 0; i < addZeros; ++i)
		{ MsgCat(MMSG::STR_0, dst); }

	MsgCatNum(num, dst);
}

void ToolStr::MsgCatNum(int num, char* dst, uint8_t base)
{
	if (nullptr == dst)
		{ return; }
		
	int toInd = strlen(dst);
	itoa(num, dst + toInd, base);
}

void ToolStr::MsgCat(MMSG name, char *dst)
{
	if (nullptr == dst)
		{ return; }

	auto val = MsgGet(name);
	if (strlen(val) > 0)
		{ strcat(dst, val); }
}

void ToolStr::MsgCat(Opcode opcode, char* dst)
{
	if (nullptr == dst)
		{ return; }

	auto val = CmdGet(opcode);
	if (strlen(val) > 0)
		{ strcat(dst, val); }
}

void ToolStr::MsgCat(bool condition, MMSG nameTrue, MMSG nameFalse, char* dst)
{
	if (nullptr == dst)
		{ return; }

	auto val = MsgGet(condition ? nameTrue : nameFalse);
	if (strlen(val) > 0)
		{ strcat(dst, val); }
}

uint16_t ToolStr::SubstringToInt(const String& val, int fr, int to)
{
	if (fr < 0)
		{ return 0; }
	auto sub = (to <= fr) ? val.substring(fr) : val.substring(fr, to);
	sub.trim();
	return (uint16_t)sub.toInt();
}

uint8_t ToolStr::SubstringToByte(const String& val, int fr, int to)
{
	if (fr < 0)
		{ return 0; }

	auto sub = (to <= fr) ? val.substring(fr) : val.substring(fr, to);
	sub.trim();
	return (uint8_t)sub.toInt();
}

void ToolStr::add0Nd(String& str, uint16_t val, size_t width)
{
	if (width >= 5 && val < 1000) 
		{ str += '0'; }
	if (width >= 4 && val < 100) 
		{ str += '0'; }
	if (width >= 3 && val < 100) 
		{ str += '0'; }
	if (width >= 2 && val < 10) 
		{ str += '0'; }
	str += val;
}

void ToolStr::AddDateTimeToStr(String& str, const DateTime *dt, bool wd)
{
	if (nullptr == dt)
		{ return; }

	if (wd)
	{
		str += MsgGet((MMSG)(dt->dayOfTheWeek() + (uint8_t)MMSG::STR_WEEK));
		str += ' ';
	}
	add0Nd(str, dt->year(), 4);
	str += '.';
	add0Nd(str, dt->month(), 2);
	str += '.';
	add0Nd(str, dt->day(), 2);
	str += ' ';
	add0Nd(str, dt->hour(), 2);
	str += ':';
	add0Nd(str, dt->minute(), 2);
}

char* ToolStr::AddDateTimeToStr(char* str, const DateTime *dt, bool wd)
{
	if (nullptr == dt)
		{ return str; }
	if (wd)
	{
		MsgCat(MMSG::STR_WEEK, str);
		strcat(str, " ");
	}

	add0Nd(str, dt->year(), 4);
	strcat(str, ".");
	add0Nd(str, dt->month(), 2);
	strcat(str, ".");
	add0Nd(str, dt->day(), 2);
	strcat(str, " ");
	add0Nd(str, dt->hour(), 2);
	strcat(str, ":");
	add0Nd(str, dt->minute(), 2);
	return str;
}

void ToolStr::add0Nd(char *str, uint16_t val, size_t width)
{
	if (width >= 5 && val < 1000)
		{ strcat(str, "0"); }
	if (width >= 4 && val < 100)
		{ strcat(str, "0"); }
	if (width >= 3 && val < 100)
		{ strcat(str, "0"); }
	if (width >= 2 && val < 10)
		{ strcat(str, "0"); }
	itoa(val, str + strlen(str), 10);
}