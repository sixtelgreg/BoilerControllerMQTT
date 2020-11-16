#ifndef ToolStrings_h
#define ToolStrings_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "DataTypes.h"

class DateTime;

class ToolStr
{
public:
	static const char* MessageReceivedOK(Opcode opcode);
	static const char* CmdGet(Opcode opcode);
	static void CmdGet(Opcode opcode, String& val);
	static const char* MsgGet(MMSG name);
	static void MsgGet(MMSG name, String& val);
	static void MsgCatDec(int num, uint8_t numSign, char *dst);
	static void MsgCatNum(int num, char *dst, uint8_t base = 10);
	static void MsgCat(MMSG name, char *dst);
	static void MsgCat(Opcode opcode, char *dst);
	static void MsgCat(bool condition, MMSG nameTrue, MMSG nameFalse, char *dst);
	static uint16_t SubstringToInt(const String& val, int fr, int to = -1);
	static uint8_t SubstringToByte(const String& val, int fr, int to = -1);
	static void AddDateTimeToStr(String& str, const DateTime *dt, bool wd = false);
	static char* AddDateTimeToStr(char *str, const DateTime* dt, bool wd = false);

	static void add0Nd(String& str, uint16_t val, size_t width);
	static void add0Nd(char *str, uint16_t val, size_t width);
};
#endif