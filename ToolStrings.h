#ifndef TOOL_STRINGS_H
#define TOOL_STRINGS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include "DataTypes.h"
#include "DateTime.h"

	const char* MessageReceivedOK(Opcode opcode);
	const char* CmdGet(Opcode opcode);
	void CmdGet(Opcode opcode, String& val);
	const char* MsgGet(MMSG name);
	void MsgGet(MMSG name, String& val);
	void MsgCatDec(int num, uint8_t numSign, char *dst);
	void MsgCatNum(int num, char *dst, uint8_t base = 10);
	void MsgCat(MMSG name, char *dst);
	void MsgCat(Opcode opcode, char *dst);
	void MsgCat(bool condition, MMSG nameTrue, MMSG nameFalse, char *dst);
	uint16_t SubstringToInt(const String& val, int fr, int to = -1);
	uint8_t SubstringToByte(const String& val, int fr, int to = -1);
	void AddDateTimeToStr(String& str, const DateTime *dt, bool wd = false);
	char* AddDateTimeToStr(char *str, const DateTime *dt, bool wd = false);
	void Add0Nd(String& str, uint16_t val, size_t width);
	void Add0Nd(char *str, uint16_t val, size_t width);

#endif //TOOL_STRINGS_H