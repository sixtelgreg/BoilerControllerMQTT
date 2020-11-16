#ifndef _BinArray_H_
#define _BinArray_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "DataTypes.h"

class DateTime;

class BinArray
{
private:
	uint8_t* Arr = nullptr;
	uint16_t ArrSize = 0;
	uint16_t NextIndex = 0;
	uint16_t UsedSize = 0;

	void Invalidate();
public:
	// ctor
	BinArray(uint16_t size = COMMUNICATION_ARR_SIZE);
	~BinArray();

	uint8_t* GetPtr(uint16_t fromIndex = 0);
	char* GetChrPtr(uint16_t fromIndex = 0);
	void Clear(uint16_t fromIndex = 0);
	uint8_t ReSize(uint16_t size);
	uint8_t* GetNextPtr() { return (Arr + NextIndex); }
	uint16_t GetNextIndex() { return NextIndex; }
	uint16_t GetArrSize() { return ArrSize; }
	uint16_t GetFreeSize() { return (ArrSize - UsedSize); }
	uint16_t GetUsedSize() { return UsedSize; }
	uint16_t SetNextIndex(uint16_t indx);
	uint16_t AddNextIndex(uint16_t indx);
	void AddUsedSize(uint16_t addSz);
	void AddUsedSize(uint16_t fromIndex, uint16_t addSz);

	uint8_t& GetByte(uint16_t fromIndex);
	uint16_t& GetWord(uint16_t fromIndex);
	uint32_t& GetDword(uint16_t fromIndex);
	DateTime* GetDateTime(uint16_t fromIndex);

	uint8_t& GetNextByte();
	uint16_t& GetNextWord();
	uint32_t& GetNextDword();
	DateTime* GetNextDateTime();

	bool Set(uint8_t data, uint16_t fromIndex = 0);
	bool Set(uint16_t data, uint16_t fromIndex = 0);
	bool Set(uint32_t data, uint16_t fromIndex = 0);
	bool Set(const DateTime *pdt, uint16_t fromIndex = 0);
	bool Set(const void *data, uint16_t dataSize, uint16_t fromIndex = 0);
 
	bool Concat(uint8_t data, uint16_t fromIndex);
	bool Concat(uint16_t data, uint16_t fromIndex);
	bool Concat(uint32_t data, uint16_t fromIndex);
	bool Concat(const DateTime *pdt, uint16_t fromIndex);
	bool Concat(const void *data, uint16_t dataSize, uint16_t fromIndex);

	bool Concat(uint8_t data);
	bool Concat(uint16_t data);
	bool Concat(uint32_t data);
	bool Concat(const DateTime *pdt);
	bool Concat(const void *data, uint16_t dataSize);

	void SetDataMask(uint8_t mask, uint16_t fromIndex);
	void SetDataMask(uint16_t mask, uint16_t fromIndex);
	void SetDataMask(uint32_t mask, uint16_t fromIndex);

	void RstDataMask(uint8_t mask, uint16_t fromIndex);
	void RstDataMask(uint16_t mask, uint16_t fromIndex);
	void RstDataMask(uint32_t mask, uint16_t fromIndex);

	bool IsDataMask(uint8_t mask, uint16_t fromIndex);
	bool IsDataMask(uint16_t mask, uint16_t fromIndex);
	bool IsDataMask(uint32_t mask, uint16_t fromIndex);

	bool IsEqual(uint8_t data, uint16_t fromIndex = 0);
	bool IsEqual(uint16_t data, uint16_t fromIndex = 0);
	bool IsEqual(uint32_t data, uint16_t fromIndex = 0);

	uint8_t&  operator [] (uint16_t index);
	BinArray& operator += (uint8_t data);
	BinArray& operator += (uint16_t data);
	BinArray& operator += (uint32_t data);
	BinArray& operator += (const DateTime *pdt);
};

#endif

