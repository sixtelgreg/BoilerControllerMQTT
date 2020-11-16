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
	uint8_t* ArrPtr = nullptr;
	uint8_t ArrSize = 0;
	uint8_t NextIndex = 0;
	//uint16_t UsedSize = 0;

	void Invalidate();
	uint8_t _restIndex() {
		uint8_t maxArrIndx = _maxArrIndex();
		return (maxArrIndx > NextIndex ? (maxArrIndx - NextIndex) : 0);
	}

	uint8_t _maxArrIndex() { return ArrSize > 1 ? ArrSize - 1U : 0; }
public:
	// ctor
	BinArray(uint8_t size = COMMUNICATION_ARR_SIZE);
	BinArray(const uint8_t* arr, uint8_t size);
	~BinArray();

	uint8_t* GetPtr(uint8_t index = 0);
	char* GetChrPtr(uint8_t index = 0);
	void Clear(uint8_t index = 0);
	void ClearData();
	uint8_t ReSize(uint8_t size);
	uint8_t* GetNextPtr()		{ return (ArrPtr + NextIndex); }
	uint8_t GetNextIndex()		{ return NextIndex; }
	uint8_t GetArrSize()		{ return ArrSize; }
	uint8_t GetFreeSize()		{ return (ArrSize - NextIndex); }
	uint8_t GetFullArraySize() { return ArrPtr[_DATA_SIZE_INDEX] + _DATA_START_INDEX; }
	uint8_t GetUsedSize()		{ return ArrPtr[_DATA_SIZE_INDEX]; }
	uint8_t SetNextIndex(uint8_t index);
	uint8_t AddNextIndex(uint8_t index);
	void AddUsedSize(uint8_t addSz);
	//void AddUsedSize(uint16_t index, uint16_t addSz);

	template <typename T>
	bool Add(T val)
	{
		auto valSize = sizeof(T);
		auto ret = Set(val, NextIndex);
		if (ret) {
			NextIndex += valSize;
		}
		return ret;
	}

	template <typename T>
	bool Set(T val, uint8_t index)
	{
		auto valSize = sizeof(T);
		auto ret = (index + valSize) < ArrSize;
		if (ret) {
			auto ptr = GetPtr(index);
			ret = (nullptr != memcpy(ptr, &val, valSize));
		}
		return ret;
	}
	
	uint8_t PinEndData()
	{
		auto usedSz = (uint8_t)(NextIndex - _DATA_START_INDEX);
		auto stt = Set(usedSz, _DATA_SIZE_INDEX);
		return stt ? usedSz : 0;
	}

	template <typename T>
	bool SetMask(T mask, uint8_t index)
	{
		T m;
		auto ret = false;
		auto valueSz = sizeof(T);
		if (GetVal(NextIndex, m)) {
			m |= mask;
			ret = Set(&m, valueSz, index);
		}
		return ret;
	}

	template <class T>
	bool GetVal(uint8_t index, T& val)
	{
		auto valueSz = sizeof(T);
		memset(&val, 0, valueSz);
		auto ret = false;
		if ((index + valueSz - 1) < ArrSize) {
			auto ptr = GetPtr(index);
			memcpy(&val, ptr, valueSz);
			ret = true;
		}
		return ret;
	}

	template <typename T>
	T GetVal(uint8_t index)
	{
		T v;
		auto valueSz = sizeof(T);
		memset(&v, 0, valueSz);

		if ((index + valueSz) < ArrSize) {
			auto ptr = GetPtr(index);
			memcpy(&v, ptr, valueSz);
		}
		return v;
	}

	template <typename T>
	T NextVal()
	{
		T v;
		auto valueSz = sizeof(T);
		if (GetVal(NextIndex, v)) {
			NextIndex += valueSz;
		}
		return v;
	}

	//uint8_t&  operator [] (uint16_t index);
	//BinArray& operator += (uint8_t data);
	//BinArray& operator += (uint16_t data);
	//BinArray& operator += (uint32_t data);
	//BinArray& operator += (const DateTime *pdt);
};

#endif

