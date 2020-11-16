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
	uint16_t ArrSize = 0;
	uint16_t NextIndex = 0;
	uint16_t UsedSize = 0;

	void Invalidate();
	uint16_t _restIndex() { 
		uint16_t maxArrIndx = _maxArrIndex();
		return (maxArrIndx > NextIndex ? (maxArrIndx - NextIndex) : 0);
	}

	uint16_t _maxArrIndex() { return ArrSize > 1 ? ArrSize - 1U : 0; }
public:
	// ctor
	BinArray(uint16_t size = COMMUNICATION_ARR_SIZE);
	BinArray(const uint8_t* arr, uint16_t size);
	~BinArray();

	uint8_t* GetPtr(uint16_t index = 0);
	char* GetChrPtr(uint16_t index = 0);
	void Clear(uint16_t index = 0);
	uint8_t ReSize(uint16_t size);
	uint8_t* GetNextPtr()   { return (ArrPtr + NextIndex); }
	uint16_t GetNextIndex() { return NextIndex; }
	uint16_t GetArrSize()   { return ArrSize; }
	uint16_t GetFreeSize()  { return (ArrSize - UsedSize); }
	uint16_t GetUsedSize()  { return UsedSize; }
	uint16_t SetNextIndex(uint16_t index);
	uint16_t AddNextIndex(uint16_t index);
	void AddUsedSize(uint16_t addSz);
	void AddUsedSize(uint16_t index, uint16_t addSz);

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
	bool Set(T val, uint16_t index)
	{
		auto valSize = sizeof(T);
		auto ret = (index + valSize) < ArrSize;
		if (ret) {
			auto ptr = GetPtr(index);
			ret = (nullptr != memcpy(ptr, &val, valSize));
		}
		return ret;
	}

	template <typename T>
	bool SetMask(T mask, uint16_t index)
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
	bool GetVal(uint16_t index, T& val)
	{
		auto valueSz = sizeof(T);
		memset(&val, 0, valueSz);
		auto ret = false;
		if ((NextIndex + valueSz - 1) < ArrSize) {
			auto ptr = GetPtr(index);
			memcpy(&val, ptr, valueSz);
			ret = true;
		}
		return ret;
	}

	template <typename T>
	T GetVal(uint16_t index)
	{
		T v;
		auto valueSz = sizeof(T);
		memset(&v, 0, valueSz);

		if ((NextIndex + valueSz) < ArrSize) {
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

