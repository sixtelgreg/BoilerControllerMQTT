#include "BinArray.h"
#include "DateTime.h"

//static uint8_t  dummy_writable_8;
//static uint16_t dummy_writable_16;
//static uint32_t dummy_writable_32;
//static DateTime dummy_writable_DT;

BinArray::BinArray(uint16_t size)
{
	ArrPtr = (uint8_t*)malloc(size);
	ArrSize = size;
	Clear(0);
}

BinArray::BinArray(const uint8_t* arr, uint16_t size)
{	
	ArrPtr = (uint8_t*)malloc(size);	
	memcpy(ArrPtr, arr, size);
	ArrSize = size;
	UsedSize = size;
	NextIndex = 0;
}

void BinArray::Clear(uint16_t index)
{
	NextIndex = (__MIN(index, _maxArrIndex()));
	memset(ArrPtr + NextIndex, 0, ArrSize - NextIndex);
	UsedSize = (__MIN(NextIndex, UsedSize));
}

BinArray::~BinArray()
{
	Invalidate();
}

void BinArray::Invalidate()
{
	if (ArrPtr) { 
		free(ArrPtr);
	}
	ArrPtr = nullptr;
	ArrSize = 0;
	UsedSize = 0;
}

uint8_t BinArray::ReSize(uint16_t size)
{
	uint8_t ret = 0;
	if (size > ArrSize){
		auto newArr = (uint8_t*)realloc(ArrPtr, size);
		
		if (nullptr != newArr){
			ArrPtr = newArr;
			ArrSize = size;
			ret = 1;
		}
	}
	Clear();
	return ret;
}

uint16_t BinArray::AddNextIndex(uint16_t index)
{ 
	NextIndex += (__MIN(index, _restIndex()));
	return NextIndex;
}

void BinArray::AddUsedSize(uint16_t addSz)
{ 
	UsedSize += (__MIN(addSz, _restIndex()));
}

void BinArray::AddUsedSize(uint16_t index, uint16_t addSz)
{
	uint16_t willUsed = (index + addSz);
	if (willUsed > UsedSize)
	{
		uint16_t addUsed = willUsed - UsedSize;
		UsedSize += (__MIN(addUsed, _restIndex()));
	}
}

uint16_t BinArray::SetNextIndex(uint16_t index)
{ 
	NextIndex = (__MIN(index, _maxArrIndex()));
	return NextIndex;
}

char* BinArray::GetChrPtr(uint16_t index)
{
	return reinterpret_cast<char*>(GetPtr(index));
}

uint8_t* BinArray::GetPtr(uint16_t index)
{
	size_t indx = (__MIN(index, _maxArrIndex()));
	return (ArrPtr + index);
}


//void BinArray::SetDataMask(uint16_t mask, uint16_t fromIndex)
//{
//	if ((fromIndex + 1) < ArrSize)
//		{ *(reinterpret_cast<uint16_t*>(ArrPtr + fromIndex)) |= mask; }
//}
//
//void BinArray::SetDataMask(uint32_t mask, uint16_t fromIndex)
//{
//	if ((fromIndex + 3) < ArrSize)
//		{ *(reinterpret_cast<uint32_t*>(ArrPtr + fromIndex)) |= mask; }
//}


//uint8_t& BinArray::operator[](uint16_t index)
//{
//	if (index < ArrSize)
//		{ return ArrPtr[index]; }
//
//	dummy_writable_8 = 0;
//	return dummy_writable_8;
//}
//
//BinArray& BinArray::operator += (uint8_t data) 
//{ 
//	Add(&data, sizeof(uint8_t));
//	return (*this);
//}
//
//BinArray& BinArray::operator += (uint16_t data)
//{
//	Add(&data, sizeof(uint16_t));
//	return (*this);
//}
//
//BinArray& BinArray::operator += (uint32_t data)
//{
//	Add(&data, sizeof(uint32_t));
//	return (*this);
//}
//BinArray& BinArray::operator += (const DateTime *data)
//{
//	Add(data, sizeof(DateTime));
//	return (*this);
//}
//
//template <class T> BinArray& BinArray::operator += (T& data)
//{
//	Add(data);
//	return (*this);
//}
//
//template <class T> T BinArray::operator [] (uint16_t index)
//{
//	T v;
//	GetVal(NextIndex, v);
//	return v;
//}
