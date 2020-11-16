#include "BinArray.h"
#include "DateTime.h"

//static uint8_t  dummy_writable_8;
//static uint16_t dummy_writable_16;
//static uint32_t dummy_writable_32;
//static DateTime dummy_writable_DT;

BinArray::BinArray(uint8_t size)
{
	ArrPtr = (uint8_t*)malloc(size);
	ArrSize = size;
	Clear(0);
}

BinArray::BinArray(const uint8_t* arr, uint8_t size)
{	
	ArrPtr = (uint8_t*)malloc(size);	
	memcpy(ArrPtr, arr, size);
	ArrSize = size;
	//UsedSize = 0;
	NextIndex = 0;
}

void BinArray::Clear(uint8_t index)
{
	NextIndex = (MIN(index, _maxArrIndex()));
	memset(ArrPtr + NextIndex, 0, ArrSize - NextIndex);
	//UsedSize = (MIN(NextIndex, UsedSize));
}

void BinArray::ClearData()
{
	Clear(_DATA_START_INDEX);
	auto ptr = GetPtr(_DATA_SIZE_INDEX);
	*ptr = 0;
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
	//UsedSize = 0;
}

uint8_t BinArray::ReSize(uint8_t size)
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

uint8_t BinArray::AddNextIndex(uint8_t index)
{ 
	NextIndex += (MIN(index, _restIndex()));
	return NextIndex;
}

void BinArray::AddUsedSize(uint8_t addSz)
{ 
	NextIndex += (MIN(addSz, _restIndex()));
}

//void BinArray::AddUsedSize(uint8_t index, uint8_t addSz)
//{
//	uint8_t willUsed = (index + addSz);
//	if (willUsed > UsedSize)
//	{
//		uint8_t addUsed = willUsed - UsedSize;
//		UsedSize += (MIN(addUsed, _restIndex()));
//	}
//}

uint8_t BinArray::SetNextIndex(uint8_t index)
{ 
	NextIndex = (MIN(index, _maxArrIndex()));
	return NextIndex;
}

char* BinArray::GetChrPtr(uint8_t index)
{
	return reinterpret_cast<char*>(GetPtr(index));
}

uint8_t* BinArray::GetPtr(uint8_t index)
{
	size_t indx = (MIN(index, _maxArrIndex()));
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
