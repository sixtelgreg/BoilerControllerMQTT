#include "BinArray.h"
#include "DateTime.h"

static uint8_t  dummy_writable_8;
static uint16_t dummy_writable_16;
static uint32_t dummy_writable_32;
static DateTime dummy_writable_DT;

BinArray::BinArray(uint16_t size)
{
	Arr = (uint8_t*)malloc(size);
	ArrSize = size;
	Clear();
}

BinArray::~BinArray()
{
	Invalidate();
}

void BinArray::Invalidate()
{
	if (Arr)
		{ free(Arr); }
	Arr = nullptr;
	ArrSize = 0;
	UsedSize = 0;
}

uint8_t BinArray::ReSize(uint16_t size)
{
	uint8_t ret = 0;
	if (size > ArrSize)
	{
		uint8_t* newArr = (uint8_t*)realloc(Arr, size);
		
		if (nullptr != newArr)
		{
			Arr = newArr;
			ArrSize = size;
			ret = 1;
		}
	}
	Clear();
	return ret;
}

uint16_t BinArray::AddNextIndex(uint16_t indx)
{ 
	NextIndex += MIN(indx, (ArrSize - NextIndex - 1));
	return NextIndex;
}

void BinArray::AddUsedSize(uint16_t addSz)
{ 
	UsedSize += MIN(addSz, (ArrSize - UsedSize - 1));
}

void BinArray::AddUsedSize(uint16_t fromIndex, uint16_t addSz)
{
	if (fromIndex + addSz > UsedSize)
	{
		auto add = (fromIndex + addSz) - UsedSize;
		UsedSize += MIN(add, (ArrSize - UsedSize - 1));
	}
}

uint16_t BinArray::SetNextIndex(uint16_t indx)
{ 
	NextIndex = MIN(indx, (ArrSize - 1));
	return NextIndex;
}

char* BinArray::GetChrPtr(uint16_t fromIndex)
{
	return (char*)GetPtr(fromIndex);
}

uint8_t* BinArray::GetPtr(uint16_t fromIndex)
{
	auto indx = MIN(fromIndex, (ArrSize - 1));
	return (Arr + indx);
}

void BinArray::Clear(uint16_t fromIndex)
{
	NextIndex = MIN(fromIndex, ArrSize - 1);
	memset(Arr + NextIndex, 0, ArrSize - NextIndex);
	UsedSize = MIN(NextIndex, UsedSize);
}

#pragma region Set
bool BinArray::Set(uint8_t data, uint16_t fromIndex)
{
	auto ret = (fromIndex <= ArrSize);
	if (ret)
	{
		AddUsedSize(fromIndex, 1);
		auto ptr = GetPtr(fromIndex);
		*ptr = data;
	}
	return ret;
}

bool BinArray::Set(uint16_t data, uint16_t fromIndex)
{
	auto ret = ((fromIndex + 1) <= ArrSize);
	if (ret)
	{
		AddUsedSize(fromIndex, 2);
		auto ptr = GetPtr(fromIndex);
		*((uint16_t*)ptr) = data;
	}
	return ret;
}

bool BinArray::Set(uint32_t data, uint16_t fromIndex)
{
	auto ret = ((fromIndex + 3) < ArrSize);
	if (ret)
	{
		AddUsedSize(fromIndex, 4);
		auto ptr = GetPtr(fromIndex);
		*((uint32_t*)ptr) = data;
	}
	return ret;
}

bool BinArray::Set(const DateTime *pdt, uint16_t fromIndex)
{
	auto ret = nullptr != pdt && ((fromIndex + sizeof(DateTime) - 1) < ArrSize);
	if (ret)
	{
		AddUsedSize(fromIndex, sizeof(DateTime));
		auto ptr = GetPtr(fromIndex);
		memcpy(ptr, pdt, sizeof(DateTime));
	}
	return ret;
}

bool BinArray::Set(const void* data, uint16_t dataSize, uint16_t fromIndex)
{
	auto ret = (nullptr != data &&
				dataSize > 0 &&
				(fromIndex + dataSize - 1) < ArrSize);
	if (ret)
	{
		AddUsedSize(fromIndex, dataSize);
		auto ptr = GetPtr(fromIndex);
		memcpy(ptr, data, dataSize);
	}
	return ret;
}
#pragma endregion

#pragma region Concat with Index
bool BinArray::Concat(uint8_t data, uint16_t fromIndex)
{
	auto ret = (fromIndex < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		*ptr = data;
		NextIndex = MAX(NextIndex, fromIndex + 1);
		AddUsedSize(fromIndex, 1);
	}
	return ret;
}

bool BinArray::Concat(uint16_t data, uint16_t fromIndex)
{
	auto ret = ((fromIndex + 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		*((uint16_t*)ptr) = data;
		NextIndex = MAX(NextIndex, fromIndex + 2);
		AddUsedSize(fromIndex, 2);
	}
	return ret;
}

bool BinArray::Concat(uint32_t data, uint16_t fromIndex)
{
	auto ret = ((fromIndex + 3) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		*((uint32_t*)ptr) = data;
		NextIndex = MAX(NextIndex, fromIndex + 4);
		AddUsedSize(fromIndex, 4);
	}
	return ret;
}

bool BinArray::Concat(const DateTime *pdt, uint16_t fromIndex)
{
	auto ret = ((fromIndex + sizeof(DateTime) - 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		memcpy(ptr, pdt, sizeof(DateTime));
		NextIndex = MAX(NextIndex, fromIndex + sizeof(DateTime));
		AddUsedSize(fromIndex, sizeof(DateTime));
	}
	return ret;
}

bool BinArray::Concat(const void* data, uint16_t dataSize, uint16_t fromIndex)
{
	auto ret = (nullptr != data &&
				dataSize > 0 &&
				(fromIndex + dataSize - 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		memcpy(ptr, data, dataSize);
		NextIndex = MAX(NextIndex, fromIndex + dataSize);
		AddUsedSize(fromIndex, dataSize);
	}
	return ret;
}
#pragma endregion

#pragma region Concat
bool BinArray::Concat(uint8_t data)
{
	auto ret = (NextIndex < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(NextIndex);
		*ptr = data;
		AddUsedSize(NextIndex, 1);
		NextIndex += 1;
	}
	return ret;
}

bool BinArray::Concat(uint16_t data)
{
	auto ret = ((NextIndex + 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(NextIndex);
		*((uint16_t*)ptr) = data;
		AddUsedSize(NextIndex, 2);
		NextIndex += 2;
	}
	return ret;
}

bool BinArray::Concat(uint32_t data)
{
	auto ret = ((NextIndex + 3) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(NextIndex);
		*((uint32_t*)ptr) = data;
		AddUsedSize(NextIndex, 4);
		NextIndex += 4;
	}
	return ret;
}

bool BinArray::Concat(const DateTime *pdt)
{
	auto ret = nullptr != pdt && ((NextIndex + sizeof(DateTime) - 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(NextIndex);
		memcpy(ptr, pdt, sizeof(DateTime));
		AddUsedSize(NextIndex, sizeof(DateTime));
		NextIndex += sizeof(DateTime);
	}
	return ret;
}

bool BinArray::Concat(const void* data, uint16_t dataSize)
{
	auto ret = (nullptr != data &&
				dataSize > 0   &&
		       (NextIndex + dataSize - 1) < ArrSize);
	if (ret)
	{
		auto ptr = GetPtr(NextIndex);
		memcpy(ptr, data, dataSize);
		AddUsedSize(NextIndex, dataSize);
		NextIndex += dataSize;
	}
	return ret;
}
#pragma endregion

void BinArray::SetDataMask(uint8_t mask, uint16_t fromIndex)
{
	if (fromIndex < ArrSize)
		{ *(Arr + fromIndex) |= mask; }
}

void BinArray::SetDataMask(uint16_t mask, uint16_t fromIndex)
{
	if ((fromIndex + 1) < ArrSize)
		{ *((uint16_t*)(Arr + fromIndex)) |= mask; }
}

void BinArray::SetDataMask(uint32_t mask, uint16_t fromIndex)
{
	if ((fromIndex + 3) < ArrSize)
		{ *((uint32_t*)(Arr + fromIndex)) |= mask; }
}

void BinArray::RstDataMask(uint8_t mask, uint16_t fromIndex)
{
	if (fromIndex < ArrSize)
		{ *(Arr + fromIndex) &= ~mask; }
}

void BinArray::RstDataMask(uint16_t mask, uint16_t fromIndex)
{
	if ((fromIndex + 1) < ArrSize)
		{ *((uint16_t*)(Arr + fromIndex)) &= ~mask; }
}

void BinArray::RstDataMask(uint32_t mask, uint16_t fromIndex)
{
	if ((fromIndex + 3) < ArrSize)
		{ *((uint32_t*)(Arr + fromIndex)) &= ~mask; }
}

bool BinArray::IsDataMask(uint8_t mask, uint16_t fromIndex)
{
	auto flag = GetByte(fromIndex);
	return (0 != (flag & mask));
}

bool BinArray::IsDataMask(uint16_t mask, uint16_t fromIndex)
{
	auto flag = GetWord(fromIndex);
	return (0 != (flag & mask));
}

bool BinArray::IsDataMask(uint32_t mask, uint16_t fromIndex)
{
	auto flag = GetDword(fromIndex);
	return (0 != (flag & mask));
}

bool BinArray::IsEqual(uint8_t data, uint16_t fromIndex)
{
	auto ret = fromIndex < ArrSize;
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		ret = ((*ptr) == data);
	}
	return ret;
}

bool BinArray::IsEqual(uint16_t data, uint16_t fromIndex)
{
	auto ret = (fromIndex + 1) < ArrSize;
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		ret = ((*((uint16_t*)ptr)) == data);
	}
	return ret;
}

bool BinArray::IsEqual(uint32_t data, uint16_t fromIndex)
{
	auto ret = (fromIndex + 3) < ArrSize;
	if (ret)
	{
		auto ptr = GetPtr(fromIndex);
		ret = ((*((uint32_t*)ptr)) == data);
	}
	return ret;
}

uint8_t& BinArray::operator[](uint16_t index)
{
	if (index < ArrSize)
		{ return Arr[index]; }

	dummy_writable_8 = 0;
	return dummy_writable_8;
}

BinArray& BinArray::operator += (uint8_t data) 
{ 
	Concat(data);
	return (*this);
}

BinArray& BinArray::operator += (uint16_t data)
{
	Concat(data);
	return (*this);
}

BinArray& BinArray::operator += (uint32_t data)
{
	Concat(data);
	return (*this);
}
BinArray& BinArray::operator += (const DateTime *pdt)
{
	Concat(pdt);
	return (*this);
}

uint8_t& BinArray::GetByte(uint16_t fromIndex = 0)
{
	if (fromIndex < ArrSize)
		{ return Arr[fromIndex]; }

	dummy_writable_8 = 0;
	return dummy_writable_8;
}

uint16_t& BinArray::GetWord(uint16_t fromIndex = 0)
{
	if ((fromIndex + 1) < ArrSize)
		{ return *((uint16_t*)(Arr + fromIndex)); }

	dummy_writable_16 = 0;
	return dummy_writable_16;
}

uint32_t& BinArray::GetDword(uint16_t fromIndex = 0)
{
	if ((fromIndex + 3) < ArrSize)
		{ return *((uint32_t*)(Arr + fromIndex)); }

	dummy_writable_32 = 0;
	return dummy_writable_32;
}

DateTime* BinArray::GetDateTime(uint16_t fromIndex = 0)
{
	if ((fromIndex + sizeof(DateTime) - 1) < ArrSize)
		{ return (DateTime*)(Arr + fromIndex); }

	dummy_writable_DT;
	return &dummy_writable_DT;
}

uint8_t& BinArray::GetNextByte()
{
	if (NextIndex < ArrSize)
	{
		auto ptr = GetPtr(NextIndex);
		++NextIndex;
		return *ptr;
	}
	dummy_writable_8 = 0;
	return dummy_writable_8;
}

uint16_t& BinArray::GetNextWord()
{
	if ((NextIndex + 1) < ArrSize)
	{
		auto ptr = GetPtr(NextIndex);
		NextIndex += 2;
		return *((uint16_t*)ptr);
	}
	dummy_writable_16 = 0;
	return dummy_writable_16;
}

uint32_t& BinArray::GetNextDword()
{
	if ((NextIndex + 3) < ArrSize)
	{
		auto ptr = GetPtr(NextIndex);
		NextIndex += 4;
		return *((uint32_t*)ptr);
	}
	dummy_writable_32 = 0;
	return dummy_writable_32;
}

DateTime* BinArray::GetNextDateTime()
{
	if ((NextIndex + sizeof(DateTime) - 1) < ArrSize)
	{
		auto ptr = GetPtr(NextIndex);
		NextIndex += sizeof(DateTime);
		return (DateTime*)ptr;
	}
	dummy_writable_DT;
	return &dummy_writable_DT;
}
