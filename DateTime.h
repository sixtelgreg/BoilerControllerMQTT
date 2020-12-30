#ifndef DATE_TIME_H
#define DATE_TIME_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

#include "TimeSpan.h"

/** Constants */
#define SECONDS_PER_DAY           86400L     ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800  ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

/**************************************************************************/
/*!
	@brief  Simple general-purpose date/time class (no TZ / DST / leap second handling!).
			See http://en.wikipedia.org/wiki/Leap_second
*/
/**************************************************************************/
#pragma pack(push, 1)
class DateTime
{
public:
	DateTime(uint32_t t = 0, bool unixTime = false);

	DateTime(
		uint16_t year, 
		uint8_t month, 
		uint8_t day,
		uint8_t hour = 0, 
		uint8_t min = 0, 
		uint8_t sec = 0);

	DateTime(const DateTime& copy);
	DateTime(const char* date, const char* time);
	DateTime(const __FlashStringHelper* date, const __FlashStringHelper* time);
    char* toString(char* buffer);
	void Set(const DateTime& copy);

	/*!
		@brief  Return the year, stored as an offset from 2000
		@return uint16_t year
	*/
	uint16_t year() const { return 2000 + yOff; }
	/*!
		@brief  Return month
		@return uint8_t month
	*/
	uint8_t month() const { return m; }
	/*!
		@brief  Return day
		@return uint8_t day
	*/
	uint8_t day() const { return d; }
	/*!
		@brief  Return hours
		@return uint8_t hours
	*/
	uint8_t hour() const { return hh; }
	/*!
		@brief  Return minutes
		@return uint8_t minutes
	*/
	uint8_t minute() const { return mm; }
	/*!
		@brief  Return seconds
		@return uint8_t seconds
	*/
	uint8_t second() const { return ss; }

	uint8_t dayOfTheWeek() const;

	/** 32-bit times as seconds since 1/1/2000 */
	long secondstime() const;

	/** 32-bit times as seconds since 1/1/1970 */
	uint32_t unixtime(void) const;

	/** ISO 8601 Timestamp function */
	enum timestampOpt 
	{
		TIMESTAMP_FULL, // YYYY-MM-DDTHH:MM:SS
		TIMESTAMP_TIME, // HH:MM:SS
		TIMESTAMP_DATE  // YYYY-MM-DD
	};

	String timestamp(timestampOpt opt = TIMESTAMP_FULL);

	DateTime operator+(const TimeSpan& span);
	DateTime operator-(const TimeSpan& span);
	TimeSpan operator-(const DateTime& right);
	bool operator<(const DateTime& right) const;
	/*!
		@brief  Test if one DateTime is greater (later) than another
		@param right DateTime object to compare
		@return True if the left object is greater than the right object, false otherwise
	*/
	bool operator>(const DateTime& right) const { return right < *this; }
	/*!
		@brief  Test if one DateTime is less (earlier) than or equal to another
		@param right DateTime object to compare
		@return True if the left object is less than or equal to the right object, false otherwise
	*/
	bool operator<=(const DateTime& right) const { return !(*this > right); }
	/*!
		@brief  Test if one DateTime is greater (later) than or equal to another
		@param right DateTime object to compare
		@return True if the left object is greater than or equal to the right object, false otherwise
	*/
	bool operator>=(const DateTime& right) const { return !(*this < right); }
	bool operator==(const DateTime& right) const;
	/*!
		@brief  Test if two DateTime objects not equal
		@param right DateTime object to compare
		@return True if the two objects are not equal, false if they are
	*/
	bool operator!=(const DateTime& right) const { return !(*this == right); }

	uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) const;
	long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) const;

protected:
	uint8_t yOff;   ///< Year offset from 2000
	uint8_t m;      ///< Month 1-12
	uint8_t d;      ///< Day 1-31
	uint8_t hh;     ///< Hours 0-23
	uint8_t mm;     ///< Minutes 0-59
	uint8_t ss;     ///< Seconds 0-59
};
#pragma pack(pop)
#endif

