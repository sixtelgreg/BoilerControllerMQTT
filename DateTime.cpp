#include "DateTime.h"

/**************************************************************************/
// utility code, some of this could be exposed in the DateTime API if needed
/**************************************************************************/

/** Number of days in each month */
const uint8_t daysInMonth[] PROGMEM = {31,28,31,30,31,30,31,31,30,31,30,31};

DateTime::DateTime(uint32_t t, bool unixTime) 
{
	if (unixTime)
	{
		t -= SECONDS_FROM_1970_TO_2000;
	}    // bring to 2000 timestamp from 1970

	ss = t % 60;
	t /= 60;
	mm = t % 60;
	t /= 60;
	hh = t % 24;
	uint16_t days = t / 24;
	uint8_t leap;
	for (yOff = 0; ; ++yOff) 
	{
		leap = yOff % 4 == 0;
		if (days < 365 + leap)
			break;
		days -= 365 + leap;
	}

	for (m = 1; ; ++m) 
	{
		uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
		if (leap && m == 2)
			++daysPerMonth;
		if (days < daysPerMonth)
			break;
		days -= daysPerMonth;
	}
	d = days + 1;
}

/**************************************************************************/
/*!
	@brief  DateTime constructor from Y-M-D H:M:S
	@param year Year, 2 or 4 digits (year 2000 or higher)
	@param month Month 1-12
	@param day Day 1-31
	@param hour 0-23
	@param min 0-59
	@param sec 0-59
*/
/**************************************************************************/
DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
	if (year >= 2000)
		year -= 2000;
	yOff = year;
	m = month;
	d = day;
	hh = hour;
	mm = min;
	ss = sec;
}

/**************************************************************************/
/*!
	@brief  DateTime copy constructor using a member initializer list
	@param copy DateTime object to copy
*/
/**************************************************************************/
DateTime::DateTime(const DateTime& copy) :
	yOff(copy.yOff),
	m(copy.m),
	d(copy.d),
	hh(copy.hh),
	mm(copy.mm),
	ss(copy.ss)
{}

void DateTime::Set(const DateTime& copy)
{
	yOff = copy.yOff;
	m = copy.m;
	d = copy.d;
	hh = copy.hh;
	mm = copy.mm;
	ss = copy.ss;
}

/**************************************************************************/
/*!
	@brief  Convert a string containing two digits to uint8_t, e.g. "09" returns 9
	@param p Pointer to a string containing two digits
*/
/**************************************************************************/
static uint8_t conv2d(const char* p) 
{
	uint8_t v = 0;
	if ('0' <= *p && *p <= '9')
		v = *p - '0';
	return 10 * v + *++p - '0';
}

/**************************************************************************/
/*!
	@brief  A convenient constructor for using "the compiler's time":
			DateTime now (__DATE__, __TIME__);
			NOTE: using F() would further reduce the RAM footprint, see below.
	@param date Date string, e.g. "Dec 26 2009"
	@param time Time string, e.g. "12:34:56"
*/
/**************************************************************************/
DateTime::DateTime(const char* date, const char* time) 
{
	// sample input: date = "Dec 26 2009", time = "12:34:56"
	yOff = conv2d(date + 9);
	// Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
	switch (date[0]) 
	{
	case 'J': m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7); break;
	case 'F': m = 2; break;
	case 'A': m = date[2] == 'r' ? 4 : 8; break;
	case 'M': m = date[2] == 'r' ? 3 : 5; break;
	case 'S': m = 9; break;
	case 'O': m = 10; break;
	case 'N': m = 11; break;
	case 'D': m = 12; break;
	}
	d = conv2d(date + 4);
	hh = conv2d(time);
	mm = conv2d(time + 3);
	ss = conv2d(time + 6);
}

/**************************************************************************/
/*!
	@brief  A convenient constructor for using "the compiler's time":
			This version will save RAM by using PROGMEM to store it by using the F macro.
			DateTime now (F(__DATE__), F(__TIME__));
	@param date Date string, e.g. "Dec 26 2009"
	@param time Time string, e.g. "12:34:56"
*/
/**************************************************************************/
DateTime::DateTime(const __FlashStringHelper* date, const __FlashStringHelper* time) 
{
	// sample input: date = "Dec 26 2009", time = "12:34:56"
	char buff[11];
	memcpy_P(buff, date, 11);
	yOff = conv2d(buff + 9);
	// Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
	switch (buff[0])
	{
	case 'J': m = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7); break;
	case 'F': m = 2; break;
	case 'A': m = buff[2] == 'r' ? 4 : 8; break;
	case 'M': m = buff[2] == 'r' ? 3 : 5; break;
	case 'S': m = 9; break;
	case 'O': m = 10; break;
	case 'N': m = 11; break;
	case 'D': m = 12; break;
	}
	d = conv2d(buff + 4);
	memcpy_P(buff, time, 8);
	hh = conv2d(buff);
	mm = conv2d(buff + 3);
	ss = conv2d(buff + 6);
}

/**************************************************************************/
/*!
    @brief  Return DateTime in based on user defined format.
    @param buffer: array of char for holding the format description and the formatted DateTime. 
                   Before calling this method, the buffer should be initialized by the user with 
                   a format string, e.g. "YYYY-MM-DD hh:mm:ss". The method will overwrite 
                   the buffer with the formatted date and/or time.
    @return a pointer to the provided buffer. This is returned for convenience, 
            in order to enable idioms such as Serial.println(now.toString(buffer));
*/
/**************************************************************************/

char* DateTime::toString(char* buffer){
		for(int i=0;i<strlen(buffer)-1;i++){
		if(buffer[i] == 'h' && buffer[i+1] == 'h'){
			buffer[i] = '0'+hh/10;
			buffer[i+1] = '0'+hh%10;
		}
		if(buffer[i] == 'm' && buffer[i+1] == 'm'){
			buffer[i] = '0'+mm/10;
			buffer[i+1] = '0'+mm%10;
		}
		if(buffer[i] == 's' && buffer[i+1] == 's'){
			buffer[i] = '0'+ss/10;
			buffer[i+1] = '0'+ss%10;
		}
    if(buffer[i] == 'D' && buffer[i+1] =='D' && buffer[i+2] =='D'){
      static PROGMEM const char day_names[] = "SunMonTueWedThuFriSat";
      const char *p = &day_names[3*dayOfTheWeek()];
      buffer[i] = pgm_read_byte(p);
      buffer[i+1] = pgm_read_byte(p+1);
      buffer[i+2] = pgm_read_byte(p+2);
    }else
		if(buffer[i] == 'D' && buffer[i+1] == 'D'){
			buffer[i] = '0'+d/10;
			buffer[i+1] = '0'+d%10;
		}
    if(buffer[i] == 'M' && buffer[i+1] =='M' && buffer[i+2] =='M'){
      static PROGMEM const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
      const char *p = &month_names[3*(m-1)];
      buffer[i] = pgm_read_byte(p);
      buffer[i+1] = pgm_read_byte(p+1);
      buffer[i+2] = pgm_read_byte(p+2);      
    }else
		if(buffer[i] == 'M' && buffer[i+1] == 'M'){
			buffer[i] = '0'+m/10;
			buffer[i+1] = '0'+m%10;
		}
		if(buffer[i] == 'Y'&& buffer[i+1] == 'Y'&& buffer[i+2] == 'Y'&& buffer[i+3] == 'Y'){
			buffer[i] = '2';
			buffer[i+1] = '0';
			buffer[i+2] = '0'+(yOff/10)%10;
			buffer[i+3] = '0'+yOff%10;
		}else
		if(buffer[i] == 'Y'&& buffer[i+1] == 'Y'){
			buffer[i] = '0'+(yOff/10)%10;
			buffer[i+1] = '0'+yOff%10;
		}

	}
	return buffer;
}

/**************************************************************************/
/*!
	@brief  Return the day of the week for this object, from 1-7.
	@return Day of week 0-6 starting with Sunday, e.g. Sunday = 1, Saturday = 7
*/
/**************************************************************************/
uint8_t DateTime::dayOfTheWeek() const 
{
	uint16_t days = (date2days(yOff, m, d) + 7U) % 7U;// Jan 1, 2000 is a Saturday, i.e. adds 7
	return  days ? days : 7;
}

/**************************************************************************/
/*!
	@brief  Return unix time, seconds since Jan 1, 1970.
	@return Number of seconds since Jan 1, 1970
*/
/**************************************************************************/
uint32_t DateTime::unixtime(void) const 
{
	uint32_t t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);
	t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

	return t;
}

/**************************************************************************/
/*!
	@brief  Convert the DateTime to seconds
	@return The object as seconds since 2000-01-01
*/
/**************************************************************************/
long DateTime::secondstime(void) const 
{
	long t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);
	return t;
}

/**************************************************************************/
/*!
	@brief  Add a TimeSpan to the DateTime object
	@param span TimeSpan object
	@return new DateTime object with span added to it
*/
/**************************************************************************/
DateTime DateTime::operator+(const TimeSpan& span) 
{
	return DateTime(unixtime() + span.totalseconds());
}

/**************************************************************************/
/*!
	@brief  Subtract a TimeSpan from the DateTime object
	@param span TimeSpan object
	@return new DateTime object with span subtracted from it
*/
/**************************************************************************/
DateTime DateTime::operator-(const TimeSpan& span) 
{
	return DateTime(unixtime() - span.totalseconds());
}

/**************************************************************************/
/*!
	@brief  Subtract one DateTime from another
	@param right The DateTime object to subtract from self (the left object)
	@return TimeSpan of the difference between DateTimes
*/
/**************************************************************************/
TimeSpan DateTime::operator-(const DateTime& right) 
{
	return TimeSpan(unixtime() - right.unixtime());
}

/**************************************************************************/
/*!
	@brief  Is one DateTime object less than (older) than the other?
	@param right Comparison DateTime object
	@return True if the left object is older than the right object
*/
/**************************************************************************/
bool DateTime::operator<(const DateTime& right) const 
{
	return unixtime() < right.unixtime();
}

/**************************************************************************/
/*!
	@brief  Is one DateTime object equal to the other?
	@param right Comparison DateTime object
	@return True if both DateTime objects are the same
*/
/**************************************************************************/
bool DateTime::operator==(const DateTime& right) const 
{
	return unixtime() == right.unixtime();
}

/**************************************************************************/
/*!
	@brief  ISO 8601 Timestamp
	@param opt Format of the timestamp
	@return Timestamp string, e.g. "2000-01-01T12:34:56"
*/
/**************************************************************************/
String DateTime::timestamp(timestampOpt opt) 
{
	char buffer[20];

	//Generate timestamp according to opt
	switch (opt) 
	{
	case TIMESTAMP_TIME:
		//Only time
		sprintf(buffer, "%02d:%02d:%02d", hh, mm, ss);
		break;
	case TIMESTAMP_DATE:
		//Only date
		sprintf(buffer, "%d.%02d.%02d", 2000 + yOff, m, d);
		break;
	default:
		//Full
		sprintf(buffer, "%d.%02d.%02d %02d:%02d:%02d", 2000 + yOff, m, d, hh, mm, ss);
	}
	return String(buffer);
}

/**************************************************************************/
/*!
	@brief  Given a date, return number of days since 2000/01/01, valid for 2001..2099
	@param y Year
	@param m Month
	@param d Day
	@return Number of days
*/
/**************************************************************************/
uint16_t DateTime::date2days(uint16_t y, uint8_t m, uint8_t d) const
{
	if (y >= 2000U)
		y -= 2000U;
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(daysInMonth + i - 1);
	if (m > 2 && y % 4 == 0)
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

/**************************************************************************/
/*!
	@brief  Given a number of days, hours, minutes, and seconds, return the total seconds
	@param days Days
	@param h Hours
	@param m Minutes
	@param s Seconds
	@return Number of seconds total
*/
/**************************************************************************/
long DateTime::time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) const
{
	return ((days * 24L + h) * 60 + m) * 60 + s;
}
