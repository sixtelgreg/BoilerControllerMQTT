/*
 * SimpleTimer.cpp
 *
 * SimpleTimer - A timer library for Arduino.
 * Author: mromani@ottotecnica.com
 * Copyright (c) 2010 OTTOTECNICA Italy
 *
 * Callback function parameters added & compiler warnings
 * removed by Bill Knight <billk@rosw.com> 20March2017
 *
 * This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser
 * General Public License along with this library; if not,
 * write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */


#include "SimpleTimer.h"


// Select time function:
//static inline unsigned long elapsed() { return micros(); }
//static inline unsigned long elapsed() { return millis(); }


SimpleTimer::SimpleTimer() 
{
	auto current_millis = millis();

	for (int i = 0; i < MAX_TIMERS; i++) 
	{
		memset(&timer[i], 0, sizeof (timer_t));
		timer[i].prev_millis = current_millis;
	}

	numTimers = 0;
}


void SimpleTimer::run() 
{
	byte i;
	//unsigned long current_millis;

	// get current time
	auto current_millis = millis();

	for (i = 0; i < MAX_TIMERS; ++i) 
	{

		timer[i].toBeCalled = DEFCALL_DONTRUN;

		// no callback == no timer, i.e. jump over empty slots
		if (timer[i].callback != NULL) 
		{

			// is it time to process this timer ?
			// see http://arduino.cc/forum/index.php/topic,124048.msg932592.html#msg932592

			if ((current_millis - timer[i].prev_millis) >= timer[i].delay) 
			{

				// update time
				timer[i].prev_millis += timer[i].delay;

				// check if the timer callback has to be executed
				if (timer[i].enabled) 
				{

					// "run forever" timers must always be executed
					if (timer[i].maxNumRuns == RUN_FOREVER) 
					{
						timer[i].toBeCalled = DEFCALL_RUNONLY;
					}
					// other timers get executed the specified number of times
					else if (timer[i].numRuns < timer[i].maxNumRuns) 
					{
						timer[i].toBeCalled = DEFCALL_RUNONLY;
						timer[i].numRuns++;

						// after the last run, delete the timer
						if (timer[i].numRuns >= timer[i].maxNumRuns) 
						{
							timer[i].toBeCalled = DEFCALL_RUNANDDEL;
						}
					}
				}
			}
		}
	}

	for (i = 0; i < MAX_TIMERS; ++i) 
	{
		if (timer[i].toBeCalled == DEFCALL_DONTRUN)
			continue;

		if (timer[i].hasParam)
			(*(timer_callback_p)timer[i].callback)(timer[i].param);
		else
			(*(timer_callback)timer[i].callback)();

		if (timer[i].toBeCalled == DEFCALL_RUNANDDEL)
			deleteTimer(i);
	}
}


// find the first available slot
// return -1 if none found
int SimpleTimer::findFirstFreeSlot() 
{
	// all slots are used
	if (numTimers >= MAX_TIMERS) 
	{
		return -1;
	}

	// return the first slot with no callback (i.e. free)
	for (byte i = 0; i < MAX_TIMERS; ++i)
	{
		if (timer[i].callback == NULL) 
		{
			return i;
		}
	}

	// no free slots found
	return -1;
}


int SimpleTimer::setupTimer(unsigned long d, void* f, void* p, boolean h, unsigned n) 
{
	int freeTimer;

	freeTimer = findFirstFreeSlot();
	if (freeTimer < 0 || f == NULL)
	{
		return -1;
	}

	timer[freeTimer].delay = d;
	timer[freeTimer].callback = f;
	timer[freeTimer].param = p;
	timer[freeTimer].hasParam = h;
	timer[freeTimer].maxNumRuns = n;
	timer[freeTimer].enabled = true;
	timer[freeTimer].prev_millis = millis();

	numTimers++;

	return freeTimer;
}


int SimpleTimer::setTimer(unsigned long d, timer_callback f, unsigned n) 
{
  return setupTimer(d, (void *)f, NULL, false, n);
}

int SimpleTimer::setTimer(unsigned long d, timer_callback_p f, void* p, unsigned n) 
{
  return setupTimer(d, (void *)f, p, true, n);
}

int SimpleTimer::setInterval(unsigned long d, timer_callback f) 
{
	return setupTimer(d, (void *)f, NULL, false, RUN_FOREVER);
}

int SimpleTimer::setInterval(unsigned long d, timer_callback_p f, void* p) 
{
  return setupTimer(d, (void *)f, p, true, RUN_FOREVER);
}

int SimpleTimer::setTimeout(unsigned long d, timer_callback f) 
{
	return setupTimer(d, (void *)f, NULL, false, RUN_ONCE);
}

int SimpleTimer::setTimeout(unsigned long d, timer_callback_p f, void* p) 
{
  return setupTimer(d, (void *)f, p, true, RUN_ONCE);
}

unsigned long SimpleTimer::leftTime(unsigned timerId)
{
	if (numTimers == 0 || timerId >= MAX_TIMERS)
		{ return 0; }

	auto pass = min((millis() - timer[timerId].prev_millis), timer[timerId].delay);

	return (timer[timerId].delay - pass);
}


void SimpleTimer::deleteTimer(unsigned timerId) 
{
	// nothing to delete if no timers are in use
	if (numTimers == 0 || timerId >= MAX_TIMERS) 
		{ return; }

	// don't decrease the number of timers if the
	// specified slot is already empty
	if (timer[timerId].callback != NULL) 
	{
		memset(&timer[timerId], 0, sizeof (timer_t));
		timer[timerId].prev_millis = millis();

		// update number of timers
		numTimers--;
	}
}


// function contributed by code@rowansimms.com
void SimpleTimer::restartTimer(unsigned timerId) 
{
	if (timerId >= MAX_TIMERS) 
		{ return; }

	timer[timerId].prev_millis = millis();
}


boolean SimpleTimer::isEnabled(unsigned timerId) 
{
	if (timerId >= MAX_TIMERS) 
		{ return false; }

	return timer[timerId].enabled;
}


void SimpleTimer::enable(unsigned timerId) 
{
	if (timerId >= MAX_TIMERS) 
		{ return; }

	timer[timerId].enabled = true;
}


void SimpleTimer::disable(unsigned timerId) 
{
	if (timerId >= MAX_TIMERS) 
		{ return; }

	timer[timerId].enabled = false;
}


void SimpleTimer::toggle(unsigned timerId)
{
	if (timerId >= MAX_TIMERS) 
		{ return; }

	timer[timerId].enabled = !timer[timerId].enabled;
}


unsigned SimpleTimer::getNumTimers() 
{
	return numTimers;
}
