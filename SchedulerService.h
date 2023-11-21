// Initial version 12-Nov-2023, Dean Gienger
//

//----------------------------------------------------------------------------
// Time trigger detectors, detect when second, minute, hour, and day tick
//
// These are used for scheduling tasks.   All must be called at least 2x as
// fast as their respective time periods.   So second detector must be called
// at least 2x per second.
//
//----------------------------------------------------------------------------

//  Use Sketch | Add file to add the file from the Components folder in the Arduino IDE
//  Use #include "SchedulerService.h" in the main folder


int lastDay = -1;
//-------------------------------------------------------------
// one day detector - return true when time goes
//   from 23:59:59 to 00:00:00
int dayDetector()
{
  // return true one time every day
  int day = rtc.getDay();
  if (day == lastDay) return false;
  if (lastDay==-1)
  {
    lastDay = day;
    return false;
  }

  lastDay = day;
  return true;
}

int lastHr = -1;
//-------------------------------------------------------------
// Top of the hour detector, return true
//   when time ticks from xx:59:59 to xx:00:00
int hourDetector()
{
  // Return true one time every hour, no mater how many
  // times this is called.
  int hr = rtc.getHour(true);
  if (hr == lastHr) return false;

  lastHr = hr;
  return true;
}


int lastMin = -1;
//-------------------------------------------------------------
// One minute detector - return true when time ticks from
//  xx:xx:59 to xx:xx:00 
int minuteDetector()
{
  // Return true one time every second, no mater how many
  // times this is called.
  int zmin = rtc.getMinute();
  if (zmin == lastMin) return false;

  lastMin = zmin;
  return true;
}

int lastSec = -1;
//-------------------------------------------------------------
// One second detector - return true once per second
int secondDetector()
{
  // Return true one time every second, no mater how many
  // times this is called.
  int sec = rtc.getSecond();
  if (sec == lastSec) return false;

  lastSec = sec;
  return true;
}

void schedulerInit()
{
  lastSec = rtc.getSecond();
  lastMin = rtc.getMinute();
  lastHr  = rtc.getHour();
  lastDay = rtc.getDay();
}

