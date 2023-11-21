// Initial version, 12-Nov-2023
// 16-Nov-2023 - ntpCompleted() added

//----------------------------------------------------------------------------
// NTP (Network Time Protocol) Handler
//----------------------------------------------------------------------------
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
#define NTPSTATE_IDLE     (0)
#define NTPSTATE_STARTED  (1)
#define NTPSTATE_COMPLETE (2)
#define NTPSTATE_TIMEOUTERROR (3)
#define NTPWAIT_MAX (30) /* seconds */

int ntpState = NTPSTATE_IDLE;
int ntpWaitSecondCounter = 0;
int ntpAttempts = 0;
int ntpSuccess = false;

void schedulerInit();

void ntpInit()
{
  ntpState = NTPSTATE_IDLE;
  ntpWaitSecondCounter = 0;
  ntpAttempts = 0;
  ntpSuccess = false;
}

void ntpService()
{
  if (ntpState == NTPSTATE_STARTED)
  {
    //Serial.println(ntpWaitSecondCounter);
    
    if (timeClient.update())
    {
      unsigned long epochTime = timeClient.getEpochTime();
      //Serial.print("NTP Time received: ");
      //Serial.println(timeClient.getFormattedDate());
      rtc.setTime(epochTime);
      Serial.println(rtc.getTime("NTP->RTC=%Y/%m/%d,%H:%M:%S"));
      schedulerInit();
      ntpSuccess = true;
      ntpAttempts++;
      ntpState=NTPSTATE_COMPLETE;
    }
    else
    {
      timeClient.forceUpdate();
      if (++ntpWaitSecondCounter >= NTPWAIT_MAX) // timeout
      {
        ntpState = NTPSTATE_TIMEOUTERROR;
        ntpSuccess = false;
      }
    }
  }
}

void ntpStart()
{
  // NTP Client
  //Serial.println("NTP Start");
  timeClient.begin();
  timeClient.setTimeOffset(0);
  ntpState = NTPSTATE_STARTED;
  ntpWaitSecondCounter = 0;
  ntpSuccess = false;
}

int ntpStarted()
{
  return ntpState == NTPSTATE_STARTED;
}

int ntpComplete()
{
  return ntpSuccess;  
}
