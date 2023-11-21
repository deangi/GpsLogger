// Initial Version 12-Nov-2023, Dean Gienger
// Add disconnect handler 15-Nov-2023, Dean Gienger
// Add error timeout handler 16-Nov-2023, Dean Gienger
//----------------------------------------------------------------------------
// WIFI connect disconnect
//
// This is a state machine that controls connecting to a WiFi AP
// There's a wifiService() to call once per second
//
//----------------------------------------------------------------------------
// Call wifiService() from one second loop
// Call wifiConnect() from setup after config file is read
//
// Needs char[64] wifissid    and  char[64] wifipwd   defined

//  Use Sketch | Add file to add the file from the Components folder in the Arduino IDE
//  Use #include "WiFiService.h" in the main folder
#define WIFISTATE_DISCONNECTED (0)
#define WIFISTATE_CONNECTING   (1)
#define WIFISTATE_CONNECTED    (2)
#define WIFISTATE_DISCOWAIT    (3)
#define WIFISTATE_ERRORTIMEOUT (4)

#define WIFIWAIT_MAX (30) /* seconds to wait before connect request times out */
#define WIFIRECONNECT_MAX (60) /* seconds to wait after a connect times out before retrying */
#define WIFIDISCOWAIT_MAX (10) /* seconds after a disconnect is discovered before trying to connect again */
int wifiState = WIFISTATE_DISCONNECTED;
int wifiWaitSecondCounter = 0;

//-- forward defs for logging
//void logMessage(char* msg);
//void logMessage(String msg);
#define WIFILOG Serial.println

void wifiInit()
{
  wifiState = WIFISTATE_DISCONNECTED;
  wifiWaitSecondCounter = 0;
}

void wifiDisconnect()
{
  // initiate a disconnect from an AP
  wifiState = WIFISTATE_DISCONNECTED;
  wifiWaitSecondCounter = 0;
  WiFi.disconnect();
  WIFILOG("Wifi disconnect");
}

void wifiConnect()
{
  // initiate a connect to an AP  
  wifiState = WIFISTATE_CONNECTING;
  wifiWaitSecondCounter = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifissid,wifipwd);
  WIFILOG("Wifi connection initiated");
}

void wifiService()
{
  //Serial.println("WiFi service");
  // call once per second
  if (wifiState == WIFISTATE_CONNECTING)
  {
    // if we are trying to connect
    // see if we are connected 
    if (WiFi.status() == WL_CONNECTED)
    {
      // if connect succeded, note that and stop waiting
      wifiState = WIFISTATE_CONNECTED;
      wifiWaitSecondCounter = 0;
      WIFILOG("Wifi connected as ");
      WIFILOG(WiFi.localIP());
    }
    else
    {
      // wifi not yet connected, just wait a bit
      if (++wifiWaitSecondCounter >= WIFIWAIT_MAX)
      {
        // or if we wait too long, declare a timeout error
        wifiState = WIFISTATE_ERRORTIMEOUT; // goto timeout mode
        wifiWaitSecondCounter = 0;
        WiFi.disconnect();
        WIFILOG("WiFi connection timed-out");
      }
    }
  }
  // handle disconnects
  if (wifiState == WIFISTATE_DISCOWAIT)
  {
    // wait a bit after disconnect is detected before we try to reconnect
    //Serial.println("Waiting after wifi disconnect is discovered");
    if (++wifiWaitSecondCounter >= WIFIDISCOWAIT_MAX) //  wait for disconnects before we try reconnecting
    {
      wifiConnect(); // start reconnect attempt
    }
  }
  if ((wifiState == WIFISTATE_CONNECTED) && (WiFi.status() != WL_CONNECTED))
  {
    // detect when connection has failed
    // should be connected, but somehow it's gotten disconnected
    WIFILOG("WiFi disconnect discovered");
    wifiState = WIFISTATE_DISCOWAIT;
    wifiWaitSecondCounter = 0;    
  }
  if (wifiState == WIFISTATE_ERRORTIMEOUT)
  {
    if (++wifiWaitSecondCounter >= WIFIRECONNECT_MAX)
    {
      wifiConnect(); // try reconnect now
    }
  }
}

int wifiIsConnected()
{
  return wifiState == WIFISTATE_CONNECTED;
}
