//----------------------------------------------------------------------------
// GPS server-logger
//
// Dean Gienger, November 9, 2023
//
// Hardware:
//
// 1) This application is for the ESP-32 eco system.
//    LOLIN32 V1.0.0 board, 12-Nov-2023
// 2) There will be a GPS module connected to an ESP32 serial port (TTL-Serial)
//
// Operational Description
// 1) ESP32 will boot-up and initialize
// 2) ESP32 will monitor the serial port that the GPS module is connected to and save the data to a log file
// 3) Periodically the ESP32 will turn on WiFi and attempt to FTP the log file to a remote server
// 4) When WiFi is on, ESP32 will attempt to get the network time via NTP
// 5) If time is available from the GPS module, update to that time
// 6) Telnet server so we can monitor real-time if desired by telnetting into it???
//
// Configuration
// 1) There will be a configuration file (config.ini) stored on the SD card which will have information
//    a) WiFi SSID and password
//    b) How many hours between attempts to upload the log files to an FTP server
//    c) FTP details (server name, user name, password)
//    d) Baud rate for GPS module
//    f) Time zone offset
//
//
// ESP32 modules needed
// 1) WiFi module
// 2) NTP
// 3) Serial port handler - to GPS module
// 4) File system
// 6) Time of day handler real-time-clock
//
//----------------------------------------------------------------------------
// Developement Log:
//  9-Nov-2023 - Initial planning and structure V0.1
// 12-Nov-2023 - Add WiFi connection and NTP V0.2
// 13-Nov-2023 - V0.3 - add GPS, partition into services with separate .h files for Wifi, NTP, Scheduler, FileSystem
// 13-Nov-2023 - V0.4 - add Telnet interface
// 14-Nov-2023 - V0.5 - LOLIN32 ESP32 board version
// 15-Nov-2023 - V1.0 - Log GPRMC (GNRMC, BNRMC) NEMA messages to flash file system
// 15-Nov-2023 - V1.1 - add sio shell commands, WIFI disconnect/reconnect
// 16-Nov-2023 - V1.2 - add event logging, wifi connect timeout handler

// Signon message with version number
#define SIGNON "\nGPS Monitor V1.2 (16Nov2023)\n\n"

//---- TODO ideas ----
// **DONE**
// - Event log file for critical events
// - New wifi test - drive off and back on property to simulate reconnect - didn't seem to work when we drove to Bruce's
//    - detect worked, but first reconnect timed out, then state machine was stuck in disconnect mode.
//    - update wifiService to have a timer for time-out mode, then reconnect
// - FTP file to server (telnet command?) - cat is NG too-slow

// **FUTURE**
// FTP client doesn't report errors very well - verify somehow that the upload worked
// - Reboot - dayly
// - update time from the GPS if we can
// - Parse the NEMA RMS command to something like 2023/11/15,09:51:00,151.991W,47.45N,20kts
// - Post to the CodeProject site with article
// - Some way to get status out of the thing - optically with LED??? optional sio port? display?  bluetooth?
// - Can possibly use the tool GPSBabel (gpsbabel.org) to convert NEMA RMC log to a GPX file to import in Google Maps to show track
//
//----------------------------------------------------------------------------
//                     G P S   M O D U L E
//
// GPS Module: Most any module that has a TTL-RS232 will work with this.
// Example I used was this one from Amazon, $15.88 for two modules:
// Teyleten Robot ATGM336H GPS+BDS Dual-Mode Module Flight Control Satellite Positioning Navigator
// https://www.amazon.com/dp/B09LQDG1HY?psc=1&ref=ppx_yo2ov_dt_b_product_details
//
// Four connections from this module to the ESP32 are
// GPS     ESP32
// ------- -----------------
// GND     GND
// VCC     3.3V
// TXD     RXD (can be mapped, ESP32 Serial 2 uses GPIO16 by default)
// RXD     TXD (can be mapped, ESP32 Serial 2 uses GPIO17 by default)
//
// Note: The 3.3V supply from an ESP32 board is somewhat limited and can
//    vary from one ESP32 implementation to the next.   Check that the
//    ESP32 board you choose has sufficient capacity to power the GPS module.
//
//----------------------------------------------------------------------------
//            F I L E   S Y S T E M
// Depending on which file system you want, uncomment one of these.
//   either the SD_MMC interface if your ESP32 has such, or the internal SPI
//   flash file system (SPIFFS)
//   - two things are stored on the file system
//   1) config.ini - read at startup to configure itself
//   2) location.log - written with location tracking information
//
//#define WANTSD_MMC 1
#define WANTSPIFFS 1
// if you have SD_MMC, put config.ini in the root folder of the SD card
// if you have SPIFFS, put config.ini in the data folder under this schetch
//   and use tools | ESP32 Sketch Data Upload to upload the initial flash file system
//

//
//----------------------------------------------------------------------------
//   L O C A T I O N   L O G G I N G
//----------------------------------------------------------------------------
// We're going to log a location (if the GPS module has one) once per minute
// to a file in the flash file system.   
// Each entry in the location log file will have the date, time, latitude, longitude
//
// We will update the flash file with logged information once per hour
//
// We can get the log file out by using the telnet connection
//
// cat /location.log

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h> // for NTP
#include <NTPClient.h> // for NTP

#include "FS.h" // for file system
#include "SPI.h"

#ifdef WANTSD_MMC
#include "SD_MMC.h" /* SD_MMC library */
#endif
#ifdef WANTSPIFFS
#include <SPIFFS.h> // on-card flash file system
#endif

#include <ESP32_FTPClient.h>

//-- forward defs for logging
void logMessage(char* msg);
void logMessage(String msg);

#include <ESP32Time.h> // real-time clock
#include "ESPTelnet.h" // telnet interface

/* comfort LED, turns on when telnet is connected */
#define LEDPIN (2) 

#define CONFIGFN "/config.ini"

#define LOGFN "/location.log"

#define EVENTFN "/event.log"

//-----------------------------------------------------------------
// real time clock (software based, not backed up for power failures
ESP32Time rtc(-8*3600);  // -8 from GMT by default

//----------------------------------------------------------------------------
// hardware serial port access for GPS module (RS232 TTL)
//----------------------------------------------------------------------------
#define GPSPORT Serial2
#define GPSESP_RXD_PIN (16)
#define GPSESP_TXD_PIN (17)

#define GPSBUFLEN (512)
char gpsRxBuf[GPSBUFLEN+2];
char gpsRxLine[GPSBUFLEN];
int gpsBufPtr = 0;
int gpsLineAvail = 0;
void gpsInit(long baudrate)
{
  GPSPORT.begin(baudrate, SERIAL_8N1, GPSESP_RXD_PIN, GPSESP_TXD_PIN);
  //GPSPORT.setRxBufferSize(GPSBUFLEN);
  gpsBufPtr = 0;
  gpsLineAvail=0;
}

char* gpsService()
{
  if (GPSPORT.available())
  {
    char c = GPSPORT.read()  & 0x7f; // 7 bits are important
    if ((c == 10) || (c == 13)) // if it's end-of-line
    {
      if (gpsBufPtr > 0) // if the line is not empty
      {
        gpsRxBuf[gpsBufPtr]=0; // copy to gpsRxLine
        strcpy(gpsRxLine, gpsRxBuf); // copy received line
        gpsLineAvail++; // set flag indicating line is available
        gpsBufPtr = 0;
        return &gpsRxLine[0];
      }
    }
    else
    {
      gpsRxBuf[gpsBufPtr] = c;
      if (gpsBufPtr < (GPSBUFLEN-1)) gpsBufPtr++;
    }
  }
  return NULL;
}



//--------------------------------------------------------------------------
//       F I L E   S Y S T E M
//
// There's a pretty simple file system - stored either on SD_MMC card or
// for the internal flash of the ESP32.   It has no folders as far as this
// application goes, only a root folder /
// For SD_MMC - the size could be quite large - like 128 Gb
// For internal mode - the size quite limited - like <2Mb
//

#ifdef WANTSD_MMC
fs::FS & fileSystem = SD_MMC;
#endif
#ifdef WANTSPIFFS
fs::FS & fileSystem = SPIFFS;
#endif

#include "FileSystemService.h"

//----------- Telnet shell command handlers

void lsCmd(String str)
{
  // "ls"
  listDir(fileSystem,"/",9);
}

void catCmd(String str)
{
  // cat /file.txt
  String fn = str.substring(/*cat */4);
  readFile(fileSystem, fn.c_str());
}

void cpCmd(String str)
{
  // cp /file1.txt /file2.txt
  String tmp = str.substring(3); // cp_
  int idx = tmp.indexOf(" ");
  if (idx < 0)
  {
    zprintln("cp command error:  cp src.file dest.file");
    return;
  }
  String srcfile = tmp.substring(0,idx);
  String destfile = tmp.substring(idx+1);
  copyFile(fileSystem, srcfile.c_str(), destfile.c_str());  
}

void rmCmd(String str)
{
  // rm /file.txt
  String fn = str.substring(3);
  deleteFile(fileSystem, fn.c_str());
}

void apCmd(String str)
{
  // ap /file1.txt some line to be added to the end of the file
  String tmp = str.substring(3); 
  int idx = tmp.indexOf(" ");
  if (idx < 0)
  {
    zprintln("ap command error:  ap src.file some line of text");
    return;
  }
  String srcfile = tmp.substring(0,idx);
  String msg = tmp.substring(idx+1);
  appendFile(fileSystem, srcfile.c_str(), msg.c_str());  
}

void ftpPut(char* fn);
void ftpCmd(String str)
{
  zprintln("FTP log file to server");
  ftpPut(LOGFN);
}

//----------------------------------------------------------------------------
//   L O G G I N G   S E R V I C E
//----------------------------------------------------------------------------
// externals required:
// rtc - ESP32Time clock
// fileSystem - file system name
//
#define LOGFILENAMELEN (64)

char logFileName[LOGFILENAMELEN+2];
int wantTimeTag = false;

void logInit(char* logfn, int timeTagWanted)
{
  strncpy(logFileName, logfn, LOGFILENAMELEN);  
  wantTimeTag = timeTagWanted != 0; // true if time tag wanted
}

void logMessage(char* msg)
{
  // time tagged loglines look like this:   2023/11/15,09:30:00,this is my message
  //  
  // rtc.getTime("%Y/%m/%d %H:%M:%S  ")

  String logmsg;
  if (wantTimeTag)
  {
    logmsg = rtc.getTime("%Y/%m/%d %H:%M:%S,")+String(msg);
  }
  else
  {
    logmsg = String(msg);
  }
  // append it to the file
  Serial.println(logmsg);
  File file = fileSystem.open(logFileName, FILE_APPEND);
  if(!file){
    Serial.println("- failed to open log file for appending");\
    return;
  }
  if(!file.println(logmsg)){
     Serial.println("- append failed");
  }
  file.close();
}

void logMessage(String msg)
{
  logMessage(msg.c_str());
}


//----------------------------------------------------------------------------
// Configuration file handler
//----------------------------------------------------------------------------
char wifissid[64];
char wifipwd[64];
char ftpServer[128];
char ftpUser[64];
char ftpPwd[64];
char tmpbuf[128];
char ftpUploadFolder[128];
long tzOffsetSec; // 32 bit
long baudRate;
int  hrsPerUploadAttempt = 1;

int readConfigFile(char* configFn)
{
  Serial.println("Reading config file");
  
  int retval = true;  // return true if successful
  retval &= readKey(configFn, "WIFISSID=", wifissid, 63);
  retval &= readKey(configFn, "WIFIPASSWORD=", wifipwd, 63);
  retval &= readKey(configFn, "TZOFFSETSEC=", tmpbuf, 63);
  tzOffsetSec = atol(tmpbuf);
  retval &= readKey(configFn, "FTPSERVER=", ftpServer, 127);
  retval &= readKey(configFn, "FTPUSER=", ftpUser, 63);
  retval &= readKey(configFn, "FTPPASSWORD=", ftpPwd, 63);
  retval &= readKey(configFn, "BAUDRATE=", tmpbuf, 63);
  retval &= readKey(configFn, "FTPFOLDER=", ftpUploadFolder, 127);
  
  baudRate = atol(tmpbuf);
  //retval &= readKey(configFn, "HOURSPERUPLOAD=", tmpbuf, 63);
  //hrsPerUploadAttempt = atoi(tmpbuf);

  // some error checking
  if (hrsPerUploadAttempt < 1) hrsPerUploadAttempt = 1;
  if (hrsPerUploadAttempt > 12) hrsPerUploadAttempt = 12;
  if (baudRate < 1200) baudRate = 1200;
  if (baudRate > 115200) baudRate = 115200;
  if (tzOffsetSec < 0) tzOffsetSec = 0;
  if (tzOffsetSec > 13*3600) tzOffsetSec = 0;

  return retval;
}

void ftpPut(char* fn)
{
  char linebuf[256];

  String targetfn = rtc.getTime("gpslog_%Y%m%d-%H%M%S.log"); // "20221116-182201"
  zprint("FTP to "); zprintln(targetfn);
  ESP32_FTPClient ftp (ftpServer,ftpUser,ftpPwd, 5000, 1); // 50 sec timeout, verbose=1
  ftp.OpenConnection();
  ftp.InitFile("Type I");
  ftp.ChangeWorkDir(ftpUploadFolder);
  ftp.NewFile(targetfn.c_str());

  // open file and read each line - send to FTP
  File finp = fileSystem.open(LOGFN, FILE_READ);
  if (!finp)
  {
    zprintln("Unable to read log file");
    return;
  }

  while (readln(finp, (uint8_t*)linebuf, 250))
  {
    strcat(linebuf,"\n");
    ftp.WriteData( (unsigned char*)linebuf, strlen(linebuf) );
  }
  finp.close();
  zprintln("FTP upload completed.");
  ftp.CloseFile();
  ftp.CloseConnection();  

}
//----------------------------------------------------------------------------
//             W I F I   S E R V I C E
#include "WiFiService.h"
//----------------------------------------------------------------------------
//             N T P   S E R V I C E
#include "NTPService.h"
// network time protocol


//----------------------------------------------------------------------------
// Telnet code
//----------------------------------------------------------------------------
// Telnet interface
ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;
int telnetConnected = false;

//---------- Telnet printing

void zprint(char * msg)
{
  Serial.print(msg);
  if (telnetConnected)
  {
    telnet.print(msg);  
  }
}

void zprint(String msg)
{
  Serial.print(msg);
  if (telnetConnected) 
    telnet.print(msg);
}

void zprintln(char * msg)
{
  Serial.println(msg);
  if (telnetConnected)
    telnet.println(msg);  
}

void zprintln(String msg)
{
  Serial.println(msg);
  if (telnetConnected) 
    telnet.println(msg);
}

void zprint(int x)
{
  char buf[16];
  sprintf(buf,"%d",x);
  zprint(buf);
}

void zprintln(int x)
{
  zprint(x); zprint("\015\012");
}

#include "sioService.h"

//---------------------------------------------------------------------
//        S I M P L E   S H E L L   C O M M A N D   H A N D L E R
//---------------------------------------------------------------------
int gpsTelnetEcho = true;
int gpsSerialEcho = false;

void handleShellCommand(String str)
{
  Serial.println(str);
  if (str=="ls")
    lsCmd(str);
  else if (str.startsWith("cat "))
    catCmd(str);
  else if (str.startsWith("cp "))
    cpCmd(str); 
  else if (str.startsWith("rm "))
    rmCmd(str);
  else if (str.startsWith("ap "))
    apCmd(str); // append one line at a time to a file
  else if (str.startsWith("on"))
    gpsTelnetEcho = true;
  else if (str.startsWith("off"))
    gpsTelnetEcho = false;
  else if (str.startsWith("son"))
    gpsSerialEcho = true;
  else if (str.startsWith("soff"))
    gpsSerialEcho = false;
  else if (str.startsWith("test"))
    WiFi.disconnect(); // TODO - add anything you want here for testing purposes
  else if (str.startsWith("ftp"))
    ftpCmd(str);
  else
    zprintln("\r\n ehh?");
  telnet.print(">"); // prompt

}


//----------------------------------------------------------------------------
//        T E L N E T   S E R V I C E
// There's a very simple shell with the following commands
//
// ls                                     - list files in the home folder
// cat /config.ini                        - dump a file to the console
//  cp /config.ini /config.bck            - copy a file
//  rm /config.bck                        - remove a file
//  ap /test.log Log line                 - append a line to a file
// -- echo GPS info to telnet switch (on by default)
//  on
//  off

void onInputReceived(String str)
{
  handleShellCommand(str);
}

void onTelnetConnect(String ip) {
  digitalWrite(LEDPIN,HIGH);
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("(Use ^] + q  to disconnect.)");
  telnet.print(">"); // prompt
}


void onTelnetDisconnect(String ip) {
  digitalWrite(LEDPIN,LOW);
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
  telnetConnected = false;
}

void onTelnetReconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
  telnetConnected = true;
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connect");
}

void setupTelnet() {  
  telnetConnected = false;
  Serial.print("- Telnet: "); Serial.print(ip); Serial.print(" "); Serial.println(port);  telnetConnected = true;
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onInputReceived);

  telnet.setLineMode(true);
  
  Serial.print("- Telnet Line Mode: "); Serial.println(telnet.isLineModeSet() ? "YES" : "NO");
  
  Serial.print("- Telnet: ");
  if (telnet.begin(port)) {
    Serial.println("running");
  } else {
    Serial.println("error.");
    //errorMsg("Will reboot...");
  }
  
}

#include "SchedulerService.h"
//----------------------------------------------------------------------------
//             L O G  F I L E  S E R V I C E
//
// We can append lines to a log file.
// Since it's a flash file system, we want to minimize the number of times
// flash is written (as it has a limited number of write cycles before it's
// worn out.
//
// So we keep some buffer of lines and stuff them in the buffer.
// Then every so often we'll dump the buffered lines to the file system.
//
// Line buffer size in characters (auto flush when full)
#define LOGBUFFERSIZE (8*1024)
char logbuffer[LOGBUFFERSIZE];
int bufferWritePosition = 0;

void gpsLogInit()
{
  bufferWritePosition = 0; // next character to write
}

void gpsLogLine(char* msg)
{
  int len = strlen(msg);
  if (len == 0) return; // nothing to log
  
  // we don't want to overflow the buffer, so if this line would overflow it,
  // then we'll flush it first
  int lastbyte = bufferWritePosition+len+8; // little buffer at the end
  if (lastbyte >= LOGBUFFERSIZE) gpsLogFlush();
  strcpy(&logbuffer[bufferWritePosition], msg);
  bufferWritePosition += len;
  logbuffer[bufferWritePosition++] = '\015'; // CR
  logbuffer[bufferWritePosition++] = '\012'; // LF
  // no \0 between lines!
}


void gpsLogFlush()
{
  if (bufferWritePosition > 0)
  {
    File file = fileSystem.open(LOGFN, FILE_APPEND);
    if(!file){
      Serial.println("- failed to open log file for appending");\
      bufferWritePosition = 0;
      return;
   }
   logbuffer[bufferWritePosition] = '\0'; // null terminate
   if(file.print(logbuffer)){
      Serial.println("- message appended");
   } else {
      Serial.println("- append failed");
   }
   file.close();
  }
  bufferWritePosition = 0;
}

int setupTelnetDone = false;
int ntpDone = false;
char rmcbuf[128]; // temporarily stores $GxRMC messages, log once per minute 

//----------------------------------------------------------------------------
// setup() - runs one time when the ESP32 boots up
//----------------------------------------------------------------------------
void setup() 
{
  ntpDone = false;
  // Set up the serial port for diagnostic purposes
  Serial.begin(115200);
  // output a signon message to diagnostic port
  Serial.println(SIGNON);

  pinMode(LEDPIN,OUTPUT); // init LED comfort pin

  //----------- initialize the file system ---------------
  // can be either on an SD card or use the built-in flash
  // with the SPI flash file service (SPIFFS)
  // If we can't connect to the file system, the boot-up
  // fails and we can't really go operational.
    
#ifdef WANTSD_MMC
  // SD card setup
  if(!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    for (;;);
  }
  uint8_t cardType = SD_MMC.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    for (;;);
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
#endif
#ifdef WANTSPIFFS
  // Initialize SPIFFS (file system)
  if(!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  else
  {
    Serial.println("SPIFFS mounted");
  }
#endif

  // initialize the RTC, uses timer 0
  rtc.setTime(00,00,00, 1, 1, 2023); // default time 00:00:00 1/1/2023

  logInit(EVENTFN, true);
  
  listDir(fileSystem, "/", 1); // for dev purposes, show the file system on boot up

  // read config file
  if (!readConfigFile(CONFIGFN))
  {
    logMessage("Unable to read config file");
  }

  schedulerInit(); // initialize the scheduler used by the loop() function

  // initiate a WIFI connect
  wifiConnect();

  gpsLogInit(); 

  // Set up serial port for connection to GPS module
  gpsInit(baudRate); 

  rmcbuf[0] = '\0';
  
  setupTelnetDone = false;
  sioInit();  // diagnostic serial port input service
  
  //log_d("Total heap: %d", ESP.getHeapSize());
  //log_d("Free heap: %d", ESP.getFreeHeap());
  //("Total PSRAM: %d", ESP.getPsramSize());
  //log_d("Free PSRAM: %d", ESP.getFreePsram());
  
  Serial.print("\n>");  // initial serial prompt
}

//----------------------------------------------------------------------------
// Main repeatitive tasks go here.  This is called over and over endlessly
// once setup() has completed.
//----------------------------------------------------------------------------
void loop() 
{
  // This is sort of a poor-person's operating system - scheduling tasks
  // at periodic intervals.

  //----------------------------
  // high rate tasks here
  //----------------------------
  char* line = gpsService();
  if (line != NULL)
  {
    if (gpsSerialEcho) Serial.println(line);
    if (gpsTelnetEcho && telnetConnected) telnet.println(line);
    // $GxRMC
    //Serial.print(line[0]); Serial.print(line[1]); Serial.print(line[3]); Serial.print(line[4]); Serial.println(line[5]);    
    if ((line[1] == 'G') &&
        (line[3] == 'R') &&
        (line[4] == 'M') &&
        (line[5] == 'C'))  strcpy(rmcbuf,line); // save for minute by minute logging
  }
  telnet.loop(); // process any telnet traffic
    
  //----------------------------
  // tasks executed once per second
  //----------------------------
  if (secondDetector())
  {
    wifiService(); // service the wifi connection controller
    
    if (wifiIsConnected() && !ntpStarted() && (ntpAttempts == 0)) ntpStart(); // first NTP request
    ntpService();
    if (!ntpDone && ntpComplete())
    {
      logMessage("NTP (bootup) completed");
      ntpDone = true;
    }

    if (wifiIsConnected() && !setupTelnetDone)
    {
      setupTelnetDone = true;
      setupTelnet();
    }

    char* sioinputline = sioService();
    if (sioinputline != NULL) handleShellCommand(String(sioinputline));
  }

  //----------------------------
  // tasks executed once per minute
  //----------------------------
  if (minuteDetector())
  {
    gpsLogLine(rmcbuf); // log position if available once per minute
  }

  //----------------------------
  // tasks executed once per hour
  //----------------------------
  if (hourDetector())
  {
    gpsLogFlush(); // flush log hourly
  }

  //----------------------------
  // tasks executed once per day
  //----------------------------
  if (dayDetector())
  {
    ntpStart(); // dayly, get an NTP update
  }
}
