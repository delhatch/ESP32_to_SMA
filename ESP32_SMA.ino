/*
 Talks to SMA Sunny Boy, Model SB 8000US-12
    - Gets AC power (only)
    - Sends the result to a custom LED bargraph

  Based on code found at https://github.com/stuartpittaway/nanodesmapvmonitor
 */

//Need to change SoftwareSerial/NewSoftSerial.h file to set buffer to 128 bytes or you will get buffer overruns!
//Find the line below and change
//#define _NewSS_MAX_RX_BUFF 128 // RX buffer size

#include "Arduino.h"
//#include <avr/pgmspace.h>
#include <ESP32Time.h>
#include "time.h"
//#include <NanodeMAC.h>
#include "bluetooth.h"
#include "SMANetArduino.h"

ESP32Time ESP32rtc;          // Time structure. Holds what time the ESP32 thinks it is.
ESP32Time nextMidnight;   // Create a time structure to hold the answer to "What time (in time_t seconds) is the upcoming midnight?"

//BST Start and end dates - this needs moving into some sort of PROGMEM array for the years or calculated based on the BST logic see 
//http://www.time.org.uk/bstgmtcodepage1.aspx
//static time_t SummerStart=1435888800;  //Sunday, 31 March 02:00:00 GMT
//static time_t SummerEnd=1425952800;  //Sunday, 27 October 02:00:00 GMT

//SMA inverter timezone (note inverter appears ignores summer time saving internally)
//Need to determine what happens when its a NEGATIVE time zone !
//Number of seconds for timezone 
//    0=UTC (London)
//19800=5.5hours Chennai, Kolkata
//36000=Brisbane (UTC+10hrs)
#define timeZoneOffset 60*60*0

#undef debugMsgln 
//#define debugMsgln(s) (__extension__(  {Serial.println(F(s));}  ))
#define debugMsgln(s) (__extension__(  {__asm__("nop\n\t"); }  ))

#undef debugMsg
//#define debugMsg(s) (__extension__(  {Serial.print(F(s));}  ))
#define debugMsg(s) (__extension__(  { __asm__("nop\n\t");  }  ))

//Do we switch off upload to sites when its dark?
#undef allowsleep

static unsigned long currentvalue=0;
static unsigned int valuetype=0;
static unsigned long value = 0;
static unsigned long oldvalue = 0;
static long lastRanTime = 0;
static long nowTime = 0;
static unsigned long spotpowerac=0;
static unsigned long spotpowerdc=0;
// "datetime" stores the number of seconds since the epoch (normally 01/01/1970), AS RETRIEVED
//     from the SMA. The value is updated when data is read from the SMA, like when
//     getInstantACPower() is called.
//static unsigned long datetime=0;   // stores the number of seconds since the epoch (normally 01/01/1970)
time_t datetime = 0;

const unsigned long seventy_years = 2208988800UL;

prog_uchar PROGMEM smanet2packetx80x00x02x00[] ={ 0x80, 0x00, 0x02, 0x00};
prog_uchar PROGMEM smanet2packet2[]  ={ 0x80, 0x0E, 0x01, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
prog_uchar PROGMEM SMANET2header[] = { 0xFF,0x03,0x60,0x65  };
prog_uchar PROGMEM  InverterCodeArray[] = { 0x5c, 0xaf, 0xf0, 0x1d, 0x50, 0x00 };  // Fake address on the SMA NETWORK
prog_uchar PROGMEM  fourzeros[]= { 0,0,0,0};
prog_uchar PROGMEM  smanet2packet6[]={ 0x54, 0x00, 0x22, 0x26, 0x00, 0xFF, 0x22, 0x26, 0x00};
prog_uchar PROGMEM  smanet2packet99[]= { 0x00,0x04,0x70,0x00};
prog_uchar PROGMEM  smanet2packet0x01000000[]=  { 0x01,0x00,0x00,0x00};

//Password needs to be 12 bytes long, with zeros as trailing bytes (Assume SMA INVERTER PIN code is 0000)
const unsigned char  SMAInverterPasscode[]={'0','0','0','0',0,0,0,0,0,0,0,0};

// Function Prototypes
void initialiseSMAConnection();
void logonSMAInverter();
void checkIfNeedToSetInverterTime();
void getInstantACPower();
void getTotalPowerGeneration();

void setup() 
{ 
  Serial.begin(115200);          //Serial port for debugging output
  ESP32rtc.setTime(30, 24, 15, 17, 1, 2021);  // 17th Jan 2021 15:24:30  // Need this to be accurate. Since connecting to the internet anyway, use NTP.
} 


void loop() 
{ 
  /* Removed by DRH
  setSyncInterval(3600*2);  //2 hours
  setSyncProvider(setTimePeriodically);  //This also fires off a refresh of the time immediately
  */

  BTStart();

  initialiseSMAConnection();

  //Dont really need this...
  //InquireBlueToothSignalStrength();

  logonSMAInverter();

  checkIfNeedToSetInverterTime();

  //getInverterName();
  //HistoricData();
  lastRanTime = millis() - 4000;

  while (1) {
    //debugMsgln("Main loop");

    //HowMuchMemory();

    //Populate datetime and spotpowerac variables with latest values
    //getInstantDCPower();
    nowTime = millis();
    if( nowTime > (lastRanTime + 4000) ) {
      lastRanTime = nowTime;
      getInstantACPower();
    }

    //digitalClockDisplay(now());
    //debugMsgln("");

    //The inverter always runs in UTC time (and so does this program!), and ignores summer time, so fix that here...
    //add 1 hour to readings if its summer
    // DRH Temp removal of: if ((datetime>=SummerStart) && (datetime<=SummerEnd)) datetime+=60*60;

#ifdef allowsleep
    if ( (ESP32rtc.getEpoch() > (datetime+3600)) && (spotpowerac==0)) {
      //Inverter output hasnt changed for an hour, so put Nanode to sleep till the morning
      //debugMsgln("Bed time");

      //sleeping=true;

      //Get midnight on the day of last solar generation
      // First, create a time structure to hold the answer to "At what time (in time_t seconds) is the upcoming midnight?"
      tmElements_t tm;     
      tm.Year = year(datetime)-1970;
      tm.Month = month(datetime);
      tm.Day = day(datetime);
      tm.Hour = 23;
      tm.Hour = hour(datetime);
      tm.Minute = 59;
      tm.Second = 59;
      time_t midnight=makeTime(tm);


      //Move to midnight
      //debugMsg("Midnight ");digitalClockDisplay( midnight );debugMsgln("");

      if (ESP32rtc.getEpoch() < midnight) {
        //Time to calculate SLEEP time, we only do this if its BEFORE midnight
        //on the day of solar generation, otherwise we might wake up and go back to sleep immediately!

        //Workout what time sunrise is and sleep till then...
        //longitude, latitude (london uk=-0.126236,51.500152)
        unsigned int minutespastmidnight=ComputeSun(mylongitude,mylatitude,datetime,true);

        //We want to wake up at least 15 minutes before sunrise, just in case...
        checktime=midnight + minutespastmidnight - 15*60;
      }
    }
#endif      

    //debugMsg("Wait for ");
    //digitalClockDisplay( checktime );
    //debugMsgln("");

    //Delay for approx. 4 seconds between instant AC power readings

  }  // end of while(1)
}   // end of loop()

//-------------------------------------------------------------------------------------------
void checkIfNeedToSetInverterTime() {
  //We dont actually use the value this returns, but "datetime" is set from its reply
  getDailyYield();

  //digitalClockDisplay(now());Serial.println("");
  //digitalClockDisplay(datetime);Serial.println("");

  unsigned long timediff;

  if (datetime > ESP32rtc.getEpoch()) timediff=datetime - ESP32rtc.getEpoch();  // DRH was now()
  else timediff = ESP32rtc.getEpoch() - datetime;                // DRH was now()

  if (timediff > 60) {
    //If inverter clock is out by more than 1 minute, set it to the time from NTP, saves filling up the 
    //inverters event log with hundred of "change time" lines...
    setInverterTime();  //Set inverter time to now()
  }
}


prog_uchar PROGMEM smanet2settime[]=  {  
  0x8c ,0x0a ,0x02 ,0x00 ,0xf0 ,0x00 ,0x6d ,0x23 ,0x00 ,0x00 ,0x6d ,0x23 ,0x00 ,0x00 ,0x6d ,0x23 ,0x00
};

void setInverterTime() {
  //Sets inverter time for those SMA inverters which don't have a realtime clock (Tripower 8000 models for instance)

  //Payload...

  //** 8C 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00 
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now)
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now) 
  //   9F AE 99 4F   ==Thu, 26 Apr 2012 20:22:55 GMT  (now)
  //   01 00         ==Timezone +1 hour for BST ?
  //   00 00 
  //   A1 A5 99 4F   ==Thu, 26 Apr 2012 19:44:33 GMT  (strange date!)
  //   01 00 00 00 
  //   F3 D9         ==Checksum
  //   7E            ==Trailer

  //Set time to Feb

  //2A 20 63 00 5F 00 B1 00 0B FF B5 01 
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00 
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 01 
  //** 8D 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00 
  //14 02 2B 4F ==Thu, 02 Feb 2012 21:37:24 GMT
  //14 02 2B 4F ==Thu, 02 Feb 2012 21:37:24 GMT 
  //14 02 2B 4F  ==Thu, 02 Feb 2012 21:37:24 GMT
  //00 00        ==No time zone/BST not applicable for Feb..
  //00 00 
  //AD B1 99 4F  ==Thu, 26 Apr 2012 20:35:57 GMT 
  //01 00 00 00 
  //F6 87        ==Checksum
  //7E

  //2A 20 63 00 5F 00 B1 00 0B FF B5 01 
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00 
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 1C 
  //** 8D 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00 
  //F5 B3 99 4F 
  //F5 B3 99 4F 
  //F5 B3 99 4F 01 00 00 00 28 B3 99 4F 01 00 00 00 
  //F3 C7 7E

  //2B 20 63 00 5F 00 DD 00 0B FF B5 01 
  //7E 5A 00 24 A3 0B 50 DD 09 00 FF FF FF FF FF FF 01 00 
  //7E FF 03 60 65 10 A0 FF FF FF FF FF FF 00 00 78 00 6E 21 96 37 00 00 00 00 00 00 08 
  //** 80 0A 02 00 F0 00 6D 23 00 00 6D 23 00 00 6D 23 00 
  //64 76 99 4F ==Thu, 26 Apr 2012 16:23:00 GMT 
  //64 76 99 4F ==Thu, 26 Apr 2012 16:23:00 GMT 
  //64 76 99 4F  ==Thu, 26 Apr 2012 16:23:00 GMT
  //58 4D   ==19800 seconds = 5.5 hours
  //00 00 
  //62 B5 99 4F 
  //01 00 00 00 
  //C3 27 7E 

  debugMsgln("setInvTime ");
  time_t currenttime = ESP32rtc.getEpoch();  // Returns the ESP32 RTC in number of seconds since the epoch (normally 01/01/1970)
  //digitalClockDisplay(currenttime);
  writePacketHeader(level1packet);
  writeSMANET2PlusPacket(level1packet,0x09, 0x00, packet_send_counter, 0, 0, 0);
  writeSMANET2ArrayFromProgmem(level1packet,smanet2settime,sizeof(smanet2settime));
  writeSMANET2Long(level1packet,currenttime);
  writeSMANET2Long(level1packet,currenttime);
  writeSMANET2Long(level1packet,currenttime); 
  writeSMANET2uint(level1packet,timeZoneOffset);  
  writeSMANET2uint(level1packet,0);
  writeSMANET2Long(level1packet,currenttime);  //No idea what this is for...
  writeSMANET2ArrayFromProgmem(level1packet,smanet2packet0x01000000,sizeof(smanet2packet0x01000000));
  writeSMANET2PlusPacketTrailer(level1packet);
  writePacketLength(level1packet);
  sendPacket(level1packet);
  packet_send_counter++;
  //debugMsgln(" done");
}


prog_uchar PROGMEM smanet2totalyieldWh[]=  {  
  0x54, 0x00, 0x01, 0x26, 0x00, 0xFF, 0x01, 0x26, 0x00};

void getTotalPowerGeneration() {
  //Gets the total kWh the SMA inverter has generated in its lifetime...
  do {
    writePacketHeader(level1packet);
    writeSMANET2PlusPacket(level1packet,0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packetx80x00x02x00,sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet,smanet2totalyieldWh,sizeof(smanet2totalyieldWh));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);

    waitForMultiPacket(0x0001);
  }
  while (!validateChecksum());
  packet_send_counter++;

  //displaySpotValues(16);
  memcpy(&datetime,&level1packet[40+1+4],4);
  memcpy(&value,&level1packet[40+1+8],3);
  //digitalClockDisplay(datetime);
  //debugMsg('=');Serial.println(value);
  currentvalue=value;
}

void initialiseSMAConnection() {

  //Wait for announcement/broadcast message from PV inverter
  waitForPacket(0x0002);

  //Extract data from the 0002 packet
  unsigned char netid=level1packet[4];
  
  // Now create a response and send it.
  writePacketHeader(level1packet,0x02,0x00,smaBTInverterAddressArray);
  writeSMANET2ArrayFromProgmem(level1packet,smanet2packet99,sizeof(smanet2packet99));
  writeSMANET2SingleByte(level1packet,netid);
  writeSMANET2ArrayFromProgmem(level1packet,fourzeros,sizeof(fourzeros));
  writeSMANET2ArrayFromProgmem(level1packet,smanet2packet0x01000000,sizeof(smanet2packet0x01000000));
  writePacketLength(level1packet);
  sendPacket(level1packet);

  // The SMA inverter will respond with a packet carrying the command '0x000A'.
  // It will rturn with cmdcode set to 0x000A.
  waitForPacket(0x000a);
  
  // The SMA inverter will now send two packets, one carrying the '0x000C' command, then the '0x0005' command.
  // Sit in the following loop until you get one of these two packets.
  while ( (cmdcode != 0x000c) && (cmdcode != 0x0005) ) {
    cmdcode = readLevel1PacketFromBluetoothStream(0);
  }

  // If the most recent packet was command code = 0x0005 skip this next line, otherwise, wait for 0x0005 packet.
  // Since the first SMA packet after a 0x000A packet will be a 0x000C packet, you'll probably sit here waiting at least once.
  if (cmdcode!=0x0005) {
    waitForPacket(0x0005);
  }

  do {
    //First SMANET2 packet
    writePacketHeader(level1packet,sixff);
    writeSMANET2PlusPacket(level1packet,0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packetx80x00x02x00,sizeof(smanet2packetx80x00x02x00));
    writeSMANET2SingleByte(level1packet,0x00);
    writeSMANET2ArrayFromProgmem(level1packet,fourzeros,sizeof(fourzeros));
    writeSMANET2ArrayFromProgmem(level1packet,fourzeros,sizeof(fourzeros));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);
    sendPacket(level1packet);

    waitForPacket(0x0001);
  } 
  while (!validateChecksum());
  packet_send_counter++;

  //Second SMANET2 packet
  writePacketHeader(level1packet,sixff);
  writeSMANET2PlusPacket(level1packet,0x08, 0xa0, packet_send_counter, 0x00, 0x03, 0x03);
  writeSMANET2ArrayFromProgmem(level1packet,smanet2packet2,sizeof(smanet2packet2));

  writeSMANET2PlusPacketTrailer(level1packet);
  writePacketLength(level1packet);
  sendPacket(level1packet);
  packet_send_counter++;
  //No reply for this message...
}


prog_uchar PROGMEM smanet2packet_logon[]={ 
  0x80, 0x0C, 0x04, 0xFD, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00,0xaa,0xaa,0xbb,0xbb};

void logonSMAInverter() {
  //Third SMANET2 packet
  debugMsg("*Logon ");
  do {
    writePacketHeader(level1packet,sixff);
    writeSMANET2PlusPacket(level1packet,0x0e, 0xa0, packet_send_counter, 0x00, 0x01, 0x01);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packet_logon,sizeof(smanet2packet_logon));
    writeSMANET2ArrayFromProgmem(level1packet,fourzeros,sizeof(fourzeros));

    //INVERTER PASSWORD
    for(int passcodeloop=0;passcodeloop<sizeof(SMAInverterPasscode);passcodeloop++) {
      unsigned char v=pgm_read_byte(SMAInverterPasscode+passcodeloop);
      writeSMANET2SingleByte(level1packet,(v + 0x88) % 0xff);
    }

    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);
    sendPacket(level1packet);

    waitForPacket(0x0001);
  } 
  while (!validateChecksum());
  packet_send_counter++;
  debugMsgln("Done");
}

void getDailyYield() {
  //Yield
  //We expect a multi packet reply to this question...
  //We ask the inverter for its DAILY yield (generation)
  //once this is returned we can extract the current date/time from the inverter and set our internal clock
  do {
    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet,0x09, 0xa0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packetx80x00x02x00,sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packet6,sizeof(smanet2packet6));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);

    waitForPacket(0x0001);
  } 
  while (!validateChecksum());
  packet_send_counter++;

  //Returns packet looking like this...
  //    7E FF 03 60 65 0D 90 5C AF F0 1D 50 00 00 A0 83 
  //    00 1E 6C 5D 7E 00 00 00 00 00 00 03 
  //    80 01 02 00 
  //    54 01 00 00 00 01 00 00 00 01 
  //    22 26  //command code 0x2622 daily yield
  //    00     //unknown
  //    D6 A6 99 4F  //Unix time stamp (backwards!) = 1335469782 = Thu, 26 Apr 2012 19:49:42 GMT
  //    D9 26 00     //Daily generation 9.945 kwh
  //    00 
  //    00 00 00 00 
  //    18 61    //checksum
  //    7E       //packet trailer

  // Does this packet contain the British Summer time flag?
  //dumpPacket('Y');

  valuetype = level1packet[40+1+1]+level1packet[40+2+1]*256;

  //Serial.println(valuetype,HEX);
  //Make sure this is the right message type
  if (valuetype==0x2622) {  
    memcpy(&value,&level1packet[40+8+1],3);
    //0x2622=Day Yield Wh
    memcpy(&datetime,&level1packet[40+4+1],4);  
    //setTime(datetime);  
  }
}

prog_uchar PROGMEM smanet2acspotvalues[]=  {  
  0x51, 0x00, 0x3f, 0x26, 0x00, 0xFF, 0x3f, 0x26, 0x00, 0x0e};

void getInstantACPower() 
{
  //Get spot value for instant AC wattage
  do {
    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet,0x09, 0xA1, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packetx80x00x02x00,sizeof(smanet2packetx80x00x02x00));
    writeSMANET2ArrayFromProgmem(level1packet,smanet2acspotvalues,sizeof(smanet2acspotvalues));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);
    waitForMultiPacket(0x0001);
  }
  while (!validateChecksum());
  packet_send_counter++;

  //value will contain instant/spot AC power generation along with date/time of reading...
  memcpy(&datetime,&level1packet[40+1+4],4);
  memcpy(&value,&level1packet[40+1+8],3);
  debugMsg("AC ");
  //Serial.println(" ");
  //Serial.println("Got AC power level. ");
  //digitalClockDisplay(datetime);
  debugMsg(" Pwr=");
  //if( value != oldvalue ) {
    //Serial.println(" ");
    //Serial.print("*** Power Level = ");
    Serial.print(value);
    //Serial.println(" Watts RMS ***");
    //Serial.print(" ");
    Serial.println("");
    oldvalue = value;
  //}
  spotpowerac=value;

  //displaySpotValues(28);
}

//Returns volts + amps
//prog_uchar PROGMEM smanet2packetdcpower[]={  0x83, 0x00, 0x02, 0x80, 0x53, 0x00, 0x00, 0x45, 0x00, 0xFF, 0xFF, 0x45, 0x00 };
// Just DC Power (watts)
prog_uchar PROGMEM smanet2packetdcpower[]={  
  0x83, 0x00, 0x02, 0x80, 0x53, 0x00, 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x25, 0x00 };
void getInstantDCPower() {

  //DC
  //We expect a multi packet reply to this question...
  do {
    writePacketHeader(level1packet);
    //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
    writeSMANET2PlusPacket(level1packet,0x09, 0xE0, packet_send_counter, 0, 0, 0);
    writeSMANET2ArrayFromProgmem(level1packet,smanet2packetdcpower,sizeof(smanet2packetdcpower));
    writeSMANET2PlusPacketTrailer(level1packet);
    writePacketLength(level1packet);

    sendPacket(level1packet);
    waitForMultiPacket(0x0001);
  }
  while (!validateChecksum());
  packet_send_counter++;

  //displaySpotValues(28);

  //float volts=0;
  //float amps=0;

  for(int i=40+1;i<packetposition-3;i+=28){
    valuetype = level1packet[i+1]+level1packet[i+2]*256;
    memcpy(&value,&level1packet[i+8],3);

    //valuetype 
    //0x451f=DC Voltage  /100
    //0x4521=DC Current  /1000
    //0x251e=DC Power /1
    //if (valuetype==0x451f) volts=(float)value/(float)100;
    //if (valuetype==0x4521) amps=(float)value/(float)1000;
    if (valuetype==0x251e) spotpowerdc=value;

    memcpy(&datetime,&level1packet[i+4],4);
  }

  //spotpowerdc=volts*amps;

  debugMsg("DC ");
  //digitalClockDisplay(datetime);
  //debugMsg(" V=");Serial.print(volts);debugMsg("  A=");Serial.print(amps);
  debugMsg(" Pwr=");
  Serial.println(spotpowerdc);  
}

/*
//Inverter name
 prog_uchar PROGMEM smanet2packetinvertername[]={   0x80, 0x00, 0x02, 0x00, 0x58, 0x00, 0x1e, 0x82, 0x00, 0xFF, 0x1e, 0x82, 0x00};  
 
 void getInverterName() {
 
 do {
 //INVERTERNAME
 debugMsgln("InvName"));
 writePacketHeader(level1packet,sixff);
 //writePacketHeader(level1packet,0x01,0x00,sixff);
 writeSMANET2PlusPacket(level1packet,0x09, 0xa0, packet_send_counter, 0, 0, 0);
 writeSMANET2ArrayFromProgmem(level1packet,smanet2packetinvertername);
 writeSMANET2PlusPacketTrailer(level1packet);
 writePacketLength(level1packet);
 sendPacket(level1packet);
 
 waitForMultiPacket(0x0001);
 } 
 while (!validateChecksum());
 packet_send_counter++;
 
 valuetype = level1packet[40+1+1]+level1packet[40+2+1]*256;
 
 if (valuetype==0x821e) {
 memcpy(invertername,&level1packet[48+1],14);
 Serial.println(invertername);
 memcpy(&datetime,&level1packet[40+4+1],4);  //Returns date/time unit switched PV off for today (or current time if its still on)
 }
 }
 
 void HistoricData() {
 
 time_t currenttime=now();
 digitalClockDisplay(currenttime);
 
 debugMsgln("Historic data...."));
 tmElements_t tm;
 if( year(currenttime) > 99)
 tm.Year = year(currenttime)- 1970;
 else
 tm.Year = year(currenttime)+ 30;  
 
 tm.Month = month(currenttime);
 tm.Day = day(currenttime);
 tm.Hour = 10;      //Start each day at 5am (might need to change this if you're lucky enough to live somewhere hot and sunny!!
 tm.Minute = 0;
 tm.Second = 0;
 time_t startTime=makeTime(tm);  //Midnight
 
 
 //Read historic data for today (SMA inverter saves 5 minute averaged data)
 //We read 30 minutes at a time to save RAM on Arduino
 
 //for (int hourloop=1;hourloop<24*2;hourloop++)
 while (startTime < now()) 
 {
 //HowMuchMemory();
 
 time_t endTime=startTime+(25*60);  //25 minutes on
 
 //digitalClockDisplay(startTime);
 //digitalClockDisplay(endTime);
 //debugMsgln(" ");
 
 do {
 writePacketHeader(level1packet);
 //writePacketHeader(level1packet,0x01,0x00,smaBTInverterAddressArray);
 writeSMANET2PlusPacket(level1packet,0x09, 0xE0, packet_send_counter, 0, 0, 0);
 
 writeSMANET2SingleByte(level1packet,0x80);
 writeSMANET2SingleByte(level1packet,0x00);
 writeSMANET2SingleByte(level1packet,0x02);
 writeSMANET2SingleByte(level1packet,0x00);
 writeSMANET2SingleByte(level1packet,0x70);
 // convert from an unsigned long int to a 4-byte array
 writeSMANET2Long(level1packet,startTime);
 writeSMANET2Long(level1packet,endTime);
 writeSMANET2PlusPacketTrailer(level1packet);
 writePacketLength(level1packet);
 sendPacket(level1packet);
 
 waitForMultiPacket(0x0001);
 }
 while (!validateChecksum());
 //debugMsg("packetlength=");    Serial.println(packetlength);
 
 packet_send_counter++;
 
 //Loop through values
 for(int x=40+1;x<(packetposition-3);x+=12){
 memcpy(&value,&level1packet[x+4],4);
 
 if (value > currentvalue) {
 memcpy(&datetime,&level1packet[x],4);
 digitalClockDisplay(datetime);
 debugMsg("=");
 Serial.println(value);
 currentvalue=value;         
 
 //uploadValueToSolarStats(currentvalue,datetime);          
 }
 }
 
 startTime=endTime+(5*60);
 delay(750);  //Slow down the requests to the SMA inverter
 }
 }
 */
