/*
 Talks to SMA Sunny Boy, Model SB 8000US-12
    - Gets AC power (only)
    - Sends the result to a custom LED bargraph

  Based on code found at https://github.com/stuartpittaway/nanodesmapvmonitor
 */

#include "Arduino.h"
#include "TimeLib.h"
#include "bluetooth.h"
#include "SMANetArduino.h"
#include "Preferences.h"

// Location = 47.4503, -122.3088  (Seattle airport)
// Local time = GMT-8 hours (Normal), GMT-7 hours (DST)

#undef debugMsgln 
//#define debugMsgln(s) (__extension__(  {Serial.println(F(s));}  ))
#define debugMsgln(s) (__extension__(  {__asm__("nop\n\t"); }  ))

#undef debugMsg
//#define debugMsg(s) (__extension__(  {Serial.print(F(s));}  ))
#define debugMsg(s) (__extension__(  { __asm__("nop\n\t");  }  ))

//Should I print the results of the time calculations?
#define ShowTimeCalculations

#define UPDATE_DELAY 4*SECS_PER_MIN*1000
#define GARBAGE_DELAY 4*1000

Preferences dailyData;

static unsigned long currentvalue = 0;   // 4 bytes
static unsigned int valuetype = 0;       // 4 bytes
static unsigned long value = 0;         // Holds the data from the SMA inverter -- the value that was requested.
static unsigned long spotpowerac = 0;     // result from getInstantACPower().
static unsigned long spotpowerdc = 0;     // result from getInstantDCPower().
static unsigned long dailyYield = 0;
static unsigned long dailyConsumed = 0;
static unsigned long yesterdayConsumed = 0;
static unsigned long topDailyYield = 0;
static unsigned long yesterdayTopDailyYield = 0;
static long int lastRanTime = 0; // The last time (in millis() ) that we checked the AC power being generated.
static long int lastGarbageTime = 0; // The last time (in millis() ) that we talked to the SMA, just to clear buffers.
static long int nowTime = 0;     // The current program time, in millis().
int bankUpdated = 0;    // if the bank value has not been updated, this value = 0;
static int bank = 0;     // Will read from EEPROM.

// "datetime" stores the number of seconds since the epoch (normally 01/01/1970), AS RETRIEVED
//     from the SMA. The value is updated when data is read from the SMA, like when
//     getInstantACPower() is called.
//static unsigned long datetime=0;   // stores the number of seconds since the epoch (normally 01/01/1970)
time_t datetime = 0;    // Time as read from the SMA inverter. In time_t seconds, UTC.   // sizeof(time_t) = 4 bytes. signed.
time_t timeNow = 0;     // Time as read from the ESP32's RTC.
time_t wakeUpNow = 0;   // Time to wake up and try to connect to the SMA inverter.

bool calc_sunrise;      // Set to "true" to calculate the sunrise time. Set to "false" to calculate the sunset time.
time_t nextSunrise;
time_t nextSunset;

const unsigned long seventy_years = 2208988800UL;
int BTNowConnected;
int trycount;
bool asleep = 0;     // If true, do not try to talk to the SMA inverter.

prog_uchar PROGMEM smanet2packetx80x00x02x00[] = { 0x80, 0x00, 0x02, 0x00};
prog_uchar PROGMEM smanet2packet2[] = { 0x80, 0x0E, 0x01, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
prog_uchar PROGMEM SMANET2header[] = { 0xFF,0x03,0x60,0x65  };
prog_uchar PROGMEM  InverterCodeArray[] = { 0x5c, 0xaf, 0xf0, 0x1d, 0x50, 0x00 };  // Fake address on the SMA NETWORK
prog_uchar PROGMEM  fourzeros[] = { 0,0,0,0};
prog_uchar PROGMEM  smanet2packet6[] = { 0x54, 0x00, 0x22, 0x26, 0x00, 0xFF, 0x22, 0x26, 0x00};
prog_uchar PROGMEM  smanet2packet99[] = { 0x00,0x04,0x70,0x00};
prog_uchar PROGMEM  smanet2packet0x01000000[] = { 0x01,0x00,0x00,0x00};
prog_uchar PROGMEM smanet2acspotvalues[] = { 0x51, 0x00, 0x3f, 0x26, 0x00, 0xFF, 0x3f, 0x26, 0x00, 0x0e };
prog_uchar PROGMEM smanet2packetdcpower[] = { 0x83, 0x00, 0x02, 0x80, 0x53, 0x00, 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x25, 0x00 };
prog_uchar PROGMEM smanet2packet_logon[] = { 0x80, 0x0C, 0x04, 0xFD, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x84, 0x03, 0x00, 0x00,0xaa,0xaa,0xbb,0xbb};
prog_uchar PROGMEM smanet2totalyieldWh[] = { 0x54, 0x00, 0x01, 0x26, 0x00, 0xFF, 0x01, 0x26, 0x00};
prog_uchar PROGMEM smanet2settime[] = { 0x8c ,0x0a ,0x02 ,0x00 ,0xf0 ,0x00 ,0x6d ,0x23 ,0x00 ,0x00 ,0x6d ,0x23 ,0x00 ,0x00 ,0x6d ,0x23 ,0x00};

//Password array needs to 12 bytes long, with zeros as trailing bytes (unused characters).
const unsigned char  SMAInverterPasscode[]={'A','s','d','f','6','g','?','?',0,0,0,0};

// Function Prototypes
void initialiseSMAConnection();
void logonSMAInverter();
void checkIfNeedToSetInverterTime();
void getInstantACPower();
void getTotalPowerGeneration();

void setup() 
{ 
  // Note: This project can only be powered-up during the daytime, when the SMA is running. This is because
  //          during the day the RTC on the SMA has been set by the SunnyBeam. So the ESP32 can set it's
  //          clock from that.
  Serial.begin(115200);          //Serial port for debugging output
  // Open Preferences with my-app namespace.
  Serial.println("\r\n-----------------------------------");
  Serial.println("In setup() code -- opening EEPROM");
  dailyData.begin("ESP32_SMA", false);
  yesterdayTopDailyYield = dailyData.getULong("topDailyYield",0);
  Serial.print("yesterdayTopDailyYield from EEPROM = ");
  Serial.println( yesterdayTopDailyYield );
  // Read the bank value from EEPROM.
  bank = dailyData.getInt("bank",1000);
  Serial.print("bank from EEPROM = ");
  Serial.println( bank );
  // Read "did I adjust the bank already" bit from EEPROM.
  bankUpdated = dailyData.getInt("bankUpdated",0);
  Serial.print("bankUpdated from EEPROM = ");
  Serial.println( bankUpdated );
  dailyData.end();    // Close the EEPROM.
        
  pinMode(output23, OUTPUT);  // Yellow LED
  pinMode(output22, OUTPUT);  // Green LED
  // Set LEDs
  digitalWrite(output23, HIGH);  // Yellow on
  digitalWrite(output22, LOW);   // Green off

  topDailyYield = 0;   // Reset the daily yield. Main loop will set updated value.
  asleep = true;   // Assume that it is nightime, until proven otherwise;
} 

void loop() 
{ 
  // The starting section will try to connect to the SMA, and will not move on until it is possible.
  while (1) {
    trycount = 0;
    do {
      trycount++;
      Serial.print("\r\nTrying to connect, attempt number: ");
      Serial.println( trycount );
      BTStart();
    } while ( (trycount <= 3) && (BTIsConnected() == false) );
    if( BTIsConnected() ) break;    // If connected, go ahead and resume running the loop() code.
    Serial.println("Failed after trying to connect. Will now wait for 15 minutes, and try again.");
    delay( 15 * 3600 * 1000 );    // Wait 15 minutes before trying again.
  };

  asleep = false;    // We know we are connected to the SMA inverter.
  
  // So make a data connection to the SMA inverter.
  initialiseSMAConnection();
  logonSMAInverter();
  // This next part is about setting the ESP32 RTC.
  getInstantACPower();
  setTime( datetime );  // Set the ESP32 RTC clock from the SMA. "datetime" is the SMA inverter time, in time_t units. This is in UTC time.
  calcSunriseSunset();

  lastRanTime = millis() - UPDATE_DELAY;
  lastGarbageTime = millis() - GARBAGE_DELAY;

  // TODO: If bankUpdated == 0 here, update the bank, and set bankUpdated to 1, and write bankUpdated to EEPROM.

  while (1) {
    // This next section is the main, important, chunk of code that runs throughout the day.
    if( asleep == false ) {
      nowTime = millis();
      if( (nowTime > (lastRanTime + UPDATE_DELAY)) && BTIsConnected()==true ) {
        lastRanTime = nowTime;
        lastGarbageTime = nowTime;
        getInstantACPower();
        //Serial.println("");
        Serial.print(datetime);
        Serial.print("  ");
        Serial.print(value);
        Serial.println(" Watts RMS ***");
        getDailyYield();
        Serial.print("Daily yield = ");
        Serial.println( dailyYield );
        if( dailyYield > topDailyYield ) {
          topDailyYield = dailyYield;
        }
        // TODO: Now display the instantaneous power on the LED bargraph.
        // TODO: Talk to the Arduio. Get the instantaneous power being consumed right now. Display it.
        // TODO: From the Arduino, get the "total power consumed today"
        // TODO: Subtract the total power consumed from the total power generated. Display it.
      }
      else
      {
        // Need to read the ESP32 BT data buffer semi-frequently to keep it (and level1packet[] buffer) from overflowing.
        if( (nowTime > (lastGarbageTime + GARBAGE_DELAY)) && BTIsConnected()==true ) {
          //Serial.println("*** Clearing buffers.");
          getInstantACPower();
          lastGarbageTime = millis();
        }
      }
    }

    // See if it is within 90 minutes of sunset AND the power level is less than 80 watts. If this is true,
    //    go ahead and shut it down for the night.
    if( (asleep == false) && ( (nextSunset - now()) < (90*SECS_PER_MIN)) && (spotpowerac < 100) ) {
      Serial.println("Almost sunset. Shutting down now.");
      BTEnd();     // Break the connection with the SMA inverter.
      
      // Store data to EEPROM
      Serial.println("In \"going to sleep\" code -- opening EEPROM");
      dailyData.begin("ESP32_SMA", false);    // Open the EEPROM
      dailyData.putULong("dailyTotal", topDailyYield);
      bankUpdated = 0;        // Indicate that when we wake up tomorrow, need to update the bank.
      dailyData.putInt("bankUpdated", bankUpdated);
      if( bank < 0 ) bank = 0;        // The bank is never negative.
      dailyData.putInt("bank", bank);
      dailyData.end();                      // Close the EEPROM.
      // Wait for EEPROM write to finish. Give it plenty of time.
      delay(10000);
      
      asleep = true;
      wakeUpNow = nextSunrise;
    }

    // See if it is time to wake up.
    if( (asleep == true) && (now() > wakeUpNow) ) {
      Serial.println("Time to wake up.");
      trycount = 0;
      do {
        trycount++;
        Serial.print("\r\nTrying to connect, attempt number: ");
        Serial.println( trycount );
        BTStart();
      } while ( (trycount <= 3) && (BTIsConnected() == false) );
      if( BTIsConnected() ) {
        // Once newly connected, login, and calculate the upcoming sunrise and sunset times.
        asleep = false;
        initialiseSMAConnection();
        logonSMAInverter();
        // This next part is about synch'ing the ESP32 RTC with the SMA inverter prior to sunrise calc.'s
        getInstantACPower();
        calcSunriseSunset();

        Serial.println("In \"just waking up now\" code -- opening EEPROM");
        dailyData.begin("ESP32_SMA", false);    // Open the EEPROM
        
        // Read yesterday's "total power consumed" from the Arduino.
        
        // Read yesterday's "total power generated" from EEPROM.
        topDailyYield = dailyData.getULong("topDailyYield",0);
        Serial.print("topDailyYield from EEPROM = ");
        Serial.println( topDailyYield );
        // Read the bank value from EEPROM.
        bank = dailyData.getInt("bank",0);
        Serial.print("bank from EEPROM = ");
        Serial.println( bank );
        // Read "did I adjust the bank already" bit from EEPROM. If not:
        bankUpdated = dailyData.getInt("bankUpdated",0);
        Serial.print("bankUpdated from EEPROM = ");
        Serial.println( bankUpdated );
        
        //    TODO:  Subtract "power consumed" from "power generated"
        //    TODO:  Calculate new bank value, and store it to EEPROM.
        //    TODO:  Set the "I already did the calculations" bit in EEPROM, so I don't repeat this.
        bankUpdated = 1;        // Indicate that the bank calculations have been done today.
        dailyData.putInt("bankUpdated", bankUpdated);
        
        dailyData.end();                      // Close the EEPROM.
        // Wait for EEPROM write to finish. Give it plenty of time.
        delay(10000);
      }
      else {
        Serial.println("Failed after trying to connect. Will now wait for 5 minutes, and try again.");
        delay( 5 * SECS_PER_MIN * 1000 );    // Wait 5 minutes before trying again.
      }
    }     // End of "waking up" section

    // If we have unsucessfully tried to connect for 4 hours after the sun rose, restart the hardware.
    if( (asleep == true) && (now() < (wakeUpNow+(4*SECS_PER_HOUR))) && (now() > wakeUpNow) ) {
      error();    // Reboot the ESP32.
    }

  }  // end of while(1)
}   // end of loop()

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
  return;
}

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
  return;
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
    dailyYield = value;
  }
  return;
}

//----------------------------------------------------------------------------------------------
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
  //Serial.print("Just read time from SMS. datetime = ");
  //Serial.println(datetime);
  memcpy(&value,&level1packet[40+1+8],3);
  debugMsg("AC ");
  //Serial.println(" ");
  //Serial.println("Got AC power level. ");
  //digitalClockDisplay(datetime);
  debugMsg(" Pwr=");
  //Serial.println(" ");
  //Serial.print("*** Power Level = ");
  //Serial.print(value);
  //Serial.println(" Watts RMS ***");
  //Serial.print(" ");
  //Serial.print(datetime);
  //Serial.print("  ");
  spotpowerac=value;

  //displaySpotValues(28);
  return;
}
//-------------------------------------------------------------------------------------------------------

//Returns volts + amps. Just DC Power (watts)

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
  return;
}

static unsigned int  ComputeSun(float longitude, float latitude, time_t when, bool rs) {
//Borrowed from TimeLord library http://swfltek.com/arduino/timelord.html
//rs=true for sunrise, false=sunset

  uint8_t a;

  float lon = -longitude/57.295779513082322;  // Convert lat/long location to radians
  float lat = latitude/57.295779513082322;

  //approximate hour;
  a=6;  //6am
  if(rs) a=18;  //6pm

  // approximate day of year
  float y= (month(when)-1) * 30.4375 + (day(when)-1)  + a/24.0; // 0... 365

  // compute fractional year
  y *= 1.718771839885e-02; // 0... 1

  // compute equation of time... .43068174
  float eqt=229.18 * (0.000075+0.001868*cos(y)  -0.032077*sin(y) -0.014615*cos(y*2) -0.040849*sin(y* 2) );

  // compute solar declination... -0.398272
  float decl=0.006918-0.399912*cos(y)+0.070257*sin(y)-0.006758*cos(y*2)+0.000907*sin(y*2)-0.002697*cos(y*3)+0.00148*sin(y*3);

  //compute hour angle
  float ha=(  cos(1.585340737228125) / (cos(lat)*cos(decl)) -tan(lat) * tan(decl)  );

  if(fabs(ha)>1.0){// we're in the (ant)arctic and there is no rise(or set) today!
    return when; 
  }

  ha=acos(ha); 
  if(rs==false) ha=-ha;

  // compute minutes from midnight
  return 60*(720+4*(lon-ha)*57.295779513082322-eqt);
}

// This routine calculates the LOCAL time of the next sunrise and sunset. Results are in UTC time.
// NOTE: If the time (UTC) has passed UTC midnight (at 5 pm DST Seattle time) then the results need to have
//         24 hours subtracted from the answer.
void calcSunriseSunset( void ) {
  tmElements_t tm;        // Temporary structure used to calculate "at what UTC time_t is the upcoming midnight?"
  time_t timeNow2;
  time_t midnight_2_local_sunrise;
  time_t midnight_2_local_sunset;
  time_t midnight;

  timeNow2 = now();   // Read the time from the ESP32 RTC. (result is in time_t seconds)
  Serial.print("Now (UTC time_t) = ");
  Serial.println( timeNow2 );
  // This next section computes the time of the upcoming midnight, at the Prime Meridian. (Midnight in/at UTC, in time_t seconds.)
  tm.Year = year( timeNow2 )-1970;
  tm.Month = month( timeNow2 );
  tm.Day = day( timeNow2 );
  tm.Hour = 23;
  tm.Minute = 59;
  tm.Second = 59;
  midnight=makeTime(tm);
  //Serial.print("Midnight UTC @ time_t = ");
  //Serial.println(midnight);

  // Now compute how many seconds past midnight (the future, upcoming UTC midnight) is the SUNRISE event *at our location*.
  calc_sunrise = true;
  midnight_2_local_sunrise = ComputeSun( -122.3088, 47.4503, timeNow2, calc_sunrise);
  nextSunrise = midnight + midnight_2_local_sunrise;
  
  // Now compute how many seconds, from now, until the SUNSET *at our location*.
  calc_sunrise = false;    // Calc sunset.
  midnight_2_local_sunset = ComputeSun( -122.3088, 47.4503, timeNow2, calc_sunrise);
  nextSunset = (midnight - SECS_PER_DAY) + midnight_2_local_sunset;
  
  // If it says the next sunrise is more than 24 hours from now, subtract 24 hours from both results.
  if( (nextSunrise - now()) > (25*SECS_PER_HOUR) ) {
    nextSunrise -= 24 * SECS_PER_HOUR;
    nextSunset  -= 24 * SECS_PER_HOUR;
  }

  //Serial.print("Num. of seconds from midnight to sunrise = ");
  //Serial.println(midnight_2_local_sunrise);
  //Serial.print("Num. of seconds from midnight to sunset = ");
  //Serial.println(midnight_2_local_sunset);
  Serial.print("So the next sunrise at this location will occur at UTC time_t = ");
  Serial.println( nextSunrise );
  Serial.print("So the next sunset at this location will occur at UTC time_t = ");
  Serial.println( nextSunset );
  
  return;
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
