/*
 NANODE SMA PV MONITOR
 Bluetooth.ino 
 */

//#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Time.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>

//SoftwareSerial blueToothSerial(RxD,TxD);

BluetoothSerial SerialBT;
uint8_t address[6]  = {0x00, 0x80, 0x25, 0x27, 0xE3, 0xC5}; // Address of my SMA inverter.
const char pinbuf[] = {'0','0','0','0'}; // BT pin, not the inverter login password. Always 0000.
const char *pin = &pinbuf[0];
bool connected;
static long charTime = 0;

//---------------------------------------------------------
int BTStart() {
  int success = 0;
  
  SerialBT.begin("ESP32test", true);   // "true" creates this device as a BT Master.
  SerialBT.setPin(pin);   // pin as in "PIN" This is the BT connection pin, not login pin. ALWAYS 0000, unchangable.
  Serial.println("The SM32 started in master mode. Now trying to connect to SMA inverter.");
  connected = SerialBT.connect(address);

  if(connected) {
    Serial.println("Connected succesfully!");
    digitalWrite(output22, HIGH);  // Green on
    digitalWrite(output23, LOW);  // Yellow off
    success = 1;
  } else {
    //while(!SerialBT.connected(2000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
      digitalWrite(output23, LOW);  // Yellow off
      digitalWrite(output22, LOW);  // Green off
      success = 0;
    //}
  }
  return success;
}
//------------------------------------------------------
bool BTEnd() {
  digitalWrite(output23, LOW);  // Yellow off
  digitalWrite(output22, LOW);  // Green off
  return ( SerialBT.disconnect() );
}
//------------------------------------------------------
bool BTIsConnected() {
  return ( SerialBT.connected(2000) );
}
//------------------------------------------------------
void sendPacket( unsigned char *btbuffer ) {
  quickblink();
  //Serial.println();
  //Serial.println("Sending: ");
  for(int i=0;i<packetposition;i++) {
    SerialBT.write( *(btbuffer+i) );  // Send message char-by-char to SMA via ESP32 bluetooth
    //Serial.print( *(btbuffer+i), HEX ); // Print out what we are sending, in hex, for inspection.
    //Serial.print(' ');
  }
  //Serial.println();
}

void quickblink() {
  digitalWrite(output23, HIGH);  // Yellow on
  delay(20);
  digitalWrite(output23, LOW);  // Yellow off
}

void writeArrayIntoEEPROM(unsigned char readbuffer[],int length,int EEPROMoffset) {
  //Writes an array into EEPROM and calculates a simple XOR checksum
  byte checksum=0;
  for(int i=0;i<length; i++) {
    EEPROM.write(EEPROMoffset+i,readbuffer[i]);
    //Serial.print(EEPROMoffset+i); Serial.print("="); Serial.println(readbuffer[i],HEX);   
    checksum^= readbuffer[i];
  }

  //Serial.print(EEPROMoffset+length); Serial.print("="); Serial.println(checksum,HEX);
  EEPROM.write(EEPROMoffset+length,checksum);
}

bool readArrayFromEEPROM(unsigned char readbuffer[],int length,int EEPROMoffset) {
  //Writes an array into EEPROM and calculates a simple XOR checksum
  byte checksum=0;
  for(int i=0;i<length; i++) {
    readbuffer[i]=EEPROM.read(EEPROMoffset+i);
    //Serial.print(EEPROMoffset+i); Serial.print("="); Serial.println(readbuffer[i],HEX);   
    checksum^= readbuffer[i];
  }
  //Serial.print(EEPROMoffset+length); Serial.print("="); Serial.println(checksum,HEX);
  return (checksum==EEPROM.read(EEPROMoffset+length)); 
}

unsigned char getByte() {
  //Returns a single byte from the bluetooth stream (with error timeout/reset)
  unsigned long time;
  int inInt = 0;  // ESP32 SerialBT.read() returns an int, not a char.

  //Max wait 60 seconds, before throwing an fatal error
  time = 60000+millis(); 

  while ( !SerialBT.available() ) {
    delay(5);  //Wait for BT byte to arrive
    if (millis() > time) { 
      debugMsgln("Timeout");
      error();
    }
  }

  //if( (millis() - charTime) > 1 ) Serial.println(); // Breaks up SMA's messages into lines.
  inInt = SerialBT.read();
  charTime = millis();  // Used to detect a gap between messages (so can print \r\n).

  if( inInt == -1 ) {
    Serial.print("ERROR: Asked for a BT char when there was none to get.");
    return '!';
  }
  else {
    //Serial.print( (unsigned char)inInt, HEX );
    //Serial.print(' ');
    return (unsigned char)inInt;
  }
}

void convertBTADDRStringToArray(char *tempbuf,unsigned char *outarray, char match) {
  //Convert BT Address into a more useful byte array, the BT address is in a really awful format to parse!
  //Must be a better way of doing this function!

  //Unit test cases...
  //Test BT address strcpy(tempbuf,"1234:56:0\x0");
  //Test BT address strcpy(tempbuf,"1010:7:310068\x0");
  //Test BT address strcpy(tempbuf,"80:25:1dac53\x0");
  //Test BT address strcpy(tempbuf,"2:72:D2224\x0");
  //Test BT address strcpy(tempbuf,"1234:56:0\x0");
  //Test BT address strcpy(tempbuf,"1:72:D2224\x0");
  //Test BT address strcpy(tempbuf,"1234:56:0\x0");
  //Test BT address strcpy(tempbuf,"1234,56,abcdef\x0");
  //Test BT address strcpy(tempbuf,"0002,72,0d2224\x0");
  int l=strlen(tempbuf);
  char *firstcolon=strchr(tempbuf,match)+1;
  char *lastcolon=strrchr(tempbuf,match)+1;

  //Could use a shared buffer here to save RAM
  char output[13]={ '0','0','0','0','0','0','0','0','0','0','0','0',0 };

  //Deliberately avoided sprintf as it adds over 1600bytes to the program size at compile time.
  int partlen=(firstcolon-tempbuf)-1;
  strncpy(output+(4-partlen),tempbuf,partlen);

  partlen=(lastcolon-firstcolon)-1;
  strncpy(output+4+(2-partlen),firstcolon,partlen);

  partlen=l-(lastcolon-tempbuf);
  strncpy(output+6+(6-partlen),lastcolon,partlen);

  //Finally convert the string (11AABB44FF66) into a real byte array
  //written backwards in the same format that the SMA packets expect it in  
  int i2=5;
  for(int i=0; i<12; i+=2){
    outarray[i2--]=hex2bin(&output[i]);
  }

  /*
Serial.print("BT StringToArray=");  
   for(int i=0; i<6; i+=1)    debugPrintHexByte(outarray[i]);   
   Serial.println("");
   */
}

int hex2bin( const char *s )
{
  int ret=0;
  int i;
  for( i=0; i<2; i++ )
  {
    char c = *s++;
    int n=0;
    if( '0'<=c && c<='9' )
      n = c-'0';
    else if( 'a'<=c && c<='f' )
      n = 10 + c-'a';
    else if( 'A'<=c && c<='F' )
      n = 10 + c-'A';
    ret = n + ret*16;
  }
  return ret;
}
