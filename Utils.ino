#include "ESP32Time.h"
#include "time.h"
#include <Esp.h>

static void error() {
  Serial.println(F("\r\n*** FATAL ERROR *** About to do a soft restart of the ESP32 now!"));
  int loop=150;
  while (loop>0) {
    digitalWrite( RED_LED, HIGH);
    delay(50);
    digitalWrite( RED_LED, LOW);
    delay(150);
    loop--;
  } 

  //softReset(); 
  ESP.restart();
}

/* Cannot get it to see the hour(t) routine in time.h, so blocked out, for now. By DRH.
static void digitalClockDisplay(time_t t){
  // digital clock display  
  printDigits(hour(t));
  Serial.print(':');
  printDigits(minute(t));
  Serial.print(':');
  printDigits(second(t));
  Serial.print(' ');
  Serial.print(year(t)); 
  printDigits(month(t));
  printDigits(day(t));
}

static void printDigits(int digits){
  if(digits < 10) Serial.print('0');
  Serial.print(digits);
}

static void printHexDigits(int digits){
  if(digits <= 0xF) Serial.print('0');
  Serial.print(digits,HEX);
}
*/

/*
static void printMacAddress(byte mymac[]) {
  Serial.print(F("MAC="));  
  for(int i=0; i<6; i++) {
    printHexDigits(mymac[i]);
  }
  Serial.println("");  
}
*/
