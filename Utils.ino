//#include "ESP32Time.h"
//#include "time.h"
#include <Esp.h>

static void error() {
  Serial.println(F("\r\n*** FATAL ERROR *** About to do a soft restart of the ESP32 now!"));
  int loop=20;
  while (loop>0) {
    digitalWrite(output23, HIGH);  // Yellow on
    delay(50);
    digitalWrite(output23, LOW);  // Yellow off
    delay(150);
    loop--;
  }

  //softReset(); 
  ESP.restart();
}

/*
static void printMacAddress(byte mymac[]) {
  Serial.print(F("MAC="));  
  for(int i=0; i<6; i++) {
    printHexDigits(mymac[i]);
  }
  Serial.println("");  
}
*/
