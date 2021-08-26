// Assign LED output variables to GPIO pins
const int output23 = 23;
const int output22 = 22;

#define INVERTERSCAN_PIN 14    //Analogue pin 1 - next to VIN on connectors
#define BT_KEY 15              //Forces BT BOARD/CHIP into AT command mode
#define RxD 16
#define TxD 17
#define BLUETOOTH_POWER_PIN 5  //pin 5

//Location in EEPROM where the 2 arrays are written
#define ADDRESS_MY_BTADDRESS  0
#define ADDRESS_SMAINVERTER_BTADDRESS  10


unsigned char smaBTInverterAddressArray[6]={  0x0,0x0,0x0,0x25,0x80,0x00 };  // BT address of my SMA.
unsigned char myBTAddress[6]={ 0x0,0x0,0x0,0xF2,0x3A,0x08 };  // BT address of my ESP32.

// Function prototypes added by DRH
int BTStart();
