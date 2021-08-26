# ESP32_to_SMA
This project uses an ESP32 to connect to an SMA SunnyBoy inverter, using Bluetooth.

I have tested it with the SMA Model SB 8000US.
This inverter model, circa 2013, has the Bluetooth feature added via an add-on module inside the inverter.

The starting point for this project was the code posted by "stuartpittaway" on github. That really well done project can be found at: https://github.com/stuartpittaway/nanodesmapvmonitor

NOTE: To connect to your ESP32 module, and your inverter, you'll need to change the MAC address constants in the bluetooth.h file, and perhaps the SunnyBoy's login password (contained in the file ESP32_SMA.ino).

CODE CHANGES:
I modified / added / subtracted features (compared to nanodesmapvmonitor) in the following ways:

1) It uses the DOIT ESP32 DEVKIT V1 module by Zerynth, and will probably run on most ESP32 modules. (The nanodesmapvmonitor project uses an Arduino board + USB-to-Bluetooth dongle.)

2) Does not interfere with, and is not confused by, an existing SMA Sunny Beam display device. No cross-interference.

3) Removed the PVoutput.org reporting.

4) I only use the "getInstantACPower()" and "getDailyYield()" functions to extract that information from the inverter. Getting other information from the inverter probably still works, but has not been tested.

5) Calculates sunrise and sunset times. After sunrise, connects to the SMA inverter. Just before sunset, disconnects the BT connection with the SMA inverter.

NOTES:

1) In the file ESP32_SMA.ino, there are some commented lines that refer to an Arduino. This refers to a seperate Arduino that is monitoring my house's power consumption, which is an upcoming feature.


TODO:

1) Fix the code that attempts to use the ESP32 EEPROM to store certain values.


KNOWN BUGS:

1) The saving of data to EEPROM (via Preferences library) does not work.
