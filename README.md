# ESP32_to_SMA
This project uses an ESP32 to connect to an SMA SunnyBoy inverter, using Bluetooth.

I have tested it with the SMA Model SB 8000US.
This inverter model, circa 2013, has the Bluetooth feature added via an add-on module inside the inverter.

The starting point for this project was the code posted by "stuartpittaway" on github. That really well done project can be found at: https://github.com/stuartpittaway/nanodesmapvmonitor

NOTE: To connect to your ESP32 module, and your inverter, you'll need to change the MAC address constants in the bluetooth.h file.

CODE CHANGES:
I modified / added / subtracted features (compared to nanodesmapvmonitor) in the following ways:

1) It uses the DOIT ESP32 DEVKIT V1 module by Zerynth, and will probably run on most ESP32 modules. (The nanodesmapvmonitor project uses an Arduino board + USB-to-Bluetooth dongle.)

2) Does not interfere with, and is not confused by, an existing SMA Sunny Beam display device. No cross-interference.

3) Removed the PVoutput.org reporting.

4) I only use the "getInstantACPower()" function to extract that information from the inverter. Getting other information from the inverter probably still works, but has not been tested.


TODO:

1) Restore the function whereby it only talks to the SMA inverter when the sun is up. Will probably use the "ESP32Time" library by fbiego while doing this.

2) Use some LEDs to indicate connection status, and perhaps blink an LED, as BT packets are exchaged.


KNOWN BUGS:

None.
