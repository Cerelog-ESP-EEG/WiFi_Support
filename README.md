# WiFi_Support (Under Dev)

## About
This is a guide to use the device wirelesly by connecting the ESP-EEG to your local network to stream data. The device defaults to USB streaming from its original firmware. Should, after setting all this up, you prefer to use USB to stream data flash this [original firmware](https://github.com/Cerelog-ESP-EEG/ESP-EEG/tree/main/firmware) back to revert changes

 
## For usage only with the forked OpenBCI Gui/LSL Streaming  

**Using the device with Brainflow API currently only supported via USB (connect to laptop not connected to mains)** 


**To use WiFi to stream to OpenBCI GUI / LSL please follow these instructions [Guide](https://github.com/Cerelog-ESP-EEG/How-to-use-OpenBCI-GUI-fork) with the below changes:**

1. You Must flash the special firmware listed here using the Arduino IDE: Prior to flashing, enter the SSID and pasword of your network into the code before running so the device can connect to your network
3. You must use the LSL python connection script listed here, don't use the python script shown in the orignal guide linked above (Its for USB connection)


