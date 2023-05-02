# ESP-WaterBucket
ESP8266 based solar charge controller mod for remote control and monitoring of PV-battery loads
```
Optional Hardware setup: 
1. Embed an ESP8266 (or similar) into a generic solar charge controller such as "Renogy Wanderer 10 Amp 12V/24V" 

2. Find and solder a wire to the 5v or 3.3v rail inside your charge controller to power the ESP. 
    Hint: This can be found by probing the charge controller's own MCU or other known IC's VCC pin.
    
3. Find and solder a 1kR resistor to the signal pin of the charge controller's own Enter button. This will toggle the load.

4. Find and solder any appropriate resistor divider between the battery pin of the charge controller, ground, and A0.
    Hint: you will need to tweak values in ADC() depending on nominal system voltage and chosen ESP. I used  10kR and 330kR.
    !Hint: a 500kR trimmer potentiometer will also suffice and allow for some easy calibration. Just be sure to set it before power on.
    !!Hint: account for Voc (max open circuit voltage) since most LiFe batteries will go open circuit when fully charged.
    
5. Find and solder a wire to the gate of the load MOSSFET inside your charge controller.
 
ESP will host an AP upon first boot. Connect to it and tell it your WiFi name/key.

Don't forget to "ESP8266 Sketch Data Upload" the html files in /data to flash.
```
