# ESP8266 MQTT Thermostat
http://ihormelnyk.com/mqtt_thermostat

Simple MQTT OpenTherm Thermostat based on [OpenTherm Adapter](http://ihormelnyk.com/opentherm_adapter), [OpenTherm Library](http://ihormelnyk.com/opentherm_library), ESP8266 (WeMos D1 Mini) and PID Temperature Controller.

## Changes compared to original
This codebase is a fork of Ihor Melnyk's example that illustrated how one could make use of his great OpenTherm adapter kit.

I've ran a slightly modified version of the example for approximately one year, after which I decided to commit my version as a fork and make some further changes.
Compared to the original code the following changes can be observed:

   - MQTT message format
     This has been adjusted to allow for easy integration into my Home Assistant setup.
   - Move of MQTT configuration to separate file
   - Fixed reconnection to MQTT server
     If the MQTT server could not be reached, the thermostat stopped talking to the boiler. This could cause the boiler to start heating indefinitely.
