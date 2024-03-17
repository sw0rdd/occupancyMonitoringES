# occupancyMonitoringES
This repo is the second repo of an embedded system project to monitor occupancy at a library, this repo contains source code for Raspberry pi pico and arduino UNO Rev 4. 

# on the picos
In the /pico_Micropython you will find 3 files

* pico_one.py --> (entrance) <br>
* pico_two.py --> (second & third floor)<br>
* pico_three.py --> (ground floor)<br>

# on the arduino
For the arduino code we acutally need one file but here we have included two

* AruinoCode.ino --> This doesn't use freeRTOS, using this code the Arduino, in turn, aggregates the collected data that is being sent from the three picos and communicates with a local server via POST requests, updating the occupancy status 
<br>
* ArduinoFreeRTOS.ino --> In this code we use freeRTOS, but the Arduino can only connect and collect data from two picos instead of three. 




