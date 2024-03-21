# Real-Time Occupancy Monitoring System for University Library

This repository is part of an academic project focused on monitoring occupancy levels in a library across different floors and zones using Raspberry Pi Pico W units with ultrasonic sensors and an Arduino board. The data collected by the Pico units is sent to a central Arduino, which then posts the information to a local web server. The server runs on Node.js and stores occupancy data in MongoDB. <br>
This project is made for the course "Project with Embedded Systems 2DT304". <br>
This [Repository](https://github.com/sw0rdd/occupancyMonitoringWebsite) contains the source code for the webserver.

## Contributers
Seif-Alamir Yousef & Ludvig Svensson

# Academic Report 
In this [Academic Report](https://docs.google.com/document/d/1X3nvK4McUbsehoqTqAhvU4GHUhgysJOzC636muut5SY/edit?usp=sharing) you will detailed information about the project.

### Raspberry Pi Pico - MicroPython Code

Inside the `/pico_Micropython` directory, you will find three distinct files corresponding to the different zones in the library:

- `pico_one.py`: Code for the entrance zone.
- `pico_two.py`: Code for the second and third floor zone.
- `pico_three.py`: Code for the ground floor zone.

These scripts are written in MicroPython and are responsible for measuring entries and exits using ultrasonic sensors attached to each Raspberry Pi Pico.

### Arduino - C++ Code

The `/Arduino` directory contains two versions of the code for the Arduino UNO Rev 4:

- `ArduinoCode.ino`: This script does not utilize FreeRTOS. It's designed to collect data from the three Picos and communicate with a local server via POST requests, updating the occupancy status accordingly.

- `ArduinoFreeRTOS.ino`: This version incorporates FreeRTOS to manage multitasking. However, due to the multitasking complexity, it is currently limited to connecting and collecting data from only two Picos.

Use only one of them on the ARduino

## Installation and Usage

To deploy the code on your microcontrollers, follow these steps:

1. **Raspberry Pi Pico:**
   - Ensure each Pico is running MicroPython. Instructions can be found on the official Raspberry Pi website.
   - Upload the respective `.py` files to each Pico device using Thonny or any other preferred MicroPython IDE.

2. **Arduino UNO Rev 4:**
   - Open the desired `.ino` file in the Arduino IDE.
   - Compile and upload the code to your Arduino UNO Rev 4.


### Configuring Arduino

Ensure the Arduino is programmed with the IP address of the device running the web server and the matching `API_KEY` for successful POST requests to the server. Also, make sure the Arduino and the server are on the same network.

