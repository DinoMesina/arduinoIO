# arduinoIO
Arduino firmware and LinuxCNC .hal

Useful for expand IO ports (digital and analogic) to LinuxCNC

This hal component is NOT REALTIME it takes around 40-150 ms to communicate the modified values to LinuxCNC, the time depends on the number of input digital and analog used.

Load the sketch (arduinoIO.ino) in the Arduino IDE, configure the pin lists and states, compile and transfer it to the Arduino board.
To use it copy arduinoIO.py to arduinoIO (without extension) in a directory on your PATH, an set it executable.

NOTE: If you don't have already installed, install python-serial

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](http://paypal.me/dinodf)
