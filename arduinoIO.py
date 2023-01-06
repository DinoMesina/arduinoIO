#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#    HAL userspace component to interface with Arduino board for IO
#
#    Copyright (C) 2022-2023 Dino del Favero <dino@mesina.net>
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# for python <3 LinuxCNC 2.7.15 
from __future__ import print_function

import serial
import sys
import time
import os 

#PORT = "/dev/ttyUSB0" # for Arduino Duemilanove/UNO
PORT = "/dev/ttyACM0" # for Arduino MEGA 2560 
BAUD = 115200
TIMEOUT = 2

if len(sys.argv) > 1:
    PORT = sys.argv[1]

nInput = 8         # input pins available on arduino 
nOutput = 7        # output pins available on arduino 
nAnalogIn = 2      # input analog pins available on arduino 
nAnalogOut = 1     # output analog pins available on arduino 

printDebug = not False
inLinuxCNC = not True

ser = serial.Serial()

ACK      = 0b00000110 # ASCII ACK
NACK     = 0b00010101 # ASCII NAK
START    = 0b01010011 # ASCII S
READDIG  = 0x01
WRITEDIG = 0x01
READANA  = 0x02
WRITEANA = 0x02

bufR = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] # buffer read
bufW = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] # buffer write
ack = False
digitalTime = 0
analogTime = 0
stateLed = 0
analogCh = 0
analogVal = 0

waitingTime = 0.040  # initial waiting time after sending bytes 

if inLinuxCNC:
    import hal
    hc = hal.component(os.path.basename(__file__))
    # digital in 
    for n in range(nInput):
        hc.newpin("digital-in-%02d" % n, hal.HAL_BIT, hal.HAL_OUT)
        hc.newpin("digital-in-%02d-not" % n, hal.HAL_BIT, hal.HAL_OUT)
        hc.newparam("digital-in-%02d-pullup" % n, hal.HAL_BIT, hal.HAL_RW)
    # digital out
    for n in range(nOutput):
        hc.newpin("digital-out-%02d" % n, hal.HAL_BIT, hal.HAL_IN)
        hc.newparam("digital-out-%02d-invert" % n, hal.HAL_BIT, hal.HAL_RW)
    # analog in
    for n in range(nAnalogIn):
        hc.newpin("analog-in-%02d" % n, hal.HAL_FLOAT, hal.HAL_OUT)
        hc.newparam("analog-in-%02d-offset" % n, hal.HAL_FLOAT, hal.HAL_RW)
        hc.newparam("analog-in-%02d-gain" % n, hal.HAL_FLOAT, hal.HAL_RW)
        hc['analog-in-%02d-gain' % n] = 1.0
    # analog out
    for n in range(nAnalogOut):
        hc.newpin("analog-out-%02d" % n, hal.HAL_FLOAT, hal.HAL_IN)
        hc.newparam("analog-out-%02d-offset" % n, hal.HAL_FLOAT, hal.HAL_RW)
        hc.newparam("analog-out-%02d-scale" % n, hal.HAL_FLOAT, hal.HAL_RW)
        hc['analog-out-%02d-scale' % n] = 1.0
    # READY 
    hc.ready()


# write on serial port the buffer 
def writeBuffer():
    # calculate the checksum
    checksum = 0
    for i in range(6):
        checksum += bufW[i]
    bufW[6] = (checksum >> 8) & 0xFF
    bufW[7] = (checksum & 0xFF)
    # send bytes
    if (sys.version_info.major < 3):
        values = bytearray([ int(bufW[0]), int(bufW[1]), int(bufW[2]), int(bufW[3]), int(bufW[4]), int(bufW[5]), int(bufW[6]), int(bufW[7]) ])
        ser.write(values)
    else:
        for b in bufW:
            ser.write(b.to_bytes(1, byteorder='big'))
    
    # sleep a little bit :-) otherwise sending data sometimes not successful
    time.sleep(waitingTime)
    
    # debug
    if printDebug:
        print("{} W-->:".format(os.path.basename(__file__)), end='  ')
        for b in bufW:
            print(str("{0:0=8b}".format(b)), end='  ')
        print()


# set digital pins state in LinuxCNC
def setDigitalIn():
    # make bit mask 
    bits = bufR[5] & 0xFF
    bits = (bits << 8) | (bufR[4] & 0xFF)
    bits = (bits << 8) | (bufR[3] & 0xFF)
    bits = (bits << 8) | (bufR[2] & 0xFF)
    if inLinuxCNC:
        for i in range(nInput):
            hc['digital-in-%02d' % i] = ((bits & (0x01 << i)) != 0)
            hc['digital-in-%02d-not' % i] = ((bits & (0x01 << i)) == 0)
    if printDebug:
        print("{} DigitalIn: ".format(os.path.basename(__file__)), end=' ')
        for i in range(nInput):
            print(((bits & (0x01 << i)) >> i), end='')
        print()


# send digital pins state to arduino 
def sendDigitalOut():
    # send pin map to arduino 
    bufW[0] = START
    bufW[1] = WRITEDIG
    bufW[2] = 0x00
    bufW[3] = 0x00
    bufW[4] = 0x00
    bufW[5] = 0x00
    if (nOutput > 24):
        for i in range((nOutput if nOutput <= 31 else 31), 23, -1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[5] = (bufW[5] | (0x01 << (i - 24)))
            else:
                bufW[5] = (bufW[5] & ~(0x01 << i))
    if (nOutput > 16):
        for i in range((nOutput if nOutput <= 23 else 23), 15, -1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[4] = (bufW[4] | (0x01 << (i - 16)))
            else:
                bufW[4] = (bufW[4] & ~(0x01 << i))
    if (nOutput > 8):
        for i in range((nOutput if nOutput <= 15 else 15), 7, -1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[3] = (bufW[3] | (0x01 << (i - 8)))
            else:
                bufW[3] = (bufW[3] & ~(0x01 << i))
    for i in range(((nOutput - 1) if nOutput < 8 else 7), -1, -1):
        if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
            bufW[2] = (bufW[2] | (0x01 << i))
        else:
            bufW[2] = (bufW[2] & ~(0x01 << i))
    # send to arduino
    writeBuffer()
    # debug
    if printDebug:
        print("{} DigitalOut: ".format(os.path.basename(__file__)), end=' ')
        for i in range(nOutput):
            if (i < 8):
                print(((bufW[2] & (0x01 << i)) >> i), end='')
            elif (i < 16):
                print(((bufW[3] & (0x01 << (i-8))) >> (i-8)), end='')
            elif (i < 24):
                print(((bufW[3] & (0x01 << (i-16))) >> (i-16)), end='')
            elif (i < 32):
                print(((bufW[3] & (0x01 << (i-24))) >> (i-24)), end='')
        print()



# set analog pins value in LinuxCNC
def setAnalogIn():
    # extract 1st analog channel
    ch1 = ((bufR[2] >> 2) & 0x1F)
    # extract 1st analog value
    val1 = ((((bufR[2] & 0x3) * 256) + (bufR[3] & 0xFF)) & 0x3FF)
    
    # extract 2st analog channel
    ch2 = ((bufR[4] >> 2) & 0x1F)
    # extract 2st analog value
    val2 = ((((bufR[4] & 0x3) * 256) + (bufR[5] & 0xFF)) & 0x3FF)
    
    if inLinuxCNC:
        # set value in LinuxCNC
        # ch1
        if (ch1 < nAnalogIn):
            gain = hc['analog-in-%02d-gain' % ch1] or 1.0
            offset = hc['analog-in-%02d-offset' % ch1]
            hc['analog-in-%02d' % ch1] = val1 / 1023.0 * 5.0 * gain + offset
        # ch2
        if (ch2 < nAnalogIn):
            gain = hc['analog-in-%02d-gain' % ch2] or 1.0
            offset = hc['analog-in-%02d-offset' % ch2]
            hc['analog-in-%02d' % ch2] = val2 / 1023.0 * 5.0 * gain + offset
    if printDebug:
        print("{} AnalogIn ch: {} val: {}".format(os.path.basename(__file__), ch1, val1))
        print("{} AnalogIn ch: {} val: {}".format(os.path.basename(__file__), ch2, val2))
        print()


# send analog value to arduino 
def sendAnalogOut(ch):
    if (ch >= nAnalogOut):
        return

    # set buffer 
    bufW[0] = START
    bufW[1] = WRITEANA
    bufW[2] = 0xFF # channel 256 do not exist (if setted to 0 arduino will set the output of the first channel to 0)
    bufW[3] = 0x00
    bufW[4] = 0xFF # channel 256 do not exist (if setted to 0 arduino will set the output of the first channel to 0)
    bufW[5] = 0x00
    
    # get analog value of channel ch 
    scale = hc['analog-out-%02d-scale' % ch] or 1.0
    offset = hc['analog-out-%02d-offset' % ch]
    data = ((hc['analog-out-%02d' % ch] - offset) / scale) / 5.0 * 255.0
    bufW[2] = ch
    bufW[3] = int(round(data))

    if ((ch + 1) < nAnalogOut):
        # get analog value of channel (ch + 1)
        scale = hc['analog-out-%02d-scale' % (ch + 1)] or 1.0
        offset = hc['analog-out-%02d-offset' % (ch + 1)]
        data = ((hc['analog-out-%02d' % (ch + 1)]- offset) / scale) / 5.0 * 255.0
        bufW[4] = ch + 1
        bufW[5] = int(round(data))
    # send to arduino
    writeBuffer()
    # debug
    if printDebug:
        print("{} AnalogOut ch: {} val: {}".format(os.path.basename(__file__), bufW[2], bufW[3]))
        print("{} AnalogOut ch: {} val: {}".format(os.path.basename(__file__), bufW[4], bufW[5]))


try:
    # clear buffer
    bufR = []
    # set serial parameters, needed for Arduino MEGA 2560 board reset
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = TIMEOUT
    ser.open()
    # wait Arduino reset completed
    time.sleep(2.0)
    # do forever
    while True:
        # read serial from arduino
        while ser.inWaiting():
            # read a byte from the serial port and convert it into int
            byte = ord(ser.read(size=1))
            bufR.append(byte)
            if ((byte == ACK) and (len(bufR) == 1)):
                ack = True
                if printDebug:
                    print("{} R<--:ACK".format(os.path.basename(__file__)))

            elif ((byte == NACK) and (len(bufR) == 1)):
                ack = False
                waitingTime += 0.002
                if printDebug:
                    print("{} R<--:NACK\n    waitingTime={:f}".format(os.path.basename(__file__), waitingTime))
            # if the first byte is not the start byte clear the buffer 
            if (bufR[0] != START):
                bufR = []
            # if have received 8 bytes check the checksum 
            if (len(bufR) == 8):
                checksum = bufR[6] * 256 + bufR[7]
                # calculate the sum of bytes received 
                sumR = 0
                for b in bufR[0:-2]:
                    sumR += b
                if (sumR == checksum):
                    # OK :) received good
                    # debug
                    if printDebug:
                        print("{} R<--:".format(os.path.basename(__file__)), end='  ')
                        for b in bufR:
                            print(str("{0:0=8b}".format(b)), end='  ')
                        print()
                    # check the command
                    if (bufR[1] == READDIG):
                        # set digital input pins
                        setDigitalIn()
                    elif (bufR[1] == READANA):
                        # set analog input pins
                        setAnalogIn()
                else:
                    print("ERROR: checksum not match!")
                # clear the buffer
                bufR=[]
        # end while

        if inLinuxCNC:
            # send digital pin stat to arduino
            sendDigitalOut()
            lastTime = time.time()
                
            # send analog pin value to arduino
            if (nAnalogOut > 0):
                sendAnalogOut(analogCh)
                # next ch 
                analogCh += 2
                if (analogCh >= nAnalogOut):
                    analogCh = 0
            
        else:
            now = time.time()
            # blink LED 
            if ((now - digitalTime) > 0.5):
                # change LED state
                stateLed = 0b0 if stateLed else 0b1000000 # 0b1000000 = 0x40 = 64 
                bufW[0] = START
                bufW[1] = WRITEDIG
                bufW[2] = stateLed 
                bufW[3] = (0x00)
                bufW[4] = (0x00)
                bufW[5] = (0x00)
                writeBuffer()
                print("LED ON" if stateLed else "LED OFF")
                digitalTime = time.time()
            # send value to 1st PWM
            if ((now - analogTime) > 5.0):
                # PWM
                bufW[0] = START
                bufW[1] = WRITEANA
                bufW[2] = (0x00)
                bufW[3] = (analogVal & 0xFF)
                bufW[4] = (0xFF)
                bufW[5] = (0x00)
                writeBuffer()
                print('PWM = {:3d} Volt = {:3.3}'.format(analogVal, (analogVal / 255.0 * 5.0)))
                analogVal += 15
                if (analogVal > 255):
                    analogVal = 0
                analogTime = time.time()
            # sleep a bit 
            time.sleep(0.1)

except (KeyboardInterrupt,):
    if inLinuxCNC:
        raise SystemExit
    else:
        print("Exit")
