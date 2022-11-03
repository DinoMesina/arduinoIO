#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#    HAL userspace component to interface with Arduino board for IO
#
#    Copyright (C) 2022 Dino del Favero <dino@mesina.net>
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

PORT = "/dev/ttyUSB0"

if len(sys.argv) > 1:
    PORT = sys.argv[1]

ser = serial.Serial(PORT, 115200, timeout=2)
nInput = 6         # input pins available on arduino 
nOutput = 3        # output pins available on arduino 
nAnalogIn = 2      # input analog pins available on arduino 
nAnalogOut = 3     # output analog pins available on arduino 

printDebug = not False
inLinuxCNC = not True 

ACK      = 0b00000110 # ASCII ACK
NACK     = 0b00010101 # ASCII NAK
START    = 0b01010011 # ASCII S
READDIG  = 0x01
WRITEDIG = 0x01
READANA  = 0x02
WRITEANA = 0x02

bufR=[] # buffer read
bufW=[0, 0, 0, 0, 0, 0, 0, 0] # buffer write
ack = False
digitalTime = 0
analogTime = 0
stateLed = 0
analogCh = 0
analogVal = 0

if (inLinuxCNC):
    import hal
    hc = hal.component("arduinoIO")
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

    # debug
    if printDebug:
        print("W-->:", end='  ')
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
    if (inLinuxCNC):
        for i in range(nInput):
            hc['digital-in-%02d' % i] = ((bits & (0x01 << i)) != 0)
            hc['digital-in-%02d-not' % i] = ((bits & (0x01 << i)) == 0)
    if printDebug:
        print("DigitalOut:", end=' ')
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
        for i in range(nOutput,23,-1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[5] = (bufW[5] | (0x01 << (i - 24)))
            else:
                bufW[5] = (bufW[5] & ~(0x01 << i))
    if (nOutput > 16):
        if (nOutput <= 23):
            start = nOutput
        else:
            start = 23
        for i in range(start,15,-1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[4] = (bufW[4] | (0x01 << (i - 16)))
            else:
                bufW[4] = (bufW[4] & ~(0x01 << i))
    if (nOutput > 8):
        if (nOutput <= 15):
            start = nOutput
        else:
            start = 15
        for i in range(start,7,-1):
            if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
                bufW[3] = (bufW[3] | (0x01 << (i - 8)))
            else:
                bufW[3] = (bufW[3] & ~(0x01 << i))
    if (nOutput < 8):
        start = nOutput - 1
    else:
        start = 7
    for i in range(start,-1,-1):
        if ((hc['digital-out-%02d' % i]) == (not hc['digital-out-%02d-invert' % i])):
            bufW[2] = (bufW[2] | (0x01 << i))
        else:
            bufW[2] = (bufW[2] & ~(0x01 << i))
    # send to arduino
    writeBuffer()


# set analog pins value in LinuxCNC
def setAnalogIn():
    # extract 1st analog channel
    ch1 = ((bufR[2] >> 2) & 0x1F)
    # extract 1st analog value
    val1 = (((bufR[2] & 0x3) * 256) + (bufR[3] & 0xFF))
    
    # extract 2st analog channel
    ch2 = ((bufR[4] >> 2) & 0x1F)
    # extract 2st analog value
    val2 = ((((bufR[4] & 0x3) *256) + (bufR[5] & 0xFF)) & 0x3FF)
    
    if (inLinuxCNC):
        # set value in LinuxCNC
        # ch1
        if (ch2 < nAnalogIn):
            gain = hc['analog-in-%02d-gain' % ch1] or 1.0
            offset = hc['analog-in-%02d-offset' % ch1]
            hc['analog-in-%02d' % ch1] = val1 / 1023.0 * 5.0 * gain + offset
        # ch2
        if (ch2 < nAnalogIn):
            gain = hc['analog-in-%02d-gain' % ch2] or 1.0
            offset = hc['analog-in-%02d-offset' % ch2]
            hc['analog-in-%02d' % ch2] = val2 / 1023.0 * 5.0 * gain + offset
    if printDebug:
        print("AnalogIn ch: " + str(ch1) + " val: " + str(val1))
        print("AnalogIn ch: " + str(ch2) + " val: " + str(val2))
        print()


# send analog value to arduino 
def sendAnalogOut(ch):
    # set buffer 
    bufW[0] = START
    bufW[1] = WRITEANA
    bufW[2] = 0xFF # channel 255 do not exist if set to 0 in arduino will set it
    bufW[3] = 0x00
    bufW[4] = 0xFF # channel 255 do not exist if set to 0 in arduino will set it
    bufW[5] = 0x00
    
    # get analog value of channel ch 
    scale = hc['analog-out-%02d-scale' % ch] or 1.0
    offset = hc['analog-out-%02d-offset' % ch]
    data = (hc['analog-out-%02d' % ch] - offset) / scale / (5.0)
    bufW[2] = ch
    bufW[3] = int(data * 255 + 0.5)
    if (printDebug):
        print("AnalogOut ch=" + str(bufW[2]) + " val=" + str(bufW[3]))
    if ((ch + 1) < nAnalogOut):
        # get analog value of channel (ch + 1)
        scale = hc['analog-out-%02d-scale' % (ch + 1)] or 1.0
        offset = hc['analog-out-%02d-offset' % (ch + 1)]
        data = (hc['analog-out-%02d' % (ch + 1)]- offset) / scale / (5.0)
        bufW[4] = ch + 1
        bufW[5] = int(data * 255 + 0.5)
        if(printDebug):
            print("AnalogOut ch=" + str(bufW[4]) + " val=" + str(bufW[5]))
    # send to arduino
    writeBuffer()


try:
    while True:
        # read from arduino 
        while ser.inWaiting():
            # read a byte from the serial port and convert it into int
            byte = ord(ser.read(size=1))
            bufR.append(byte)
            if ((byte == ACK) and (len(bufR) == 1)):
                ack = True
                if printDebug:
                    print("ACK")
            elif ((byte == NACK) and (len(bufR) == 1)):
                ack = False
                if printDebug:
                    print("NACK")
            # if the first byte is not the start byte clear the buffer 
            if (bufR[0] != START):
                bufR=[]
            # if have received 8 bytes check the checksum 
            if (len(bufR) == 8):
                checksum = bufR[6] * 256 + bufR[7]
                sumR = 0
                # calculate the sum of bytes received 
                for b in bufR[0:-2]:
                    sumR += b
                if (sumR == checksum):
                    # OK :) received good

                    # debug
                    if printDebug:
                        print("R<--:", end='  ')
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

        if (inLinuxCNC):
            # send digital Pin stat to arduino
            sendDigitalOut()
            # sleep a bit :-)
            time.sleep(0.025)
            # send analog Pin stat to arduino
            if (nAnalogOut > 0):
                sendAnalogOut(analogCh)
                # sleep a bit :-)
                time.sleep(0.025)
                analogCh += 2
                if (analogCh >= nAnalogOut):
                    analogCh = 0

        else:
            # TEST blink LED and send value to 1st PWM
            now = time.time_ns()
            if ((now - digitalTime) > 500000000):
                # change LED state
                stateLed = 0x00 if stateLed else 0x01 
                bufW[0] = START
                bufW[1] = WRITEDIG
                bufW[2] = stateLed 
                bufW[3] = (0x00)
                bufW[4] = (0x00)
                bufW[5] = (0x00)
                writeBuffer()
                print("LED=" + ("ON" if stateLed else "OFF"))
                digitalTime = time.time_ns()
            
            if ((now - analogTime) > 1500000000):
                # PWM
                bufW[0] = START
                bufW[1] = WRITEANA
                bufW[2] = (0x00)
                bufW[3] = (analogVal & 0xFF)
                analogVal += 12
                if (analogVal > 255):
                    analogVal = 0
                bufW[4] = (0xFF)
                bufW[5] = (0x00)
                writeBuffer()
                print("PWM=" + str(analogVal))
                analogTime = time.time_ns()
                
            time.sleep(0.025)
                                
except (KeyboardInterrupt,):
    if (inLinuxCNC):
        raise SystemExit
    else:
        print("Exit")
