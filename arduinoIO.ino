/*
 *   Copyright (C) 2022-2023 Dino del Favero <dino@mesina.net>
 *   
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *   
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * 
 * Protocol:
 *  8bit  start = 0b01010011 (ASCII 'S')
 *  8bit command
 *  8bit  data0 
 *  8bit  data1 
 *  8bit  data2 
 *  8bit  data3 
 * 16bit checksum (uint16_t sum of prev 6 uint8_t)
 * 
 * command 0:
 *   nothing 
 * command 1:
 *   read/write digital pin 
 *   (32bit pin mask MSB..LSB MSB..LSB MSB..LSB MSB..LSB : MSB=8th pin LSB=1st pin)
 * command 2:
 *   read/write analog pin 
 *   (32bit xxxxxxyy yyyyyyyy uuuuuuvv vvvvvvvv : x|u=channel y|v=value)
 * 
 */

#define ACK   0b00000110 // ASCII ACK
#define NACK  0b00010101 // ASCII NAK
#define START 0b01010011 // ASCII 'S'

// Commands
#define READDIG  0x01
#define WRITEDIG 0x01
#define READANA  0x02
#define WRITEANA 0x02

#define BAUD 115200

#define MAXDELAY 100    // max time to wait in millis 
#define ANALOGREADS 20  // how many times do I have to read the analog data 

int pinIn[]        = { 12, 11, 10,  8, A0, A1, A2, A3 }; // pin list digital input 
int inActive[]     = {  0,  0,  0,  0,  0,  0,  0,  0 }; // 1=Active HIGH; 0=Active LOW 
int pinOut[]       = {  7,  6,  5,  4,  3,  2, 13 }; // pin list digital output 
int pinInitState[] = {  1,  1,  1,  1,  1,  1,  1 }; // state of the output pins on initialization 
int pinOutInvert[] = {  1,  1,  1,  1,  1,  1,  1 }; // 1=Invert state 
int anaIn[]        = { A6, A7 }; // pin list analog input 
int anaOut[]       = {  9 }; // pin list analog output 

int ack=0;
unsigned int ind = 0;
int maxRead = 0;
int maxWrite = 0;
int maxAnaRead = 0;
int maxAnaWrite = 0;
int indAnaRead = 0;

uint8_t sbuf[8];
unsigned long lastSend = 0;

void setup() {
  // initialize the serial communication
  Serial.begin(BAUD);

  // set digital input pins
  maxRead = sizeof(pinIn) / sizeof(int);
  for (int i=0; i<maxRead; i++){
    pinMode(pinIn[i], INPUT);      // set pin to input 
    digitalWrite(pinIn[i], HIGH);  // turn on pullup resistors 
  }

  // set digital output pins 
  maxWrite = sizeof(pinOut) / sizeof(int);
  for (int i=0; i<maxWrite; i++){
    pinMode(pinOut[i], OUTPUT);      // set pin to output 
    if (pinInitState[i]) {
      digitalWrite(pinOut[i], HIGH); // turn on pin 
    } else {
      digitalWrite(pinOut[i], LOW);  // turn off pin 
    }
  }

  // set analog input pins
  maxAnaRead = sizeof(anaIn) / sizeof(int);
  for (int i=0; i<maxAnaRead; i++){
    pinMode(anaIn[i], INPUT);     // set pin to input 
  }

  // set analog output pins 
  maxAnaWrite = sizeof(anaOut) / sizeof(int);
  for (int i=0; i<maxAnaWrite; i++){
    pinMode(anaOut[i], OUTPUT);   // set pin to output 
  }

}

void loop() {
  uint16_t sum = 0;
  // read data from serial
  while ((Serial.available() > 0) && (ind < 8)) {
    sbuf[ind] = Serial.read();
    if ((ind < 8) && (sbuf[0] == START)){
      ind++;
      //Serial.write(ACK);
    } else {
      ind = 0;
      Serial.write(NACK);
    }
  }
  // if have readed all 8 bytes do 
  if (ind == 8){
    // reset ind
    ind = 0;
    // calculate the sum and check if it match with checksum sended
    sum = uint16_t(sbuf[0]);
    for (int j=1; j<6; j++) {
      sum += uint16_t(sbuf[j]);
    }
    uint16_t checksum = uint16_t(sbuf[6] * 256 + uint16_t(sbuf[7]));
    if ((sbuf[0] == START) && (sum == checksum)) {
      // OK :) send ACK
      ack = 1;
      Serial.write(ACK);

      // do command 
      switch (sbuf[1]) {
        case WRITEDIG:
          writeDig(); // write digital pins 
          break;
        case WRITEANA:
          writeAna(); // write analog pins 
          break;
      }
            
    } else {
      // error :( send NACK
      Serial.write(NACK);
    }
  }

  // have to update the data?
  if (ack or (millis() - lastSend > MAXDELAY)) {
    // read the digital input pins
    if (maxRead > 0) {
      readDig();
    }
    // read the analog input pins 
    if (maxAnaRead > 0) {
      readAna(indAnaRead);
      indAnaRead += 2;
      if (indAnaRead >= maxAnaRead) {
        indAnaRead = 0;
      }
    }
    // reset vars
    ack = 0;
    lastSend = millis();
  }
}

/*
 * read digital pins 
 */
void readDig(){
  int i, j, k;
  // read inut pins 
  uint8_t input[4];
  for (i=0; i<4; i++) {
    input[i] = 0x00;
    for (j=0; j<8; j++) {
      k = ((i * 8) + j);
      if (k < maxRead) {
        if (digitalRead(pinIn[k]) == inActive[k])
          input[i] |= (0x01<<j);
      }
    }
  }
  
  // send data
  Serial.write(START);
  Serial.write(READDIG);
  Serial.write(input[0]);
  Serial.write(input[1]);
  Serial.write(input[2]);
  Serial.write(input[3]);
  uint16_t sum = START + READDIG + input[0] + input[1] + input[2] + input[3];
  Serial.write((sum >> 8) & 0xFF);
  Serial.write(sum & 0xFF);
}

/*
 * write digital pins
 */
void writeDig(){
  int i, j, k;
  for (i=0; i<4; i++) {
    for (j=0; j<8; j++) {
      k = ((i * 8) + j);
      if (k < maxWrite) {
        if (pinOutInvert[k]) {
          digitalWrite(pinOut[k], (!(sbuf[2 + i] & (0x01<<j))));
        } else {
          digitalWrite(pinOut[k], (sbuf[2 + i] & (0x01<<j)));
        }
      }
    } 
  }
}

/*
 * read analog pins 
 */
void readAna(int p) {
  long int tmpVal;
  int i;
  if (p >= maxAnaRead)
    return;
  uint8_t output[] = { 0xF0, 0x00, 0xF0, 0x00 };
  uint16_t analog = 0;
  
  // first analog 
  tmpVal = 0;
  for (i=0; i<ANALOGREADS; i++) {
    tmpVal += analogRead(anaIn[p]);
    delay(1);
  }
  analog = (int)(tmpVal / ANALOGREADS);
  /*
   *   analog bytes
   *   xxxxxxyy  yyyyyyyy
   *   \----/\----------/
   *   index     value   
   * 0<=index<32  0<=value<1024
   */
  output[0] = (((p & 0xFF) << 2) | ((analog >> 8) & 0x03));
  output[1] = (analog & 0xFF);

  // second analog 
  if ((p + 1) < maxAnaRead) {
    tmpVal = 0;
    for (i=0; i<ANALOGREADS; i++) {
      tmpVal += analogRead(anaIn[p + 1]);
      delay(1);
    }
    analog = (int)(tmpVal / ANALOGREADS);
    output[2] = ((((p + 1) & 0xFF) << 2) | ((analog >> 8) & 0x03));
    output[3] = (analog & 0xFF);
  }
  
  // send data
  Serial.write(START);
  Serial.write(READANA);
  Serial.write(output[0]);
  Serial.write(output[1]);
  Serial.write(output[2]);
  Serial.write(output[3]);
  uint16_t sum = START + READANA + output[0] + output[1] + output[2] + output[3];
  Serial.write((sum >> 8) & 0xFF);
  Serial.write(sum & 0xFF);
}

/*
 * write analog pins 
 */
void writeAna() {
  int pwm;
  
  // first PWM 
  if (sbuf[2] < maxAnaWrite) {
    pwm = sbuf[3];
    if (pwm < 0) 
      pwm = 0;
    if (pwm > 255) 
      pwm = 255;
    analogWrite(anaOut[sbuf[2]], pwm);
  }
  
  // second PWM 
  if (sbuf[4] < maxAnaWrite) {
    pwm = sbuf[5];
    if (pwm < 0) 
      pwm = 0;
    if (pwm > 255) 
      pwm = 255;
    analogWrite(anaOut[sbuf[4]], pwm);
  }
}
