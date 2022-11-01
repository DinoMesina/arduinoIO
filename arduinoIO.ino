/*
 *   Copyright (C) 2022 Dino del Favero <dino@mesina.net>
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
 *   nulla
 * command 1:
 *   legge/scrive pin digitali
 * command 2:
 *   legge/scrive pin analogici
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

uint8_t sbuf[8];
int pinIn[4][8]  = { { 2, 3, 4, 5, 6, 7,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 } };

int pinOut[4][8] = { {13,12, 8,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 } };

int anaIn[4][8]  = { {A0,A1,A2,A3,A4,A5,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 } };

int anaOut[4][8] = { { 9,10,11,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 },\
                     {-1,-1,-1,-1,-1,-1,-1,-1 } };

unsigned int ind=0;
unsigned int indAnaRead=0;
unsigned int maxAnaRead=0;

void setup() {
  // initialize the serial communication
  Serial.begin(115200);

  // set digital input pins
  for (int i=0; i<4; i++){
    for (int j=0; j<8; j++) {
      if (pinIn[i][j] != -1) {
        pinMode(pinIn[i][j], INPUT);      // set pin to input 
        digitalWrite(pinIn[i][j], HIGH);  // turn on pullup resistors 
      }
    } 
  }

  // set digital output pins 
  for (int i=0; i<4; i++){
    for (int j=0; j<8; j++) {
      if (pinOut[i][j] != -1) {
        pinMode(pinOut[i][j], OUTPUT);   // set pin to output 
        digitalWrite(pinOut[i][j], LOW); // turn off pin 
      }
    } 
  }

  // set analog input pins
  maxAnaRead=0;
  for (int i=0; i<4; i++){
    for (int j=0; j<8; j++) {
      if (anaIn[i][j] != -1) {
        pinMode(anaIn[i][j], INPUT);      // set pin to input 
        maxAnaRead++;
      }
    } 
  }

  // set analog output pins 
  for (int i=0; i<4; i++){
    for (int j=0; j<8; j++) {
      if (anaOut[i][j] != -1) {
        pinMode(anaOut[i][j], OUTPUT);   // set pin to output 
      }
    } 
  }

}

void loop() {
  uint16_t sum = 0;
  // read data from serial
  while(Serial.available() > 0) {
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
      Serial.write(ACK);

      // do command 
      switch (sbuf[1]) {
        case WRITEDIG:
          writeDig(); // write digital pin
          break;
        case WRITEANA:
          writeAna(); // write analog pin
          break;
        default:
        
          break;
      }
            
    } else {
      // error :( send NACK
      Serial.write(NACK);
    }
  }

  // devo aggiornare i dati?
  if (1) {
    // read the digital input pins
    readDig();
    // read the analog input pins 
    if (indAnaRead >= maxAnaRead) {
      indAnaRead = 0;
    }
    readAna(indAnaRead);
    indAnaRead += 2;
    // wait a bit 
    delay(20);
  }
}

/*
 * read digital pins 
 */
void readDig(){
  // read inut pins 
  uint8_t input[4];
  for (int i=0; i<4; i++) {
    input[i] = 0;
    for (int j=8; j>=0; j--) {
      if ((pinIn[i][j] != -1) && (digitalRead(pinIn[i][j]))) {
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
  for (int i=0; i<4; i++) {
    for (int j=0; j<8; j++) {
      if (pinOut[i][j] != -1) {
        digitalWrite(pinOut[i][j], (sbuf[2 + i] & (0x01<<j)));
      }
    }
  }
}

/*
 * read analog pins 
 */
void readAna(int p) {
  if (p > 31)
    return;
  uint8_t output[4];
  
  // first ana 
  int i = 3;
  if (p < 8)
    i = 0;
  else if (p < 16)
    i = 1;
  else if (p < 24)
    i = 2;
  int j = p % 8;
  uint16_t analog;
  analog = analogRead(anaIn[i][j]);
  /*
   * analog bytes
   *   xxxxxxyy  yyyyyyyy
   *   \----/\----------/
   *   index     value   
   * 0<=index<32  0<=value<1024
   */
  output[0] = (((p & 0xFF) << 2) | ((analog >> 8) & 0x03));
  output[1] = (analog & 0xFF);

  // second ana 
  if ((p + 1) < 32) {
    i = 3;
    if ((p + 1) < 8)
      i = 0;
    else if ((p + 1) < 16)
      i = 1;
    else if ((p + 1) < 24)
      i = 2;
    j = (p + 1) % 8;
    analog = analogRead(anaIn[i][j]);
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
  int i = 3;
  if (sbuf[2] < 8)
    i = 0;
  else if (sbuf[2] < 16)
    i = 1;
  else if (sbuf[2] < 24)
    i = 2;
  int j = sbuf[2] % 8;
  if (anaOut[i][j] != -1) {
    int pwm = sbuf[3];
    if (pwm < 0) 
      pwm = 0;
    if (pwm > 255) 
      pwm = 255;
    analogWrite(anaOut[i][j], pwm);
  }
  // second PWM 
  i = 3;
  if (sbuf[4] < 8)
    i = 0;
  else if (sbuf[4] < 16)
    i = 1;
  else if (sbuf[4] < 24)
    i = 2;
  j = sbuf[4] % 8;
  if (anaOut[i][j] != -1) {
    pwm = sbuf[5];
    if (pwm < 0) 
      pwm = 0;
    if (pwm > 255) 
      pwm = 255;
    analogWrite(anaOut[i][j], pwm);
  }
}
