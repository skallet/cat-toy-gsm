//#include <sim900_Suli.h>
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Suli.h>

#define pinTX 2
#define pinRX 3
#define pinPower 9
#define simSpeed 9600
#define optoPin 11

char buffer[512];
char numberMemory[16];
int sendRun = 0;

GPRS sim900(pinTX, pinRX, simSpeed);

void sendCmd(String s) {
  Serial.print("Sending: ");
  Serial.println(s);
  char cmds [64];
  s.toCharArray(cmds, 64); 

  if (s == "Test\r\n") {
    Serial.println("end");
    sim900_send_cmd("Test sms");
    suli_delay_ms(500);
    sim900_send_End_Mark();
  } else {
    sim900_send_cmd(cmds);
  }

  sim900_clean_buffer(buffer,512);
  sim900_read_buffer(buffer,512);
  Serial.println(buffer);
}

void setup() {
  pinMode(optoPin, OUTPUT);
  Serial.begin(9600);
  
  Serial.println("Booting up...");
  Serial.println("Power up...");
  sim900.powerUpDown(pinPower);
  
  while(!sim900.init()) {
      Serial.println("Error!");
      delay(1000);
  }
  
  delay(2000);
  Serial.println("Boot done!");
  sendCmd("AT+COLP=1\r\n");
  delay(1000);
}

void loop() {  
  if (Serial.available()) {
    sendCmd(Serial.readString());
  }

  if (sim900.isCallActive(numberMemory) == 0) {
    delay(1000);
    sim900.hangup();
    Serial.println(numberMemory);

    if (strcmp(numberMemory, "+420*********") == 0) {
      if (!sendRun) {
        sendRun = 1;
        Serial.println("Run rutine started");
      }
    }
  }

  if (sendRun) {
    Serial.println("Power up toy..");
    digitalWrite(optoPin, HIGH);
    delay(100);
    digitalWrite(optoPin, LOW);
    delay(10000);
    sendRun = 0;
    sendCmd("AT+COLP=1\r\n");
  }
}
