//#include <sim900_Suli.h>
#include <GPRS_Shield_Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Suli.h>

#define pinTX 2
#define pinRX 3
#define pinPower 9
#define pinMotorA 11
#define pinMotorB 12
#define simSpeed 9600
#define optoPin 11
#define STAY 0
#define FORWARD 1
#define BACKWARDS 2

char buffer[512];
char numberMemory[16];
int sendRun = 0;
int runSeconds = 0;
int timeLimit = 300;
int currentDir = STAY;

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

ISR(TIMER1_OVF_vect)         // timer compare interrupt service routine
{
  TCNT1 = 65536;            // preload timer
  if (sendRun == 0) {
    digitalWrite(pinMotorA, LOW);
    digitalWrite(pinMotorB, LOW);
    runSeconds = 0;
  } else {
    int prob = !currentDir ? 40 : 70;
    if (random(100) > prob) {
      // change direction
      int prevDir = currentDir;
      while (prevDir == currentDir) {
        currentDir = random(BACKWARDS + 1);
      }
    }
    
    runSeconds += 1;

    if (currentDir == STAY) {
      digitalWrite(pinMotorA, LOW);
      digitalWrite(pinMotorB, LOW);
    } else if (currentDir == FORWARD) {
      digitalWrite(pinMotorA, HIGH);
      digitalWrite(pinMotorB, LOW);
    } else {
      digitalWrite(pinMotorA, LOW);
      digitalWrite(pinMotorB, HIGH);
    }

    if (runSeconds >= timeLimit) {
      Serial.println("Run rutine finished");
      sendRun = 0;
    }
  }
}

void setup() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65536;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();
  
  pinMode(pinPower, OUTPUT);
  pinMode(optoPin, OUTPUT);
  pinMode(pinMotorA, OUTPUT);
  pinMode(pinMotorB, OUTPUT);
  
  Serial.begin(9600);
  
  Serial.println("Booting up...");
  Serial.println("Power up...");
  
  while(!sim900.init()) {
      Serial.println("Error!");
      sim900.powerUpDown(pinPower);
  }
  
  delay(2000);
  Serial.println("Boot done!");
  sendCmd("AT+COLP=1\r\n");
  sendCmd("AT+COPS\r\n");
  sendCmd("AT+CSQ\r\n");
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

    if (strcmp(numberMemory, "+420***") == 0) {
      if (!sendRun) {
        sendRun = 1;
        Serial.println("Run rutine started");
      }
    }
  }
}
