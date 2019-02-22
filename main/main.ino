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

#define NUM_OF_MOVES 6
#define STAY 0
#define FORWARD 1
#define BACKWARD 2
#define ATTENTION 3
#define BAIT_FORWARD 4
#define BAIT_BACKWARD 5

char buffer[512];
char numberMemory[16];
int sendRun = 0;
float runSeconds = 0;
int timeLimit = 900;
int currentDir = STAY;
int currentSpeed = 0;
int notChangeForSteps = 0;

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
  TCNT1 = 0;            // preload timer
  if (sendRun == 0) {
    digitalWrite(pinMotorA, LOW);
    digitalWrite(pinMotorB, LOW);
    runSeconds = 0;
  } else {
    int prob = 0;
    switch (currentDir) {
      case FORWARD:
      case BACKWARD:
      case ATTENTION:
        prob = 60;
      case BAIT_FORWARD:
      case BAIT_BACKWARD:
        prob = 50;
      default:
        prob = 30;
    }
    
    if (!notChangeForSteps && random(101) > prob) {
      // change direction
      int prevDir = currentDir;
      while (prevDir == currentDir) {
        currentDir = random(NUM_OF_MOVES);
        currentSpeed = random(86) + 170;

        if (currentDir > BACKWARD) {
          notChangeForSteps = 12;
        } else {
          notChangeForSteps = 0;
        }
      }
    }

    Serial.print("STEP => Dir: ");
    Serial.print(currentDir);
    Serial.print("; seconds: ");
    Serial.print(runSeconds);
    Serial.print("; Speed: ");
    Serial.println(currentSpeed);
    if (currentDir == STAY) {
      digitalWrite(pinMotorA, LOW);
      digitalWrite(pinMotorB, LOW);
    } else if (currentDir == FORWARD) {
      analogWrite(pinMotorA, currentSpeed);
      digitalWrite(pinMotorB, LOW);
    } else if (currentDir == BACKWARD) {
      analogWrite(pinMotorA, 256 - currentSpeed);
      digitalWrite(pinMotorB, HIGH);
    } else if (currentDir == ATTENTION) {
      if (notChangeForSteps % 2 == 0) {
        digitalWrite(pinMotorA, HIGH);
        digitalWrite(pinMotorB, LOW);
      } else {
        digitalWrite(pinMotorA, LOW);
        digitalWrite(pinMotorB, HIGH);
      }
      TCNT1 = 56250;
      runSeconds -= 0.9;
    } else if (currentDir == BAIT_FORWARD) {
      if (notChangeForSteps % 2 == 0) {
        digitalWrite(pinMotorA, HIGH);
        digitalWrite(pinMotorB, LOW);
      } else {
        digitalWrite(pinMotorA, LOW);
        digitalWrite(pinMotorB, LOW);
      }
      TCNT1 = 56250;
      runSeconds -= 0.9;
    } else {
      if (notChangeForSteps % 2 == 0) {
        digitalWrite(pinMotorA, LOW);
        digitalWrite(pinMotorB, HIGH);
      } else {
        digitalWrite(pinMotorA, LOW);
        digitalWrite(pinMotorB, LOW);
      }
      TCNT1 = 56250;
      runSeconds -= 0.9;
    }

    runSeconds += 1;
    notChangeForSteps = max(0, notChangeForSteps - 1);
    
    if (runSeconds >= timeLimit) {
      Serial.println("Run rutine finished");
      sendRun = 0;
    }
  }
}

void setup() {
  randomSeed(analogRead(0));
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 0;            // preload timer 65536-16MHz/256/2Hz
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
  delay(1000);
  sendCmd("AT+CSQ\r\n");
  delay(1000);
  //sendRun = 1;
}

void loop() {  
  if (Serial.available()) {
    sendCmd(Serial.readString());
  }

  if (sim900.isCallActive(numberMemory) == 0) {
    delay(1000);
    sim900.hangup();
    Serial.println(numberMemory);

    if (strcmp(numberMemory, "+*") == 0
      || strcmp(numberMemory, "+*") == 0) {
      // rebooting ending timer
      runSeconds = 0;
      if (!sendRun) {
        sendRun = 1;
        Serial.println("Run rutine started");
      }
    }
  }
}
