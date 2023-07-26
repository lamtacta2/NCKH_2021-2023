#include <PID_v1.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include "std_msgs/Float32MultiArray.h"

// setup PIN
const byte ENCA = 2;
const byte ENCB = 4;
const byte ENCA1 = 3;
const byte ENCB1 = 5;

int IN_R = 7;
int IN_F = 6;
int IN_R1 = 8;
int IN_F1 = 9;
int pwm_1 = 10;
int pwm_2 = 11;
volatile float posi = 0;
volatile float posi1 = 0;
unsigned long currentTime = micros();

//Define Variables we'll be connecting to
float Setpoint1, Input1, Output1;
float Setpoint2, Input2, Output2;
//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1, 3, 5, 1, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, 10, 5, 1, DIRECT);

void setup() {
  Serial.begin(57600);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(IN_R, OUTPUT);
  pinMode(IN_F, OUTPUT);

  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(IN_R1, OUTPUT);
  pinMode(IN_F1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
}


void loop() {
  Setpoint1 = 200;
  Setpoint2 = 100;
  Input1 = posi;
  Input2 = posi1;
  myPID1.Compute();
  myPID2.Compute();
  Motor1(Output1);
  Motor2(Output2);
  Serial.print("p1: ");
  Serial.println(posi);
  Serial.print("p2: ");
  Serial.println(posi1);
  Serial.println();
}

// Setup readEncoder
void readEncoder() {
  if (digitalRead(ENCB)) {
    posi--;
  } else {
    posi++;
  }
}

void readEncoder1() {
  if (digitalRead(ENCB1)) {
    posi1--;
  } else {
    posi1++;
  }
}

// Setup Control Motor
void Motor1(float pwm) {
  if (pwm > 0) {
    digitalWrite(IN_F, HIGH);
    digitalWrite(IN_R, LOW);
    analogWrite(pwm_1, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN_R, HIGH);
    digitalWrite(IN_F, LOW);
    analogWrite(pwm_1, pwm * -1);
  } else {
    analogWrite(IN_F, 0);
    analogWrite(IN_R, 0);
  }
}

void Motor2(float pwm) {
  if (pwm < 0) {
    digitalWrite(IN_F1, HIGH);
    digitalWrite(IN_R1, LOW);
    analogWrite(pwm_2, pwm*-1);
  } else if (pwm > 0) {
    digitalWrite(IN_R1, HIGH);
    digitalWrite(IN_F1, LOW);
    analogWrite(pwm_2, pwm);
  } else {
    analogWrite(IN_F1, 0);
    analogWrite(IN_R1, 0);
  }
}
