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
int Hz = 0;
int hz = 0;
volatile float posi = 0;
volatile float posi1 = 0;

uint32_t times = millis();
uint32_t times_hz = millis();
// Setup rosserial and lister
ros::NodeHandle nh;
float value[3];
void servo_cb(const std_msgs::Float32MultiArray& cmd_msg) {
  Motor1(cmd_msg.data[0]);
  Motor2(cmd_msg.data[1]);
}

// Setup rosserial
std_msgs::Float32MultiArray str_msg;
ros::Publisher chatter("arduino_chatter", &str_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("arduino_listener", servo_cb);

void setup() {
  Serial.begin(57600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IN_R, OUTPUT);
  pinMode(IN_F, OUTPUT);
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(IN_R1, OUTPUT);
  pinMode(IN_F1, OUTPUT);

  attachInterrupt(0, readEncoder, RISING);
  attachInterrupt(1, readEncoder1, RISING);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop() {
  // Serial.println(posi);
  Serial.println(posi1);
  value[0] = posi;
  value[1] = posi1;
  value[2] = millis();
  str_msg.data = value;
  str_msg.data_length = 3;
  if (millis() - times >= 20.0) {
    chatter.publish(&str_msg);
    times = millis();
    // currentTime = micros();
    Hz++;
    // Serial.println(hz);
  }
  if (millis() - times_hz >= 1000.0) {
    hz = Hz;
    Hz = 0;
    times_hz = millis();
  }
  // Serial.println(posi);
    //       digitalWrite(IN_F, HIGH);
    // digitalWrite(IN_R, LOW);
    // analogWrite(pwm_1, 90);
  nh.spinOnce();
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
  if (pwm > 0) {
    digitalWrite(IN_F1, HIGH);
    digitalWrite(IN_R1, LOW);
    analogWrite(pwm_2, pwm);
  } else if (pwm < 0) {
    digitalWrite(IN_R1, HIGH);
    digitalWrite(IN_F1, LOW);
    analogWrite(pwm_2, pwm * -1);
  } else {
    analogWrite(IN_F1, 0);
    analogWrite(IN_R1, 0);
  }
}
