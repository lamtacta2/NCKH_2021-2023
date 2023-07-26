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

uint32_t times = millis();

int IN_R = 7;
int IN_F = 6;
int dir = 0;

int IN_R1 = 8;
int IN_F1 = 9;

int pwm_1 = 10;
int pwm_2 = 11;


// Setup counter encoder
volatile float posi = 0;
volatile float posi1 = 0;

// Setup PID
float pwm = 0;
float prevT = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
float target = 0.0;
float target1 = 0.0;
float posion = 0.0;
float posion1 = 0.0;

// Setup rosserial and lister
ros::NodeHandle nh;
float value[2];
void servo_cb(const std_msgs::Float32MultiArray& cmd_msg) {
  target = cmd_msg.data[0];
  target1 = cmd_msg.data[1];
}

void set_ori(const std_msgs::Float32MultiArray& cmd_msg) {
  target = 0.0;
  target1 = 0.0;
  posi = 0.0;
  posi1 = 0.0;
}
// Setup rosserial
std_msgs::Float32MultiArray str_msg;
ros::Publisher chatter("arduino_chatter", &str_msg);
ros::Subscriber<std_msgs::Float32MultiArray> sub("arduino_listener", servo_cb);
ros::Subscriber<std_msgs::Float32MultiArray> sub1("arduino_ori", set_ori);

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

  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.advertise(chatter);
}

void loop() {
  value[0] = posi;
  value[1] = posi1;
  str_msg.data = value;
  str_msg.data_length = 2;
  // chatter.publish( &str_msg );
  if (millis() - times >= 100) {
    chatter.publish(&str_msg);
    times = millis();
  }
  nh.spinOnce();
  // Read the position
  noInterrupts();  // disable interrupts temporarily while reading
  posion = posi;
  posion1 = posi1;
  interrupts();  // turn interrupts back on
  PID(target, target1, posion, posion1);
  // Serial.println(posion);
  // Serial.println(posion1);
  nh.spinOnce();
}

// Setup readEncoder
void readEncoder() {
  if (digitalRead(ENCB)) {
    posi++;
  } else {
    posi--;
  }
}

void readEncoder1() {
  if (digitalRead(ENCB1)) {
    posi1++;
  } else {
    posi1--;
  }
}

// Setup Control Motor
void MOTOR(int dir, int pwm) {
  if (dir == 1) {
    digitalWrite(IN_F, HIGH);
    digitalWrite(IN_R, LOW);
    analogWrite(pwm_1, pwm);
  } else if (dir == -1) {
    digitalWrite(IN_R, HIGH);
    digitalWrite(IN_F, LOW);
    analogWrite(pwm_1, pwm);
  } else {
    analogWrite(IN_F, 0);
    analogWrite(IN_R, 0);
  }
}

void MOTOR1(int dir, int pwm) {
  if (dir == 1) {
    digitalWrite(IN_F1, HIGH);
    digitalWrite(IN_R1, LOW);
    analogWrite(pwm_2, pwm);
  } else if (dir == -1) {
    digitalWrite(IN_R1, HIGH);
    digitalWrite(IN_F1, LOW);
    analogWrite(pwm_2, pwm);
  } else {
    analogWrite(IN_F1, 0);
    analogWrite(IN_R1, 0);
  }
}

// Setup PID
void PID(float Ang1, float Ang2, float pos1, float pos2) {
  float kp = 0.5;
  float kd = 0.2;
  float ki = 0.1;
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  float e1 = pos1 - Ang1;
  float e2 = pos2 - Ang2;

  float dedt1 = (e1 - eprev1) / (deltaT);
  float dedt2 = (e2 - eprev2) / (deltaT);

  eintegral1 = eintegral1 + e1 * deltaT;
  eintegral2 = eintegral2 + e2 * deltaT;

  float u1 = kp * e1 + kd * dedt1 + ki * eintegral1;
  float u2 = kp * e2 + kd * dedt2 + ki * eintegral2;

  if (u1 < 0) {
    dir = -1;
    pwm = round(u1 * -1 / 12 * 255);
  } else if (u1 > 0) {
    dir = 1;
    pwm = round(u1 / 12 * 255);
  } else {
    dir = 0;
  }

  if (pwm > 255) {
    pwm = 255;
  }

  MOTOR(dir, pwm);

  if (u2 < 0) {
    dir = -1;
    pwm = round(u2 * -1 / 12 * 255);
  } else if (u2 > 0) {
    dir = 1;
    pwm = round(u2 / 12 * 255);
  } else {
    dir = 0;
  }
  
  if (pwm > 255) {
    pwm = 255;
  }

  MOTOR1(dir, pwm);

  eprev1 = e1;
  eprev2 = e2;
}
