#include <Arduino.h>

// L298N 的 ENA、ENB 跳线帽保持短接。速度用 IN 脚上的 PWM 实现：
// 正转时给 IN1（或 IN3）输出 PWM、另一脚拉低；反转时反过来。
// PB6~PB9 分别是 TIM4 的通道 1~4，左右轮可以给不同占空比。
//
//   PB6  IN1   电机 A，这里当作左轮
//   PB7  IN2
//   PB8  IN3   电机 B，这里当作右轮
//   PB9  IN4
//
// 左右和实物相反时，对调 drive() 里的两个参数。
// 某一侧正反转反了，对调该侧两个 IN 引脚常量。
// 循迹：RO -> PB12，LO -> PB13。这个程序只前进，还不根据传感器转弯。

static const uint32_t LEFT_IN1 = PB6;
static const uint32_t LEFT_IN2 = PB7;
static const uint32_t RIGHT_IN1 = PB8;
static const uint32_t RIGHT_IN2 = PB9;

static const uint32_t TRACK_RO = PB12;
static const uint32_t TRACK_LO = PB13;

static const uint32_t LED_PIN = PC13;

// 直行速度，0~255。先架起来试，再加大。
static const int SPEED = 160;

static int clampSpeed(int speed) {
  if (speed > 255) {
    return 255;
  }
  if (speed < -255) {
    return -255;
  }
  return speed;
}

// speed > 0 正转，< 0 反转，0 停止。数值是占空比。
static void setMotor(uint32_t inFwd, uint32_t inRev, int speed) {
  speed = clampSpeed(speed);
  analogWrite(inFwd, 0);
  analogWrite(inRev, 0);

  if (speed > 0) {
    digitalWrite(inRev, LOW);
    analogWrite(inFwd, speed);
  } else if (speed < 0) {
    digitalWrite(inFwd, LOW);
    analogWrite(inRev, -speed);
  } else {
    digitalWrite(inFwd, LOW);
    digitalWrite(inRev, LOW);
  }
}

// 左、右轮速度各自 -255~255。两轮不同速就会转弯。
static void drive(int left, int right) {
  setMotor(LEFT_IN1, LEFT_IN2, left);
  setMotor(RIGHT_IN1, RIGHT_IN2, right);
}

void setup() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(TRACK_RO, INPUT);
  pinMode(TRACK_LO, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  drive(SPEED, SPEED);       // 直行
  // drive(SPEED, SPEED / 3); // 右转：右轮更慢
  // drive(SPEED, -SPEED);    // 原地右转：右轮反转
}

void loop() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(500);
}
