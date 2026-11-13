#include <Arduino.h>

// ============================== 接线 ==============================
// L298N 的 ENA、ENB 跳线帽保持短接。速度用 IN 脚上的 PWM 实现：
// 正转时给 IN1（或 IN3）输出 PWM、另一脚拉低；反转时反过来。
// PB6~PB9 正好是 TIM4 的通道 1~4，四个脚可以各自给不同占空比。
//
//   PB6  IN1   电机 A，这里当作左轮
//   PB7  IN2
//   PB8  IN3   电机 B，这里当作右轮
//   PB9  IN4
//
// 循迹模块（两路 TCRT5000）：RO -> PB12，LO -> PB13，VCC 接 5V，GND 与板子共地。
// PB12/PB13 是 F103 上的 5V 容忍脚，模块输出 5V 也能直接接。

static const uint32_t LEFT_IN1 = PB6;   // TIM4_CH1
static const uint32_t LEFT_IN2 = PB7;   // TIM4_CH2
static const uint32_t RIGHT_IN1 = PB8;  // TIM4_CH3
static const uint32_t RIGHT_IN2 = PB9;  // TIM4_CH4

static const uint32_t TRACK_RO = PB12;  // 右探头
static const uint32_t TRACK_LO = PB13;  // 左探头

static const uint32_t LED_PIN = PC13;   // 板载 LED，低电平点亮
static const bool LED_ACTIVE_LOW = true;

// ========================== 需要现场调的参数 ==========================

// 1) 探头极性：探头压在黑线上时，数字输出是高电平还是低电平。
//    不同厂家的模块不一样。先把 TRACK_DEBUG 改成 1 烧进去，打开串口，
//    把探头放到黑线上看打印里 black 那一栏是不是 1，不是就改这里。
static const bool TRACK_BLACK_IS_HIGH = false;

// 2) 速度，0~255。先架起来试，再落地；落地后从 120 左右往上加。
static const int BASE_SPEED = 150;  // 直行
static const int TURN_OUTER = 210;  // 转弯时外侧轮
static const int TURN_INNER = 40;   // 转弯时内侧轮，越小转得越急

// 3) 加减速斜率：每个控制周期最多变化多少。太小则反应慢，太大则
//    起步顿挫、轮胎打滑。
static const int RAMP_STEP = 25;

// 4) 控制周期与去抖次数。去抖 2 表示要连续两次读到同样的结果才认。
static const uint32_t PERIOD_MS = 10;
static const uint8_t DEBOUNCE_SAMPLES = 2;

// 5) 上电后等多久才开动，这段时间可以把车摆正到起跑线上。
static const uint32_t ARM_DELAY_MS = 1000;

// 6) 串口调试：1 = 打开（PA9/PA10，115200），只打印探头状态、电机不转，
//    用来确认极性和左右；正式跑车时改回 0。
#define TRACK_DEBUG 0

// ========================== 控制逻辑 ==========================
// 两个探头骑跨在黑线两侧（都压在白底上）时直行。哪个探头压到黑线，
// 说明车已经偏到线的另一边，就往回修：
//
//   LO 黑、RO 白  ->  线在左边  ->  左轮慢、右轮快（向左修）
//   LO 白、RO 黑  ->  线在右边  ->  左轮快、右轮慢（向右修）
//   两个都黑（十字路口、宽线）或两个都白（正常骑跨）-> 直行
//
// 如果实测发现车越修越偏，把 TRACK_LO / TRACK_RO 两个常量对调即可。
// 如果整个车前进变成后退，对调某一侧的两个 IN 常量。

enum TrackState : uint8_t {
  TRACK_STRAIGHT,
  TRACK_TURN_LEFT,
  TRACK_TURN_RIGHT,
};

static int clampSpeed(int speed) {
  if (speed > 255) {
    return 255;
  }
  if (speed < -255) {
    return -255;
  }
  return speed;
}

// 让 current 朝 target 走一步，最多 step，用来做加减速。
static int approach(int current, int target, int step) {
  if (step <= 0) {
    return target;
  }
  if (current + step < target) {
    return current + step;
  }
  if (current - step > target) {
    return current - step;
  }
  return target;
}

// speed > 0 正转，< 0 反转，0 停。数值就是占空比。
//
// 两侧都用 analogWrite：引脚一旦被 analogWrite 交给 TIM4，就变成复用输出，
// 这时 digitalWrite 只写 ODR、改不动引脚电平，所以拉低也要用 analogWrite(x, 0)。
static void setMotor(uint32_t inFwd, uint32_t inRev, int speed) {
  speed = clampSpeed(speed);
  if (speed > 0) {
    analogWrite(inFwd, speed);
    analogWrite(inRev, 0);
  } else if (speed < 0) {
    analogWrite(inFwd, 0);
    analogWrite(inRev, -speed);
  } else {
    // 两脚都低：L298N 两个输出都拉低，电机被短接，属于刹车。
    analogWrite(inFwd, 0);
    analogWrite(inRev, 0);
  }
}

// 左、右轮速度各自 -255~255。两轮不同速就会转弯。
static void drive(int left, int right) {
  setMotor(LEFT_IN1, LEFT_IN2, left);
  setMotor(RIGHT_IN1, RIGHT_IN2, right);
}

static bool seesBlack(uint32_t pin) {
  const bool high = digitalRead(pin) == HIGH;
  return TRACK_BLACK_IS_HIGH ? high : !high;
}

// 看一次，不做去抖。
static TrackState sampleTrack() {
  const bool left = seesBlack(TRACK_LO);
  const bool right = seesBlack(TRACK_RO);
  if (left == right) {
    return TRACK_STRAIGHT;
  }
  return left ? TRACK_TURN_LEFT : TRACK_TURN_RIGHT;
}

static TrackState trackState = TRACK_STRAIGHT;
static uint8_t trackSamples = 0;

// 去抖后的状态：新结果要连续出现 DEBOUNCE_SAMPLES 次才生效。
static TrackState readTrack() {
  const TrackState raw = sampleTrack();
  if (raw == trackState) {
    trackSamples = 0;
    return trackState;
  }
  if (++trackSamples >= DEBOUNCE_SAMPLES) {
    trackState = raw;
    trackSamples = 0;
  }
  return trackState;
}

static void trackTargets(TrackState state, int &left, int &right) {
  switch (state) {
    case TRACK_TURN_LEFT:
      left = TURN_INNER;
      right = TURN_OUTER;
      break;
    case TRACK_TURN_RIGHT:
      left = TURN_OUTER;
      right = TURN_INNER;
      break;
    default:
      left = BASE_SPEED;
      right = BASE_SPEED;
      break;
  }
}

static void setLed(bool on) {
  digitalWrite(LED_PIN, on != LED_ACTIVE_LOW ? HIGH : LOW);
}

// 正常运行时的指示灯：每 500ms 短闪一下，表示程序还活着。
static void updateLed(uint32_t now) {
  setLed((now % 500) < 60);
}

#if TRACK_DEBUG
static const char *stateName(TrackState state) {
  switch (state) {
    case TRACK_TURN_LEFT:
      return "left";
    case TRACK_TURN_RIGHT:
      return "right";
    default:
      return "straight";
  }
}

static void printTrack(uint32_t now) {
  const bool leftBlack = seesBlack(TRACK_LO);
  const bool rightBlack = seesBlack(TRACK_RO);

  Serial.print("raw LO=");
  Serial.print(digitalRead(TRACK_LO) == HIGH ? 1 : 0);
  Serial.print(" RO=");
  Serial.print(digitalRead(TRACK_RO) == HIGH ? 1 : 0);
  Serial.print("  black LO=");
  Serial.print(leftBlack ? 1 : 0);
  Serial.print(" RO=");
  Serial.print(rightBlack ? 1 : 0);
  Serial.print("  -> ");
  Serial.println(stateName(readTrack()));
  (void)now;
}
#endif

static int currentLeft = 0;
static int currentRight = 0;
static uint32_t lastTick = 0;

void setup() {
  // 先把四路 IN 定死为低，避免上电瞬间电机乱转。
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  drive(0, 0);

  // 探头是推挽输出，内部上拉只是为了断线时也有一个确定的电平。
  pinMode(TRACK_LO, INPUT_PULLUP);
  pinMode(TRACK_RO, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  // 想把 PWM 调离人耳敏感的 1kHz 时取消注释。L298N 撑得住 20kHz，只是会热一点。
  // analogWriteFrequency(20000);

#if TRACK_DEBUG
  Serial.begin(115200);
  setLed(true);  // 调试模式灯常亮，电机不动
#else
  // 灯亮着等一小会儿，用来把车摆到线上。
  setLed(true);
  const uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ARM_DELAY_MS) {
  }
  setLed(false);
  lastTick = millis();
#endif
}

void loop() {
  const uint32_t now = millis();

#if TRACK_DEBUG
  static uint32_t lastPrint = 0;
  if ((uint32_t)(now - lastPrint) >= 100) {
    lastPrint = now;
    printTrack(now);
  }
#else
  updateLed(now);

  if ((uint32_t)(now - lastTick) < PERIOD_MS) {
    return;
  }
  lastTick = now;

  const TrackState state = readTrack();

  int targetLeft = 0;
  int targetRight = 0;
  trackTargets(state, targetLeft, targetRight);

  currentLeft = approach(currentLeft, targetLeft, RAMP_STEP);
  currentRight = approach(currentRight, targetRight, RAMP_STEP);
  drive(currentLeft, currentRight);
#endif
}
