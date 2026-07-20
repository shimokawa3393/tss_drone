// ================================================================
// RC → モーター 連動テスト + IMU（コンパスは磁気干渉のため無効化）
// setMotor()のチャンネルバグは修正済み(pin直指定方式)
// 【必ずプロペラを外した状態でテストすること】
// ================================================================

#include <Wire.h>

#define SDA_PIN 32
#define SCL_PIN 33

#define MPU_ADDR 0x68

#define ESC_M1 5
#define ESC_M2 12
#define ESC_M3 26
#define ESC_M4 4

#define RC_ROLL     34
#define RC_PITCH    39
#define RC_THROTTLE 35
#define RC_YAW      36
#define RC_ARM      2

#define PULSE_TIMEOUT 25000

const int THROTTLE_MIN   = 1000;
const int THROTTLE_IDLE  = 1080;
const int THROTTLE_MAX   = 1900;
const int ARM_THRESHOLD  = 1500;
const int ARM_SAFE_THROTTLE = 1100;

const float MIX_GAIN_RP  = 0.4;
const float MIX_GAIN_YAW = 0.3;

enum FlightState { DISARMED, IDLE, FLYING };
FlightState currentState = DISARMED;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(RC_ROLL, INPUT);
  pinMode(RC_PITCH, INPUT);
  pinMode(RC_THROTTLE, INPUT);
  pinMode(RC_YAW, INPUT);
  pinMode(RC_ARM, INPUT);

  ledcAttach(ESC_M1, 50, 16);
  ledcAttach(ESC_M2, 50, 16);
  ledcAttach(ESC_M3, 50, 16);
  ledcAttach(ESC_M4, 50, 16);

  setMotor(ESC_M1, THROTTLE_MIN);
  setMotor(ESC_M2, THROTTLE_MIN);
  setMotor(ESC_M3, THROTTLE_MIN);
  setMotor(ESC_M4, THROTTLE_MIN);
  delay(3000);

  initMPU6050();
  calibrateGyro();
  // コンパスは磁気干渉のため無効化中。将来GPS+コンパス一体モジュールを
  // マストで導入する際に復活させる予定

  Serial.println("=== RC-モーター連動 + IMUテスト開始 ===");
  Serial.println("【警告】プロペラが外れていることを確認してください");
}

void setMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  int duty = map(us, 1000, 2000, 3277, 6553);
  ledcWrite(pin, duty);
}

void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
}

void calibrateGyro() {
  Serial.println("ジャイロキャリブレーション中...機体を動かさないでください");
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    readMPU6050(ax, ay, az, gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;

  Serial.printf("キャリブレーション完了 offsetX:%.2f offsetY:%.2f offsetZ:%.2f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

void loop() {
  int roll     = pulseIn(RC_ROLL,     HIGH, PULSE_TIMEOUT);
  int pitch    = pulseIn(RC_PITCH,    HIGH, PULSE_TIMEOUT);
  int throttle = pulseIn(RC_THROTTLE, HIGH, PULSE_TIMEOUT);
  int yaw      = pulseIn(RC_YAW,      HIGH, PULSE_TIMEOUT);
  int arm      = pulseIn(RC_ARM,      HIGH, PULSE_TIMEOUT);

  if (roll == 0)     roll = 1500;
  if (pitch == 0)    pitch = 1500;
  if (throttle == 0) throttle = THROTTLE_MIN;
  if (yaw == 0)       yaw = 1500;
  if (arm == 0)       arm = 1000;

  bool armSwitchOn = (arm > ARM_THRESHOLD);

  float ax, ay, az, gx, gy, gz;
  readMPU6050(ax, ay, az, gx, gy, gz);
  gx -= gyroOffsetX;
  gy -= gyroOffsetY;
  gz -= gyroOffsetZ;

  switch (currentState) {
    case DISARMED:
      if (armSwitchOn && throttle < ARM_SAFE_THROTTLE) {
        currentState = IDLE;
        Serial.println(">>> ARMED (IDLE状態へ移行)");
      }
      break;
    case IDLE:
      if (!armSwitchOn) {
        currentState = DISARMED;
        Serial.println(">>> DISARMED");
      } else if (throttle > ARM_SAFE_THROTTLE + 50) {
        currentState = FLYING;
        Serial.println(">>> FLYING状態へ移行");
      }
      break;
    case FLYING:
      if (!armSwitchOn) {
        currentState = DISARMED;
        Serial.println(">>> 緊急DISARMED");
      } else if (throttle < ARM_SAFE_THROTTLE) {
        currentState = IDLE;
      }
      break;
  }

  int m1, m2, m3, m4;

  if (currentState == DISARMED) {
    m1 = m2 = m3 = m4 = THROTTLE_MIN;
  } else if (currentState == IDLE) {
    m1 = m2 = m3 = m4 = THROTTLE_MIN;
  } else {
    float rollInput  = (roll  - 1500) * MIX_GAIN_RP;
    float pitchInput = (pitch - 1500) * MIX_GAIN_RP;
    float yawInput   = (yaw   - 1500) * MIX_GAIN_YAW;
    int throttleBase = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);

    m1 = throttleBase - rollInput - pitchInput + yawInput;
    m2 = throttleBase + rollInput + pitchInput + yawInput;
    m3 = throttleBase + rollInput - pitchInput - yawInput;
    m4 = throttleBase - rollInput + pitchInput - yawInput;

    m1 = constrain(m1, THROTTLE_IDLE, THROTTLE_MAX);
    m2 = constrain(m2, THROTTLE_IDLE, THROTTLE_MAX);
    m3 = constrain(m3, THROTTLE_IDLE, THROTTLE_MAX);
    m4 = constrain(m4, THROTTLE_IDLE, THROTTLE_MAX);
  }

  setMotor(ESC_M1, m1);
  setMotor(ESC_M2, m2);
  setMotor(ESC_M3, m3);
  setMotor(ESC_M4, m4);

  const char* stateStr = (currentState == DISARMED) ? "DISARMED" :
                          (currentState == IDLE)     ? "IDLE"     : "FLYING";
  Serial.printf(
    "[%s] R:%d P:%d T:%d Y:%d Arm:%d | M1:%d M2:%d M3:%d M4:%d | "
    "gx:%.1f gy:%.1f gz:%.1f\n",
    stateStr, roll, pitch, throttle, yaw, arm, m1, m2, m3, m4,
    gx, gy, gz);

  delay(20);
}