// ================================================================
// Phase 2-3 FC本体（統合版・第1弾）
// RC入力 → 状態遷移(DISARMED/IDLE/FLYING) → ミキシング+Rateダンピング → モーター出力
//
// 統合済み：
//   ・setMotor()のチャンネルバグ修正（pin直指定方式）
//   ・RC5ch読み取り（デフォルトAETR：CH1=Roll,CH2=Pitch,CH3=Throttle,CH4=Yaw,CH5=Arm）
//   ・MPU-6050ジャイロ、起動時キャリブレーション
//   ・Roll/Pitchダンピング（実機テストで符号検証済み）
//   ・安全設計：MOTOR_STOP方式（Arm済みでもスロットル最低なら完全停止）
//
// 未統合（次のステップ）：
//   ・コンパス（磁気干渉のため保留、GPS導入時にマスト化して復活予定）
//   ・LoRaテレメトリ送信（Chapter2の実配線でのCS/RST/DIO0ピンが未確認のため保留。
//     GPIO26は現在ESC_M3で使用中なので、LoRaのピン割り当ては要再確認）
//   ・BMP280/AHT20（テレメトリ送信の枠組みと一緒に統合予定）
//
// 【必ずプロペラを外した状態で最初のテストを行うこと】
// ================================================================

#include <Wire.h>

// --- I2Cバス ---
#define SDA_PIN 32
#define SCL_PIN 33

// --- センサーアドレス ---
#define MPU_ADDR 0x68

// --- ESC(モーター出力)GPIO割り当て ---
// フレーム配置：①右前CCW=M1  ③左前CW=M3
//              ②左後CCW=M2  ④右後CW=M4
#define ESC_M1 5    // 右前 CCW
#define ESC_M2 12   // 左後 CCW
#define ESC_M3 26   // 左前 CW
#define ESC_M4 4    // 右後 CW

// --- RC入力 GPIO割り当て（基板シルク順 35,34,VN,VP）---
#define RC_ROLL     34  // CH1
#define RC_PITCH    39  // CH2 (VN)
#define RC_THROTTLE 35  // CH3
#define RC_YAW      36  // CH4 (VP)
#define RC_ARM      2   // CH5

#define PULSE_TIMEOUT 25000  // pulseInタイムアウト(us)

// --- 安全パラメータ ---
const int THROTTLE_MIN      = 1000;  // 完全停止
const int THROTTLE_IDLE     = 1080;  // FLYING中の出力下限（完全停止は避ける）
const int THROTTLE_MAX      = 1900;  // 出力上限（2000ではなく余裕を持たせる）
const int ARM_THRESHOLD     = 1500;  // これを超えたらArmed判定
const int ARM_SAFE_THROTTLE = 1100;  // Armを許可する条件：スロットルがこれ未満

// --- ミキシング・ダンピングのゲイン ---
const float MIX_GAIN_RP  = 0.4;   // roll/pitch用ゲイン
const float MIX_GAIN_YAW = 0.3;   // yaw用ゲイン
const float DAMPING_GAIN = 1.5;   // Rateダンピングのゲイン（実機テストで確認済みの値）

// --- 機体の状態 ---
enum FlightState { DISARMED, IDLE, FLYING };
FlightState currentState = DISARMED;

// ジャイロのオフセット(バイアス)値。起動時に静止状態で計測し、以後毎回このズレを引く
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // RC入力ピン設定
  pinMode(RC_ROLL, INPUT);
  pinMode(RC_PITCH, INPUT);
  pinMode(RC_THROTTLE, INPUT);
  pinMode(RC_YAW, INPUT);
  pinMode(RC_ARM, INPUT);

  // ESC(PWM)初期化
  ledcAttach(ESC_M1, 50, 16);
  ledcAttach(ESC_M2, 50, 16);
  ledcAttach(ESC_M3, 50, 16);
  ledcAttach(ESC_M4, 50, 16);

  // 起動時は必ず最低値を送ってESCをアーム待機状態にする
  setMotor(ESC_M1, THROTTLE_MIN);
  setMotor(ESC_M2, THROTTLE_MIN);
  setMotor(ESC_M3, THROTTLE_MIN);
  setMotor(ESC_M4, THROTTLE_MIN);
  delay(3000);

  initMPU6050();
  calibrateGyro();  // 静止状態でのジャイロオフセットを計測

  Serial.println("=== Phase 2-3 FC本体 起動完了 ===");
  Serial.println("【警告】プロペラが外れていることを確認してください");
}

// ---------------- モーター出力 ----------------
void setMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);  // 異常値の暴走を防ぐ安全クランプ
  int duty = map(us, 1000, 2000, 3277, 6553);
  ledcWrite(pin, duty);
}

// ---------------- MPU-6050 ----------------
void initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1レジスタ
  Wire.write(0x00);  // スリープ解除
  Wire.endTransmission();
}

// 加速度(g)・角速度(deg/s)を取得
void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H から連続読み出し開始
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // 温度レジスタは今回使わないのでスキップ
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 16384.0;  // ±2gレンジ → 16384 LSB/g
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
  gx = rawGx / 131.0;    // ±250deg/sレンジ → 131 LSB/(deg/s)
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
}

// 起動直後、機体を静止させた状態で複数回サンプリングして平均を取る
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
  // ---------- 1. RC入力読み取り ----------
  int roll     = pulseIn(RC_ROLL,     HIGH, PULSE_TIMEOUT);
  int pitch    = pulseIn(RC_PITCH,    HIGH, PULSE_TIMEOUT);
  int throttle = pulseIn(RC_THROTTLE, HIGH, PULSE_TIMEOUT);
  int yaw      = pulseIn(RC_YAW,      HIGH, PULSE_TIMEOUT);
  int arm      = pulseIn(RC_ARM,      HIGH, PULSE_TIMEOUT);

  // 信号ロスト（0が返ってきた）場合は安全な値にフォールバック
  if (roll == 0)     roll = 1500;
  if (pitch == 0)    pitch = 1500;
  if (throttle == 0) throttle = THROTTLE_MIN;
  if (yaw == 0)       yaw = 1500;
  if (arm == 0)       arm = 1000;

  bool armSwitchOn = (arm > ARM_THRESHOLD);

  // ---------- 2. IMU読み取り ----------
  float ax, ay, az, gx, gy, gz;
  readMPU6050(ax, ay, az, gx, gy, gz);
  gx -= gyroOffsetX;
  gy -= gyroOffsetY;
  gz -= gyroOffsetZ;

  // ---------- 3. 状態遷移 ----------
  switch (currentState) {
    case DISARMED:
      // スロットルが最低付近、かつArmスイッチONの時だけIDLEへ移行
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
        // 飛行中でもArmスイッチが切れたら即座に停止（緊急停止）
        currentState = DISARMED;
        Serial.println(">>> 緊急DISARMED");
      } else if (throttle < ARM_SAFE_THROTTLE) {
        currentState = IDLE;
      }
      break;
  }

  // ---------- 4. モーター出力の計算 ----------
  int m1, m2, m3, m4;

  if (currentState == DISARMED) {
    // 非アーム時は問答無用で全停止
    m1 = m2 = m3 = m4 = THROTTLE_MIN;

  } else if (currentState == IDLE) {
    // アイドル：Arm済みだがスロットル最低なので完全停止（MOTOR_STOP方式）
    m1 = m2 = m3 = m4 = THROTTLE_MIN;

  } else {  // FLYING
    float rollInput  = (roll  - 1500) * MIX_GAIN_RP;
    float pitchInput = (pitch - 1500) * MIX_GAIN_RP;
    float yawInput   = (yaw   - 1500) * MIX_GAIN_YAW;
    int throttleBase = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);

    // Rateダンピング：角速度(gx,gy)に比例した補正で急な傾きを打ち消す
    // 符号は実機テストで検証済み（Rollは反転済み、Pitchは元のまま正しかった）
    float rollDamping  = gy * DAMPING_GAIN;   // Y軸の角速度 → ロール方向の補正
    float pitchDamping = -gx * DAMPING_GAIN;  // X軸の角速度 → ピッチ方向の補正

    // X配置ミキシング（フレーム配置と回転方向に基づく）
    // M1:右前CCW  M2:左後CCW  M3:左前CW  M4:右後CW
    m1 = throttleBase - rollInput - pitchInput + yawInput - rollDamping - pitchDamping;
    m2 = throttleBase + rollInput + pitchInput + yawInput + rollDamping + pitchDamping;
    m3 = throttleBase + rollInput - pitchInput - yawInput + rollDamping - pitchDamping;
    m4 = throttleBase - rollInput + pitchInput - yawInput - rollDamping + pitchDamping;

    m1 = constrain(m1, THROTTLE_IDLE, THROTTLE_MAX);
    m2 = constrain(m2, THROTTLE_IDLE, THROTTLE_MAX);
    m3 = constrain(m3, THROTTLE_IDLE, THROTTLE_MAX);
    m4 = constrain(m4, THROTTLE_IDLE, THROTTLE_MAX);
  }

  // ---------- 5. モーターへ出力 ----------
  setMotor(ESC_M1, m1);
  setMotor(ESC_M2, m2);
  setMotor(ESC_M3, m3);
  setMotor(ESC_M4, m4);

  // ---------- 6. ログ出力 ----------
  const char* stateStr = (currentState == DISARMED) ? "DISARMED" :
                          (currentState == IDLE)     ? "IDLE"     : "FLYING";
  Serial.printf(
    "[%s] R:%d P:%d T:%d Y:%d Arm:%d | M1:%d M2:%d M3:%d M4:%d | "
    "gx:%.1f gy:%.1f gz:%.1f\n",
    stateStr, roll, pitch, throttle, yaw, arm, m1, m2, m3, m4,
    gx, gy, gz);

  delay(20);  // 約50Hzループ
}
