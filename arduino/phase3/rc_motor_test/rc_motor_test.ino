// ================================================================
// RC → モーター 連動テスト
// スティック操作(RC入力)を読み取り、X配置4モーターの出力に変換する
// setMotor()のチャンネルバグは修正済み(pin直指定方式)
// 【必ずプロペラを外した状態でテストすること】
// ================================================================

// --- ESC(モーター出力)GPIO割り当て ---
// フレーム配置：①右前CCW=M1  ③左前CW=M3
//              ②左後CCW=M2  ④右後CW=M4
#define ESC_M1 5    // 右前 CCW
#define ESC_M2 12   // 左後 CCW
#define ESC_M3 26   // 左前 CW
#define ESC_M4 4    // 右後 CW

// --- RC入力 GPIO割り当て（新割り当て：基板シルク順 35,34,VN,VP）---
#define RC_ROLL     34  // CH1
#define RC_PITCH    39  // CH2 (VN)
#define RC_THROTTLE 35  // CH3
#define RC_YAW      36  // CH4 (VP)
#define RC_ARM      2   // CH5（現状VrA割り当て。将来SWAに変更予定）

#define PULSE_TIMEOUT 25000  // pulseInタイムアウト(us)

// --- 安全パラメータ ---
const int THROTTLE_MIN   = 1000;  // 完全停止
const int THROTTLE_IDLE  = 1080;  // アイドル回転（Arm直後の最低回転数）
const int THROTTLE_MAX   = 1900;  // 出力上限（2000ではなく余裕を持たせる）
const int ARM_THRESHOLD  = 1500;  // これを超えたらArmed判定
const int ARM_SAFE_THROTTLE = 1100; // Armを許可する条件：スロットルがこれ未満

// ミキシングのゲイン（roll/pitch/yawの入力をどれだけモーター差に反映するか）
// 値が大きいほど反応が強くなる。プロペラ無しテストなので大きめでもOK、
// プロペラ装着後は小さめに再調整する前提
const float MIX_GAIN_RP = 0.4;   // roll/pitch用ゲイン
const float MIX_GAIN_YAW = 0.3;  // yaw用ゲイン

// --- 機体の状態 ---
enum FlightState { DISARMED, IDLE, FLYING };
FlightState currentState = DISARMED;

void setup() {
  Serial.begin(115200);

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

  Serial.println("=== RC-モーター連動テスト開始 ===");
  Serial.println("【警告】プロペラが外れていることを確認してください");
}

// マイクロ秒(1000〜2000)をduty値に変換してPWM出力
void setMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);  // 異常値の暴走を防ぐ安全クランプ
  int duty = map(us, 1000, 2000, 3277, 6553);
  ledcWrite(pin, duty);
}

void loop() {
  // ---------- 1. RC入力読み取り ----------
  int roll     = pulseIn(RC_ROLL,     HIGH, PULSE_TIMEOUT);
  int pitch    = pulseIn(RC_PITCH,    HIGH, PULSE_TIMEOUT);
  int throttle = pulseIn(RC_THROTTLE, HIGH, PULSE_TIMEOUT);
  int yaw      = pulseIn(RC_YAW,      HIGH, PULSE_TIMEOUT);
  int arm      = pulseIn(RC_ARM,      HIGH, PULSE_TIMEOUT);

  // 信号ロスト（0が返ってきた）場合はニュートラル/最低値扱いにして安全側に倒す
  if (roll == 0)     roll = 1500;
  if (pitch == 0)    pitch = 1500;
  if (throttle == 0) throttle = THROTTLE_MIN;
  if (yaw == 0)       yaw = 1500;
  if (arm == 0)       arm = 1000;

  bool armSwitchOn = (arm > ARM_THRESHOLD);

  // ---------- 2. 状態遷移 ----------
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

  // ---------- 3. モーター出力の計算 ----------
  int m1, m2, m3, m4;

  if (currentState == DISARMED) {
    // 非アーム時は問答無用で全停止
    m1 = m2 = m3 = m4 = THROTTLE_MIN;

  } else if (currentState == IDLE) {
     // アイドル：Arm済みだがスロットル最低なので完全停止（MOTOR_STOP方式）
     // ミキシングはせず、指を近づけても安全な状態を優先する
     m1 = m2 = m3 = m4 = THROTTLE_MIN;

  } else {  // FLYING
    // roll/pitch/yawをニュートラル(1500)からの差分に変換
    float rollInput  = (roll  - 1500) * MIX_GAIN_RP;
    float pitchInput = (pitch - 1500) * MIX_GAIN_RP;
    float yawInput   = (yaw   - 1500) * MIX_GAIN_YAW;

    // スロットルは上限に余裕を持たせてミキシング分の伸びしろを確保
    int throttleBase = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);

    // X配置ミキシング（フレーム配置と回転方向に基づく）
    // M1:右前CCW  M2:左後CCW  M3:左前CW  M4:右後CW
    m1 = throttleBase - rollInput - pitchInput + yawInput;  // 右前 CCW
    m2 = throttleBase + rollInput + pitchInput + yawInput;  // 左後 CCW
    m3 = throttleBase + rollInput - pitchInput - yawInput;  // 左前 CW
    m4 = throttleBase - rollInput + pitchInput - yawInput;  // 右後 CW

    // 出力範囲を安全にクランプ（下限はアイドル値、上限はTHROTTLE_MAX）
    m1 = constrain(m1, THROTTLE_IDLE, THROTTLE_MAX);
    m2 = constrain(m2, THROTTLE_IDLE, THROTTLE_MAX);
    m3 = constrain(m3, THROTTLE_IDLE, THROTTLE_MAX);
    m4 = constrain(m4, THROTTLE_IDLE, THROTTLE_MAX);
  }

  // ---------- 4. モーターへ出力 ----------
  setMotor(ESC_M1, m1);
  setMotor(ESC_M2, m2);
  setMotor(ESC_M3, m3);
  setMotor(ESC_M4, m4);

  // ---------- 5. ログ出力 ----------
  const char* stateStr = (currentState == DISARMED) ? "DISARMED" :
                          (currentState == IDLE)     ? "IDLE"     : "FLYING";
  Serial.printf("[%s] R:%d P:%d T:%d Y:%d Arm:%d | M1:%d M2:%d M3:%d M4:%d\n",
                stateStr, roll, pitch, throttle, yaw, arm, m1, m2, m3, m4);

  delay(20);  // 約50Hzループ
}