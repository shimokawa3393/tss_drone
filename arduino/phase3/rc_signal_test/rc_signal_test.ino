// ================================================================
// RC受信機 信号テスト用コード
// FS-iA10B の各チャンネル(CH1〜CH5)からPWM信号を読み取り、
// シリアルモニターにパルス幅(us)を表示する
// モーター・ESCは接続しない状態でのテストを想定
// ================================================================

// 新しいGPIO割り当て（基板シルクの物理的な並び順 35,34,VN,VP に合わせた配線）
#define RC_ROLL     34  // CH1（デフォルトAETR: Roll）
#define RC_PITCH    39  // CH2（デフォルトAETR: Pitch）シルク表記:VN
#define RC_THROTTLE 35  // CH3（デフォルトAETR: Throttle）
#define RC_YAW      36  // CH4（デフォルトAETR: Yaw）シルク表記:VP
#define RC_ARM      2   // CH5（Armスイッチ）

// pulseInのタイムアウト値（us）
// 受信機からの信号は約50Hz(20ms周期)なので、それより少し長めに設定
#define PULSE_TIMEOUT 25000

void setup() {
  Serial.begin(115200);

  // 全RC入力ピンを入力モードに設定
  pinMode(RC_ROLL, INPUT);
  pinMode(RC_PITCH, INPUT);
  pinMode(RC_THROTTLE, INPUT);
  pinMode(RC_YAW, INPUT);
  pinMode(RC_ARM, INPUT);

  Serial.println("=== RC信号テスト開始 ===");
  Serial.println("送信機の電源を入れ、各スティックを動かして数値の変化を確認してください");
  delay(1000);
}

void loop() {
  // 各チャンネルのHIGHパルス幅（マイクロ秒）を計測
  // 信号が来ていない場合はタイムアウトして0が返る
  int roll     = pulseIn(RC_ROLL,     HIGH, PULSE_TIMEOUT);
  int pitch    = pulseIn(RC_PITCH,    HIGH, PULSE_TIMEOUT);
  int throttle = pulseIn(RC_THROTTLE, HIGH, PULSE_TIMEOUT);
  int yaw      = pulseIn(RC_YAW,      HIGH, PULSE_TIMEOUT);
  int arm      = pulseIn(RC_ARM,      HIGH, PULSE_TIMEOUT);

  // 信号が来ていないチャンネルには [NG] を付けて分かりやすくする
  Serial.printf(
    "Roll:%5d%s  Pitch:%5d%s  Throttle:%5d%s  Yaw:%5d%s  Arm:%5d%s\n",
    roll,     roll     == 0 ? "[NG]" : "    ",
    pitch,    pitch    == 0 ? "[NG]" : "    ",
    throttle, throttle == 0 ? "[NG]" : "    ",
    yaw,      yaw      == 0 ? "[NG]" : "    ",
    arm,      arm      == 0 ? "[NG]" : "    "
  );

  delay(100);  // 約10Hzで表示更新
}