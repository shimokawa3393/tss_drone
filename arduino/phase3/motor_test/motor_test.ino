// モーター診断テスト用コード
// シリアルコマンドで個別モーター・スイープ・キャリブレーションを試せる

#define ESC_M1 5
#define ESC_M2 12
#define ESC_M3 26
#define ESC_M4 4

void setup() {
  Serial.begin(115200);

  ledcAttach(ESC_M1, 50, 16);
  ledcAttach(ESC_M2, 50, 16);
  ledcAttach(ESC_M3, 50, 16);
  ledcAttach(ESC_M4, 50, 16);

  // アーム：4本まとめて最低スロットルを3秒送る
  setMotorUS(ESC_M1, 1000);
  setMotorUS(ESC_M2, 1000);
  setMotorUS(ESC_M3, 1000);
  setMotorUS(ESC_M4, 1000);
  delay(3000);

  Serial.println("=== モーター診断テスト ===");
  printHelp();
}

void printHelp() {
  Serial.println("コマンド一覧:");
  Serial.println("  1500          -> 全モーターに1500usを送る");
  Serial.println("  m1 1200       -> M1だけ1200usを送る（m2,m3,m4も同様）");
  Serial.println("  sweep1〜4     -> 指定モーターだけスイープ（1000〜2000us）");
  Serial.println("  sweepall      -> M1→M2→M3→M4の順に1台ずつスイープ");
  Serial.println("  cal           -> ESCキャリブレーションシーケンス実行（M1のみ）");
  Serial.println("  stop          -> 全モーター停止（1000us）");
}

// us値をduty値に変換してPWM出力、送った値をログ表示
void setMotorUS(int pin, int us) {
  int duty = map(us, 1000, 2000, 3277, 6553);
  ledcWrite(pin, duty);
  Serial.printf("  -> pin=%d us=%d duty=%d\n", pin, us, duty);
}

// 指定したモーター1台だけをスイープさせる（他は停止のまま）
// label はログ表示用（"M1"など）
void sweepMotor(int pin, const char* label) {
  Serial.printf("%sスイープ開始 (1000→2000, 50刻み, 各300ms)\n", label);
  for (int us = 1000; us <= 2000; us += 50) {
    setMotorUS(pin, us);
    delay(300);
  }
  setMotorUS(pin, 1000);  // 終わったら必ず最低値に戻す
  Serial.printf("%sスイープ終了\n", label);
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("m1 ")) {
    setMotorUS(ESC_M1, cmd.substring(3).toInt());
  } else if (cmd.startsWith("m2 ")) {
    setMotorUS(ESC_M2, cmd.substring(3).toInt());
  } else if (cmd.startsWith("m3 ")) {
    setMotorUS(ESC_M3, cmd.substring(3).toInt());
  } else if (cmd.startsWith("m4 ")) {
    setMotorUS(ESC_M4, cmd.substring(3).toInt());

  } else if (cmd == "sweep1") {
    sweepMotor(ESC_M1, "M1");
  } else if (cmd == "sweep2") {
    sweepMotor(ESC_M2, "M2");
  } else if (cmd == "sweep3") {
    sweepMotor(ESC_M3, "M3");
  } else if (cmd == "sweep4") {
    sweepMotor(ESC_M4, "M4");

  } else if (cmd == "sweepall") {
    // 1台ずつ、他は必ず停止させた状態で確認する
    sweepMotor(ESC_M1, "M1");
    delay(1000);
    sweepMotor(ESC_M2, "M2");
    delay(1000);
    sweepMotor(ESC_M3, "M3");
    delay(1000);
    sweepMotor(ESC_M4, "M4");
    Serial.println("=== 全モーター確認完了 ===");

  } else if (cmd == "cal") {
    Serial.println("キャリブレーション開始：最大値を送信...");
    setMotorUS(ESC_M1, 2000);
    delay(3000);
    Serial.println("最低値を送信...");
    setMotorUS(ESC_M1, 1000);
    delay(3000);
    Serial.println("キャリブレーション完了");

  } else if (cmd == "stop") {
    setMotorUS(ESC_M1, 1000);
    setMotorUS(ESC_M2, 1000);
    setMotorUS(ESC_M3, 1000);
    setMotorUS(ESC_M4, 1000);
    Serial.println("全モーター停止");

  } else {
    int us = cmd.toInt();
    if (us >= 1000 && us <= 2000) {
      setMotorUS(ESC_M1, us);
      setMotorUS(ESC_M2, us);
      setMotorUS(ESC_M3, us);
      setMotorUS(ESC_M4, us);
    } else {
      Serial.println("不明なコマンド");
      printHelp();
    }
  }
}