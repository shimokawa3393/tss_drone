// ================================================================
// TSS-550 Chapter 2 - ドローン用フライトコンピュータ
// Phase 2-3 : データ収集・可視化FC改造版
// ================================================================

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// ================================================================
// ピン定義
// ================================================================
#define SDA_PIN 32
#define SCL_PIN 33
#define LORA_CS  15
#define LORA_RST 14
#define LORA_DIO 13

// ESC出力ピン（PWM）
#define ESC_M1 25 // 左前
#define ESC_M2 12 // 右前
#define ESC_M3 27 // 左後
#define ESC_M4 4 // 右後

// RC受信機入力ピン
#define RC_THROTTLE 34
#define RC_ROLL     35
#define RC_PITCH    36
#define RC_YAW      39
#define RC_ARM      2  // アームスイッチ

// ================================================================
// センサーアドレス
// ================================================================
#define MPU_ADDR 0x68  // MPU-6050
#define BMP_ADDR 0x77  // BMP280
#define HMC_ADDR 0x1E  // HMC5883L（コンパス）

// ================================================================
// 状態機械の定義
// ================================================================
enum FlightState {
    IDLE,       // 待機中・モーター停止
    ARMED,      // アーム済み・モーター最低回転
    HOVER,      // ホバリング中（手動操縦）
    LAND,       // 着陸シーケンス
    EMERGENCY   // 緊急停止
};
FlightState state = IDLE;

// ================================================================
// PIDゲイン
// ================================================================
float Kp_roll = 2.0, Ki_roll = 0.05, Kd_roll = 0.5;
float Kp_pitch = 2.0, Ki_pitch = 0.05, Kd_pitch = 0.5;
float Kp_yaw = 3.0, Ki_yaw = 0.01, Kd_yaw = 0.3;
float Kp_alt = 1.5, Ki_alt = 0.05, Kd_alt = 0.3;

// PID計算用変数（積分・前回誤差）
float integral_roll = 0, integral_pitch = 0, integral_yaw = 0, integral_alt = 0;
float prev_error_roll = 0, prev_error_pitch = 0, prev_error_yaw = 0, prev_error_alt = 0;

// ================================================================
// センサー値
// ================================================================
float ax, ay, az;        // 加速度 [g]
float gx, gy, gz;        // ジャイロ [deg/s]
float roll, pitch, yaw;  // 姿勢角 [deg]
float altitude;          // 高度 [m]
float heading;           // コンパス方位 [deg]

// ================================================================
// RC入力値（PWM幅 1000〜2000us）
// ================================================================
int rc_throttle = 1000;
int rc_roll_in  = 1500;  // 中央1500
int rc_pitch_in = 1500;
int rc_yaw_in   = 1500;
bool rc_armed   = false;

// ================================================================
// モーター出力値（PWM幅 1000〜2000us）
// ================================================================
int m1, m2, m3, m4;  // 左前・右前・左後・右後

// ================================================================
// ループ管理
// ================================================================
#define LOOP_INTERVAL_MS 10  // 100Hz
unsigned long lastTime = 0;

// ================================================================
// setup
// ================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    // センサー初期化
    initMPU6050();
    initBMP280();
    initHMC5883L();

    // ESC初期化（PWM設定）
    ledcAttach(ESC_M1, 50, 16);
    ledcAttach(ESC_M2, 50, 16);
    ledcAttach(ESC_M3, 50, 16);
    ledcAttach(ESC_M4, 50, 16);

    // ESCアーム（最低スロットルを一定時間送る）
    setMotor(0, 1000); setMotor(1, 1000);
    setMotor(2, 1000); setMotor(3, 1000);
    delay(3000);

    // LoRa初期化
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO);
    LoRa.begin(433E6);
    LoRa.setSpreadingFactor(9);


    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("デバイス発見: 0x%02X\n", addr);
        }
    }


    Serial.println("FC起動完了");
}

// ================================================================
// loop
// ================================================================
void loop() {
    unsigned long now = millis();
    if (now - lastTime < LOOP_INTERVAL_MS) return;
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 1. センサー読み取り
    readMPU6050();
    readBMP280();
    readHMC5883L();

    // 2. 姿勢推定
    roll  = atan2(ay, az) * 180.0 / PI;
    pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
    yaw   = heading;  // コンパスから取得

    // 3. RC入力読み取り
    rc_throttle = pulseIn(RC_THROTTLE, HIGH, 25000);
    rc_roll_in  = pulseIn(RC_ROLL,     HIGH, 25000);
    rc_pitch_in = pulseIn(RC_PITCH,    HIGH, 25000);
    rc_yaw_in   = pulseIn(RC_YAW,      HIGH, 25000);
    rc_armed    = (pulseIn(RC_ARM, HIGH, 25000) > 1500);

    // 4. 状態機械の更新
    updateFlightState();

    // 5. PID計算 + モーター分配
    if (state == HOVER) {
        // RC入力を目標角度に変換（±30deg）
        float target_roll  = map(rc_roll_in,  1000, 2000, -30, 30);
        float target_pitch = map(rc_pitch_in, 1000, 2000, -30, 30);
        float target_yaw   = map(rc_yaw_in,   1000, 2000, -30, 30);

        // PID計算
        float out_roll  = computePID(target_roll,  roll,  integral_roll,  prev_error_roll,  Kp_roll,  Ki_roll,  Kd_roll,  dt);
        float out_pitch = computePID(target_pitch, pitch, integral_pitch, prev_error_pitch, Kp_pitch, Ki_pitch, Kd_pitch, dt);
        float out_yaw   = computePID(target_yaw,   yaw,   integral_yaw,   prev_error_yaw,   Kp_yaw,   Ki_yaw,   Kd_yaw,   dt);

        // モーター分配式
        // M1（左前）= throttle + roll - pitch - yaw
        // M2（右前）= throttle - roll - pitch + yaw
        // M3（左後）= throttle + roll + pitch + yaw
        // M4（右後）= throttle - roll + pitch - yaw
        m1 = constrain(rc_throttle + out_roll - out_pitch - out_yaw, 1000, 2000);
        m2 = constrain(rc_throttle - out_roll - out_pitch + out_yaw, 1000, 2000);
        m3 = constrain(rc_throttle + out_roll + out_pitch + out_yaw, 1000, 2000);
        m4 = constrain(rc_throttle - out_roll + out_pitch - out_yaw, 1000, 2000);

        setMotor(0, m1); setMotor(1, m2);
        setMotor(2, m3); setMotor(3, m4);

    } else if (state == ARMED) {
        // アーム済みだがホバーしていない→最低回転維持
        setMotor(0, 1100); setMotor(1, 1100);
        setMotor(2, 1100); setMotor(3, 1100);

    } else {
        // IDLE / EMERGENCY → 全モーター停止
        setMotor(0, 1000); setMotor(1, 1000);
        setMotor(2, 1000); setMotor(3, 1000);
    }

    // 6. LoRaテレメトリ送信
    LoRa.beginPacket();
    LoRa.printf("%lu,%.1f,%.1f,%.1f,%.1f,%d",
        now, roll, pitch, yaw, altitude, (int)state);
    LoRa.endPacket();


    Serial.printf("roll=%.1f pitch=%.1f heading=%.1f alt=%.1f state=%d\n",
    roll, pitch, heading, altitude, (int)state);
}

// ================================================================
// 状態機械の更新
// ================================================================
void updateFlightState() {
    switch (state) {
        case IDLE:
            // アームスイッチON + スロットル最小でARMEDへ
            if (rc_armed && rc_throttle < 1100) {
                state = ARMED;
                Serial.println("→ ARMED");
            }
            break;

        case ARMED:
            // アームスイッチOFFでIDLEへ戻る
            if (!rc_armed) { state = IDLE; break; }
            // スロットルを上げたらHOVERへ
            if (rc_throttle > 1200) {
                state = HOVER;
                Serial.println("→ HOVER");
            }
            break;

        case HOVER:
            // スロットル最小でLANDへ
            if (rc_throttle < 1100) {
                state = LAND;
                Serial.println("→ LAND");
            }
            // アームスイッチOFFで緊急停止
            if (!rc_armed) {
                state = EMERGENCY;
                Serial.println("→ EMERGENCY");
            }
            break;

        case LAND:
            // 高度が十分低くなったらIDLEへ
            if (altitude < 0.3) {
                state = IDLE;
                Serial.println("→ IDLE（着陸完了）");
            }
            break;

        case EMERGENCY:
            // 緊急停止は手動リセットのみ復帰
            break;
    }
}

// ================================================================
// PID計算
// ================================================================
float computePID(float target, float measured,
                 float &integral, float &prev_error,
                 float Kp, float Ki, float Kd, float dt) {
    float error = target - measured;

    // 積分ワインドアップ対策
    integral += error * dt;
    integral = constrain(integral, -100, 100);

    // 微分（前回誤差との差分）
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    return constrain(output, -300, 300);  // 出力リミット
}

// ================================================================
// モーター出力（PWM）
// ================================================================
void setMotor(int ch, int pwm_us) {
    // 1000〜2000usをledcWrite値に変換（50Hz・16bit）
    // 1000us = 3277, 2000us = 6554
    int val = map(pwm_us, 1000, 2000, 3277, 6554);
    ledcWrite(ESC_M1, val);  // チャンネル番号じゃなくてピン番号を渡す
}

// ================================================================
// MPU-6050 初期化・読み取り
// ================================================================
void initMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);  // スリープ解除
    Wire.endTransmission();
}

void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

    ax = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    ay = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    az = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0;
    Wire.read(); Wire.read();  // 温度スキップ
    gx = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    gy = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
    gz = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
}

// ================================================================
// BMP280 初期化・読み取り（簡易版）
// ================================================================
void initBMP280() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4); Wire.write(0x27);  // 通常モード
    Wire.endTransmission();
}

void readBMP280() {
    // ※簡易実装。実際はキャリブレーション補正が必要
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF7);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP_ADDR, (uint8_t)6);
    int32_t raw = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);
    float pressure = raw / 100.0;  // 仮換算（要キャリブ）
    altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
}

// ================================================================
// HMC5883L（コンパス）初期化・読み取り
// ================================================================
void initHMC5883L() {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x02); Wire.write(0x00);  // 連続計測モード
    Wire.endTransmission();
}

void readHMC5883L() {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x03);  // データレジスタ
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)HMC_ADDR, (uint8_t)6);
    int16_t mx = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // Z軸スキップ
    int16_t my = (Wire.read() << 8) | Wire.read();
    heading = atan2(my, mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
}