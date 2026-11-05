/*
 * ============================================================================
 * 使用ピン一覧 ＆ 配線リファレンス (Arduino Uno / Nano 等 ATmega328P基準)
 * ============================================================================
 * 
 * 【1. シリアル通信 (Serial)】
 *  - D0 (RX)  : シリアル受信 (USBシリアル通信で使用)
 *  - D1 (TX)  : シリアル送信 (USBシリアル通信で使用)
 * 
 * 【2. 超音波センサー HC-SR04 × 4個 (時分割・ピン変化割り込み方式)】
 *  - D4       : Sonar 0 (前) Trig出力
 *  - D3       : Sonar 1 (左側) Trig出力
 *  - D7       : Sonar 2 (右側) Trig出力
 *  - D8       : Sonar 3 (後) Trig出力 (※シールドと干渉する場合はA4等へ変更可)
 *  - A0 (PC0) : Sonar 0 (前) Echo入力 (PCINT8  ピン変化割り込み)
 *  - A1 (PC1) : Sonar 1 (左側) Echo入力 (PCINT9  ピン変化割り込み)
 *  - A2 (PC2) : Sonar 2 (右側) Echo入力 (PCINT10 ピン変化割り込み)
 *  - A3 (PC3) : Sonar 3 (後) Echo入力 (PCINT11 ピン変化割り込み)
 *  - 5V / GND : 各センサーの VCC を Arduino 5V へ、GND を共通 GND へ接続
 * 
 * 【3. サーボ・ESC制御ピン (Servo.h ＆ ユーザー定義)】
 *  - D5       : ステアリングサーボ 信号線 (白 または オレンジ)
 *  - D6       : ESC (アンプ)       信号線 (白 または オレンジ)
 *  ※ 注意: Servoライブラリ使用時は内部で「Timer1」を占有するため、
 *          D9, D10ピンのPWM出力(analogWrite)は使用不可になります。
 * 
 * 【4. サーボ・ESC 電源(BEC) ＆ GND 配線ルール (重要)】
 *  [ステアリングサーボ]                 [Arduino (＋Jetson USB接続)]
 *    ├─ 信号 (白/橙) ───────────────────────> D5 ピン
 *    ├─ 電源 (赤)    ───┐ (※直結してArduinoの5Vからは浮かす)
 *    └─ GND  (黒/茶) ───┼───┬───────────────> GND ピン
 *                       │   │
 *  [ESC (アンプ)]       │   │
 *    ├─ 信号 (白/橙) ───┼───┼───────────────> D6 ピン
 *    ├─ BEC電源 (赤) ───┘   │
 *    └─ GND  (黒/茶) ───────┘
 * 
 *  - 信号線 (白/橙) : サーボは D5、ESCは D6 へ接続。
 *  - GND線  (黒/茶) : サーボ・ESC・Arduinoの「GND」をすべて共通化(結合)する。
 *  - 電源線 (赤)    : サーボの赤線とESCの赤線(BEC出力)の2本だけを直結する。
 *    ※ 警告1: サーボの赤線をArduinoの5Vに直結すると電圧降下でマイコンが再起動します。
 *    ※ 警告2: PCとUSB接続中にESCの赤線をArduinoの5Vに繋ぐと電源が衝突し故障します。
 *    ※ 補足 : 単体走行時はArduinoへ別電源(モバイルバッテリー等)からUSB給電するか、
 *             ESCのBEC電圧が5.0V仕様であることを確認した上でPCを抜いて5Vピンへ給電。
 * 
 * 【5. コントローラー・Jetsonとの入力】
 *  - スロットル : L2前進/R2後退で手動操作。OPTIONSで25%自動前進を切替
 *  - タッチパッド : 押すと自動スロットルを解除し、押している間は緊急停止
 *  - 正面200cm以内 : 前進上限30%、100cm以内 : 15%、15cm以内 : 停止
 *  - 横ソナー : スロットル制限なし。壁回避操舵に使用
 *  - ×ボタン : PS4操舵のみ、Sonar制御OFF (標準色)
 *  - ○ボタン : PS4操舵、Sonar制御ON (赤色)
 *  - □ボタン : Jetson操舵のみ、Sonar制御OFF (ピンク色)
 *  - △ボタン : Jetson操舵、Sonar制御ON (緑色)
 *  - ライトバー: 選択中のモード色を維持
 *  - PS4接続    : USB Host Shield 2.0 (SPI: D9〜D13)
 *  - Jetson操舵 : USBシリアル、115200bps、ASCII、LF (\n) 終端
 *  - Jetson形式 : S<0〜255> (例: S127\n)。S<値>,T<値>形式も受信し、操舵値Sを使用
 *  - Jetson値   : 0〜255をステアリングパルス1200〜1800usに変換
 *  - Jetson無通信: 最後に受信した操舵値を保持。まだ受信がなければ中央にする
 *  - Arduino返送: S操舵値,Tスロットル値,F/L/R/B各ソナー距離(cm),
 *                  PはPS4接続状態、JはJetson操舵指令受信状態
 * 
 * 【空きピン (I2Cデバイスや追加センサー等で利用可能)】
 *  - デジタル : D2 (外部割り込みINT0)
 *  - アナログ : A4 (I2C SDA), A5 (I2C SCL)
 * ============================================================================
 */

#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>
#include <Servo.h>

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

Servo steeringServo; // D5: ステアリング
Servo escServo;      // D6: ESC

const int SERVO_PIN = 5; // ステアリング
const int ESC_PIN   = 6; // ESC

const int STEERING_CENTER_US = 1500;
const int STEERING_RANGE_US  = 400; // 舵角範囲 (1200us ～ 1800us)

const int ESC_STOP_US  = 1500;
const int ESC_RANGE_US = 400;  // 出力範囲
const int ESC_FORWARD_LIMIT_PERCENT = 100; // 前進の最大出力上限 (%)
const int ESC_REVERSE_LIMIT_PERCENT = 100; // 後進の最大出力上限 (%)
const int AUTO_THROTTLE_PERCENT = 25;
const int FRONT_WALL_THROTTLE_LIMIT_PERCENT = 30;
const int FRONT_WALL_NEAR_THROTTLE_LIMIT_PERCENT = 15;
const int ESC_FORWARD_LIMIT_RANGE_US = ESC_RANGE_US * ESC_FORWARD_LIMIT_PERCENT / 100;
const int ESC_REVERSE_LIMIT_RANGE_US = ESC_RANGE_US * ESC_REVERSE_LIMIT_PERCENT / 100;
const int FRONT_WALL_THROTTLE_LIMIT_RANGE_US =
  ESC_RANGE_US * FRONT_WALL_THROTTLE_LIMIT_PERCENT / 100;
const int FRONT_WALL_NEAR_THROTTLE_LIMIT_RANGE_US =
  ESC_RANGE_US * FRONT_WALL_NEAR_THROTTLE_LIMIT_PERCENT / 100;

const int STEERING_INPUT_MIN = 0;
const int STEERING_INPUT_MAX = 255;
const int STEERING_INPUT_CENTER = 127;
const int DEADZONE = 12;
const float STEERING_FILTER_ALPHA = 0.1;
const float SIDE_WALL_CURVE_OPEN_DISTANCE_CM = 100.0;
const float FRONT_WALL_THROTTLE_LIMIT_DISTANCE_CM = 200.0;
const float FRONT_WALL_NEAR_THROTTLE_LIMIT_DISTANCE_CM = 100.0;
const float SIDE_WALL_SLOW_DISTANCE_CM = 30.0;
const float SIDE_WALL_STOP_DISTANCE_CM = 10.0;
const int SIDE_WALL_STEERING_CORRECTION_US = 150;
const unsigned long TELEMETRY_INTERVAL_MS = 50;
const uint8_t SERIAL_LINE_BUFFER_SIZE = 16;

char serialLine[SERIAL_LINE_BUFFER_SIZE];
uint8_t serialLineLength = 0;
bool serialLineOverflow = false;

int jetsonSteeringValue = STEERING_INPUT_CENTER;
bool hasJetsonSteeringCommand = false;
bool useJetsonSteering = false;
bool sonarControlEnabled = true;
bool autoThrottleEnabled = false;
float filteredSteeringUs = STEERING_CENTER_US;
unsigned long lastTelemetryMs = 0;
uint8_t selectedLedMode = 0;
uint8_t lastLedMode = 255;

// ----------------------------------------------------------------------------
// 4連超音波センサー (HC-SR04) 設定
// ----------------------------------------------------------------------------
const int NUM_SONARS = 4;
const int trigPins[NUM_SONARS] = {4, 3, 7, 8};     // Sonar 0〜3 の Trigピン
const int echoPins[NUM_SONARS] = {A0, A1, A2, A3}; // Sonar 0〜3 の Echoピン (A0〜A3固定)

const unsigned long SONAR_INTERVAL_MS = 30; // センサー切り替え間隔 (30ms × 4 = 1周120ms)
unsigned long lastSonarSwitchTime = 0;

volatile uint8_t currentSonar = 0;          // 現在計測中のセンサー番号 (0〜3)
volatile unsigned long echoStartUs = 0;     // Echo立ち上がり時刻
volatile unsigned long echoDurationUs = 0;  // 計測されたパルス幅
volatile bool echoReceived = false;         // 計測完了フラグ

float distanceCm[NUM_SONARS] = {999.0, 999.0, 999.0, 999.0}; // 4つの距離データ(cm)

// ----------------------------------------------------------------------------
// ピン変化割り込み (PORTC: A0〜A3 のいずれかが変化した時に自動で呼ばれる)
// ----------------------------------------------------------------------------
ISR(PCINT1_vect) {
  // 現在アクティブなセンサーのEchoピンに対応するビットマスク (A0=bit0, A1=bit1, A2=bit2, A3=bit3)
  uint8_t mask = (1 << currentSonar);

  // PINCレジスタを直接読み取ってHIGH/LOWを高速判定
  if (PINC & mask) {
    // LOW -> HIGH (立ち上がり)
    echoStartUs = micros();
  } else {
    // HIGH -> LOW (立ち下がり)
    echoDurationUs = micros() - echoStartUs;
    echoReceived = true;
  }
}

// ----------------------------------------------------------------------------
// 4つのHC-SR04を30msごとに順番にトリガー＆距離更新する関数 (待ち時間ゼロ)
// ----------------------------------------------------------------------------
void updateMultiSonar() {
  unsigned long now = millis();

  // 30ms経過するごとに「前センサーの結果回収」と「次センサーのトリガー発射」を行う
  if (now - lastSonarSwitchTime >= SONAR_INTERVAL_MS) {
    lastSonarSwitchTime = now;

    // 1. 直前に鳴らしたセンサーの結果を回収
    if (echoReceived) {
      noInterrupts();
      unsigned long dur = echoDurationUs;
      echoReceived = false;
      interrupts();

      float tempCm = (float)dur * 0.01715; // 距離(cm)に変換
      if (tempCm >= 2.0 && tempCm <= 450.0) {
        distanceCm[currentSonar] = tempCm;
      } else {
        distanceCm[currentSonar] = 999.0;
      }
    } else {
      // 30ms以内にEchoが返ってこなかった場合（タイムアウト・範囲外）
      distanceCm[currentSonar] = 999.0;
    }

    // 2. 計測するセンサーを次の番号 (0 -> 1 -> 2 -> 3 -> 0...) に切り替え
    uint8_t nextSonar = (currentSonar + 1) % NUM_SONARS;

    noInterrupts();
    currentSonar = nextSonar;
    echoReceived = false;
    // 現在計測するセンサーのEchoピン(A0〜A3)だけ割り込みを許可し、他3つのノイズを遮断
    PCMSK1 = (1 << currentSonar);
    interrupts();

    // 3. 次のセンサーのTrigパルス(10us)を発射
    int trigPin = trigPins[currentSonar];
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // 4つの超音波センサーのピン初期化
  for (int i = 0; i < NUM_SONARS; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }

  // ピン変化割り込み (PCINT1グループ = PORTC: A0〜A5) の有効化
  noInterrupts();
  PCICR |= (1 << PCIE1);   // PORTCのピン変化割り込みを許可
  PCMSK1 = (1 << 0);       // 最初はSonar 0 (A0) のみ割り込みを許可
  interrupts();
  
  steeringServo.attach(SERVO_PIN);
  escServo.attach(ESC_PIN);

  steeringServo.writeMicroseconds(STEERING_CENTER_US);
  escServo.writeMicroseconds(ESC_STOP_US);

  if (Usb.Init() == -1) {
    while (1) {
      steeringServo.writeMicroseconds(STEERING_CENTER_US);
      escServo.writeMicroseconds(ESC_STOP_US);
    }
  }
  delay(1000);
  delay(1000);
}

void loop() {
  Usb.Task();

  // バックグラウンドで4つのソナーを順番に更新 (待ち時間ゼロ)
  updateMultiSonar();

  // LFまで受信した1行を解析。バッファ超過行は破棄する。
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\r') {
      continue;
    }

    if (receivedChar == '\n') {
      if (!serialLineOverflow && serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        char *numberEnd;

        if (serialLine[0] == 'S') {
          long steeringValue = strtol(serialLine + 1, &numberEnd, 10);
          bool steeringDigits = numberEnd != serialLine + 1 &&
                                (*numberEnd == '\0' ||
                                 (numberEnd[0] == ',' && numberEnd[1] == 'T'));
          for (char *digit = serialLine + 1; digit < numberEnd; digit++) {
            if (*digit < '0' || *digit > '9') {
              steeringDigits = false;
              break;
            }
          }

          bool validLine = steeringDigits;
          if (validLine && *numberEnd != '\0') {
            char *throttleStart = numberEnd + 2;
            char *throttleEnd;
            long throttleValue = strtol(throttleStart, &throttleEnd, 10);
            bool throttleDigits = throttleEnd != throttleStart;
            char *digit = throttleStart;
            if (*digit == '-') {
              digit++;
            }
            for (; digit < throttleEnd; digit++) {
              if (*digit < '0' || *digit > '9') {
                throttleDigits = false;
                break;
              }
            }
            validLine = throttleDigits &&
                        *throttleEnd == '\0' &&
                        throttleValue >= -255 &&
                        throttleValue <= 255;
          }

          if (validLine &&
              steeringValue >= STEERING_INPUT_MIN &&
              steeringValue <= STEERING_INPUT_MAX) {
            jetsonSteeringValue = steeringValue;
            hasJetsonSteeringCommand = true;
          }
        }
      }

      serialLineLength = 0;
      serialLineOverflow = false;
    } else if (!serialLineOverflow) {
      if (serialLineLength < SERIAL_LINE_BUFFER_SIZE - 1) {
        serialLine[serialLineLength++] = receivedChar;
      } else {
        serialLineOverflow = true;
      }
    }
  }

  uint8_t controllerSteeringValue = STEERING_INPUT_CENTER;
  uint8_t r2Val = 0;
  uint8_t l2Val = 0;
  int escUs = ESC_STOP_US;

  if (PS4.connected()) {
    if (PS4.getButtonClick(OPTIONS)) {
      autoThrottleEnabled = !autoThrottleEnabled;
    }

    if (PS4.getButtonClick(CROSS)) {
      autoThrottleEnabled = false;
      useJetsonSteering = false;
      sonarControlEnabled = false;
      selectedLedMode = 0;
    } else if (PS4.getButtonClick(CIRCLE)) {
      autoThrottleEnabled = false;
      useJetsonSteering = false;
      sonarControlEnabled = true;
      selectedLedMode = 3;
    } else if (PS4.getButtonClick(SQUARE)) {
      autoThrottleEnabled = false;
      useJetsonSteering = true;
      sonarControlEnabled = false;
      selectedLedMode = 2;
    } else if (PS4.getButtonClick(TRIANGLE)) {
      autoThrottleEnabled = false;
      useJetsonSteering = true;
      sonarControlEnabled = true;
      selectedLedMode = 1;
    }

    if (selectedLedMode != lastLedMode) {
      if (selectedLedMode == 3) {
        PS4.setLed(255, 0, 0);
      } else if (selectedLedMode == 1) {
        PS4.setLed(0, 255, 0);
      } else if (selectedLedMode == 2) {
        PS4.setLed(255, 0, 128);
      } else {
        PS4.setLed(Blue);
      }
      lastLedMode = selectedLedMode;
    }

    controllerSteeringValue = PS4.getAnalogHat(LeftHatX);
    r2Val = PS4.getAnalogButton(R2);
    l2Val = PS4.getAnalogButton(L2);

    int baseEscUs = ESC_STOP_US;
    bool forwardThrottleRequested = false;
    if (autoThrottleEnabled) {
      baseEscUs = ESC_STOP_US -
        ESC_RANGE_US * AUTO_THROTTLE_PERCENT / 100;
      forwardThrottleRequested = true;
    } else if (l2Val > 15) {
      baseEscUs = map(
        l2Val, 0, 255,
        ESC_STOP_US, ESC_STOP_US - ESC_FORWARD_LIMIT_RANGE_US);
      forwardThrottleRequested = true;
    }

    if (forwardThrottleRequested) {
      float frontDist = distanceCm[0];

      if (sonarControlEnabled) {
        if (frontDist >= 2.0 && frontDist <= 15.0) {
          escUs = ESC_STOP_US;
        } else {
          int forwardDiff = ESC_STOP_US - baseEscUs;
          if (frontDist >= 2.0 &&
              frontDist <= FRONT_WALL_THROTTLE_LIMIT_DISTANCE_CM) {
            forwardDiff = min(
              forwardDiff, FRONT_WALL_THROTTLE_LIMIT_RANGE_US);
          }
          if (frontDist >= 2.0 &&
              frontDist <= FRONT_WALL_NEAR_THROTTLE_LIMIT_DISTANCE_CM) {
            forwardDiff = min(
              forwardDiff, FRONT_WALL_NEAR_THROTTLE_LIMIT_RANGE_US);
          }
          escUs = ESC_STOP_US - forwardDiff;
        }
      } else {
        escUs = baseEscUs;
      }
    } else if (!autoThrottleEnabled && r2Val > 15) {
      escUs = map(
        r2Val, 0, 255,
        ESC_STOP_US, ESC_STOP_US + ESC_REVERSE_LIMIT_RANGE_US);
    }
  } else {
    // 再接続後も現在選択中のモード色をコントローラーへ再適用する。
    lastLedMode = 255;
    autoThrottleEnabled = false;
  }

  unsigned long now = millis();
  bool steeringInputAvailable = useJetsonSteering
    ? hasJetsonSteeringCommand
    : PS4.connected();
  int steeringUs = STEERING_CENTER_US;

  if (useJetsonSteering) {
    if (hasJetsonSteeringCommand) {
      if (jetsonSteeringValue <= STEERING_INPUT_CENTER) {
        steeringUs = map(
          jetsonSteeringValue,
          STEERING_INPUT_MIN, STEERING_INPUT_CENTER,
          STEERING_CENTER_US - STEERING_RANGE_US,
          STEERING_CENTER_US);
      } else {
        steeringUs = map(
          jetsonSteeringValue,
          STEERING_INPUT_CENTER, STEERING_INPUT_MAX,
          STEERING_CENTER_US,
          STEERING_CENTER_US + STEERING_RANGE_US);
      }
    }
  } else if (PS4.connected() &&
             (controllerSteeringValue < STEERING_INPUT_CENTER - DEADZONE ||
              controllerSteeringValue > STEERING_INPUT_CENTER + DEADZONE)) {
    steeringUs = map(
      controllerSteeringValue,
      STEERING_INPUT_MIN, STEERING_INPUT_MAX,
      STEERING_CENTER_US - STEERING_RANGE_US,
      STEERING_CENTER_US + STEERING_RANGE_US);
  }

  bool emergencySteering = false;
  // Jetsonが左右ソナーで検知した壁側へ操舵する時、開けた側へ最大操舵する。
  // 横壁回避は自動前進中にも適用し、スロットル停止中も操舵を続ける。
  if (sonarControlEnabled &&
      steeringInputAvailable &&
      (autoThrottleEnabled || l2Val > 15)) {
    float leftDist = distanceCm[1];  // Sonar 1 (左側)
    float rightDist = distanceCm[2]; // Sonar 2 (右側)
    float closestSideDist = min(leftDist, rightDist);
    bool leftSideOpen = leftDist > SIDE_WALL_CURVE_OPEN_DISTANCE_CM;
    bool rightSideOpen = rightDist > SIDE_WALL_CURVE_OPEN_DISTANCE_CM;
    bool jetsonSteeringLeft =
      useJetsonSteering &&
      hasJetsonSteeringCommand &&
      jetsonSteeringValue < STEERING_INPUT_CENTER - DEADZONE;
    bool jetsonSteeringRight =
      useJetsonSteering &&
      hasJetsonSteeringCommand &&
      jetsonSteeringValue > STEERING_INPUT_CENTER + DEADZONE;

    if (leftSideOpen && rightDist <= SIDE_WALL_CURVE_OPEN_DISTANCE_CM &&
        jetsonSteeringRight) {
      steeringUs = STEERING_CENTER_US - STEERING_RANGE_US;
      emergencySteering = true;
    } else if (rightSideOpen &&
               leftDist <= SIDE_WALL_CURVE_OPEN_DISTANCE_CM &&
               jetsonSteeringLeft) {
      steeringUs = STEERING_CENTER_US + STEERING_RANGE_US;
      emergencySteering = true;
    } else if (closestSideDist < SIDE_WALL_SLOW_DISTANCE_CM &&
               leftDist != rightDist) {
      float avoidanceScale = constrain(
        (SIDE_WALL_SLOW_DISTANCE_CM - closestSideDist) /
          (SIDE_WALL_SLOW_DISTANCE_CM - SIDE_WALL_STOP_DISTANCE_CM),
        0.0, 1.0);
      int avoidanceCorrection =
        (int)(avoidanceScale * SIDE_WALL_STEERING_CORRECTION_US);

      steeringUs += leftDist < rightDist
        ? avoidanceCorrection
        : -avoidanceCorrection;
      steeringUs = constrain(
        steeringUs,
        STEERING_CENTER_US - STEERING_RANGE_US,
        STEERING_CENTER_US + STEERING_RANGE_US);
    }
  }

  // タッチパッドを押している間は自動モードを解除して緊急停止する。
  if (PS4.connected() && PS4.getButtonPress(TOUCHPAD)) {
    autoThrottleEnabled = false;
    escUs = ESC_STOP_US;
  }

  if (emergencySteering) {
    filteredSteeringUs = steeringUs;
  } else {
    filteredSteeringUs +=
      (steeringUs - filteredSteeringUs) * STEERING_FILTER_ALPHA;
    steeringUs = (int)filteredSteeringUs;
  }

  steeringServo.writeMicroseconds(steeringUs);
  escServo.writeMicroseconds(escUs);

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS &&
      Serial.availableForWrite() >= 48) {
    lastTelemetryMs = now;

    int steeringOutputValue;
    if (useJetsonSteering && hasJetsonSteeringCommand) {
      steeringOutputValue = jetsonSteeringValue;
    } else if (steeringUs <= STEERING_CENTER_US) {
      steeringOutputValue = map(
        steeringUs,
        STEERING_CENTER_US - STEERING_RANGE_US, STEERING_CENTER_US,
        STEERING_INPUT_MIN, STEERING_INPUT_CENTER);
    } else {
      steeringOutputValue = map(
        steeringUs,
        STEERING_CENTER_US, STEERING_CENTER_US + STEERING_RANGE_US,
        STEERING_INPUT_CENTER, STEERING_INPUT_MAX);
    }

    int throttleOutputValue = 0;
    if (escUs < ESC_STOP_US) {
      throttleOutputValue = map(
        escUs,
        ESC_STOP_US, ESC_STOP_US - ESC_FORWARD_LIMIT_RANGE_US,
        0, 255);
    } else if (escUs > ESC_STOP_US) {
      throttleOutputValue = map(
        escUs,
        ESC_STOP_US, ESC_STOP_US + ESC_REVERSE_LIMIT_RANGE_US,
        0, -255);
    }

    Serial.print('S');
    Serial.print(constrain(
      steeringOutputValue, STEERING_INPUT_MIN, STEERING_INPUT_MAX));
    Serial.print(F(",T"));
    Serial.print(constrain(throttleOutputValue, -255, 255));
    Serial.print(F(",F"));
    Serial.print(distanceCm[0], 1);
    Serial.print(F(",L"));
    Serial.print(distanceCm[1], 1);
    Serial.print(F(",R"));
    Serial.print(distanceCm[2], 1);
    Serial.print(F(",B"));
    Serial.print(distanceCm[3], 1);
    Serial.print(F(",P"));
    Serial.print(PS4.connected() ? 1 : 0);
    Serial.print(F(",J"));
    Serial.println(hasJetsonSteeringCommand ? 1 : 0);
  }
}
