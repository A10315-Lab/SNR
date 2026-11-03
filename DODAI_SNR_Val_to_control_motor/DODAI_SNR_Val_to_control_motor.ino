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
 *  - D3       : Sonar 1 (左前45度) Trig出力
 *  - D7       : Sonar 2 (右前45度) Trig出力
 *  - D8       : Sonar 3 (後) Trig出力 (※シールドと干渉する場合はA4等へ変更可)
 *  - A0 (PC0) : Sonar 0 (前) Echo入力 (PCINT8  ピン変化割り込み)
 *  - A1 (PC1) : Sonar 1 (左前45度) Echo入力 (PCINT9  ピン変化割り込み)
 *  - A2 (PC2) : Sonar 2 (右前45度) Echo入力 (PCINT10 ピン変化割り込み)
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
 *  - スロットル : PS4コントローラーのR2/L2で操作
 *  - ステアリング入力切替 : PS4コントローラーの△ボタン
 *  - PS4接続    : USB Host Shield 2.0 (SPI: D9〜D13)
 *  - Jetson操舵 : USBシリアル、115200bps、ASCII、LF (\n) 終端
 *  - Jetson形式 : S<0〜255> (例: S127\n)
 *  - Jetson値   : 0〜255をステアリングパルス1200〜1800usに変換
 *  - Jetson無通信: Jetson操舵選択中に500ms受信がなければ中央に戻す
 *  - Arduino返送: S<最終操舵0〜255>,T<最終スロットル-255〜255>\n
 *  - 返送例     : S127,T-100 (Sonar回避・減速反映後の出力値)
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
const int STEERING_RANGE_US  = 300; // 舵角範囲 (1200us ～ 1800us)

const int ESC_STOP_US  = 1500;
const int ESC_RANGE_US = 400;  // 出力範囲
const int ESC_FORWARD_LIMIT_PERCENT = 40; // 前進の最大出力上限 (%)
const int ESC_REVERSE_LIMIT_PERCENT = 50; // 後進の最大出力上限 (%)
const int ESC_FORWARD_LIMIT_RANGE_US = ESC_RANGE_US * ESC_FORWARD_LIMIT_PERCENT / 100;
const int ESC_REVERSE_LIMIT_RANGE_US = ESC_RANGE_US * ESC_REVERSE_LIMIT_PERCENT / 100;

const int STEERING_INPUT_MIN = 0;
const int STEERING_INPUT_MAX = 255;
const int STEERING_INPUT_CENTER = 127;
const int DEADZONE = 12;
const float SIDE_WALL_SLOW_DISTANCE_CM = 80.0;
const float SIDE_WALL_STOP_DISTANCE_CM = 20.0;
const int SIDE_WALL_STEERING_CORRECTION_US = 150;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long TELEMETRY_INTERVAL_MS = 50;
const uint8_t SERIAL_LINE_BUFFER_SIZE = 8;

char serialLine[SERIAL_LINE_BUFFER_SIZE];
uint8_t serialLineLength = 0;
bool serialLineOverflow = false;

int jetsonSteeringValue = STEERING_INPUT_CENTER;
bool hasJetsonSteeringCommand = false;
bool useJetsonSteering = false;
unsigned long lastJetsonSteeringCommandMs = 0;
unsigned long lastTelemetryMs = 0;

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
                                *numberEnd == '\0';
          for (char *digit = serialLine + 1; digit < numberEnd; digit++) {
            if (*digit < '0' || *digit > '9') {
              steeringDigits = false;
              break;
            }
          }

          if (steeringDigits &&
              steeringValue >= STEERING_INPUT_MIN &&
              steeringValue <= STEERING_INPUT_MAX) {
            jetsonSteeringValue = steeringValue;
            hasJetsonSteeringCommand = true;
            lastJetsonSteeringCommandMs = millis();
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
    if (PS4.getButtonClick(TRIANGLE)) {
      useJetsonSteering = !useJetsonSteering;
    }

    controllerSteeringValue = PS4.getAnalogHat(LeftHatX);
    r2Val = PS4.getAnalogButton(R2);
    l2Val = PS4.getAnalogButton(L2);

    if (r2Val > 15) {
      int baseEscUs = map(
        r2Val, 0, 255,
        ESC_STOP_US, ESC_STOP_US - ESC_FORWARD_LIMIT_RANGE_US);
      float frontDist = distanceCm[0];
      float leftDist = distanceCm[1];
      float rightDist = distanceCm[2];

      float frontScale = 1.0;
      if (frontDist <= 20.0) {
        frontScale = 0.0;
      } else if (frontDist < 100.0) {
        if (frontDist >= 50.0) {
          frontScale = 0.5 + 0.5 * (frontDist - 50.0) / 50.0;
        } else {
          frontScale = 0.5 * (frontDist - 20.0) / 30.0;
        }
      }

      float closestSideDist = min(leftDist, rightDist);
      float sideScale = 1.0;
      if (closestSideDist <= SIDE_WALL_STOP_DISTANCE_CM) {
        sideScale = 0.0;
      } else if (closestSideDist < SIDE_WALL_SLOW_DISTANCE_CM) {
        sideScale = (closestSideDist - SIDE_WALL_STOP_DISTANCE_CM) /
                    (SIDE_WALL_SLOW_DISTANCE_CM - SIDE_WALL_STOP_DISTANCE_CM);
      }

      int forwardDiff = ESC_STOP_US - baseEscUs;
      escUs = ESC_STOP_US - (int)(forwardDiff * frontScale * sideScale);
    } else if (l2Val > 15) {
      escUs = map(
        l2Val, 0, 255,
        ESC_STOP_US, ESC_STOP_US + ESC_REVERSE_LIMIT_RANGE_US);
    }
  }

  unsigned long now = millis();
  bool jetsonCommandFresh = hasJetsonSteeringCommand &&
    now - lastJetsonSteeringCommandMs <= COMMAND_TIMEOUT_MS;
  bool steeringInputFresh = useJetsonSteering
    ? jetsonCommandFresh
    : PS4.connected();
  int steeringUs = STEERING_CENTER_US;

  if (useJetsonSteering) {
    if (jetsonCommandFresh) {
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

  // 前進中はSonar 1・2で壁から離れる方向へステアリングを補正する。
  if (steeringInputFresh && escUs < ESC_STOP_US) {
    float leftDist = distanceCm[1];  // Sonar 1 (左前45度)
    float rightDist = distanceCm[2]; // Sonar 2 (右前45度)
    float closestSideDist = min(leftDist, rightDist);

    if (closestSideDist < SIDE_WALL_SLOW_DISTANCE_CM &&
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

  // 前進指令は前方・左右のソナー距離に応じて減速または停止する。
  steeringServo.writeMicroseconds(steeringUs);
  escServo.writeMicroseconds(escUs);

  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS &&
      Serial.availableForWrite() >= 20) {
    lastTelemetryMs = now;

    int steeringOutputValue;
    if (steeringUs <= STEERING_CENTER_US) {
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
    Serial.println(constrain(throttleOutputValue, -255, 255));
  }
}
