const int trigPin = 2;
const int echoPin[] = {3,5,6,9};

int i = 0;
int stop_counter = 0;
double duration[] = {0,0,0,0};
double distance[] = {0,0,0,0};

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  for(i = 0; i < 4; i++) pinMode(echoPin[i], INPUT);
}

void loop() {
  while(stop_counter < 100) {
      // 1つずつのセンサを順番に処理していく
    for(int i = 0; i < 4; i++) {
    
      // トリガーピンを一度LOWにする
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
    
     // 超音波パルスを発射（10usで十分です）
     digitalWrite(trigPin, HIGH); 
     delayMicroseconds(10);  
     digitalWrite(trigPin, LOW); 

      // 発射直後に、対応するエコーピンの信号をすぐに聴き取る
     // 第3引数に「20000us（20ms）」を指定して、空振りの時の1秒フリーズを防ぐ
     duration[i] = pulseIn(echoPin[i], HIGH, 20000);  
    
      // 距離計算（音速 340m/s = 0.034cm/us）
     distance[i] = duration[i] * 0.034 / 2;

     // センサ同士の「音の混信」を防ぐため、次のセンサに移る前に少しだけ待つ
     delay(10); 
   }

    // 4つすべての計測が終わったら、まとめてシリアルモニタに表示
  for(int i = 0; i < 4; i++) {
    Serial.print("distance_");
    Serial.print(i + 1);
    Serial.print(" : ");
    Serial.print(distance[i]);
    Serial.print(" cm\n");
  }
  Serial.println("--------------------"); // 区切り線

  // 全体のループ速度（ここを小さくするほどログが早く流れます）
  delay(100); 
  stop_counter++;
  }
}
