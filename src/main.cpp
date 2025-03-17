#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h> // GPSパース用ライブラリ

//-----------------------------------------
// ピン定義
//-----------------------------------------
const int auxPin = 17; // LoRaモジュールのAUXピン
const int m0Pin = 9;   // LoRaモジュールのM0ピン
const int m1Pin = 3;   // LoRaモジュールのM1ピン
const int ledPin = 47; // LED
const int GPSPin = 1;  // GPS電源制御用ピン

// UART設定
// LoRa用UART：UART1（RX=18, TX=8）
HardwareSerial mySerial(1);
// GPS用UART：UART2（RX=48, TX=45）
HardwareSerial mySerial2(2);

// GPSパーサーオブジェクト
TinyGPSPlus gps;

// GPSデータ格納用
struct GPSData_t
{
  double latitude;
  double longitude;
  bool isValid;
};
volatile GPSData_t gGPSData = {0.0, 0.0, false};

// ミューテックス（mutexとは複数のタスクが同時にアクセスするのを防ぐための仕組み）
SemaphoreHandle_t xGPSMutex = NULL;
SemaphoreHandle_t xSerialMutex = NULL;

//-----------------------------------------
// 関数プロトタイプ
//-----------------------------------------
void enterConfigurationMode();
void exitConfigurationMode();
uint8_t calculateChecksum(const uint8_t *data, size_t length);
void sendCommand(const uint8_t *command, size_t len);
void configureLoRaModule();
void Addaddressandchannel();
void vGPSReceiverTask(void *pvParameters);
void vGPSTask(void *pvParameters);

//-----------------------------------------
// LoRaモジュール設定用関数群
//-----------------------------------------
void enterConfigurationMode()
{
  digitalWrite(m0Pin, HIGH);
  digitalWrite(m1Pin, HIGH);
  delay(1000);
}

void exitConfigurationMode()
{
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  delay(1000);
}

uint8_t calculateChecksum(const uint8_t *data, size_t length)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++)
  {
    sum += data[i];
  }
  return sum & 0xFF;
}

void sendCommand(const uint8_t *command, size_t len)
{
  mySerial.write(command, len);
  delay(10);
  Serial.print("Send Command: ");
  for (size_t i = 0; i < len; i++)
  {
    Serial.printf("0x%02X ", command[i]);
  }
  Serial.println();
  mySerial.flush();
}

void configureLoRaModule()
{
  enterConfigurationMode();
  uint8_t command[] = {
      0xC0, // 書き込みコマンドヘッダ
      0x00, // 開始レジスタアドレス
      0x06, // 書き込み数（6バイト）
      0x00, // アドレス上位バイト
      0x03, // アドレス下位バイト
      0x62, // UART Serial Port Rate 9600, Air Data Rate 62500 bps
      0xC1, // ペイロード長等の設定
      0x00, // チャンネル設定 0
      0xC0  // RSSI有効化, 固定送信モード
  };
  // チェックサムを計算して最後のバイトに設定
  command[9] = calculateChecksum(command, sizeof(command) - 1);
  sendCommand(command, sizeof(command));
  Serial.println("Configuration command sent.");
  delay(100);
  uint8_t response[10] = {0};
  size_t responseIndex = 0;
  while (mySerial.available() && responseIndex < sizeof(response))
  {
    response[responseIndex++] = mySerial.read();
  }
  Serial.print("Response: ");
  for (size_t i = 0; i < responseIndex; i++)
  {
    Serial.printf("0x%02X ", response[i]);
  }
  Serial.println();
  exitConfigurationMode();
}

// 送信パケット先頭に固定のアドレス＆チャンネル情報を付加
void Addaddressandchannel()
{
  uint8_t addressHigh = 0x00;
  uint8_t addressLow = 0x03;
  uint8_t channel = 0x00;
  mySerial.write(addressHigh);
  mySerial.write(addressLow);
  mySerial.write(channel);
}

// GPS受信専用タスク（10ms周期）
void vGPSReceiverTask(void *pvParameters)
{
  for (;;)
  {
    while (mySerial2.available() > 0)
    {
      char c = mySerial2.read();
      gps.encode(c);
    }

    // 最新のGPSデータをグローバル変数に更新（ミューテックス保護）
    if (xSemaphoreTake(xGPSMutex, portMAX_DELAY) == pdTRUE)
    {
      gGPSData.latitude = gps.location.lat();
      gGPSData.longitude = gps.location.lng();
      gGPSData.isValid = gps.location.isValid();
      xSemaphoreGive(xGPSMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms周期
  }
}

// GPS位置情報送信タスク（100ms周期）
void vGPSTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  for (;;)
  {
    double latitude, longitude;
    bool isGPSValid;

    // グローバル変数から最新のGPSデータを取得（ミューテックス保護）
    if (xSemaphoreTake(xGPSMutex, portMAX_DELAY) == pdTRUE)
    {
      latitude = gGPSData.latitude;
      longitude = gGPSData.longitude;
      isGPSValid = gGPSData.isValid;
      xSemaphoreGive(xGPSMutex);
    }

    // ASCII形式でGPSデータを送信
    if (isGPSValid)
    {
      // ASCII形式で緯度経度を文字列に変換（小数点以下6桁）
      char gpsBuffer[50];
      snprintf(gpsBuffer, sizeof(gpsBuffer), "%.6f,%.6f", latitude, longitude);

      if (xSerialMutex != NULL)
      {
        xSemaphoreTake(xSerialMutex, portMAX_DELAY);
        // アドレスとチャンネルを先に送信
        Addaddressandchannel();
        // ASCII形式の位置情報を送信
        mySerial.write(gpsBuffer, strlen(gpsBuffer));
        mySerial.flush();
        xSemaphoreGive(xSerialMutex);
      }

      // デバッグ出力
      Serial.print("Sending GPS data: ");
      Serial.println(gpsBuffer);
    }
    else
    {
      Serial.println("GPS data not valid. Nothing sent.");
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//====================================================
// setup() 関数
//====================================================
void setup()
{
  Serial.begin(9600);
  Serial.println("GPS Sender System starting...");

  // ピン初期設定
  pinMode(auxPin, INPUT);
  pinMode(m0Pin, OUTPUT);
  pinMode(m1Pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(GPSPin, OUTPUT);
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(GPSPin, HIGH); // GPS電源をON

  // UART初期化
  mySerial.begin(9600, SERIAL_8N1, 18, 8);   // LoRa用UART
  mySerial2.begin(9600, SERIAL_8N1, 48, 45); // GPS用UART

  // LoRaモジュール設定
  configureLoRaModule();

  // GPS用ミューテックスの作成
  xGPSMutex = xSemaphoreCreateMutex();
  if (xGPSMutex == NULL)
  {
    Serial.println("[ERROR] xGPSMutex の作成に失敗");
    while (1)
      ;
  }

  // シリアル用ミューテックスの作成
  xSerialMutex = xSemaphoreCreateMutex();
  if (xSerialMutex == NULL)
  {
    Serial.println("[ERROR] xSerialMutex の作成に失敗");
    while (1)
      ;
  }

  // FreeRTOSタスクの作成
  xTaskCreate(vGPSReceiverTask, "GPSReceiverTask", 4096, nullptr, 3, nullptr);
  xTaskCreate(vGPSTask, "GPSTask", 8192, nullptr, 2, nullptr);

  Serial.println("Setup complete. GPS data will be sent every 100ms.");
}

//====================================================
void loop()
{
  // FreeRTOSタスクで処理するため、loop内では何もしない
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(ledPin, !digitalRead(ledPin)); // 動作確認用LED点滅
}