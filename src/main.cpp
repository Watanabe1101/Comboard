#include <Arduino.h>
#include <HardwareSerial.h>
#include "CanComm.hpp"       // CAN通信クラス（定義済み）
#include "LoRa.h"            // LoRa設定用定数など（LoRa.h内に定義済み）
#include <TinyGPS++.h>       // GPSパース用ライブラリ
#include <SD_MMC.h>          // SDカード用ライブラリ（ESP32用SD_MMC）
#include <Preferences.h>     // Flash（NVS）への状態保存用ライブラリ
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// FreeRTOS関連ヘッダ
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"



//====================================================
// ピン定義
// -----------------------------------------------------
const int auxPin = 17;    // LoRaモジュールのAUXピン
const int m0Pin  = 9;     // LoRaモジュールのM0ピン
const int m1Pin  = 3;     // LoRaモジュールのM1ピン
const int ledPin = 47;    // LED
const int GPSPin = 1;     // GPS電源制御用ピン

// SDカード用ピン（SD_MMC.setPins用）
int clk = 14, cmd = 15, d0 = 2, d1 = 4, d2 = 12, d3 = 13;

// CAN通信用ピン（ESP32-S3用、gpio_num_t型）
#define CAN_RX_PIN GPIO_NUM_5
#define CAN_TX_PIN GPIO_NUM_6

// 自基板のID
#define SELF_BOARD_ID 0x001

//====================================================
// UARTインスタンスの生成
// LoRa用UART：UART1（RX=18, TX=8）
HardwareSerial mySerial(1);
// GPS用UART：UART2（RX=48, TX=45）
HardwareSerial mySerial2(2);

//====================================================
// 構造体定義：LoRa受信コマンドとRSSIをひとまとめに
typedef struct {
  uint8_t cmd;   // 受信コマンド
  uint8_t rssi;  // RSSI値
} LoraCommand_t;

// ********** TelemetryData構造体 **********
// GPSとクォータニオン(4要素, int16_t[4])をまとめる
typedef struct {
  double latitude;
  double longitude;
  bool   gpsValid;
  int16_t quat[4]; // -32768 ~ 32767 → -1.0 ~ 1.0
  ModeCommand mode;
} TelemetryData;

// Telemetryキュー（送信・記録専用）
QueueHandle_t telemetryQueue = nullptr;

// グローバル変数とミューテックスによる保護対象の構造体
struct GPSData_t {
  double latitude;
  double longitude;
  float altitude;
  bool isValid;
};
volatile GPSData_t gGPSData = {0.0, 0.0, false};
SemaphoreHandle_t xGPSMutex = NULL;

//====================================================
// グローバル変数
volatile ModeCommand currentMode = ModeCommand::START; // 現在の動作モード
ModeCommand lastSavedMode = ModeCommand::START;        // Flashに保存された直前のモード
CanComm can(BoardID::COM, CAN_TX_PIN, CAN_RX_PIN, BoardID::UNKNOWN, BoardID::UNKNOWN);
TinyGPSPlus gps;                                      // GPSパース用
uint8_t latestQuaternion[8] = {0};                    // クォータニオン（8バイト）
bool newQuaternionAvailable = false;                  // 新規クォータニオン受信フラグ
volatile uint8_t prepCounter = 0;                     // PREPARATIONモード用カウンタ
const char* filename = "/gps_log.txt";                // SDログ保存ファイルパス
Preferences preferences;                              // Flash保存用

// LoRa受信用キュー（LoraCommand_t型）
QueueHandle_t loraQueue = NULL; // キューサイズ：10件

// クォータニオンデータ用キュー（最大10件保持、各要素は int16_t[4]）
QueueHandle_t quaternionQueue = NULL;

// シリアル送信用ミューテックス（mySerialへの同時アクセスを防止）
SemaphoreHandle_t xSerialMutex = NULL;

//------------------------------------------------------
// 以下、ステータス集約用グローバル変数
//------------------------------------------------------
#define NUM_STATUS 5  // [0]: COM（現在のモード）, [1]: PARA, [2]: POWER, [3]: GIMBAL, [4]: CAMERA

// 既存の aggregatedStatus, received 配列を利用
char aggregatedStatus[NUM_STATUS];
bool received[NUM_STATUS] = {false, false, false, false, false}; // 0番は常に現在モード

// 新たに、集約処理用ミューテックスとタイマーを追加
SemaphoreHandle_t xStatusMutex = NULL;
TimerHandle_t xStatusTimer = NULL;

//------------------------------------------------------
// LoRa受信コールバック関数（mySerial.onReceive用）
// ※ 受信バッファに2バイト以上あれば、先頭1バイトをコマンド、2バイト目をRSSIとして
//    1つの構造体にまとめ、キューに送信します。
void onLoraReceive() {
  while (mySerial.available() >= 2) {
    LoraCommand_t loraCmd;
    int lenCmd = mySerial.readBytes(&loraCmd.cmd, 1);
    int lenRSSI = mySerial.readBytes(&loraCmd.rssi, 1);
    
    if (lenCmd == 1 && lenRSSI == 1) {
      Serial.print("[DEBUG] 受信コマンド: ");
      Serial.print((char)loraCmd.cmd);
      Serial.print(", RSSI: ");
      Serial.println((int)loraCmd.rssi);

      if (loraQueue != NULL) {
        xQueueSend(loraQueue, &loraCmd, 0);
      }
    }
  }
}

//------------------------------------------------------
// LoRa関連関数
//------------------------------------------------------

// 設定モードへ移行する関数
void enterConfigurationMode() {
  digitalWrite(m0Pin, HIGH);
  digitalWrite(m1Pin, HIGH);
  delay(1000); // 設定モード移行のための待機
}

// 通常動作モードへ戻す関数
void exitConfigurationMode() {
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  delay(1000);
}

// 受信データのチェックサム計算
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum & 0xFF;
}

// LoRaモジュールへコマンド送信
void sendCommand(const uint8_t *command, size_t len) {
  mySerial.write(command, len);
  delay(10);
  Serial.print("Send Command: ");
  for (size_t i = 0; i < len; i++) {
    Serial.printf("0x%02X ", command[i]);
  }
  Serial.println();
  mySerial.flush();
}

// LoRaモジュール初期設定
void configureLoRaModule() {
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
  while (mySerial.available() && responseIndex < sizeof(response)) {
    response[responseIndex++] = mySerial.read();
  }
  Serial.print("Response: ");
  for (size_t i = 0; i < responseIndex; i++) {
    Serial.printf("0x%02X ", response[i]);
  }
  Serial.println();
  exitConfigurationMode();
}

// 送信パケット先頭に固定のアドレス＆チャンネル情報を付加
void Addaddressandchannel(){
  uint8_t addressHigh = 0x00;
  uint8_t addressLow = 0x03;
  uint8_t channel = 0x00;
  mySerial.write(addressHigh);
  mySerial.write(addressLow);
  mySerial.write(channel);
}

//------------------------------------------------------
// SDカードログ追記
void SD_AppendLog(const char* text) {
    Serial.println(text);
    Serial.println("Opening file for append...");
    
    File file = SD_MMC.open(filename, FILE_APPEND);
    if (!file) {
        Serial.println("[ERROR] SD_MMC.open() 失敗 - ファイルを開けませんでした。");
        return;
    }

    size_t writtenBytes = file.print(text);
    file.flush();
    file.close();

    if (writtenBytes != strlen(text)) {
        Serial.print("[ERROR] SDカードに書き込んだバイト数が不足: ");
        Serial.print(writtenBytes);
        Serial.print(" / ");
        Serial.println(strlen(text));
    }
}

// SDカードへのログ保存（GPS + クォータニオンデータ）
void saveToSDCard(double latitude, double longitude, uint8_t* quaternion) {
    char logLine[96];

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
        snprintf(logLine, sizeof(logLine), 
                 "%04d/%02d/%02d %02d:%02d:%02d, Lat=%.6f, Lon=%.6f, Q=0x%02X%02X%02X%02X\n",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second(),
                 latitude, longitude,
                 quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    } 
    else {
        snprintf(logLine, sizeof(logLine), 
                 "Time N/A, Lat=%.6f, Lon=%.6f, Q=0x%02X%02X%02X%02X\n",
                 latitude, longitude,
                 quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    }

    delay(1);
    SD_AppendLog(logLine);
}

//------------------------------------------------------
// LoRa送信（集約済みステータスを送信）
//------------------------------------------------------
void sendLoRaStatus() {
    Serial.print("[DEBUG] LoRa送信データ: ");
    for (int i = 0; i < NUM_STATUS; i++) {
        Serial.printf("0x%02X ", (uint8_t)aggregatedStatus[i]);
    }
    Serial.println();
  
    if (xSerialMutex != NULL) {
        xSemaphoreTake(xSerialMutex, portMAX_DELAY);
        Addaddressandchannel();
        mySerial.write(reinterpret_cast<uint8_t*>(aggregatedStatus), NUM_STATUS);
        xSemaphoreGive(xSerialMutex);
    }
    Serial.println("[DEBUG] 基板ステータスのLoRa返信完了");
}

//------------------------------------------------------
// 新規関数：タイマーコールバック（タイムアウト時に集約結果を送信）
//------------------------------------------------------
void vStatusTimerCallback(TimerHandle_t xTimer) {
    if (xStatusMutex != NULL) {
        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
    }
    Serial.println("[DEBUG] タイムアウトにより集約結果を送信");
    sendLoRaStatus();
    // 次回用に集約結果をリセット（COMは currentMode で更新されるので、1～4のみ）
    for (int i = 1; i < NUM_STATUS; i++) {
        aggregatedStatus[i] = '-';
        received[i] = false;
    }
    if (xStatusMutex != NULL) {
        xSemaphoreGive(xStatusMutex);
    }
}

//------------------------------------------------------
// FreeRTOSタスク：CAN受信処理タスク（10ms周期）
//------------------------------------------------------
void vGeneralTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        CanRxFrame rxFrame;
        if (can.readFrameNoWait(rxFrame) == ESP_OK) {
            Serial.print("[DEBUG] CAN RX: ContentID = ");
            Serial.print((int)rxFrame.content_id);
            Serial.print(", DLC = ");
            Serial.println(rxFrame.dlc);

            if (rxFrame.content_id == ContentID::QUATERNION && rxFrame.dlc == 8) {
                int16_t rawQuat[4];
                memcpy(rawQuat, rxFrame.data, sizeof(rawQuat));
                // 最新のクォータニオンをキューに送信（非ブロッキング）
                if (quaternionQueue != NULL) {
                    xQueueSend(quaternionQueue, rawQuat, 0);
                }

                Serial.print("[DEBUG] Quaternion Received & Stored in Queue: ");
                for (int i = 0; i < 4; i++) {
                    Serial.print(rawQuat[i] / 32768.0f, 6);
                    Serial.print(" ");
                }
                Serial.println();
            }
            else if (rxFrame.content_id == ContentID::VOLTAGE && rxFrame.dlc == 8) {
                float voltages[2];
                memcpy(voltages, rxFrame.data, sizeof(voltages));
                Serial.print("[DEBUG] Voltage CAN frame受信: Voltage1 = ");
                Serial.print(voltages[0], 2);
                Serial.print(" V, Voltage2 = ");
                Serial.print(voltages[1], 2);
                Serial.println(" V");

                // シリアル送信処理をミューテックスで保護
                if (xSerialMutex != NULL) {
                    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
                    Addaddressandchannel();
                    mySerial.write(rxFrame.data, 8);
                    xSemaphoreGive(xSerialMutex);
                }
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(5));
                digitalWrite(ledPin, LOW);
            }
            // ---- ここが修正対象：BOARD_STATE の集約処理 ----
            else if (rxFrame.content_id == ContentID::BOARD_STATE && rxFrame.dlc == 1) {
                Serial.println("[DEBUG] CANから BOARD_STATE を受信、応答を待機します。");

                // 集約用変数へのアクセスを保護
                if (xStatusMutex != NULL) {
                    xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                }

                // 先頭は常に現在のモード
                aggregatedStatus[0] = static_cast<char>(currentMode);

                BoardID senderID = rxFrame.sender_board_id;
                char statusChar = static_cast<char>(rxFrame.data[0]);

                switch (senderID) {
                    case BoardID::PARA:
                        aggregatedStatus[1] = statusChar;
                        received[1] = true;
                        break;
                    case BoardID::POWER:
                        aggregatedStatus[2] = statusChar;
                        received[2] = true;
                        break;
                    case BoardID::GIMBAL:
                        aggregatedStatus[3] = statusChar;
                        received[3] = true;
                        break;
                    case BoardID::CAMERA:
                        aggregatedStatus[4] = statusChar;
                        received[4] = true;
                        break;
                    default:
                        break;
                }

                Serial.print("[DEBUG] 更新されたステータス: COM=");
                Serial.print(aggregatedStatus[0]);
                Serial.print(" PARA=");
                Serial.print(aggregatedStatus[1]);
                Serial.print(" POWER=");
                Serial.print(aggregatedStatus[2]);
                Serial.print(" GIMBAL=");
                Serial.print(aggregatedStatus[3]);
                Serial.print(" CAMERA=");
                Serial.println(aggregatedStatus[4]);

                // 全ての基板からの応答が揃ったかチェック
                bool allReceived = received[1] && received[2] && received[3] && received[4];

                if (allReceived) {
                    // 応答が全て揃っている場合、タイマーを停止して即座に送信
                    if (xStatusTimer != NULL) {
                        xTimerStop(xStatusTimer, 0);
                    }
                    if (xStatusMutex != NULL) {
                        xSemaphoreGive(xStatusMutex);
                    }
                    sendLoRaStatus();
                    // 次回用にリセット
                    if (xStatusMutex != NULL) {
                        xSemaphoreTake(xStatusMutex, portMAX_DELAY);
                    }
                    for (int i = 1; i < NUM_STATUS; i++) {
                        aggregatedStatus[i] = '-';
                        received[i] = false;
                    }
                    if (xStatusMutex != NULL) {
                        xSemaphoreGive(xStatusMutex);
                    }
                }
                else {
                    // 応答が足りない場合、タイマーを起動してタイムアウト後に送信
                    if (xStatusTimer != NULL) {
                        xTimerStart(xStatusTimer, 0);
                    }
                    if (xStatusMutex != NULL) {
                        xSemaphoreGive(xStatusMutex);
                    }
                }
            }
            // ---------------------------------------------------
            else {
                if (xSerialMutex != NULL) {
                    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
                    Addaddressandchannel();
                    mySerial.write(rxFrame.data, rxFrame.dlc);
                    xSemaphoreGive(xSerialMutex);
                }
                Serial.println("[DEBUG] その他 CAN frame送信");
                digitalWrite(ledPin, HIGH);
                vTaskDelay(pdMS_TO_TICKS(5));
                digitalWrite(ledPin, LOW);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ★ GPS受信専用タスク（10ms周期） ★
void vGPSReceiverTask(void* pvParameters) {
    // 10ms周期でシリアル受信バッファを処理する
    for (;;) {
        while (mySerial2.available() > 0) {
            char c = mySerial2.read();
            gps.encode(c);
        }
        // 最新のGPSデータをグローバル変数に更新（ミューテックス保護）
        if (xSemaphoreTake(xGPSMutex, portMAX_DELAY) == pdTRUE) {
            gGPSData.latitude = gps.location.lat();
            gGPSData.longitude = gps.location.lng();
            gGPSData.isValid   = gps.location.isValid();
            xSemaphoreGive(xGPSMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms周期
    }
}

// ----------------------------------------------------
//  LoRa送信・SDログ処理タスク（100ms周期） 
void vGPSTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  for (;;) {
      double latitude, longitude;
      float altitude;
      bool isGPSValid;

      // グローバル変数から最新のGPSデータを取得（ミューテックス保護）
      if (xSemaphoreTake(xGPSMutex, portMAX_DELAY) == pdTRUE) {
          latitude  = gGPSData.latitude;
          longitude = gGPSData.longitude;
          altitude  = static_cast<float>(gGPSData.altitude);
          isGPSValid = gGPSData.isValid;
          xSemaphoreGive(xGPSMutex);
      }
      
      // クォータニオンの取得（CANタスク側でキューに最新値を送っている）
      int16_t rawQuat[4] = {0, 0, 0, 0};
      if (quaternionQueue != NULL) {
          xQueueReceive(quaternionQueue, rawQuat, 0);
      }
      
      // LoRa送信用パケットを作成
      if (currentMode == ModeCommand::LOGGING) {
          uint8_t txBuffer[28]; // 32 → 28 に変更（floatにしたため）
          if (isGPSValid) {
              memcpy(txBuffer, &latitude, sizeof(latitude));
              memcpy(txBuffer + sizeof(latitude), &longitude, sizeof(longitude));
              memcpy(txBuffer + sizeof(latitude) + sizeof(longitude), &altitude, sizeof(altitude));
          } else {
              double defaultValue = 0.0;
              float defaultAltitude = 0.0f;
              memcpy(txBuffer, &defaultValue, sizeof(defaultValue));
              memcpy(txBuffer + sizeof(defaultValue), &defaultValue, sizeof(defaultValue));
              memcpy(txBuffer + sizeof(defaultValue) * 2, &defaultAltitude, sizeof(defaultAltitude));
          }
          // txBuffer[20～27] にクォータニオンデータ（8バイト）を格納
          memcpy(txBuffer + 20, rawQuat, 8);
          
          if (xSerialMutex != NULL) {
              xSemaphoreTake(xSerialMutex, portMAX_DELAY);
              Addaddressandchannel();
              mySerial.write(txBuffer, sizeof(txBuffer));
              xSemaphoreGive(xSerialMutex);
          }
      }
      else if (currentMode == ModeCommand::PREPARATION) {
          prepCounter++;
          if (prepCounter >= 20) {  // 100ms x 20 = 0.5Hz
              uint8_t dataBuffer[28]; 
              if (isGPSValid) {
                  memcpy(dataBuffer, &latitude, sizeof(latitude));
                  memcpy(dataBuffer + sizeof(latitude), &longitude, sizeof(longitude));
                  memcpy(dataBuffer + sizeof(latitude) + sizeof(longitude), &altitude, sizeof(altitude));
              } else {
                  double defaultValue = 0.0;
                  float defaultAltitude = 0.0f;
                  memcpy(dataBuffer, &defaultValue, sizeof(defaultValue));
                  memcpy(dataBuffer + sizeof(defaultValue), &defaultValue, sizeof(defaultValue));
                  memcpy(dataBuffer + sizeof(defaultValue) * 2, &defaultAltitude, sizeof(defaultAltitude));
              }
              memcpy(dataBuffer + 20, rawQuat, 8);
              
              if (xSerialMutex != NULL) {
                  xSemaphoreTake(xSerialMutex, portMAX_DELAY);
                  Addaddressandchannel();
                  mySerial.write(dataBuffer, sizeof(dataBuffer));
                  xSemaphoreGive(xSerialMutex);
              }
              prepCounter = 0;
          }
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


//------------------------------------------------------
// LoRa受信コマンドの処理
void processLoraCommand(uint8_t cmd, uint8_t rssi) {
  Serial.print("[DEBUG] processLoraCommand: コマンド = ");
  Serial.print((char)cmd);
  Serial.print(", RSSI = ");
  Serial.println((int)rssi);

  if (cmd == static_cast<uint8_t>(ModeCommand::START) ||
      cmd == static_cast<uint8_t>(ModeCommand::PREPARATION) ||
      cmd == static_cast<uint8_t>(ModeCommand::LOGGING)) {
    currentMode = static_cast<ModeCommand>(cmd);
    Serial.print("[DEBUG] 動作モード変更: ");
    Serial.println((char)currentMode);
    can.send(ContentID::MODE_TRANSITION, &cmd, 1);
    if (currentMode == ModeCommand::START) {
      digitalWrite(GPSPin, LOW);
    } else {
      digitalWrite(GPSPin, HIGH);
    }
  }
  else if (cmd == static_cast<uint8_t>(ServoCommand::CLOSE_SERVO)  ||
           cmd == static_cast<uint8_t>(ServoCommand::ANGLE_MINUS_1) ||
           cmd == static_cast<uint8_t>(ServoCommand::ANGLE_PLUS_1)  ||
           cmd == static_cast<uint8_t>(ServoCommand::CLOSE_ANGLE_PLUS)||
           cmd == static_cast<uint8_t>(ServoCommand::CLOSE_ANGLE_MINUS)) {
    can.send(ContentID::KAIHOU_COMMAND, &cmd, 1);
    Serial.println("[DEBUG] サーボ指令送信");
  }
  else if (cmd == 'v') {
    can.send(ContentID::VOLTAGE, nullptr, 0);
    Serial.println("[DEBUG] 電圧要求CANリクエスト送信");
  }
  else if (cmd == 'b') {
    Serial.println("[DEBUG] ステータス要求受信");
    esp_err_t err = can.send(ContentID::BOARD_STATE, nullptr, 0);
    if (err == ESP_OK) {
        Serial.println("[DEBUG] 他基板へステータス要求 CAN 送信成功");
    } else {
        Serial.print("[ERROR] CAN 送信失敗: ");
        Serial.println(esp_err_to_name(err));
    }
  }
}

//------------------------------------------------------
// FreeRTOSタスク：LoRa受信キュー処理タスク
void vLoRaTask(void *pvParameters) {
  LoraCommand_t receivedCmd;
  for (;;) {
    if (xQueueReceive(loraQueue, &receivedCmd, portMAX_DELAY) == pdPASS) {
      processLoraCommand(receivedCmd.cmd, receivedCmd.rssi);
    }
  }
}



//====================================================
// setup() 関数
//====================================================
void setup() {
  Serial.begin(9600);
  Serial.println("System starting...");



  // --- ピン初期設定 ---
  pinMode(auxPin, INPUT);
  pinMode(m0Pin, OUTPUT);
  pinMode(m1Pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(GPSPin, OUTPUT);
  digitalWrite(m0Pin, LOW);
  digitalWrite(m1Pin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(GPSPin, LOW);
  
  // --- UART初期化 ---
  mySerial.begin(9600, SERIAL_8N1, 18, 8);   // LoRa用UART
  mySerial2.begin(9600, SERIAL_8N1, 48, 45);  // GPS用UART
  
  // --- LoRaモジュール設定 ---
  configureLoRaModule();
  
  // --- CAN初期化 ---
  if (can.begin() == ESP_OK) {
    Serial.println("CAN driver started.");
  } else {
    Serial.println("CAN driver start FAILED.");
  }
  
  // --- onReceive コールバック登録 ---
  mySerial.onReceive(onLoraReceive);
  
  // --- GPS電源初期設定 ---
  if (currentMode == ModeCommand::START) {
    digitalWrite(GPSPin, LOW);
  } else {
    digitalWrite(GPSPin, HIGH);
  }
  
  // --- SD_MMC初期化 ---
  if (!SD_MMC.setPins(clk, cmd, d0, d1, d2, d3)) {
    Serial.println("SD_MMC setPins failed!");
  }
  Serial.println("SDMMC Initiating...");
  if (SD_MMC.begin("/sdcard", true, false)) {
    Serial.println("SD_MMC init OK!");
  } else {
    Serial.println("SD_MMC begin failed!");
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
  } else {
    Serial.print("SD_MMC Card Type: ");
    if (cardType == CARD_MMC)      Serial.println("MMC");
    else if (cardType == CARD_SD)  Serial.println("SDSC");
    else if (cardType == CARD_SDHC)Serial.println("SDHC");
    else                         Serial.println("UNKNOWN");
  }

  // GPSデータ用ミューテックスの作成
  xGPSMutex = xSemaphoreCreateMutex();
  if (xGPSMutex == NULL) {
      Serial.println("[ERROR] xGPSMutex の作成に失敗");
      while (1);
  }
  // --- ミューテックス、キューの作成 ---
  xSerialMutex = xSemaphoreCreateMutex();
  if (xSerialMutex == NULL) {
      Serial.println("[ERROR] xSerialMutex の作成に失敗");
      while (1);
  }
  // ステータス集約用ミューテックスの作成
  xStatusMutex = xSemaphoreCreateMutex();
  if (xStatusMutex == NULL) {
      Serial.println("[ERROR] xStatusMutex の作成に失敗");
      while (1);
  }

  quaternionQueue = xQueueCreate(10, sizeof(int16_t) * 4);
  if (quaternionQueue == NULL) {
      Serial.println("[ERROR] quaternionQueue の作成に失敗");
      while (1);
  }

  // --- LoRa受信用キューの作成 ---
  loraQueue = xQueueCreate(10, sizeof(LoraCommand_t));
  if (loraQueue == NULL) {
      Serial.println("[ERROR] loraQueue の作成に失敗");
      while (1);
  }
  
  // ステータスタイマーの作成（例：100msタイムアウトのタイマー）
  xStatusTimer = xTimerCreate("StatusTimer", pdMS_TO_TICKS(100), pdFALSE, (void*)0, vStatusTimerCallback);
  if (xStatusTimer == NULL) {
      Serial.println("[ERROR] xStatusTimer の作成に失敗");
      while (1);
  }

  // --- FreeRTOSタスクの作成 ---
  xTaskCreate(vGPSReceiverTask, "GPSReceiverTask", 4096, nullptr, 3, nullptr);
  xTaskCreate(vGeneralTask, "GeneralTask", 8192, nullptr, 2, nullptr);
  xTaskCreate(vGPSTask, "GPSTask", 8192, nullptr, 2, nullptr);
  xTaskCreate(vLoRaTask, "LoRaTask", 2048, nullptr, 1, nullptr);
  
  Serial.println("Setup complete.");
}

//====================================================
void loop() {
    vTaskDelay(pdMS_TO_TICKS(50));
}
