#ifndef LoRa_H
#define LoRa_H

#include <Arduino.h>

//----------------------------------------------------------
// ピン定義
//----------------------------------------------------------
extern const int m0Pin; // M0ピン番号
extern const int m1Pin; // M1ピン番号
extern const int ledPin; // LEDピン番号

//----------------------------------------------------------
// 定数定義 (アドレス、チャンネル)
//----------------------------------------------------------
#define DEVICE_ADDRESS_HIGH  (0x00)
#define DEVICE_ADDRESS_LOW   (0x03)
#define CHANNEL_NUMBER       (0x00)

//----------------------------------------------------------
// typedef enum定義
//----------------------------------------------------------

// UARTシリアルボーレート設定 (REG0 のbit7-5)
typedef enum {
    UART_SERIAL_1200    = (0x00 << 5), // 000b
    UART_SERIAL_2400    = (0x01 << 5), // 001b
    UART_SERIAL_4800    = (0x02 << 5), // 010b
    UART_SERIAL_9600    = (0x03 << 5), // 011b (default)
    UART_SERIAL_19200   = (0x04 << 5), // 100b
    UART_SERIAL_38400   = (0x05 << 5), // 101b
    UART_SERIAL_57600   = (0x06 << 5), // 110b
    UART_SERIAL_115200  = (0x07 << 5)  // 111b
} UARTSerialRate_t;

// Air Data Rate設定 (REG0 のbit4-0)
typedef enum {
    // BW=125kHz系
    AIR_DATA_RATE_15625_BPS_125KHZ = 0x00, // 00000b:15,625bps SF=5  BW=125kHz
    AIR_DATA_RATE_9375_BPS_125KHZ  = 0x04, // 00100b:9,375bps SF=6  BW=125kHz
    AIR_DATA_RATE_5469_BPS_125KHZ  = 0x08, // 01000b:5,469bps SF=7  BW=125kHz
    AIR_DATA_RATE_3125_BPS_125KHZ  = 0x0C, // 01100b:3,125bps SF=8  BW=125kHz
    AIR_DATA_RATE_1758_BPS_125KHZ  = 0x10, // 10000b:1,758bps(default) SF=9  BW=125kHz

    // BW=250kHz系
    AIR_DATA_RATE_31250_BPS_250KHZ = 0x01, // 00001b:31,250bps SF=5  BW=250kHz
    AIR_DATA_RATE_18750_BPS_250KHZ = 0x05, // 00101b:18,750bps SF=6  BW=250kHz
    AIR_DATA_RATE_10938_BPS_250KHZ = 0x09, // 01001b:10,938bps SF=7  BW=250kHz
    AIR_DATA_RATE_6250_BPS_250KHZ  = 0x0D, // 01101b:6,250bps SF=8  BW=250kHz
    AIR_DATA_RATE_3516_BPS_250KHZ  = 0x11, // 10001b:3,516bps SF=9  BW=250kHz
    AIR_DATA_RATE_1953_BPS_250KHZ  = 0x15, // 10101b:1,953bps SF=10 BW=250kHz

    // BW=500kHz系
    AIR_DATA_RATE_62500_BPS_500KHZ = 0x02, // 00010b:62,500bps SF=5  BW=500kHz
    AIR_DATA_RATE_37500_BPS_500KHZ = 0x06, // 00110b:37,500bps SF=6  BW=500kHz
    AIR_DATA_RATE_21875_BPS_500KHZ = 0x0A, // 01010b:21,875bps SF=7  BW=500kHz
    AIR_DATA_RATE_12500_BPS_500KHZ = 0x0E, // 01110b:12,500bps SF=8  BW=500kHz
    AIR_DATA_RATE_7031_BPS_500KHZ  = 0x12, // 10010b:7,031bps SF=9  BW=500kHz
    AIR_DATA_RATE_3906_BPS_500KHZ  = 0x16, // 10110b:3,906bps SF=10 BW=500kHz
    AIR_DATA_RATE_2148_BPS_500KHZ  = 0x1A  // 11010b:2,148bps SF=11 BW=500kHz
} AirDataRate_t;

// ペイロード長 (REG1 のbit7-6)
typedef enum {
    PAYLOAD_LEN_200_BYTE = (0x00 << 6), // 00b (default)
    PAYLOAD_LEN_128_BYTE = (0x01 << 6), // 01b
    PAYLOAD_LEN_64_BYTE  = (0x02 << 6), // 10b
    PAYLOAD_LEN_32_BYTE  = (0x03 << 6)  // 11b
} PayloadLength_t;


// RSSI環境ノイズ有効化 (REG1 のbit5)
typedef enum {
    RSSI_ENV_NOISE_DISABLE = (0x00 << 5), // 0: 無効 (default)
    RSSI_ENV_NOISE_ENABLE  = (0x01 << 5)  // 1: 有効
} RssiEnvEnable_t;

// 出力パワー設定 (REG1 のbit1-0)
typedef enum {
    TX_POWER_NOT_AVAILABLE = 0x00, // 00b: Not available
    TX_POWER_13DBM         = 0x01, // 01b: 13dBm (default)
    TX_POWER_7DBM          = 0x02, // 10b: 7dBm
    TX_POWER_0DBM          = 0x03  // 11b: 0dBm
} TransmitPower_t;

// RSSIバイト有効化 (REG3 のbit7)
typedef enum {
    RSSI_BYTE_DISABLE = (0x00 << 7), // 0: RSSIバイト無効 (default)
    RSSI_BYTE_ENABLE  = (0x01 << 7)  // 1: RSSIバイト有効
} RssiByteEnable_t;

// 送信モード (REG3 のbit6)
typedef enum {
    TRANSMISSION_MODE_TRANSPARENT = (0x00 << 6), // 0: トランスペアレント送信 (default)
    TRANSMISSION_MODE_FIXED       = (0x01 << 6)  // 1: Fixed-block送信
} TransmissionMode_t;

// WORサイクル (REG3 のbit2-0)
typedef enum {
    WOR_CYCLE_500MS  = 0x00, // 000b
    WOR_CYCLE_1000MS = 0x01, // 001b
    WOR_CYCLE_1500MS = 0x02, // 010b
    WOR_CYCLE_2000MS = 0x03, // 011b (default)
    WOR_CYCLE_2500MS = 0x04, // 100b
    WOR_CYCLE_3000MS = 0x05  // 101b
} WorCycle_t;


//----------------------------------------------------------
// 関数プロトタイプ
//----------------------------------------------------------
void enterConfigurationMode();
void exitConfigurationMode();
uint8_t calculateChecksum(const uint8_t* data, size_t length);
void sendCommand(const uint8_t* cmd, size_t len);
void configureLoRaModule();
void Addaddressandchannel();

#endif // LoRa_H
