#include "CanComm.hpp"
#include <cstring>

//---------------------------------------
// ID変換
//---------------------------------------
uint16_t CanComm::makeStdId(BoardID board, ContentID content)
{
    uint16_t b = static_cast<uint8_t>(board) & 0x07;
    uint16_t c = static_cast<uint8_t>(content);
    return (b << 8) | c; // 3bit + 8bit = 11bit
}

void CanComm::parseStdId(uint16_t sid, BoardID &bOut, ContentID &cOut)
{
    uint8_t b = (sid >> 8) & 0x07;
    uint8_t c = sid & 0xFF;
    bOut = static_cast<BoardID>(b);
    cOut = static_cast<ContentID>(c);
}

//---------------------------------------
// コンストラクタ
//---------------------------------------
CanComm::CanComm(BoardID self_board_id,
                 int tx_gpio,
                 int rx_gpio,
                 BoardID filter_board1,
                 BoardID filter_board2,
                 uint32_t tx_queue_len,
                 uint32_t rx_queue_len)
    : self_board_id_(self_board_id),
      fb1_(filter_board1),
      fb2_(filter_board2),
      driver_installed_(false)
{
    // config 初期化（Arduino 環境用）
    g_config_ = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_gpio, (gpio_num_t)rx_gpio, TWAI_MODE_NORMAL);
    g_config_.tx_queue_len = tx_queue_len;
    g_config_.rx_queue_len = rx_queue_len;
    g_config_.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_OFF;
    g_config_.intr_flags = ESP_INTR_FLAG_LOWMED;

    t_config_ = TWAI_TIMING_CONFIG_1MBITS();

    f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
}

//---------------------------------------
// ドライバ初期化
//---------------------------------------
esp_err_t CanComm::begin()
{
    if (driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // フィルタ設定を適用
    setupFilter();

    // ドライバインストール
    esp_err_t err = twai_driver_install(&g_config_, &t_config_, &f_config_);
    if (err != ESP_OK)
    {
        return err;
    }

    // TWAIの開始
    err = twai_start();
    if (err == ESP_OK)
    {
        driver_installed_ = true;
    }
    else
    {
        twai_driver_uninstall();
    }
    return err;
}

//---------------------------------------
// メッセージ送信
//---------------------------------------
esp_err_t CanComm::send(ContentID content, const uint8_t *data, size_t len)
{
    if (!driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 8)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t sid = makeStdId(self_board_id_, content);

    twai_message_t tx_msg = {};
    tx_msg.extd = 0; // standard
    tx_msg.rtr = 0;  // data frame
    tx_msg.identifier = sid;
    tx_msg.data_length_code = len;

    if (data && len > 0)
    {
        memcpy(tx_msg.data, data, len);
    }

    return twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
}

//---------------------------------------
// メッセージ受信（非ブロッキング）
//---------------------------------------
esp_err_t CanComm::readFrameNoWait(CanRxFrame &outFrame)
{
    if (!driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, 0);
    if (err != ESP_OK)
    {
        return err;
    }
    if (rx_msg.extd)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }

    parseStdId(rx_msg.identifier, outFrame.sender_board_id, outFrame.content_id);
    outFrame.dlc = rx_msg.data_length_code;
    memcpy(outFrame.data, rx_msg.data, rx_msg.data_length_code);

    for (int i = rx_msg.data_length_code; i < 8; i++)
    {
        outFrame.data[i] = 0;
    }

    return ESP_OK;
}

//---------------------------------------
// バスオフリカバリ
//---------------------------------------
esp_err_t CanComm::recover()
{
    return twai_initiate_recovery();
}

//---------------------------------------
// フィルタ設定
//---------------------------------------
void CanComm::setupFilter()
{
    if (fb1_ == BoardID::UNKNOWN && fb2_ == BoardID::UNKNOWN)
    {
        f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        return;
    }

    if (fb1_ != BoardID::UNKNOWN && fb2_ == BoardID::UNKNOWN)
    {
        f_config_.single_filter = true;
        uint16_t boardSid = (static_cast<uint8_t>(fb1_) & 0x07) << 8;
        uint16_t maskVal = 0x700;

        uint32_t code = (boardSid & 0x7FF) << 21;
        uint32_t mask = (maskVal & 0x7FF) << 21;

        f_config_.acceptance_code = code;
        f_config_.acceptance_mask = mask;
        return;
    }

    if (fb1_ != BoardID::UNKNOWN && fb2_ != BoardID::UNKNOWN)
    {
        f_config_.single_filter = false;

        auto encodeBoard = [&](BoardID b)
        {
            return static_cast<uint16_t>((static_cast<uint8_t>(b) & 0x07) << 8);
        };
        uint16_t sidA = encodeBoard(fb1_);
        uint16_t sidB = encodeBoard(fb2_);

        uint16_t maskVal = 0x700;

        uint32_t codeA = (sidA & 0x7FF) << 19;
        uint32_t codeB = (sidB & 0x7FF) << 3;
        uint32_t code = codeA | codeB;

        uint32_t maskA = (maskVal & 0x7FF) << 19;
        uint32_t maskB = (maskVal & 0x7FF) << 3;
        uint32_t mask = maskA | maskB;

        f_config_.acceptance_code = code;
        f_config_.acceptance_mask = mask;
        return;
    }

    f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
}
