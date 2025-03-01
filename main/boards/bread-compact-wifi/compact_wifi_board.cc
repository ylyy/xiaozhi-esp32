#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/ssd1306_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "csi/csi_detector.h"
#include "rc522_device.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>

#define TAG "CompactWifiBoard"

// RC522 引脚定义 - 使用软件 SPI
#define RC522_PIN_MOSI    GPIO_NUM_11
#define RC522_PIN_MISO    GPIO_NUM_12

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    Button boot_button_;
    Button touch_button_;
    Button tw_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    CsiDetector csi_detector_;
    Rc522Device* rc522_;

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeRc522() {
        // 创建RC522设备，使用软件 SPI
        rc522_ = new Rc522Device(RC522_PIN_MOSI, RC522_PIN_MISO);
        if (!rc522_->Initialize()) {
            ESP_LOGE(TAG, "Failed to initialize RC522");
            return;
        }
        ESP_LOGI(TAG, "RC522 initialized successfully");

        // 启动RFID卡片检测任务
        xTaskCreate([](void* arg) {
            auto board = static_cast<CompactWifiBoard*>(arg);
            uint8_t card_type;
            uint8_t serial_no[5];
            
            while (true) {
                if (board->rc522_->DetectCard(&card_type)) {
                    if (board->rc522_->ReadCardSerial(serial_no)) {
                        ESP_LOGI(TAG, "Card detected! Type: 0x%02X", card_type);
                        ESP_LOGI(TAG, "Serial: %02X:%02X:%02X:%02X:%02X",
                                serial_no[0], serial_no[1], serial_no[2],
                                serial_no[3], serial_no[4]);
                        
                        // 显示卡片信息
                        char display_text[64];
                        snprintf(display_text, sizeof(display_text),
                                "Card: %02X:%02X:%02X:%02X:%02X",
                                serial_no[0], serial_no[1], serial_no[2],
                                serial_no[3], serial_no[4]);
                        board->GetDisplay()->ShowNotification(display_text);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(100));  // 100ms检测间隔
            }
        }, "rc522_task", 4096, this, 5, nullptr);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

        tw_button_.OnClick([this]() {
            Application::GetInstance().ToggleChatState();
        });

        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification("音量 " + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification("最大音量");
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification("音量 " + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification("已静音");
        });
    }

    void InitializeCsiDetector() {
        // 等待WiFi初始化完成后再初始化CSI
        auto& wifi = WifiStation::GetInstance();
        wifi.OnConnected([this](const std::string& ssid) {
            ESP_LOGI(TAG, "WiFi connected to %s, initializing CSI detector", ssid.c_str());
            csi_detector_.Initialize();
            csi_detector_.OnObjectDetected([this]() {
                ESP_LOGI(TAG, "CSI detected object, toggling chat state");
                auto& app = Application::GetInstance();
                app.Schedule([&app]() {
                    app.ToggleChatState();
                });
            });
            csi_detector_.StartDetection();
            ESP_LOGI(TAG, "CSI detection started");
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        tw_button_(TW_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeRc522();
        InitializeButtons();
        InitializeCsiDetector();
        InitializeIot();
    }

    ~CompactWifiBoard() {
        if (rc522_) {
            delete rc522_;
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        static Ssd1306Display display(display_i2c_bus_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                    &font_puhui_14_1, &font_awesome_14_1);
        return &display;
    }
};

DECLARE_BOARD(CompactWifiBoard);