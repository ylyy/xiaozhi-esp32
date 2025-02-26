#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/ssd1306_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "settings.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>

#define TAG "XminiC3Board"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class XminiC3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    bool press_to_talk_enabled_ = false;
    Button spihd_button_;

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeButtons() {
        ESP_LOGI(TAG, "Initializing buttons...");
        
        boot_button_.OnClick([this]() {
            ESP_LOGI(TAG, "Boot button clicked");
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            if (!press_to_talk_enabled_) {
                app.ToggleChatState();
            }
        });
        boot_button_.OnPressDown([this]() {
            ESP_LOGI(TAG, "Boot button pressed down");
            if (press_to_talk_enabled_) {
                Application::GetInstance().StartListening();
            }
        });
        boot_button_.OnPressUp([this]() {
            ESP_LOGI(TAG, "Boot button released");
            if (press_to_talk_enabled_) {
                Application::GetInstance().StopListening();
            }
        });

        // 添加 SPIHD 按钮的调试日志
        ESP_LOGI(TAG, "Setting up SPIHD button on GPIO %d", SPIHD_BUTTON_GPIO);
        spihd_button_.OnClick([this]() {
            ESP_LOGI(TAG, "SPIHD button clicked");
            auto& app = Application::GetInstance();
            app.Schedule([&app]() {
                app.HandleWakeWordDetected(app.GetWakeWordDetect().GetLastDetectedWakeWord());
            });
        });
        ESP_LOGI(TAG, "Buttons initialization completed");
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        Settings settings("vendor");
        press_to_talk_enabled_ = settings.GetInt("press_to_talk", 0) != 0;

        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("PressToTalk"));
    }

public:
    XminiC3Board() : boot_button_(BOOT_BUTTON_GPIO), spihd_button_(SPIHD_BUTTON_GPIO) {  
        ESP_LOGI(TAG, "Initializing XminiC3Board...");
        // 把 ESP32C3 的 VDD SPI 引脚作为普通 GPIO 口使用
        esp_efuse_write_field_bit(ESP_EFUSE_VDD_SPI_AS_GPIO);

        // 初始化 SPIHD 引脚为输入模式
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << SPIHD_BUTTON_GPIO);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        ESP_LOGI(TAG, "Configuring SPIHD GPIO %d as input with pull-up", SPIHD_BUTTON_GPIO);
        gpio_config(&io_conf);

        InitializeCodecI2c();
        InitializeButtons();
        InitializeIot();
    }

    virtual Led* GetLed() override {
        static SingleLed led_strip(BUILTIN_LED_GPIO);
        return &led_strip;
    }

    virtual Display* GetDisplay() override {
        static Ssd1306Display display(codec_i2c_bus_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                    &font_puhui_14_1, &font_awesome_14_1);
        return &display;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    void SetPressToTalkEnabled(bool enabled) {
        press_to_talk_enabled_ = enabled;

        Settings settings("vendor", true);
        settings.SetInt("press_to_talk", enabled ? 1 : 0);
        ESP_LOGI(TAG, "Press to talk enabled: %d", enabled);
    }

    bool IsPressToTalkEnabled() {
        return press_to_talk_enabled_;
    }
};

DECLARE_BOARD(XminiC3Board);


namespace iot {

class PressToTalk : public Thing {
public:
    PressToTalk() : Thing("PressToTalk", "控制对话模式，一种是长按对话，一种是单击后连续对话。") {
        // 定义设备的属性
        properties_.AddBooleanProperty("enabled", "true 表示长按说话模式，false 表示单击说话模式", []() -> bool {
            auto board = static_cast<XminiC3Board*>(&Board::GetInstance());
            return board->IsPressToTalkEnabled();
        });

        // 定义设备可以被远程执行的指令
        methods_.AddMethod("SetEnabled", "启用或禁用长按说话模式，调用前需要经过用户确认", ParameterList({
            Parameter("enabled", "true 表示长按说话模式，false 表示单击说话模式", kValueTypeBoolean, true)
        }), [](const ParameterList& parameters) {
            bool enabled = parameters["enabled"].boolean();
            auto board = static_cast<XminiC3Board*>(&Board::GetInstance());
            board->SetPressToTalkEnabled(enabled);
        });
    }
};

} // namespace iot

DECLARE_THING(PressToTalk);
