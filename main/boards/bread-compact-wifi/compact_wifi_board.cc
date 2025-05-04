#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "system_info.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/event_groups.h>
#include <esp_smartconfig.h>
#include <esp_wifi.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>
#include <wifi_configuration_ap.h>

#define TAG "CompactWifiBoard"

// 定义事件组位
#define SMARTCONFIG_CONNECTED_BIT BIT0
#define SMARTCONFIG_DONE_BIT BIT1

// 创建事件组句柄
static EventGroupHandle_t s_wifi_event_group;

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

// UDP发送设备ID相关定义
#define UDP_PORT 18266
#define MAX_RETRY 3

// 发送UDP包含设备ID到配网APP
static void SendDeviceIdUdp(const char* ip_addr, const char* device_id) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ip_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "无法创建socket: %d", sock);
        return;
    }

    // 准备发送的数据包，包含设备ID
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "{\"deviceId\":\"%s\"}", device_id);
    
    ESP_LOGI(TAG, "发送设备ID到 %s: %s", ip_addr, buffer);
    
    // 尝试发送，失败重试几次
    for (int i = 0; i < MAX_RETRY; i++) {
        int err = sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "发送UDP数据包失败: %d", err);
        } else {
            ESP_LOGI(TAG, "UDP数据包发送成功");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待短暂时间后重试
    }
    
    close(sock);
}

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    Button tw_wakeup_button_;
    bool smartconfig_running_ = false;
    
    // 存储最后获取的SSID信息和当前连接的IP
    char last_connected_ip[16] = {0};
    char phone_ip[16] = {0};  // 存储手机IP地址，用于UDP回传
    
    // 注册SmartConfig事件处理函数
    static void SmartConfigEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        CompactWifiBoard* board = static_cast<CompactWifiBoard*>(arg);
        if (event_base == SC_EVENT) {
            switch (event_id) {
                case SC_EVENT_SCAN_DONE:
                    ESP_LOGI(TAG, "SC_EVENT_SCAN_DONE: 扫描完成");
                    board->GetDisplay()->ShowNotification("WiFi扫描完成，正在搜索配网信号...");
                    break;
                case SC_EVENT_FOUND_CHANNEL:
                    ESP_LOGI(TAG, "SC_EVENT_FOUND_CHANNEL: 已找到信道");
                    board->GetDisplay()->ShowNotification("已找到配网信道，等待接收WiFi信息...");
                    break;
                case SC_EVENT_GOT_SSID_PSWD: {
                    ESP_LOGI(TAG, "SC_EVENT_GOT_SSID_PSWD: 已获取SSID和密码");
                    
                    smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
                    
                    // 获取SSID和密码
                    uint8_t ssid[33] = { 0 };
                    uint8_t password[65] = { 0 };
                    memcpy(ssid, evt->ssid, sizeof(evt->ssid));
                    memcpy(password, evt->password, sizeof(evt->password));
                    
                    // 保存手机端IP地址，用于UDP回传
                    memcpy(board->phone_ip, evt->cellphone_ip, sizeof(evt->cellphone_ip));
                    ESP_LOGI(TAG, "手机IP: %s", board->phone_ip);
                    
                    // 打印SSID和密码信息
                    ESP_LOGI(TAG, "SSID: %s", ssid);
                    ESP_LOGI(TAG, "PASSWORD: %s", password);
                    
                    // 显示获取到的SSID
                    std::string notification = "已获取WiFi信息: ";
                    notification += (char*)ssid;
                    board->GetDisplay()->ShowNotification(notification.c_str());
                    
                    // 如果有设备ID，发送到配网手机
                    if (board->phone_ip[0] != '\0') {
                        std::string mac_address = SystemInfo::GetMacAddress();
                        if (!mac_address.empty()) {
                            SendDeviceIdUdp(board->phone_ip, mac_address.c_str());
                        }
                    }
                    
                    // 创建延迟重启任务
                    xTaskCreate([](void *) {
                        ESP_LOGI(TAG, "设备将在3秒后重启...");
                        vTaskDelay(pdMS_TO_TICKS(3000));
                        esp_restart();
                    }, "restart_task", 4096, NULL, 5, NULL);
                    
                    break;
                }
                case SC_EVENT_SEND_ACK_DONE:
                    ESP_LOGI(TAG, "SC_EVENT_SEND_ACK_DONE: 发送ACK完成");
                    board->GetDisplay()->ShowNotification("SmartConfig配网完成!");
                    board->smartconfig_running_ = false;
                    break;
            }
        }
    }

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

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
            {&font_puhui_14_1, &font_awesome_14_1});
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            else 
            {
            app.ToggleChatState();
            }
        });

        // 长按Boot按钮进入SmartConfig配网模式
        boot_button_.OnLongPress([this]() {
            StartSmartConfig();
        });

        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
        
        tw_wakeup_button_.OnPressDown([this]() {
            Application::GetInstance().WakeWordInvoke("こんにちは");
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
    }

    // 启动SmartConfig配网
    void StartSmartConfig() {
        if (smartconfig_running_) {
            ESP_LOGI(TAG, "SmartConfig已经在运行中");
            return;
        }
        
        smartconfig_running_ = true;
        GetDisplay()->ShowNotification("正在启动SmartConfig配网...");
        
        // 确保之前的WiFi连接已经断开
        auto& wifi_station = WifiStation::GetInstance();
        if (wifi_station.IsConnected()) {
            wifi_station.Stop();
            vTaskDelay(pdMS_TO_TICKS(500)); // 等待WiFi停止
        }
        
        // 注册SmartConfig事件处理函数
        ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &SmartConfigEventHandler, this));
        
        // 启动SmartConfig
        auto& wifi_config_ap = WifiConfigurationAp::GetInstance();
        wifi_config_ap.StartSmartConfig();
        
        GetDisplay()->ShowNotification("请打开手机App进行配网操作...");
    }

public:
    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO),
        tw_wakeup_button_(TW_WAKEUP_GPIO, false) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeIot();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        //static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
        //    AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        static NoAudioCodecSimplexPdm audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, GPIO_NUM_2, GPIO_NUM_3);
            
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    // 重写StartNetwork方法来检查是否需要启动SmartConfig
    virtual void StartNetwork() override {
        // 首先调用父类方法
        WifiBoard::StartNetwork();
        
        // 如果WiFi没有连接成功，可以自动启动SmartConfig配网
        auto& wifi_station = WifiStation::GetInstance();
        if (!wifi_station.IsConnected()) {
            GetDisplay()->ShowNotification("WiFi未连接，长按BOOT按钮进行SmartConfig配网");
        }
    }
};

DECLARE_BOARD(CompactWifiBoard);
