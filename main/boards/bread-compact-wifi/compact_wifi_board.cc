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
#include <esp_task_wdt.h>

#define TAG "CompactWifiBoard"

// RC522 引脚定义 - 使用SPI通信
#define RC522_PIN_RST     GPIO_NUM_11  // 复位引脚
#define RC522_PIN_CS      GPIO_NUM_8   // 片选信号
#define RC522_PIN_MOSI    GPIO_NUM_9   // 主机输出，从机输入
#define RC522_PIN_MISO    GPIO_NUM_10  // 主机输入，从机输出
#define RC522_PIN_SCK     GPIO_NUM_12  // 时钟信号
#define RC522_PIN_IRQ     GPIO_NUM_NC  // 中断信号，暂不使用

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
    std::string first_card_id_;  // 存储第一张卡的ID
    bool first_card_registered_; // 标记是否已注册第一张卡

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
        // 初始化新增的成员变量
        first_card_registered_ = false;
        first_card_id_ = "";

        // 设置RC522日志级别为DEBUG
        esp_log_level_set("RC522", ESP_LOG_DEBUG);
        
        ESP_LOGI(TAG, "Initializing RC522 with RST:%d, CS:%d, MOSI:%d, MISO:%d, SCK:%d, IRQ:%d", 
                 RC522_PIN_RST, RC522_PIN_CS, RC522_PIN_MOSI, RC522_PIN_MISO, RC522_PIN_SCK, RC522_PIN_IRQ);
        
        // 创建RC522设备，使用SPI通信
        rc522_ = new Rc522Device(RC522_PIN_RST, RC522_PIN_CS, RC522_PIN_MOSI, RC522_PIN_MISO, RC522_PIN_SCK, RC522_PIN_IRQ);
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
            int retry_count = 0;
            int detection_interval = 1000;  // 增加到1000ms，降低扫描频率
            bool last_card_state = false;
            int init_retry_count = 0;
            
            ESP_LOGI(TAG, "RC522 card detection task started");
            
            ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
            ESP_ERROR_CHECK(esp_task_wdt_reset());
            
            if (board->GetDisplay()) {
                board->GetDisplay()->ShowNotification("RC522 Ready");
            }
            
            if (!board->rc522_->Initialize()) {
                ESP_LOGE(TAG, "Initial RC522 initialization failed, will retry");
                if (board->GetDisplay()) {
                    board->GetDisplay()->ShowNotification("RC522 Init Failed");
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (!board->rc522_->Initialize()) {
                    ESP_LOGE(TAG, "Second RC522 initialization failed");
                    if (board->GetDisplay()) {
                        board->GetDisplay()->ShowNotification("RC522 Failed");
                    }
                    init_retry_count = 5;
                }
            }
            
            while (true) {
                ESP_ERROR_CHECK(esp_task_wdt_reset());
                
                if (init_retry_count > 0) {
                    init_retry_count--;
                    if (init_retry_count == 0) {
                        ESP_LOGI(TAG, "Retrying RC522 initialization");
                        if (board->rc522_->Initialize()) {
                            ESP_LOGI(TAG, "RC522 initialization succeeded after retry");
                            if (board->GetDisplay()) {
                                board->GetDisplay()->ShowNotification("RC522 Ready");
                            }
                        } else {
                            ESP_LOGE(TAG, "RC522 initialization failed after retry");
                            if (board->GetDisplay()) {
                                board->GetDisplay()->ShowNotification("RC522 Failed");
                            }
                            init_retry_count = 10;
                        }
                    }
                    ESP_ERROR_CHECK(esp_task_wdt_reset());
                    vTaskDelay(pdMS_TO_TICKS(detection_interval));
                    continue;
                }
                
                // 使用try-catch风格的错误处理
                bool card_detected = false;
                try {
                    card_detected = board->rc522_->DetectCard(&card_type);
                } catch (...) {
                    ESP_LOGE(TAG, "Exception during card detection, reinitializing RC522");
                    // 重新初始化RC522
                    board->rc522_->Initialize();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    ESP_ERROR_CHECK(esp_task_wdt_reset());
                    continue;
                }
                
                ESP_ERROR_CHECK(esp_task_wdt_reset());
                
                if (card_detected && !last_card_state) {
                    ESP_LOGI(TAG, "Card detected! Type: 0x%02X", card_type);
                    retry_count = 0;
                    
                    if (board->GetDisplay()) {
                        board->GetDisplay()->ShowNotification("Card Detected");
                    }
                    
                    bool read_success = false;
                    try {
                        read_success = board->rc522_->ReadCardSerial(serial_no);
                    } catch (...) {
                        ESP_LOGE(TAG, "Exception during card serial reading");
                        read_success = false;
                    }
                    
                    if (read_success) {
                        // 将卡片ID转换为字符串
                        char card_id[16];
                        snprintf(card_id, sizeof(card_id), "%02X%02X%02X%02X%02X",
                                serial_no[0], serial_no[1], serial_no[2],
                                serial_no[3], serial_no[4]);
                        std::string current_card_id = card_id;
                        
                        ESP_LOGI(TAG, "Read card ID: %s", current_card_id.c_str());
                        
                        // 检查是否是第一张卡
                        if (!board->first_card_registered_) {
                            // 记录第一张卡的ID
                            board->first_card_registered_ = true;
                            board->first_card_id_ = current_card_id;
                            ESP_LOGI(TAG, "First card registered: %s, sending wake word: 你好可莉", current_card_id.c_str());
                            
                            // 发送"你好可莉"
                            if (board->GetDisplay()) {
                                board->GetDisplay()->ShowNotification("你好可莉");
                            }
                            // 触发唤醒词
                            Application::GetInstance().WakeWordInvoke("你好可莉");
                            ESP_LOGI(TAG, "Wake word sent: 你好可莉");
                        } else if (current_card_id != board->first_card_id_) {
                            ESP_LOGI(TAG, "Different card detected: %s (first card was: %s), sending wake word: 你好派蒙", 
                                    current_card_id.c_str(), board->first_card_id_.c_str());
                            
                            // 如果是不同的卡，发送"你好派蒙"
                            if (board->GetDisplay()) {
                                board->GetDisplay()->ShowNotification("你好派蒙");
                            }
                            // 触发唤醒词
                            Application::GetInstance().WakeWordInvoke("你好派蒙");
                            ESP_LOGI(TAG, "Wake word sent: 你好派蒙");
                        } else {
                            ESP_LOGI(TAG, "Same card detected: %s, no wake word sent", current_card_id.c_str());
                        }
                        
                        // 显示卡片信息
                        char display_text[64];
                        snprintf(display_text, sizeof(display_text),
                                "Card: %02X:%02X:%02X:%02X:%02X",
                                serial_no[0], serial_no[1], serial_no[2],
                                serial_no[3], serial_no[4]);
                        
                        if (board->GetDisplay()) {
                            board->GetDisplay()->ShowNotification(display_text);
                        }
                        
                        // 增加检测间隔，避免重复读取
                        vTaskDelay(pdMS_TO_TICKS(2000));  // 增加到2秒
                    } else {
                        ESP_LOGW(TAG, "Failed to read card serial number");
                    }
                } else if (!card_detected && last_card_state) {
                    ESP_LOGI(TAG, "Card removed");
                    if (board->GetDisplay()) {
                        board->GetDisplay()->ShowNotification("Card Removed");
                    }
                }
                
                last_card_state = card_detected;
                ESP_ERROR_CHECK(esp_task_wdt_reset());
                vTaskDelay(pdMS_TO_TICKS(detection_interval));
            }
        }, "rc522_task", 4096, this, 2, nullptr);
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