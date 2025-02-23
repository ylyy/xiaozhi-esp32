#include "csi_detector.h"
#include <esp_log.h>
#include <math.h>
#include <esp_wifi.h>
#include "esp_wifi_types.h"

#define TAG "CsiDetector"
#define DETECTION_RUNNING_EVENT BIT0

CsiDetector::CsiDetector() : is_running_(false) {
    event_group_ = xEventGroupCreate();
}

CsiDetector::~CsiDetector() {
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

void CsiDetector::Initialize() {
    // 配置CSI
    wifi_csi_config_t csi_config = {0};  // 先全部初始化为0
    csi_config.channel_filter_en = true;
    csi_config.htltf_en = true;
    csi_config.lltf_en = true;
    csi_config.ltf_merge_en = true;
    csi_config.manu_scale = false;
    csi_config.shift = false;
    csi_config.stbc_htltf2_en = true;
    
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(CsiCallback, this));
    
    ESP_LOGI(TAG, "CSI detector initialized");
}

void CsiDetector::StartDetection() {
    if (!is_running_) {
        ESP_ERROR_CHECK(esp_wifi_set_csi(true));
        xEventGroupSetBits(event_group_, DETECTION_RUNNING_EVENT);
        is_running_ = true;
        ESP_LOGI(TAG, "CSI detection started");
    }
}

void CsiDetector::StopDetection() {
    if (is_running_) {
        ESP_ERROR_CHECK(esp_wifi_set_csi(false));
        xEventGroupClearBits(event_group_, DETECTION_RUNNING_EVENT);
        is_running_ = false;
        ESP_LOGI(TAG, "CSI detection stopped");
    }
}

bool CsiDetector::IsDetectionRunning() {
    return (xEventGroupGetBits(event_group_) & DETECTION_RUNNING_EVENT);
}

void CsiDetector::OnObjectDetected(std::function<void()> callback) {
    object_detected_callback_ = callback;
}

void CsiDetector::CsiCallback(void* ctx, wifi_csi_info_t* data) {
    CsiDetector* detector = static_cast<CsiDetector*>(ctx);
    detector->ProcessCsiData(data);
}

void CsiDetector::ProcessCsiData(wifi_csi_info_t* data) {
    if (!data || !data->buf || data->len <= 0) {
        return;
    }

    ESP_LOGD(TAG, "CSI data received: len=%d, first_word_invalid=%d", 
             data->len, data->first_word_invalid);

    if (DetectObject(data)) {
        ESP_LOGI(TAG, "Object detected!");
        if (object_detected_callback_) {
            object_detected_callback_();
        }
    }
}

bool CsiDetector::DetectObject(wifi_csi_info_t* data) {
    static const int WINDOW_SIZE = 3;  // 减小窗口大小以提高响应速度
    static float magnitudes[WINDOW_SIZE] = {0};
    static int window_index = 0;
    static bool window_filled = false;
    
    // 计算当前CSI幅度
    float current_magnitude = 0;
    int8_t* buf = data->buf;
    int len = data->len;
    
    // 每个子载波的CSI包含实部和虚部
    for (int i = 0; i < len; i += 2) {
        float real = buf[i];
        float imag = buf[i + 1];
        current_magnitude += sqrt(real * real + imag * imag);
    }
    current_magnitude /= (len / 2);
    
    ESP_LOGD(TAG, "Current magnitude: %.2f", current_magnitude);
    
    // 更新滑动窗口
    magnitudes[window_index] = current_magnitude;
    window_index = (window_index + 1) % WINDOW_SIZE;
    if (window_index == 0) {
        window_filled = true;
    }
    
    if (!window_filled) {
        return false;
    }
    
    // 计算平均幅度和标准差
    float avg_magnitude = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        avg_magnitude += magnitudes[i];
    }
    avg_magnitude /= WINDOW_SIZE;
    
    float variance = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float diff = magnitudes[i] - avg_magnitude;
        variance += diff * diff;
    }
    variance /= WINDOW_SIZE;
    float std_dev = sqrt(variance);
    
    // 降低检测阈值，提高灵敏度
    float diff = fabs(current_magnitude - avg_magnitude);
    if (diff > 1.0 * std_dev && std_dev > 0.03) {  // 进一步降低阈值
        ESP_LOGI(TAG, "Movement detected: magnitude=%.2f, avg=%.2f, std_dev=%.2f", 
                 current_magnitude, avg_magnitude, std_dev);
        return true;
    }
    
    return false;
} 