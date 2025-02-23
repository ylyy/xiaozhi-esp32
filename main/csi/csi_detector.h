#ifndef CSI_DETECTOR_H
#define CSI_DETECTOR_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <functional>

class CsiDetector {
public:
    CsiDetector();
    ~CsiDetector();

    void Initialize();
    void StartDetection();
    void StopDetection();
    bool IsDetectionRunning();
    void OnObjectDetected(std::function<void()> callback);

private:
    static void CsiCallback(void* ctx, wifi_csi_info_t* data);
    void ProcessCsiData(wifi_csi_info_t* data);
    bool DetectObject(wifi_csi_info_t* data);
    
    EventGroupHandle_t event_group_;
    std::function<void()> object_detected_callback_;
    bool is_running_;
    static constexpr float DETECTION_THRESHOLD = 50.0f; // 50cm检测阈值
};

#endif 