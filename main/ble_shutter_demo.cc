#include "ble_shutter.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>

#define TAG "BLE_SHUTTER_DEMO"

// 按键状态回调函数
void OnButtonStateChanged(shutter_button_state_t state) {
    switch (state) {
        case SHUTTER_BUTTON_PRESSED:
            ESP_LOGI(TAG, "按钮被按下！");
            // 在这里添加您的按钮按下处理代码
            break;
            
        case SHUTTER_BUTTON_RELEASED:
            ESP_LOGI(TAG, "按钮被释放！");
            // 在这里添加您的按钮释放处理代码
            break;
            
        case SHUTTER_BUTTON_UNKNOWN:
            ESP_LOGW(TAG, "未知按钮状态");
            break;
    }
}

// BLE Shutter任务
void BleShutterTask(void* pvParameters) {
    ESP_LOGI(TAG, "BLE Shutter任务启动");
    
    // 获取BleShutter实例
    BleShutter& shutter = BleShutter::GetInstance();
    
    // 初始化BLE
    if (!shutter.Initialize()) {
        ESP_LOGE(TAG, "BLE初始化失败");
        vTaskDelete(NULL);
        return;
    }
    
    // 设置按键状态回调
    shutter.SetButtonCallback(OnButtonStateChanged);
    
    // 开始扫描AB Shutter3设备
    ESP_LOGI(TAG, "开始扫描AB Shutter3设备...");
    if (!shutter.StartScan()) {
        ESP_LOGE(TAG, "扫描失败");
        vTaskDelete(NULL);
        return;
    }
    
    // 等待用户输入设备地址
    ESP_LOGI(TAG, "请输入要连接的AB Shutter3设备地址（例如：11:22:33:44:55:66）：");
    
    // 在实际应用中，您可以从控制台读取用户输入的设备地址
    // 这里我们假设已经找到了设备地址
    std::string device_addr = "11:22:33:44:55:66";  // 替换为实际的设备地址
    
    // 连接到AB Shutter3设备
    ESP_LOGI(TAG, "正在连接到设备：%s", device_addr.c_str());
    if (!shutter.Connect(device_addr)) {
        ESP_LOGE(TAG, "连接失败");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "连接成功！等待按键事件...");
    
    // 主循环
    while (true) {
        // 检查连接状态
        if (!shutter.IsConnected()) {
            ESP_LOGW(TAG, "设备已断开连接，尝试重新连接...");
            
            // 尝试重新连接
            if (!shutter.Connect(device_addr)) {
                ESP_LOGE(TAG, "重新连接失败");
                vTaskDelay(pdMS_TO_TICKS(5000));  // 等待5秒后再次尝试
                continue;
            }
            
            ESP_LOGI(TAG, "重新连接成功！");
        }
        
        // 休眠一段时间
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒
    }
    
    // 任务不应该到达这里
    vTaskDelete(NULL);
}

// 启动BLE Shutter任务
void StartBleShutterDemo() {
    xTaskCreate(BleShutterTask, "ble_shutter_task", 4096, NULL, 5, NULL);
} 