#ifndef _BLE_SHUTTER_H_
#define _BLE_SHUTTER_H_

#include <esp_log.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatt_common_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <functional>
#include <string>

#define BLE_SHUTTER_TAG "BLE_SHUTTER"

// AB Shutter3的服务UUID和特征UUID
#define AB_SHUTTER_SERVICE_UUID      0x1812  // HID服务
#define AB_SHUTTER_CHAR_UUID         0x2A4D  // HID报告特征

// 按键状态定义
typedef enum {
    SHUTTER_BUTTON_RELEASED = 0,
    SHUTTER_BUTTON_PRESSED = 1,
    SHUTTER_BUTTON_UNKNOWN = 2
} shutter_button_state_t;

// 回调函数类型定义
typedef std::function<void(shutter_button_state_t)> shutter_callback_t;

class BleShutter {
public:
    // 单例模式获取实例
    static BleShutter& GetInstance();

    // 初始化BLE
    bool Initialize();

    // 开始扫描AB Shutter3设备
    bool StartScan();

    // 停止扫描
    void StopScan();

    // 连接到指定的AB Shutter3设备
    bool Connect(const std::string& device_addr);

    // 断开连接
    void Disconnect();

    // 设置按键状态回调
    void SetButtonCallback(shutter_callback_t callback);

    // 获取当前连接状态
    bool IsConnected() const;

    // 获取当前按键状态
    shutter_button_state_t GetButtonState() const;

private:
    BleShutter();
    ~BleShutter();

    // 禁止拷贝和赋值
    BleShutter(const BleShutter&) = delete;
    BleShutter& operator=(const BleShutter&) = delete;

    // 处理GAP事件
    static void GapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    
    // 处理GATTC事件
    static void GattcEventHandler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);

    // 解析按键数据
    shutter_button_state_t ParseButtonData(uint8_t* data, uint16_t length);

    // 成员变量
    static BleShutter* instance_;
    esp_gatt_if_t gattc_if_;
    uint16_t conn_id_;
    esp_bd_addr_t remote_bda_;
    uint16_t shutter_handle_;
    bool is_connected_;
    shutter_button_state_t button_state_;
    shutter_callback_t button_callback_;
    EventGroupHandle_t event_group_;
};

#endif // _BLE_SHUTTER_H_ 