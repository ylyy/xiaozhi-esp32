#include "ble_shutter.h"
#include <cstring>

#define BLE_SCAN_DURATION 10  // 扫描持续时间（秒）
#define GATTC_APP_ID 0x55     // GATTC应用ID
#define BLE_EVENT_CONNECT_BIT BIT0
#define BLE_EVENT_DISCONNECT_BIT BIT1
#define BLE_EVENT_SCAN_DONE_BIT BIT2

// 定义AB Shutter3的名称前缀，用于过滤设备
#define AB_SHUTTER_NAME_PREFIX "AB Shutter"

// 静态成员初始化
BleShutter* BleShutter::instance_ = nullptr;

// 获取单例实例
BleShutter& BleShutter::GetInstance() {
    if (instance_ == nullptr) {
        instance_ = new BleShutter();
    }
    return *instance_;
}

// 构造函数
BleShutter::BleShutter() 
    : gattc_if_(ESP_GATT_IF_NONE), 
      conn_id_(0), 
      shutter_handle_(0), 
      is_connected_(false), 
      button_state_(SHUTTER_BUTTON_UNKNOWN) {
    
    // 创建事件组
    event_group_ = xEventGroupCreate();
    
    // 初始化远程设备地址
    memset(remote_bda_, 0, sizeof(esp_bd_addr_t));
}

// 析构函数
BleShutter::~BleShutter() {
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
    }
}

// 初始化BLE
bool BleShutter::Initialize() {
    ESP_LOGI(BLE_SHUTTER_TAG, "Initializing BLE...");
    
    // 释放蓝牙控制器内存
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to release BT controller memory: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 启用蓝牙控制器
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 初始化蓝牙协议栈
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 启用蓝牙协议栈
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 注册GAP回调
    ret = esp_ble_gap_register_callback(GapEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 注册GATTC回调
    ret = esp_ble_gattc_register_callback(GattcEventHandler);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 注册GATTC应用
    ret = esp_ble_gattc_app_register(GATTC_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to register GATTC app: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 设置扫描参数
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to set scan params: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGI(BLE_SHUTTER_TAG, "BLE initialized successfully");
    return true;
}

// 开始扫描AB Shutter3设备
bool BleShutter::StartScan() {
    ESP_LOGI(BLE_SHUTTER_TAG, "Starting BLE scan...");
    
    // 清除扫描完成事件标志
    xEventGroupClearBits(event_group_, BLE_EVENT_SCAN_DONE_BIT);
    
    // 开始扫描
    esp_err_t ret = esp_ble_gap_start_scanning(BLE_SCAN_DURATION);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to start scanning: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 等待扫描完成
    EventBits_t bits = xEventGroupWaitBits(
        event_group_,
        BLE_EVENT_SCAN_DONE_BIT,
        pdTRUE,
        pdTRUE,
        pdMS_TO_TICKS(BLE_SCAN_DURATION * 1000 + 1000));  // 额外等待1秒
    
    return (bits & BLE_EVENT_SCAN_DONE_BIT) != 0;
}

// 停止扫描
void BleShutter::StopScan() {
    ESP_LOGI(BLE_SHUTTER_TAG, "Stopping BLE scan...");
    esp_ble_gap_stop_scanning();
}

// 连接到指定的AB Shutter3设备
bool BleShutter::Connect(const std::string& device_addr) {
    ESP_LOGI(BLE_SHUTTER_TAG, "Connecting to device: %s", device_addr.c_str());
    
    // 解析设备地址
    uint8_t addr[6];
    sscanf(device_addr.c_str(), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
           &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
    
    // 清除连接事件标志
    xEventGroupClearBits(event_group_, BLE_EVENT_CONNECT_BIT);
    
    // 连接设备
    esp_err_t ret = esp_ble_gattc_open(gattc_if_, addr, BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SHUTTER_TAG, "Failed to connect to device: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 等待连接完成
    EventBits_t bits = xEventGroupWaitBits(
        event_group_,
        BLE_EVENT_CONNECT_BIT,
        pdTRUE,
        pdTRUE,
        pdMS_TO_TICKS(10000));  // 等待10秒
    
    return (bits & BLE_EVENT_CONNECT_BIT) != 0;
}

// 断开连接
void BleShutter::Disconnect() {
    if (is_connected_) {
        ESP_LOGI(BLE_SHUTTER_TAG, "Disconnecting...");
        
        // 清除断开连接事件标志
        xEventGroupClearBits(event_group_, BLE_EVENT_DISCONNECT_BIT);
        
        // 断开连接
        esp_ble_gattc_close(gattc_if_, conn_id_);
        
        // 等待断开连接完成
        xEventGroupWaitBits(
            event_group_,
            BLE_EVENT_DISCONNECT_BIT,
            pdTRUE,
            pdTRUE,
            pdMS_TO_TICKS(5000));  // 等待5秒
    }
}

// 设置按键状态回调
void BleShutter::SetButtonCallback(shutter_callback_t callback) {
    button_callback_ = callback;
}

// 获取当前连接状态
bool BleShutter::IsConnected() const {
    return is_connected_;
}

// 获取当前按键状态
shutter_button_state_t BleShutter::GetButtonState() const {
    return button_state_;
}

// 处理GAP事件
void BleShutter::GapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    BleShutter& instance = GetInstance();
    
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(BLE_SHUTTER_TAG, "Scan param set complete");
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_SHUTTER_TAG, "Scan start failed: %d", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(BLE_SHUTTER_TAG, "Scan started successfully");
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // 获取设备名称
                uint8_t* adv_name = NULL;
                uint8_t adv_name_len = 0;
                adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                
                if (adv_name != NULL) {
                    std::string device_name((char*)adv_name, adv_name_len);
                    
                    // 检查是否是AB Shutter设备
                    if (device_name.find(AB_SHUTTER_NAME_PREFIX) != std::string::npos) {
                        char bda_str[18];
                        sprintf(bda_str, "%02x:%02x:%02x:%02x:%02x:%02x",
                                param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                                param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
                        
                        ESP_LOGI(BLE_SHUTTER_TAG, "Found AB Shutter device: %s, RSSI: %d", bda_str, param->scan_rst.rssi);
                        
                        // 保存设备地址
                        memcpy(instance.remote_bda_, param->scan_rst.bda, sizeof(esp_bd_addr_t));
                    }
                }
            } else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
                ESP_LOGI(BLE_SHUTTER_TAG, "Scan completed");
                xEventGroupSetBits(instance.event_group_, BLE_EVENT_SCAN_DONE_BIT);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_SHUTTER_TAG, "Scan stop failed: %d", param->scan_stop_cmpl.status);
            } else {
                ESP_LOGI(BLE_SHUTTER_TAG, "Scan stopped successfully");
                xEventGroupSetBits(instance.event_group_, BLE_EVENT_SCAN_DONE_BIT);
            }
            break;
            
        default:
            break;
    }
}

// 处理GATTC事件
void BleShutter::GattcEventHandler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) {
    BleShutter& instance = GetInstance();
    
    // 保存GATT接口
    if (event == ESP_GATTC_REG_EVT && param->reg.status == ESP_GATT_OK && param->reg.app_id == GATTC_APP_ID) {
        instance.gattc_if_ = gattc_if;
    }
    
    // 处理事件
    switch (event) {
        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(BLE_SHUTTER_TAG, "GATTC connected to device");
            instance.conn_id_ = param->connect.conn_id;
            memcpy(instance.remote_bda_, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
            break;
            
        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(BLE_SHUTTER_TAG, "GATTC disconnected from device");
            instance.is_connected_ = false;
            instance.button_state_ = SHUTTER_BUTTON_UNKNOWN;
            xEventGroupSetBits(instance.event_group_, BLE_EVENT_DISCONNECT_BIT);
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT:
            // 查找HID服务
            if (param->search_res.srvc_id.uuid.uuid.uuid16 == AB_SHUTTER_SERVICE_UUID) {
                ESP_LOGI(BLE_SHUTTER_TAG, "Found HID service");
                
                // 创建UUID用于查找HID报告特征
                esp_bt_uuid_t char_uuid;
                char_uuid.len = ESP_UUID_LEN_16;
                char_uuid.uuid.uuid16 = AB_SHUTTER_CHAR_UUID;
                
                // 查找特征 - 使用正确的参数调用
                esp_gattc_char_elem_t char_elem_result;
                uint16_t count = 1;
                esp_gatt_status_t status = esp_ble_gattc_get_all_char(
                    gattc_if,
                    param->search_res.conn_id,
                    param->search_res.start_handle,
                    param->search_res.end_handle,
                    &char_elem_result,
                    &count,
                    0
                );
                
                if (status == ESP_GATT_OK && count > 0) {
                    for (int i = 0; i < count; i++) {
                        if (char_elem_result.uuid.uuid.uuid16 == AB_SHUTTER_CHAR_UUID) {
                            ESP_LOGI(BLE_SHUTTER_TAG, "Found HID report characteristic");
                            instance.shutter_handle_ = char_elem_result.char_handle;
                            
                            // 注册通知
                            esp_ble_gattc_register_for_notify(gattc_if, instance.remote_bda_, char_elem_result.char_handle);
                            break;
                        }
                    }
                } else {
                    ESP_LOGE(BLE_SHUTTER_TAG, "Failed to get characteristics: %d", status);
                }
            }
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(BLE_SHUTTER_TAG, "GATT search completed");
            break;
            
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            ESP_LOGI(BLE_SHUTTER_TAG, "Registered for notification");
            instance.is_connected_ = true;
            xEventGroupSetBits(instance.event_group_, BLE_EVENT_CONNECT_BIT);
            break;
            
        case ESP_GATTC_NOTIFY_EVT:
            // 处理通知数据
            if (param->notify.handle == instance.shutter_handle_) {
                instance.button_state_ = instance.ParseButtonData(param->notify.value, param->notify.value_len);
                
                // 调用回调函数
                if (instance.button_callback_) {
                    instance.button_callback_(instance.button_state_);
                }
                
                ESP_LOGI(BLE_SHUTTER_TAG, "Button state: %d", instance.button_state_);
            }
            break;
            
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status == ESP_GATT_OK) {
                ESP_LOGI(BLE_SHUTTER_TAG, "GATTC open success");
                
                // 开始搜索服务
                esp_ble_gattc_search_service(gattc_if, param->open.conn_id, NULL);
            } else {
                ESP_LOGE(BLE_SHUTTER_TAG, "GATTC open failed: %d", param->open.status);
            }
            break;
            
        default:
            break;
    }
}

// 解析按键数据
shutter_button_state_t BleShutter::ParseButtonData(uint8_t* data, uint16_t length) {
    if (data == nullptr || length == 0) {
        return SHUTTER_BUTTON_UNKNOWN;
    }
    
    // AB Shutter3通常发送的数据格式为：
    // 按下按钮时: 0x01
    // 释放按钮时: 0x00
    if (length >= 1) {
        if (data[0] == 0x01) {
            return SHUTTER_BUTTON_PRESSED;
        } else if (data[0] == 0x00) {
            return SHUTTER_BUTTON_RELEASED;
        }
    }
    
    return SHUTTER_BUTTON_UNKNOWN;
} 