#include "rc522_device.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rom_sys.h>
#include <esp_log.h>

#define TAG "RC522"

// I2C通信延时（微秒）
#define I2C_DELAY_US 100  // 增加到100微秒，提高通信可靠性
#define VERIFY_WRITES false

Rc522Device::Rc522Device(gpio_num_t rst_pin, gpio_num_t cs_pin, gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sck_pin, gpio_num_t irq_pin)
    : rst_pin_(rst_pin), cs_pin_(cs_pin), mosi_pin_(mosi_pin), miso_pin_(miso_pin), sck_pin_(sck_pin), irq_pin_(irq_pin), spi_initialized_(false) {
    ESP_LOGI(TAG, "RC522 created with RST:%d, CS:%d, MOSI:%d, MISO:%d, SCK:%d, IRQ:%d", 
             rst_pin_, cs_pin_, mosi_pin_, miso_pin_, sck_pin_, irq_pin_);
}

Rc522Device::~Rc522Device() {
    if (spi_initialized_ && spi_device_) {
        // 等待所有挂起的传输完成
        spi_device_acquire_bus(spi_device_, portMAX_DELAY);
        spi_device_release_bus(spi_device_);
        
        // 移除设备
        spi_bus_remove_device(spi_device_);
        spi_device_ = nullptr;
        spi_initialized_ = false;
    }
}

void Rc522Device::SpiInit() {
    ESP_LOGI(TAG, "Initializing SPI for RC522...");
    
    // 如果设备已经初始化，直接返回
    if (spi_initialized_) {
        ESP_LOGI(TAG, "SPI already initialized");
        return;
    }
    
    // SPI总线配置
    spi_bus_config_t bus_config = {
        .mosi_io_num = mosi_pin_,
        .miso_io_num = miso_pin_,
        .sclk_io_num = sck_pin_,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };
    
    // 初始化SPI总线
    esp_err_t ret = spi_bus_initialize(RC522_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized, continuing...");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    
    // SPI设备配置
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,               // CPOL=0, CPHA=0 (模式0)
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 1,    // CS预传输时间
        .cs_ena_posttrans = 1,   // CS后传输时间
        .clock_speed_hz = 100000,  // 降低到100kHz以提高稳定性
        .input_delay_ns = 0,     // 不使用输入延时
        .spics_io_num = cs_pin_,
        .flags = 0,              // 使用全双工模式
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    // 添加SPI设备
    ret = spi_bus_add_device(RC522_SPI_HOST, &dev_config, &spi_device_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add RC522 device to SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    
    // 配置RST引脚为输出
    gpio_config_t rst_conf = {};
    rst_conf.intr_type = GPIO_INTR_DISABLE;
    rst_conf.mode = GPIO_MODE_OUTPUT;
    rst_conf.pin_bit_mask = (1ULL << rst_pin_);
    rst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&rst_conf);
    
    // 如果IRQ引脚有效，配置为输入
    if (irq_pin_ != GPIO_NUM_NC) {
        gpio_config_t irq_conf = {};
        irq_conf.intr_type = GPIO_INTR_NEGEDGE;
        irq_conf.mode = GPIO_MODE_INPUT;
        irq_conf.pin_bit_mask = (1ULL << irq_pin_);
        irq_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        irq_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&irq_conf);
    }
    
    // 硬件复位序列
    gpio_set_level(rst_pin_, 1);  // 先设置为高
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(rst_pin_, 0);  // 拉低复位
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(rst_pin_, 1);  // 恢复高电平
    vTaskDelay(pdMS_TO_TICKS(50));
    
    spi_initialized_ = true;
    ESP_LOGI(TAG, "SPI initialized for RC522");
}

bool Rc522Device::SpiWrite(uint8_t reg, uint8_t value) {
    spi_transaction_t trans = {};
    uint8_t tx_data[2];
    
    // 写命令：地址的最高位为0
    tx_data[0] = (reg << 1) & 0x7E;  // 左移一位，清除最高位
    tx_data[1] = value;
    
    trans.flags = 0;
    trans.length = 16;        // 总共发送16位
    trans.tx_buffer = tx_data;
    trans.rx_buffer = NULL;
    
    esp_err_t ret = spi_device_transmit(spi_device_, &trans);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 添加短暂延时确保写入完成
    esp_rom_delay_us(100);
    return true;
}

uint8_t Rc522Device::SpiRead(uint8_t reg) {
    spi_transaction_t trans = {};
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    
    // 读命令：地址的最高位为1
    tx_data[0] = ((reg << 1) & 0x7E) | 0x80;  // 左移一位，设置最高位
    tx_data[1] = 0x00;
    
    trans.flags = 0;
    trans.length = 16;        // 总共发送16位
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;
    
    esp_err_t ret = spi_device_transmit(spi_device_, &trans);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SPI read failed: %s", esp_err_to_name(ret));
        return 0xFF;
    }
    
    // 添加短暂延时确保读取完成
    esp_rom_delay_us(100);
    return rx_data[1];
}

// 使用SPI读取寄存器
uint8_t Rc522Device::ReadRegister(uint8_t reg) {
    uint8_t result = SpiRead(reg);
    
    // 记录读取的寄存器值（调试用）
    if (reg != RC522_REG_FIFO_DATA) {  // 避免FIFO数据过多的日志
        ESP_LOGD(TAG, "Read Reg[0x%02X] = 0x%02X", reg, result);
    }
    
    return result;
}

// 使用SPI写入寄存器
void Rc522Device::WriteRegister(uint8_t reg, uint8_t value) {
    // 记录写入的寄存器值（调试用）
    if (reg != RC522_REG_FIFO_DATA) {  // 避免FIFO数据过多的日志
        ESP_LOGD(TAG, "Write Reg[0x%02X] = 0x%02X", reg, value);
    }
    
    SpiWrite(reg, value);
    
    // 验证写入是否成功（可选）
    if (VERIFY_WRITES && reg != RC522_REG_COMMAND && reg != RC522_REG_FIFO_DATA) {
        // 等待一段时间，确保写入完成
        esp_rom_delay_us(I2C_DELAY_US * 5);
        
        uint8_t readValue = ReadRegister(reg);
        if (readValue != value) {
            ESP_LOGW(TAG, "Write verification failed: reg=0x%02X, wrote=0x%02X, read=0x%02X", 
                    reg, value, readValue);
            
            // 尝试再次写入
            ESP_LOGI(TAG, "Retrying write to register 0x%02X", reg);
            SpiWrite(reg, value);
            
            // 再次验证
            esp_rom_delay_us(I2C_DELAY_US * 5);
            readValue = ReadRegister(reg);
            if (readValue != value) {
                ESP_LOGW(TAG, "Second write verification failed: reg=0x%02X, wrote=0x%02X, read=0x%02X", 
                        reg, value, readValue);
            } else {
                ESP_LOGI(TAG, "Second write verification succeeded");
            }
        }
    }
}

void Rc522Device::SetBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = ReadRegister(reg);
    WriteRegister(reg, tmp | mask);
}

void Rc522Device::ClearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = ReadRegister(reg);
    WriteRegister(reg, tmp & (~mask));
}

void Rc522Device::AntennaOn() {
    SetBitMask(RC522_REG_TX_CONTROL, 0x03);
}

void Rc522Device::AntennaOff() {
    ClearBitMask(RC522_REG_TX_CONTROL, 0x03);
}

void Rc522Device::ResetDevice() {
    // 硬件复位
    gpio_set_level(rst_pin_, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(rst_pin_, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 软复位
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool Rc522Device::DetectCard(uint8_t* card_type) {
    // 重置命令寄存器
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    // 清除所有中断标志
    WriteRegister(RC522_REG_COM_IRQ, 0x7F);
    // 设置位帧调整为7位
    WriteRegister(RC522_REG_BIT_FRAMING, 0x07);
    
    // 准备REQA命令
    uint8_t buff[2] = {0};
    buff[0] = 0x26; // REQA command
    uint8_t result_len = sizeof(buff);
    
    // 执行命令
    bool status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
    
    // 检查结果
    if (status && (result_len == 2)) {
        *card_type = buff[0];
        ESP_LOGD(TAG, "Card detected with type: 0x%02X, 0x%02X", buff[0], buff[1]);
        
        // 如果卡片类型是0xFF，可能是通信问题，尝试再次读取
        if (*card_type == 0xFF) {
            // 短暂延时后重试
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // 重置并再次尝试
            WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
            WriteRegister(RC522_REG_COM_IRQ, 0x7F);
            WriteRegister(RC522_REG_BIT_FRAMING, 0x07);
            
            // 尝试WUPA命令
            buff[0] = 0x52; // WUPA command
            result_len = sizeof(buff);
            
            status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
            
            if (status && (result_len == 2)) {
                *card_type = buff[0];
                ESP_LOGD(TAG, "Card detected with WUPA! Type: 0x%02X, 0x%02X", buff[0], buff[1]);
                return true;
            }
            
            // 如果WUPA失败，尝试增加RF功率
            WriteRegister(RC522_REG_RF_CFG, 0x70); // 最大RF输出功率
            
            // 再次尝试REQA
            buff[0] = 0x26; // REQA command
            result_len = sizeof(buff);
            
            status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
            
            // 恢复RF功率
            WriteRegister(RC522_REG_RF_CFG, 0x70); // 保持最大功率
            
            if (status && (result_len == 2)) {
                *card_type = buff[0];
                ESP_LOGD(TAG, "Card detected with high power! Type: 0x%02X, 0x%02X", buff[0], buff[1]);
                return true;
            }
            
            return false;
        }
        
        return true;
    } else {
        // 如果REQA失败，尝试WUPA命令
        WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
        WriteRegister(RC522_REG_COM_IRQ, 0x7F);
        WriteRegister(RC522_REG_BIT_FRAMING, 0x07);
        
        buff[0] = 0x52; // WUPA command
        result_len = sizeof(buff);
        
        status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
        
        if (status && (result_len == 2)) {
            *card_type = buff[0];
            ESP_LOGD(TAG, "Card detected with WUPA! Type: 0x%02X, 0x%02X", buff[0], buff[1]);
            return true;
        }
    }
    
    return false;
}

bool Rc522Device::ReadCardSerial(uint8_t* serial_no) {
    // 尝试使用标准的ANTICOLLISION命令读取卡片序列号
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    WriteRegister(RC522_REG_COM_IRQ, 0x7F);
    WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
    
    // 准备ANTICOLLISION命令
    uint8_t buff[9] = {0};
    buff[0] = 0x93; // ANTICOLLISION command
    buff[1] = 0x20; // NVB (Number of Valid Bits)
    uint8_t result_len = sizeof(buff);
    
    ESP_LOGD(TAG, "Trying ANTICOLLISION command");
    
    // 执行命令
    bool status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 2, buff, &result_len);
    
    // 检查结果
    if (status && (result_len >= 5)) {
        ESP_LOGD(TAG, "ANTICOLLISION success, result_len=%d", result_len);
        
        // 复制前5个字节作为序列号
        memcpy(serial_no, buff, 5);
        
        // 检查序列号是否全为0或全为FF
        bool all_zero = true;
        bool all_ff = true;
        
        for (int i = 0; i < 5; i++) {
            if (serial_no[i] != 0x00) all_zero = false;
            if (serial_no[i] != 0xFF) all_ff = false;
            ESP_LOGD(TAG, "Serial[%d] = 0x%02X", i, serial_no[i]);
        }
        
        if (all_zero || all_ff) {
            ESP_LOGW(TAG, "Invalid card serial (all zeros or all FFs)");
            
            // 尝试使用不同的命令格式
            WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
            
            // 使用WUPA命令代替REQA
            buff[0] = 0x52; // WUPA command
            result_len = 2;
            
            ESP_LOGD(TAG, "Trying WUPA command");
            status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
            
            if (status && (result_len == 2)) {
                ESP_LOGD(TAG, "WUPA success, result_len=%d", result_len);
                
                // 如果WUPA成功，再次尝试ANTICOLLISION
                WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
                buff[0] = 0x93;
                buff[1] = 0x20;
                result_len = sizeof(buff);
                
                ESP_LOGD(TAG, "Trying ANTICOLLISION after WUPA");
                status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 2, buff, &result_len);
                
                if (status && (result_len >= 5)) {
                    ESP_LOGD(TAG, "ANTICOLLISION after WUPA success, result_len=%d", result_len);
                    
                    memcpy(serial_no, buff, 5);
                    
                    // 再次检查序列号
                    all_zero = true;
                    all_ff = true;
                    
                    for (int i = 0; i < 5; i++) {
                        if (serial_no[i] != 0x00) all_zero = false;
                        if (serial_no[i] != 0xFF) all_ff = false;
                        ESP_LOGD(TAG, "Serial[%d] = 0x%02X", i, serial_no[i]);
                    }
                    
                    if (!all_zero && !all_ff) {
                        ESP_LOGI(TAG, "Card serial (WUPA method): %02X:%02X:%02X:%02X:%02X",
                                serial_no[0], serial_no[1], serial_no[2], serial_no[3], serial_no[4]);
                        return true;
                    }
                }
            }
            
            // 如果仍然失败，尝试增加延时和调整参数
            vTaskDelay(pdMS_TO_TICKS(20));
            
            // 调整RF配置
            WriteRegister(RC522_REG_RF_CFG, 0x70); // 增加RF输出功率
            
            // 再次尝试ANTICOLLISION
            WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
            buff[0] = 0x93;
            buff[1] = 0x20;
            result_len = sizeof(buff);
            
            ESP_LOGD(TAG, "Trying ANTICOLLISION with high power");
            status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 2, buff, &result_len);
            
            if (status && (result_len >= 5)) {
                ESP_LOGD(TAG, "ANTICOLLISION with high power success, result_len=%d", result_len);
                
                memcpy(serial_no, buff, 5);
                
                // 再次检查序列号
                all_zero = true;
                all_ff = true;
                
                for (int i = 0; i < 5; i++) {
                    if (serial_no[i] != 0x00) all_zero = false;
                    if (serial_no[i] != 0xFF) all_ff = false;
                    ESP_LOGD(TAG, "Serial[%d] = 0x%02X", i, serial_no[i]);
                }
                
                if (!all_zero && !all_ff) {
                    ESP_LOGI(TAG, "Card serial (high power): %02X:%02X:%02X:%02X:%02X",
                            serial_no[0], serial_no[1], serial_no[2], serial_no[3], serial_no[4]);
                    
                    // 保持高功率
                    // WriteRegister(RC522_REG_RF_CFG, 0x59); // 恢复默认值
                    return true;
                }
                
                // 恢复RF配置
                // WriteRegister(RC522_REG_RF_CFG, 0x59); // 恢复默认值
            }
            
            return false;
        }
        
        ESP_LOGI(TAG, "Card serial: %02X:%02X:%02X:%02X:%02X",
                serial_no[0], serial_no[1], serial_no[2], serial_no[3], serial_no[4]);
        return true;
    }
    
    // 如果标准方法失败，尝试SELECT2命令
    WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
    buff[0] = 0x95; // SELECT2命令
    buff[1] = 0x20;
    result_len = sizeof(buff);
    
    ESP_LOGD(TAG, "Trying SELECT2 command");
    status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 2, buff, &result_len);
    
    if (status && (result_len >= 5)) {
        ESP_LOGD(TAG, "SELECT2 success, result_len=%d", result_len);
        
        memcpy(serial_no, buff, 5);
        
        // 检查序列号是否全为0或全为FF
        bool all_zero = true;
        bool all_ff = true;
        
        for (int i = 0; i < 5; i++) {
            if (serial_no[i] != 0x00) all_zero = false;
            if (serial_no[i] != 0xFF) all_ff = false;
            ESP_LOGD(TAG, "Serial[%d] = 0x%02X", i, serial_no[i]);
        }
        
        if (all_zero || all_ff) {
            ESP_LOGW(TAG, "Invalid card serial (all zeros or all FFs)");
            return false;
        }
        
        ESP_LOGI(TAG, "Card serial (SELECT2): %02X:%02X:%02X:%02X:%02X",
                serial_no[0], serial_no[1], serial_no[2], serial_no[3], serial_no[4]);
        return true;
    }
    
    // 如果所有方法都失败，尝试直接读取FIFO
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    WriteRegister(RC522_REG_COM_IRQ, 0x7F);
    
    // 尝试读取FIFO中的数据
    uint8_t fifo_level = ReadRegister(RC522_REG_FIFO_LEVEL);
    ESP_LOGD(TAG, "FIFO level: %d", fifo_level);
    
    if (fifo_level >= 5) {
        for (int i = 0; i < 5; i++) {
            serial_no[i] = ReadRegister(RC522_REG_FIFO_DATA);
            ESP_LOGD(TAG, "FIFO Serial[%d] = 0x%02X", i, serial_no[i]);
        }
        
        // 检查序列号是否全为0或全为FF
        bool all_zero = true;
        bool all_ff = true;
        
        for (int i = 0; i < 5; i++) {
            if (serial_no[i] != 0x00) all_zero = false;
            if (serial_no[i] != 0xFF) all_ff = false;
        }
        
        if (!all_zero && !all_ff) {
            ESP_LOGI(TAG, "Card serial (FIFO): %02X:%02X:%02X:%02X:%02X",
                    serial_no[0], serial_no[1], serial_no[2], serial_no[3], serial_no[4]);
            return true;
        }
    }
    
    return false;
}

bool Rc522Device::ExecuteCommand(uint8_t command, uint8_t* data, uint8_t data_len, uint8_t* result, uint8_t* result_len) {
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    
    if (command == RC522_CMD_TRANSCEIVE) {
        irq = 0x77;
        irq_wait = 0x30;
    }
    
    ESP_LOGD(TAG, "Executing command: 0x%02X, data_len: %d", command, data_len);
    
    // 清除所有中断标志
    WriteRegister(RC522_REG_COM_IRQ, 0x7F);
    // 禁用所有中断
    WriteRegister(RC522_REG_COM_IE, 0x00);
    // 清除FIFO缓冲区
    SetBitMask(RC522_REG_FIFO_LEVEL, 0x80);
    // 空闲命令
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    
    // 写入数据到FIFO
    for (uint8_t i = 0; i < data_len; i++) {
        WriteRegister(RC522_REG_FIFO_DATA, data[i]);
    }
    
    // 执行命令
    WriteRegister(RC522_REG_COMMAND, command);
    
    // 如果是发送接收命令，启动发送
    if (command == RC522_CMD_TRANSCEIVE) {
        SetBitMask(RC522_REG_BIT_FRAMING, 0x80);
    }
    
    // 等待命令完成
    uint16_t wait_count = 2000;  // 增加超时时间
    uint8_t n;
    do {
        n = ReadRegister(RC522_REG_COM_IRQ);
        wait_count--;
        
        // 检查错误和超时
        if (wait_count == 0 || n & 0x01) {
            ESP_LOGW(TAG, "Command timeout or error, IRQ: 0x%02X", n);
            break;
        }
        
        // 检查命令完成
        if (n & irq_wait) {
            break;
        }
        
        // 短暂延时
        esp_rom_delay_us(100);
    } while (true);
    
    // 停止发送
    ClearBitMask(RC522_REG_BIT_FRAMING, 0x80);
    
    // 检查错误
    if (wait_count == 0) {
        ESP_LOGW(TAG, "Command execution timeout");
        return false;
    }
    
    uint8_t error = ReadRegister(RC522_REG_ERROR);
    if (error & 0x1B) {
        ESP_LOGW(TAG, "Command execution error: 0x%02X", error);
        return false;
    }
    
    // 获取接收到的数据长度
    uint8_t fifo_level = ReadRegister(RC522_REG_FIFO_LEVEL);
    last_bits = ReadRegister(RC522_REG_CONTROL) & 0x07;
    
    if (last_bits) {
        *result_len = (fifo_level - 1) * 8 + last_bits;
    } else {
        *result_len = fifo_level;
    }
    
    // 读取结果
    if (fifo_level > 0) {
        uint8_t count = (fifo_level > *result_len) ? *result_len : fifo_level;
        for (uint8_t i = 0; i < count; i++) {
            result[i] = ReadRegister(RC522_REG_FIFO_DATA);
        }
        ESP_LOGD(TAG, "Command executed successfully, result_len: %d", count);
        return true;
    }
    
    ESP_LOGW(TAG, "No data in FIFO");
    return false;
}

bool Rc522Device::Initialize() {
    ESP_LOGI(TAG, "Initializing RC522...");
    
    // 初始化SPI接口
    SpiInit();
    
    // 等待RC522上电稳定
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 软复位RC522
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 检查复位是否成功
    uint8_t command_reg = ReadRegister(RC522_REG_COMMAND);
    ESP_LOGI(TAG, "After reset, command register: 0x%02X", command_reg);
    
    // 如果复位后命令寄存器不为0，再次尝试复位
    if (command_reg != 0x00) {
        ESP_LOGW(TAG, "Reset may not be complete, trying again");
        WriteRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        command_reg = ReadRegister(RC522_REG_COMMAND);
        ESP_LOGI(TAG, "After second reset, command register: 0x%02X", command_reg);
        
        if (command_reg != 0x00) {
            ESP_LOGW(TAG, "Reset still not complete, trying hardware reset");
            // 尝试硬件复位
            gpio_set_level(rst_pin_, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(rst_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
            
            // 再次软复位
            WriteRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
            vTaskDelay(pdMS_TO_TICKS(50));
            
            command_reg = ReadRegister(RC522_REG_COMMAND);
            ESP_LOGI(TAG, "After hardware reset, command register: 0x%02X", command_reg);
        }
    }
    
    // 配置接收器增益
    WriteRegister(RC522_REG_RF_CFG, 0x70);  // 最大增益
    
    // 配置天线
    WriteRegister(RC522_REG_TX_ASK, 0x40);  // Force100ASK = 1
    
    // 配置调制
    WriteRegister(RC522_REG_MOD_WIDTH, 0x26);
    
    // 设置定时器
    WriteRegister(RC522_REG_T_MODE, 0x80);
    WriteRegister(RC522_REG_T_PRESCALER, 0xA9);
    WriteRegister(RC522_REG_T_RELOAD_H, 0x03);
    WriteRegister(RC522_REG_T_RELOAD_L, 0xE8);
    
    // 设置CRC预置值
    WriteRegister(RC522_REG_MODE, 0x3D);
    
    // 使能天线
    AntennaOn();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 读取版本信息
    uint8_t version = ReadRegister(RC522_REG_VERSION);
    ESP_LOGI(TAG, "RC522 version: 0x%02X", version);
    
    // 验证通信
    WriteRegister(RC522_REG_FIFO_DATA, 0x55);
    uint8_t test = ReadRegister(RC522_REG_FIFO_DATA);
    
    if (test != 0x55) {
        ESP_LOGW(TAG, "Communication test failed: wrote 0x55, read 0x%02X", test);
        return false;
    }
    
    ESP_LOGI(TAG, "RC522 initialized successfully");
    return true;
} 