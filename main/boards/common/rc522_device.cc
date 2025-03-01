#include "rc522_device.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_rom_sys.h>

#define TAG "RC522"

Rc522Device::Rc522Device(gpio_num_t mosi_pin, gpio_num_t miso_pin)
    : mosi_pin_(mosi_pin), miso_pin_(miso_pin) {
}

Rc522Device::~Rc522Device() {
}

void Rc522Device::SoftwareSpiInit() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << mosi_pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << miso_pin_);
    gpio_config(&io_conf);
}

uint8_t Rc522Device::SoftwareSpiTransfer(uint8_t data) {
    uint8_t received = 0;
    
    // 按位传输数据
    for (int i = 7; i >= 0; i--) {
        // 设置 MOSI
        gpio_set_level(mosi_pin_, (data >> i) & 1);
        
        // 短暂延时以确保信号稳定
        esp_rom_delay_us(1);
        
        // 读取 MISO
        received = (received << 1) | gpio_get_level(miso_pin_);
        
        // 短暂延时以完成一个时钟周期
        esp_rom_delay_us(1);
    }
    
    return received;
}

bool Rc522Device::Initialize() {
    SoftwareSpiInit();
    
    // 配置RC522
    WriteRegister(RC522_REG_TX_MODE, 0x00);
    WriteRegister(RC522_REG_RX_MODE, 0x00);
    WriteRegister(RC522_REG_MOD_WIDTH, 0x26);
    
    WriteRegister(RC522_REG_T_MODE, 0x80);
    WriteRegister(RC522_REG_T_PRESCALER, 0xA9);
    WriteRegister(RC522_REG_T_RELOAD_H, 0x03);
    WriteRegister(RC522_REG_T_RELOAD_L, 0xE8);
    
    WriteRegister(RC522_REG_TX_ASK, 0x40);
    WriteRegister(RC522_REG_MODE, 0x3D);
    
    AntennaOn();
    return true;
}

uint8_t Rc522Device::ReadRegister(uint8_t reg) {
    uint8_t address = ((reg << 1) & 0x7E) | 0x80;
    return SoftwareSpiTransfer(address);
}

void Rc522Device::WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t address = (reg << 1) & 0x7E;
    SoftwareSpiTransfer(address);
    SoftwareSpiTransfer(value);
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
    uint8_t temp = ReadRegister(RC522_REG_TX_CONTROL);
    if (!(temp & 0x03)) {
        SetBitMask(RC522_REG_TX_CONTROL, 0x03);
    }
}

void Rc522Device::AntennaOff() {
    ClearBitMask(RC522_REG_TX_CONTROL, 0x03);
}

bool Rc522Device::DetectCard(uint8_t* card_type) {
    WriteRegister(RC522_REG_BIT_FRAMING, 0x07);
    uint8_t buff[2];
    buff[0] = 0x26; // REQA command
    uint8_t result_len = sizeof(buff);
    
    bool status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 1, buff, &result_len);
    if (status && (result_len == 2)) {
        *card_type = buff[0];
        return true;
    }
    return false;
}

bool Rc522Device::ReadCardSerial(uint8_t* serial_no) {
    uint8_t buff[9];
    uint8_t result_len = sizeof(buff);
    
    WriteRegister(RC522_REG_BIT_FRAMING, 0x00);
    buff[0] = 0x93; // Anti-collision command
    buff[1] = 0x20;
    
    bool status = ExecuteCommand(RC522_CMD_TRANSCEIVE, buff, 2, buff, &result_len);
    if (status && (result_len == 5)) {
        memcpy(serial_no, buff, 5);
        return true;
    }
    return false;
}

bool Rc522Device::ExecuteCommand(uint8_t command, uint8_t* data, uint8_t data_len, uint8_t* result, uint8_t* result_len) {
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t n = 0;
    
    if (command == RC522_CMD_TRANSCEIVE) {
        irq = 0x77;
        irq_wait = 0x30;
    }
    
    WriteRegister(RC522_REG_COM_IE, irq | 0x80);
    ClearBitMask(RC522_REG_COM_IRQ, 0x80);
    SetBitMask(RC522_REG_FIFO_LEVEL, 0x80);
    
    WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);
    
    // 写入数据到FIFO
    for (uint8_t i = 0; i < data_len; i++) {
        WriteRegister(RC522_REG_FIFO_DATA, data[i]);
    }
    
    // 执行命令
    WriteRegister(RC522_REG_COMMAND, command);
    if (command == RC522_CMD_TRANSCEIVE) {
        SetBitMask(RC522_REG_BIT_FRAMING, 0x80);
    }
    
    // 等待命令完成
    uint16_t i = 1000;
    do {
        n = ReadRegister(RC522_REG_COM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & irq_wait));
    
    ClearBitMask(RC522_REG_BIT_FRAMING, 0x80);
    
    if (i != 0) {
        if (!(ReadRegister(RC522_REG_ERROR) & 0x1B)) {
            if (n & irq_wait) {
                if (n & 0x01) {
                    return false;
                }
                
                n = ReadRegister(RC522_REG_FIFO_LEVEL);
                last_bits = ReadRegister(RC522_REG_CONTROL) & 0x07;
                if (last_bits) {
                    *result_len = (n - 1) * 8 + last_bits;
                } else {
                    *result_len = n * 8;
                }
                
                if (n == 0) {
                    n = 1;
                }
                if (n > *result_len) {
                    n = *result_len;
                }
                
                // 从FIFO读取结果
                for (uint8_t i = 0; i < n; i++) {
                    result[i] = ReadRegister(RC522_REG_FIFO_DATA);
                }
                
                return true;
            }
        }
    }
    
    return false;
} 