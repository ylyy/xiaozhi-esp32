#ifndef RC522_DEVICE_H
#define RC522_DEVICE_H

#include <driver/gpio.h>
#include <esp_log.h>

// RC522 寄存器地址定义
#define RC522_REG_COMMAND        0x01
#define RC522_REG_COM_IE        0x02
#define RC522_REG_DIV_IE        0x03
#define RC522_REG_COM_IRQ       0x04
#define RC522_REG_DIV_IRQ       0x05
#define RC522_REG_ERROR         0x06
#define RC522_REG_STATUS1       0x07
#define RC522_REG_STATUS2       0x08
#define RC522_REG_FIFO_DATA     0x09
#define RC522_REG_FIFO_LEVEL    0x0A
#define RC522_REG_CONTROL       0x0C
#define RC522_REG_BIT_FRAMING   0x0D
#define RC522_REG_MODE          0x11
#define RC522_REG_TX_MODE       0x12
#define RC522_REG_RX_MODE       0x13
#define RC522_REG_TX_CONTROL    0x14
#define RC522_REG_TX_ASK        0x15
#define RC522_REG_T_MODE        0x2A
#define RC522_REG_T_PRESCALER   0x2B
#define RC522_REG_T_RELOAD_H    0x2C
#define RC522_REG_T_RELOAD_L    0x2D
#define RC522_REG_CRC_RESULT_H  0x21
#define RC522_REG_CRC_RESULT_L  0x22
#define RC522_REG_MOD_WIDTH     0x24
#define RC522_REG_RF_CFG        0x26
#define RC522_REG_VERSION       0x37

// RC522 命令定义
#define RC522_CMD_IDLE          0x00
#define RC522_CMD_CALC_CRC      0x03
#define RC522_CMD_TRANSMIT      0x04
#define RC522_CMD_RECEIVE       0x08
#define RC522_CMD_TRANSCEIVE    0x0C
#define RC522_CMD_SOFT_RESET    0x0F

class Rc522Device {
public:
    Rc522Device(gpio_num_t mosi_pin, gpio_num_t miso_pin);
    ~Rc522Device();

    bool Initialize();
    bool DetectCard(uint8_t* card_type);
    bool ReadCardSerial(uint8_t* serial_no);
    
private:
    gpio_num_t mosi_pin_;
    gpio_num_t miso_pin_;
    
    uint8_t ReadRegister(uint8_t reg);
    void WriteRegister(uint8_t reg, uint8_t value);
    void SetBitMask(uint8_t reg, uint8_t mask);
    void ClearBitMask(uint8_t reg, uint8_t mask);
    bool ExecuteCommand(uint8_t command, uint8_t* data, uint8_t data_len, uint8_t* result, uint8_t* result_len);
    void AntennaOn();
    void AntennaOff();

    // 软件 SPI 函数
    void SoftwareSpiInit();
    uint8_t SoftwareSpiTransfer(uint8_t data);
};

#endif // RC522_DEVICE_H 