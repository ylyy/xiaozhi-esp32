#pragma once

#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/spi_master.h>

// RC522寄存器定义
#define RC522_REG_COMMAND        0x01
#define RC522_REG_COM_IE         0x02
#define RC522_REG_DIV_IE         0x03
#define RC522_REG_COM_IRQ        0x04
#define RC522_REG_DIV_IRQ        0x05
#define RC522_REG_ERROR          0x06
#define RC522_REG_STATUS1        0x07
#define RC522_REG_STATUS2        0x08
#define RC522_REG_FIFO_DATA      0x09
#define RC522_REG_FIFO_LEVEL     0x0A
#define RC522_REG_CONTROL        0x0C
#define RC522_REG_BIT_FRAMING    0x0D
#define RC522_REG_COLL           0x0E
#define RC522_REG_MODE           0x11
#define RC522_REG_TX_MODE        0x12
#define RC522_REG_RX_MODE        0x13
#define RC522_REG_TX_CONTROL     0x14
#define RC522_REG_TX_ASK         0x15
#define RC522_REG_TX_SEL         0x16
#define RC522_REG_RX_SEL         0x17
#define RC522_REG_RX_THRESHOLD   0x18
#define RC522_REG_DEMOD          0x19
#define RC522_REG_MOD_WIDTH      0x24
#define RC522_REG_RF_CFG         0x26
#define RC522_REG_GS_N           0x27
#define RC522_REG_CW_GS_P        0x28
#define RC522_REG_MOD_GS_P       0x29
#define RC522_REG_T_MODE         0x2A
#define RC522_REG_T_PRESCALER    0x2B
#define RC522_REG_T_RELOAD_H     0x2C
#define RC522_REG_T_RELOAD_L     0x2D
#define RC522_REG_T_COUNTER_H    0x2E
#define RC522_REG_T_COUNTER_L    0x2F
#define RC522_REG_VERSION        0x37

// RC522命令定义
#define RC522_CMD_IDLE           0x00
#define RC522_CMD_MEM            0x01
#define RC522_CMD_GENERATE_RANDOM_ID 0x02
#define RC522_CMD_CALC_CRC       0x03
#define RC522_CMD_TRANSMIT       0x04
#define RC522_CMD_NO_CMD_CHANGE  0x07
#define RC522_CMD_RECEIVE        0x08
#define RC522_CMD_TRANSCEIVE     0x0C
#define RC522_CMD_MF_AUTHENT     0x0E
#define RC522_CMD_SOFT_RESET     0x0F

// 调试选项
#define RC522_DEBUG_REGISTERS    0   // 设置为1启用寄存器读写调试
#define RC522_DEBUG_COMMANDS     1   // 设置为1启用命令执行调试

// SPI通信相关定义
#define RC522_SPI_HOST           SPI2_HOST  // 使用SPI2
#define RC522_SPI_DMA_CHAN       SPI_DMA_CH_AUTO

class Rc522Device {
public:
    Rc522Device(gpio_num_t rst_pin, gpio_num_t cs_pin, gpio_num_t mosi_pin, gpio_num_t miso_pin, gpio_num_t sck_pin, gpio_num_t irq_pin = GPIO_NUM_NC);
    ~Rc522Device();

    bool Initialize();
    bool DetectCard(uint8_t* card_type);
    bool ReadCardSerial(uint8_t* serial_no);

private:
    void SpiInit();
    bool SpiWrite(uint8_t reg, uint8_t value);
    uint8_t SpiRead(uint8_t reg);
    uint8_t ReadRegister(uint8_t reg);
    void WriteRegister(uint8_t reg, uint8_t value);
    void SetBitMask(uint8_t reg, uint8_t mask);
    void ClearBitMask(uint8_t reg, uint8_t mask);
    void AntennaOn();
    void AntennaOff();
    bool ExecuteCommand(uint8_t command, uint8_t* data, uint8_t data_len, uint8_t* result, uint8_t* result_len);
    void ResetDevice();

    gpio_num_t rst_pin_;
    gpio_num_t cs_pin_;    // 片选信号
    gpio_num_t mosi_pin_;  // 主机输出，从机输入
    gpio_num_t miso_pin_;  // 主机输入，从机输出
    gpio_num_t sck_pin_;   // 时钟信号
    gpio_num_t irq_pin_;   // 中断信号
    spi_device_handle_t spi_device_;
    bool spi_initialized_;
};
