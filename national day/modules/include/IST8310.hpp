#ifndef IST8310_HPP
#define IST8310_HPP

#include "IMU.hpp"
#include "i2c.h"

/**
 * @brief 硬件连接
 * @note  I2C3：PA8 = SCL、PC9 = SDA；复位脚 PG6
 */
#define IST8310_IIC_ADDR      (0x0E << 1)   /* HAL 用 8 位地址（7 位左移 1） */
#define IST8310_WHO_AM_I_ADDR 0x00
#define IST8310_WHO_AM_I_VAL  0x10
#define IST8310_DATA_ADDR     0x03          /* X_LSB 起始 */
#define IST8310_RST_PORT      GPIOG
#define IST8310_RST_PIN       GPIO_PIN_6

/**
 * @brief IST8310 三轴磁力计
 */
class cIST8310
{
public:
    mag_data_t  mag_data;     ///< 三轴磁场（µT）
    mag_error_t self_test;    ///< 错误状态

    
    void Config();

   
    void ReadMagData(mag_data_t *data);

    /**
     * @brief 校验 WHO_AM_I，若不对则置 CHIP_ID_ERR / INIT_ERR
     */
    void VerifyMagChipID();

private:
    /**
     * @brief 读多个寄存器
     * @param reg 寄存器起始地址
     * @param buf 读取缓冲区
     * @param len 读取长度
     * @retval true 成功，false 失败
     */
    bool ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);

    /**
     * @brief 写单个寄存器
     * @param reg 寄存器地址
     * @param data 写入值
     * @retval true 成功，false 失败
     */
    bool WriteReg(uint8_t reg, uint8_t data);
};

#endif // IST8310_HPP