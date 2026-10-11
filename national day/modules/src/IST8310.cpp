#include "IST8310.hpp"
#include "tx_api.h"

bool cIST8310::ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK;
}

bool cIST8310::WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK;
}

void cIST8310::VerifyMagChipID()
{
    uint8_t id = 0;

    if (!ReadRegs(IST8310_WHO_AM_I_ADDR, &id, 1))
    {
        self_test.COMM_ERR    = true;
        self_test.CHIP_ID_ERR = true;
        self_test.INIT_ERR    = true;
        return;
    }

    if (id != IST8310_WHO_AM_I_VAL)
    {
        self_test.CHIP_ID_ERR = true;
        self_test.INIT_ERR    = true;
    }
    else
    {
        self_test.CHIP_ID_ERR = false;
    }
}

void cIST8310::Config()
{
    /* 1. 硬复位：拉低 50ms → 拉高 50ms */
    HAL_GPIO_WritePin(IST8310_RST_PORT, IST8310_RST_PIN, GPIO_PIN_RESET);
    tx_thread_sleep(50);
    HAL_GPIO_WritePin(IST8310_RST_PORT, IST8310_RST_PIN, GPIO_PIN_SET);
    tx_thread_sleep(50);

    /* 2. 校验芯片 ID */
    VerifyMagChipID();
    if (self_test.CHIP_ID_ERR) return;

    /* 3. 配置寄存器（照官方例程），每个写完读回校验 */
    const uint8_t cfg[4][2] = {
        {0x0B, 0x08},   /* 使能中断，低电平有效 */
        {0x41, 0x09},   /* 平均次数 2 */
        {0x42, 0xC0},   /* 必须写 0xC0 */
        {0x0A, 0x0B},   /* 200Hz 输出速率 */
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!WriteReg(cfg[i][0], cfg[i][1])) { self_test.COMM_ERR = true; return; }
        tx_thread_sleep(1);

        uint8_t back = 0;
        if (!ReadRegs(cfg[i][0], &back, 1))  { self_test.COMM_ERR = true; return; }
        if (back != cfg[i][1])               { self_test.INIT_ERR = true; return; }
    }

    self_test.INIT_ERR = false;
}

void cIST8310::ReadMagData(mag_data_t *data)
{
    if (data == nullptr) return;

    uint8_t buf[6] = {0};
    if (!ReadRegs(IST8310_DATA_ADDR, buf, 6))
    {
        self_test.COMM_ERR = true;
        self_test.DATA_ERR = true;
        return;                       /* 读失败就保留上一次的值 */
    }

    /* 低字节在前 */
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    data->x = 0.3f * raw_x;           /* raw × 0.3 = µT */
    data->y = 0.3f * raw_y;
    data->z = 0.3f * raw_z;

    mag_data = *data;                 /* 和 ReadAccData 里 acc_data = *data; 一致 */

    self_test.COMM_ERR = false;
    self_test.DATA_ERR = (data->x == 0.0f && data->y == 0.0f && data->z == 0.0f);
}