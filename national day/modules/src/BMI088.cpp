#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include <cmath>


namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        // TODO: 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        float temp = 0.0f;

        ReadAccTemperature(&temp);

        TempPid.ref = target_temp;
        TempPid.fdb = temp;
        TempPid.UpdateResult();

        /* 输出当占空比用，限制到 [0, 1]；负值表示不用加热 */
        float duty = TempPid.result;
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID，
        //但readreg中已经丢弃

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &pRxData, 1); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData()
    {
        // TODO: 检查加速度数据是否有效，并更新 ACC_DATA_ERR。
        if (acc_data.x == 0.0f && acc_data.y == 0.0f && acc_data.z == 0.0f)
            self_test.ACC_DATA_ERR = true;
        else
            self_test.ACC_DATA_ERR = false;
    }

    void cBMI088::VerifyGyroData()
    {
        // TODO: 检查角速度数据是否有效，并更新 GYRO_DATA_ERR。
        if (gyro_data.x == 0.0f && gyro_data.y == 0.0f && gyro_data.z == 0.0f)
            self_test.GYRO_DATA_ERR = true;
        else
            self_test.GYRO_DATA_ERR = false;
    }



    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 写协议发送地址和数据，
        // 最后释放片选，并处理通信失败与必要的延时。
        if (data == nullptr || len == 0) return;

        uint8_t reg = addr & BMI088_SPI_WRITE_CODE;   /* 写：最高位清 0 */

        /* 拉低对应片选 */
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg, 1, 100);     /* 发地址 */
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, data, len, 100);   /* 发数据 */

        /* 拉高片选 */
        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 读协议发送地址，
        // 丢弃加速度计返回的首个无效字节，读取 len 字节并释放片选。
        // 未实现前清零缓冲区，避免芯片 ID 校验读取未初始化数据。
        if (data == nullptr || len == 0) return;

        uint8_t reg   = addr | BMI088_SPI_READ_CODE;   /* 读：最高位置 1 */
        uint8_t dummy = 0;

        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg, 1, 100);   /* 发地址 */

        /* 加速度计的真数据比陀螺仪晚一拍，先收掉一个无效字节 */
        if (cs == BMI088_CS_ACC)
            HAL_SPI_Receive(&BMI088_SPI_HANDLE, &dummy, 1, 100);

        HAL_SPI_Receive(&BMI088_SPI_HANDLE, data, len, 100);  /* 收数据 */

        if (cs == BMI088_CS_ACC)
            HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        uint8_t pTxData;

        /* 1. 软复位 */
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100);

        /* 2. 打开加速度计电源，写两次（防止软复位后第一次写入被吞） */
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(50);
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(50);

//active
         pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(50);

        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5);

        pTxData = 0xAB;                      
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5);

       
       
        /* 5. 中断引脚 */
        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5);

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5);

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms


    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        // TODO: 按 BMI088 加速度计 SPI 协议丢弃首个无效字节(readreg中)，拼接三轴有符号原始值，
        // 再按配置的量程换算为 m/s²，写入 data。
        if (data == nullptr) return;

        uint8_t buf[ACC_XYZ_LEN];   /* 6 个字节 */
        ReadReg(BMI088_CS_ACC, ACC_X_LSB_ADDR, buf, ACC_XYZ_LEN);
        /* ↑ 这一步内部已经帮你丢掉了那个无效字节 */

        /* 拼接：低字节在前 */
        int16_t raw_x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
        int16_t raw_y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
        int16_t raw_z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

        
        data->x = raw_x * IMU_ACCEL_6G_SEN;
        data->y = raw_y * IMU_ACCEL_6G_SEN;
        data->z = raw_z * IMU_ACCEL_6G_SEN;

        acc_data = *data;

       
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        // TODO: 读取并拼接陀螺仪三轴原始值，按配置量程换算为 rad/s，
        // 减去 Gyro_offset 后写入 data。
        if (data == nullptr) return;

        uint8_t buf[GYRO_XYZ_LEN];  /* 6 */
        ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR, buf, GYRO_XYZ_LEN);

        int16_t raw_x = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
        int16_t raw_y = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
        int16_t raw_z = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

        data->x = raw_x * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = raw_y * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = raw_z * IMU_GYRO_2000_SEN - Gyro_offset[2];

        gyro_data = *data;
        
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        // TODO: 丢弃加速度计读取时的首个无效字节，解析 11 位有符号温度并换算为摄氏度。
        if (temp == nullptr) return;

        uint8_t buf[TEMP_LEN];      /* 2 */
        ReadReg(BMI088_CS_ACC, TEMP_MSB_ADDR, buf, TEMP_LEN);

        int16_t raw = (int16_t)((uint16_t)buf[0] << 3 | (buf[1] >> 5));

        /* 11 位补码转有符号 */
        if (raw > 1023) raw -= 2048;

        *temp = raw * TEMP_UNIT + TEMP_BIAS;   /* 0.125 ℃/LSB + 23℃ 偏置 */
    }

}

