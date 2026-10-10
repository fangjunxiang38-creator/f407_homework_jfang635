#include "threads.hpp"
#include "BMI088.hpp"

/* ============ 线程三件套 ============ */

static TX_THREAD imu_thread;
static uint8_t   imu_stack[1024];

/* IMU 对象 */
BMI088::cBMI088 imu;

/* ============ 线程函数 ============ */

static void imu_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    /* --- 开机时做一次：初始化 --- */
    imu.Config();               /* 写一堆配置寄存器 */
    imu.VerifyAccChipID();      /* 检查加速度计 ID */
    imu.VerifyGyroChipID();     /* 检查陀螺仪 ID */
    imu.Calibrate();            /* 陀螺仪零偏标定（此时车必须静止） */
    
        /* 初始化成功就告诉 alive 一声 */
    if (!imu.self_test.ACC_CHIP_ID_ERR &&
        !imu.self_test.GYRO_CHIP_ID_ERR &&
        !imu.self_test.CALIBRATE_ERR)
    {
        tx_semaphore_put(&imu_init_sem);
    }
    uint8_t temp_counter = 0;
    

    /* --- 主循环 --- */
    while (1)
    {
        /* 读数据 */
        imu.ReadAccData(&imu.acc_data);
        imu.ReadGyroData(&imu.gyro_data);

        /* 校验（把错误标志写进 imu.self_test） */
        imu.VerifyAccData();
        imu.VerifyGyroData();

        /* 温控：加热是慢过程，不用每 1ms 都算，10ms 一次就够 */
        if (++temp_counter >= 10)
        {
            temp_counter = 0;
            imu.TemperatureControl(40.0f);   /* 目标 40℃ */
        }

        tx_thread_sleep(1);
        
        tx_semaphore_put(&imu_alive_sem);                 
    }
}

/* ============ 创建函数 ============ */

extern "C" void imu_thread_create(void)
{
    tx_thread_create(&imu_thread,
                     (CHAR *)"imu",
                     imu_thread_entry,
                     0,
                     imu_stack,
                     sizeof(imu_stack),
                     10,                     /* 优先级 10，比 control 高一点 */
                     10,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);
}