#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"

#ifdef __cplusplus
#include "BMI088.hpp"      
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 三个线程的创建函数，给 app_threadx.c（C 文件）调用 */
void control_thread_create(void);

void imu_thread_create(void);
void alive_thread_create(void);

/* 心跳信号量：被监控的线程往里喂，alive 线程取 */
extern TX_SEMAPHORE imu_alive_sem;
extern TX_SEMAPHORE control_alive_sem;
extern TX_SEMAPHORE imu_init_sem;   

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/* ★ IMU 对象：imu 线程负责更新，其他线程可以直接读 */
extern BMI088::cBMI088 imu;
#endif

#endif /* THREADS_HPP */