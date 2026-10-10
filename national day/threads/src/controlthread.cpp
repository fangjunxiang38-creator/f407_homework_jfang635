#include "threads.hpp"
#include "M2006.hpp"
#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"


static TX_THREAD control_thread;          /* 控制块：RTOS 用它管理这个线程 */
static uint8_t   control_stack[1024];     /* 栈：1KB，存这个线程的现场 */

/* 电机对象（放在文件里，全局可见） */
static M2006 motor;
static DJIMotorHandler *handler = nullptr;

/* 调试开关：一次只验证一种模式 */
#define CTRL_TEST_MODE  DJIMotor::POS_MODE   /* RELAX_MODE / SPD_MODE / POS_MODE */

/* ============ 线程函数 ============ */

static void control_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    /* --- 开机时做一次：注册电机 --- */
    handler = DJIMotorHandler::Instance();
    handler->registerMotor(&motor, &hcan1, 0x201);

    motor.controlMode = CTRL_TEST_MODE;

    if (motor.controlMode == DJIMotor::SPD_MODE)
        motor.speedSet = 5.0f;                /* 速度环：输出轴 5 rad/s */
    else if (motor.controlMode == DJIMotor::POS_MODE)
        motor.positionSet = 1.0f;             /* 双环：输出轴转到 1 rad */

    /* --- 主循环 --- */
    while (1)
    {
        motor.setOutput();                 /* 算电流（PID） */
        handler->sendControlData();        /* 打包成 CAN 报文发出去 */
        handler->AllMotorAliveCheck();     /* 查一下电机在线状态 */

        tx_thread_sleep(1);                /* 睡 1ms，让出 CPU */
        tx_semaphore_put(&control_alive_sem);
    }
}

/* ============ 创建函数（给 app_threadx.c 调用） ============ */

extern "C" void control_thread_create(void)
{
    tx_thread_create(&control_thread,          /* 控制块 */
                     (CHAR *)"control",         /* 名字（调试用） */
                     control_thread_entry,      /* 入口函数 */
                     0,                         /* 传给线程的参数 */
                     control_stack,             /* 栈的起始地址 */
                     sizeof(control_stack),     /* 栈大小 */
                     11,                        /* 优先级（数字越小越高） */
                     11,                        /* 抢占阈值，一般等于优先级 */
                     TX_NO_TIME_SLICE,          /* 不用时间片轮转 */
                     TX_AUTO_START);            /* 创建后立刻启动 */
}