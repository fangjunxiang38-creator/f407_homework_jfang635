#include "threads.hpp"
#include "M2006.hpp"
#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"

#include "ref.hpp"

static RefCurve curve{RefShape::Triangle, -1.0f, 1.0f, 0, 2000};

static TX_THREAD control_thread;          /* 控制块：RTOS 用它管理这个线程 */
static uint8_t   control_stack[1024];     /* 栈：1KB，存这个线程的现场 */

/* 电机对象 */
M2006 motor;

static DJIMotorHandler *handler = nullptr;

/* PID 参数 */
PIDTuning speed_tuning{1000.0f, 3.0f, 0.0f, 5000.0f, 500.0f};
PIDTuning position_tuning{20.0f, 0.0f, 0.0f, 5.0f, 5.0f};


/* ---- 给 PnX 曲线监视用的镜像变量（普通结构体 + 显式初始化，工具才认） ---- */
struct DebugData
{
    float ref;
    float fdb;
    float result;
};
DebugData dbg_spd{};
DebugData dbg_pos{};


/* 调试开关：一次只验证一种模式 */
#define CTRL_TEST_MODE  DJIMotor::POS_MODE   /* RELAX_MODE / SPD_MODE / POS_MODE */

/* ============ 线程函数 ============ */

static void control_thread_entry(ULONG thread_input)
{
    (void)thread_input;

    /* --- 开机时做一次：注册电机 --- */
    handler = DJIMotorHandler::Instance();
    handler->registerMotor(&motor, &hcan1, 0x203);

    motor.controlMode = CTRL_TEST_MODE;

    if (motor.controlMode == DJIMotor::SPD_MODE)
        motor.speedSet = 1.0f;                /* 速度环：输出轴 5 rad/s */
    else if (motor.controlMode == DJIMotor::POS_MODE)
        motor.positionSet = 1.0f;             /* 双环：输出轴转到 1 rad */

    uint32_t start_ms = tx_time_get();

    /* --- 主循环 --- */
    while (1)
    {
        motor.positionSet = curve.Value(tx_time_get() - start_ms);
        speed_tuning.ApplyTo(motor.speedPid);
        position_tuning.ApplyTo(motor.positionPid);

        motor.setOutput();                 /* 算电流（PID） */

        dbg_spd.ref    = motor.speedPid.ref;
        dbg_spd.fdb    = motor.speedPid.fdb;
        dbg_spd.result = motor.speedPid.result;

        dbg_pos.ref    = motor.positionPid.ref;
        dbg_pos.fdb    = motor.positionPid.fdb;
        dbg_pos.result = motor.positionPid.result;

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