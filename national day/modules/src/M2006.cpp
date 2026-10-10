#include "M2006.hpp"

M2006::M2006()
{
    // TODO: 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
     /* M2006 配 C610 电调：电流指令是原始值，范围 ±10000*/
    maxCurrent = 10000;

    /* 减速比 36:1。DJIMotorHandler 解析反馈时要靠它做换算 */
    gearBox = GearBox_M2006;


}

void M2006::setOutput()
{
    // TODO: 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // TODO: 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。

    switch (controlMode)
    {
    case RELAX_MODE:
        currentSet = 0;
        speedPid.Clear();
        positionPid.Clear();
        break;

    case SPD_MODE:
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)speedPid.result;
        break;

    case POS_MODE:
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = positionPid.result;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();

        currentSet = (int16_t)speedPid.result;
        break;

    default:
        currentSet = 0;
        break;
    }

    /* 兜底限幅，保证不超出电调允许的范围 */
    if (currentSet >  (int16_t)maxCurrent) currentSet =  (int16_t)maxCurrent;
    if (currentSet < -(int16_t)maxCurrent) currentSet = -(int16_t)maxCurrent;
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
    if (AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;        /* 计数没变 → 没收到新反馈 */
    }
    else
    {
        Pre_AliveFlag = AliveFlag;
        MotorState = MOTOR_ONLINE;
    }
    return MotorState;
}
