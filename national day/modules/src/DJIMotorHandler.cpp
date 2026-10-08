#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"

#define MOTOR_ID_MIN  0x201
#define MOTOR_ID_MAX  0x208

#define ECD_RANGE     8192

/* M2006 换算系数（36:1 减速比） */
#define M2006_RPM2RAD 0.002908882f    /* 2π/60/36    rpm → rad/s */
#define M2006_ECD2RAD 0.000021308f    /* 2π/8191/36  编码器计数 → rad */

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // TODO: 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // TODO: 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
    /* 先校验三个参数 */
    if (motor == nullptr)                             return;
    if (hcan != &hcan1 && hcan != &hcan2)             return;
    if (canId < MOTOR_ID_MIN || canId > MOTOR_ID_MAX) return;

    /* 记下这个电机挂在哪条总线、是几号 */
    motor->hcan  = hcan;
    motor->canId = canId;

    const uint8_t bus  = (hcan == &hcan1) ? 0 : 1;
    const uint8_t slot = (uint8_t)(canId - MOTOR_ID_MIN);

    DJIMotorList[bus][slot] = motor;

    /* 设置"这条控制报文上有电机"的标志 */
    if (canId <= 0x204)          /* 0x201~0x204 归 0x200 报文管 */
    {
        if (bus == 0) CAN1_0x200_Exist = true;
        else          CAN2_0x200_Exist = true;
    }
    else                          /* 0x205~0x208 归 0x1FF 报文管 */
    {
        if (bus == 0) CAN1_0x1FF_Exist = true;
        else          CAN2_0x1FF_Exist = true;
    }
}

void DJIMotorHandler::sendControlData()
{
    // TODO: 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // TODO: 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
    /* 把每个电机的 currentSet 填进对应的控制帧 */
    for (uint8_t bus = 0; bus < 2; bus++)
    {
        for (uint8_t slot = 0; slot < 8; slot++)
        {
            DJIMotor *motor = DJIMotorList[bus][slot];
            if (motor == nullptr) continue;

            uint8_t *frame;
            uint8_t  offset;

            if (motor->canId <= 0x204)
            {
                frame  = (bus == 0) ? can1_send_data_0 : can2_send_data_0;
                offset = (uint8_t)((motor->canId - 0x201) * 2);
            }
            else
            {
                frame  = (bus == 0) ? can1_send_data_1 : can2_send_data_1;
                offset = (uint8_t)((motor->canId - 0x205) * 2);
            }

            /* 大端序：高字节在前 */
            frame[offset]     = (uint8_t)((uint16_t)motor->currentSet >> 8);
            frame[offset + 1] = (uint8_t)((uint16_t)motor->currentSet & 0xFF);
        }
    }

    /* 只发"有电机"的报文 */
    if (CAN1_0x200_Exist) CAN_Transmit(&hcan1, 0x200, can1_send_data_0, 8);
    if (CAN1_0x1FF_Exist) CAN_Transmit(&hcan1, 0x1FF, can1_send_data_1, 8);
    if (CAN2_0x200_Exist) CAN_Transmit(&hcan2, 0x200, can2_send_data_0, 8);
    if (CAN2_0x1FF_Exist) CAN_Transmit(&hcan2, 0x1FF, can2_send_data_1, 8);
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    // TODO: 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
    if (rx_data == nullptr)      return;
    if (index < 0 || index >= 8) return;

    uint8_t bus;
    if      (hcan == &hcan1) bus = 0;
    else if (hcan == &hcan2) bus = 1;
    else                     return;

    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr) return;

    UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    motor->AliveFlag++;

    /* 记下上次编码器值，下一步要算差值 */
    motor->motorFeedback.last_ecd = motor->motorFeedback.ecd;

    /* 拆字节（大端序） */
    motor->motorFeedback.ecd        = (uint16_t)(can_data[0] << 8 | can_data[1]);
    motor->motorFeedback.speed_rpm  = (int16_t)(can_data[2] << 8 | can_data[3]);
    motor->motorFeedback.currentFdb = (float)(int16_t)(can_data[4] << 8 | can_data[5]);
    motor->motorFeedback.temperatureFdb = (float)can_data[6];

    /* 换算成输出轴单位 */
    motor->motorFeedback.speedFdb = motor->motorFeedback.speed_rpm * M2006_RPM2RAD;

    /* 编码器差值，处理 8192 回绕 */
    int16_t delta = (int16_t)(motor->motorFeedback.ecd - motor->motorFeedback.last_ecd);
    if (delta >  ECD_RANGE / 2) delta -= ECD_RANGE;
    if (delta < -ECD_RANGE / 2) delta += ECD_RANGE;

    /* 位置累加 */
    motor->motorFeedback.positionFdb += delta * M2006_ECD2RAD;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    // TODO: 遍历已注册电机并调用 AliveCheck。
    for (uint8_t bus = 0; bus < 2; bus++)
        for (uint8_t i = 0; i < 8; i++)
            if (DJIMotorList[bus][i] != nullptr)
                DJIMotorList[bus][i]->AliveCheck();
}
