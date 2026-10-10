#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    if (mode == PID_POSITION)
    {
        pResult = kp * err[0];
        iResult = Numeric::LimitABS(iResult + ki * err[0], maxIOut);
        dResult = kd * (err[0] - err[1]);
        result  = Numeric::LimitABS(pResult + iResult + dResult, maxOut);
    }
    else   /* PID_DELTA 增量式 */
    {
        dResult = kp * (err[0] - err[1])
                + ki * err[0]
                + kd * (err[0] - 2.0f * err[1] + err[2]);
        result  = Numeric::LimitABS(result + dResult, maxOut);
    }
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
