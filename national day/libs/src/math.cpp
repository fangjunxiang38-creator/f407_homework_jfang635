#include "math.hpp"

namespace Numeric
{
    float LimitABS(float input, float maxValue)
    {
        if (input > maxValue)
            return maxValue;
        if (input < -maxValue)
            return -maxValue;
        return input;
    }

}
