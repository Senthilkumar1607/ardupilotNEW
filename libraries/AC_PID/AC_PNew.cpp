/// @file	AC_PNew.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PNew.h"

const AP_Param::GroupInfo AC_PNew ::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PNew, _kp, default_kp),
    AP_GROUPEND
};

float AC_PNew::get_p(float error) const
{
    return (float)error * _kp;
}

void AC_PNew::load_gains()
{
    _kp.load();
}

void AC_PNew::save_gains()
{
    _kp.save();
}
