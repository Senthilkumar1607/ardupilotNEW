/// @file	AC_PNew.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PNew.h"

const AP_Param::GroupInfo AC_PNew ::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PNew, _kp, default_kp),
   
    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, AC_PNew, _kd, default_kd),

    AP_GROUPEND
};
AC_PNew::AC_PNew(float initial_p,float initial_d) :
    default_kp(initial_p),
    default_kd(initial_d)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_pid_info, 0, sizeof(_pid_info));

}
float AC_PNew::update_all(float target, float measurement, float dt)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }
    float error = _error;
    // float target_last = _target;
    //float error = _target - measurement;

    float P_out = (error * _kp);
    float D_out = (_derivative * _kd);

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + D_out;
}
float AC_PNew::update_error(float error, float dt)
{
      // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }
    _target = 0.0;
    const float output = update_all(0.0, -error, dt);

    // Make sure logged target and actual are still 0 to maintain behaviour
    _pid_info.target = 0.0;
    _pid_info.actual = 0.0;

    return output;
}
//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
/*float AC_PNew::get_p(float error) const
{
    return (float)error * _kp;
}*/
float AC_PNew::get_p() const
{
    return _pid_info.P;
}

float AC_PNew::get_d() const
{
    return _pid_info.D;
}

void AC_PNew::load_gains()
{
    _kp.load();
    _kd.load();
}

void AC_PNew::save_gains()
{
    _kp.save();
    _kd.save();
}
void AC_PNew::operator()(float p_val, float d_val)
{
    _kp.set(p_val);
    _kd.set(d_val);
}
