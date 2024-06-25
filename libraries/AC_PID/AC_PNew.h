#pragma once

/// @file	AC_PD.h
/// @brief	Generic P controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AP_PIDInfo.h"

/// @class	AC_PNew
/// @brief	Object managing one P controller
class AC_PNew {
public:
    struct Defaults {
        float p;
        float i;
        float d;
    };
    /// Constructor for P that saves its settings to EEPROM
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    ///
    AC_PNew(float initial_p, float initial_d) ;
    AC_PNew(const AC_PNew::Defaults &defaults) :
        AC_PNew(
            defaults.p,
            defaults.d
    )
    {}
    /*{
        AP_Param::setup_object_defaults(this, var_info);
    }*/
    CLASS_NO_COPY(AC_PNew);
        
    float update_all(float target, float measurement, float dt);

    float get_p() const;
    float get_d() const;
    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    ///
    /// @returns		The updated control output.
    ///
    float update_error(float error, float dt);
    float get_p(float error, float dt) const;
    float get_d(float error, float dt) const;
    //float       get_p(float error) const;

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float d_val);


   /* /// @name	parameter accessors*/
  /*  //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator() (const float p) { _kp.set(p); }*/

    // accessors
    AP_Float    &kP() { return _kp; }
    const AP_Float &kP() const { return _kp; }
    AP_Float &kD() { return _kd; }

    void kP(const float v) { _kp.set(v); }
    void kD(const float v) { _kd.set(v); }

    const AP_PIDInfo& get_pid_info(void) const {return _pid_info;}

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Float  _kp;
    AP_Float _kd;

    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering
private:

    const float default_kp;
    const float default_kd;
    float _last_error;
    AP_PIDInfo _pid_info;
};
