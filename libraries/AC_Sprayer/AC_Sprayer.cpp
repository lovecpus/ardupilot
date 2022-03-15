#include "AC_Sprayer.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when traveling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_BACK",   5, AC_Sprayer, _pump_back_rate, 12),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
    _manual_speed = 300.0f;
    _flags.test_empty = false;
    _flags.manual = false;
}

/*
 * Get the AP_Sprayer singleton
 */
AC_Sprayer *AC_Sprayer::_singleton;
AC_Sprayer *AC_Sprayer::get_singleton()
{
    return _singleton;
}

void AC_Sprayer::active(const bool true_false) {
    _flags.active = true_false;
}

void AC_Sprayer::run(const bool true_false)
{
    // return immediately if no change
    if (true_false == _flags.running) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = true_false && _enabled && _flags.active;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);

    _flags.spraying = false;
}

bool AC_Sprayer::test_sensor(float cn) {
    if (_spinner_pwm.get() == 0)
        return true;
    return (cn >= _spinner_pwm.get() * 1.0f);
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static int8_t bForeback = 0;
#endif    

    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump) && !SRV_Channels::function_assigned(SRV_Channel::k_sprayer_spinner)) {
        return;
    }

    // get horizontal velocity
    Vector3f velocity;
    if (!AP::ahrs().get_velocity_NED(velocity)) {
        // treat unknown velocity as zero which should lead to pump stopping
        // velocity will already be zero but this avoids a coverity warning
        velocity.zero();
    }
    float ground_speed = norm(velocity.x * 100.0f, velocity.y * 100.0f);

    // get the current time
    const uint32_t now = AP_HAL::millis();
    bool should_be_spraying = _flags.spraying;
    // check foreback
    float vx = velocity.x * 100.0f / ground_speed;
    float vy = velocity.y * 100.0f / ground_speed;
    float vw = AP::ahrs().yaw_sensor;
    float ax = cosf(vw * M_PI / 18000.0f);
    float ay = sinf(vw * M_PI / 18000.0f);
    float aw = ax*vx + ay*vy;

    int8_t should_foreback = 0;
    if (ground_speed >= _speed_min) {
        if (aw >= 0.85f) should_foreback = 1;
        else if (aw <= -0.85f) should_foreback = -1;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (bForeback != should_foreback) {
        bForeback = should_foreback;
        const char *msg = (bForeback == +1) ? "FWD" : ((bForeback == -1) ? "BAK" : "SIDE");
        gcs().send_text(MAV_SEVERITY_INFO, "SPRAY a:%0.2f,s:%0.3f[%s]", aw, ground_speed, msg);
    }
#endif

    // check our speed vs the minimum
    if (ground_speed >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    should_be_spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    } else {
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            } else {
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }

    float pct = _pump_pct_1ms.get();
    // if testing pump output speed as if traveling at 1m/s
    if (_flags.testing) {
        ground_speed = 200.0f;
        should_be_spraying = true;
        should_foreback = 1;
        pct = 20.0f;
    }

    bool bFull = _flags.fullspray != 0;
    if (_flags.manual) {
        ground_speed = _manual_speed;
        should_be_spraying = true;
        pct = 20.0f;
        bFull = true;
    }

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        float pos = ground_speed * pct;
        pos = MAX(pos, 100 * _pump_min_pct.get()); // ensure min pump speed
        pos = MIN(pos, 10000); // clamp to range
        float back = _pump_back_rate.get() * 100;
        back = MAX(back, 0); // ensure min pump speed
        back = MIN(back, 10000); // clamp to range

        if (_pump_back_rate.get() < 0) {
            // 전후방 분사
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
            SRV_Channels::move_servo(SRV_Channel::k_sprayer_spinner, pos, 0, 10000);
        } else {
            if (bFull || _flags.testing) {
                // 전후방 분사
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_spinner, pos, 0, 10000);
            } else if (should_foreback == -1) {
                // 후방 문사
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, back, 0, 10000);
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_spinner, pos, 0, 10000);
            } else if (should_foreback == +1) {
                // 전방 분사
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_spinner, back, 0, 10000);
            } else {
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, 0, 0, 10000);
                SRV_Channels::move_servo(SRV_Channel::k_sprayer_spinner, 0, 0, 10000);
            }
        }

        _flags.spraying = true;
        _flags.test_empty = pos >= (100 * _pump_min_pct.get() + (_pump_pct_1ms.get() * _speed_min.get()));
    } else {
        _flags.test_empty = false;
        stop_spraying();
    }
}

namespace AP {

AC_Sprayer *sprayer()
{
    return AC_Sprayer::get_singleton();
}

};
