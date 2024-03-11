#include "Copter.h"
#include <stdio.h>
#include <../libraries/AP_RangeFinder/AP_RangeFinder_ETRI.h>
#include <AP_Logger/AP_Logger.h>

#if MODE_CNDN_ENABLED == ENABLED

#define USE_CNDN    1

const AP_Param::GroupInfo ModeCNDN::var_info[] = {
    // @Param: METHOD
    // @DisplayName: Mode using method
    // @Description: Mode using method of CNDN & ETRI Mission computer
    // @Values: 0: Disable, 1: All enable, 2: Take picture only, 2: Edge follow only, 3: Take picture after Edge following
    // @User: Advance
    AP_GROUPINFO_FLAGS("METHOD", 0, ModeCNDN, _method, 2, AP_PARAM_FLAG_ENABLE),

    // @Param: TAKE_ALT
    // @DisplayName: Take picture altitute
    // @Description: Altitute of take picture
    // @Units: cm
    // @Range: 100 2000
    // @User: Advance
    AP_GROUPINFO("TAKEOFF_ALT", 1, ModeCNDN, _take_alt_cm, 200),

    // @Param: MISSION_ALT
    // @DisplayName: Mission altitute
    // @Description: Altitute of mission planning
    // @Units: cm
    // @Range: 200 1000
    // @User: Advance
    AP_GROUPINFO("MISSION_ALT", 2, ModeCNDN, _mission_alt_cm, 300),

    // @Param: SPRAY_WIDTH
    // @DisplayName: Spray width
    // @Description: Mission planning width of spraying
    // @Units: cm
    // @Range: 3000 8000
    // @User: Advance
    AP_GROUPINFO("SPRAY_WIDTH", 3, ModeCNDN, _spray_width_cm, 400),

    // @Param: DIS_EDGE
    // @DisplayName: Distance Edge
    // @Description: Distance from Edge
    // @Units: cms
    // @Range: 100 800
    // @User: Advance
    AP_GROUPINFO("DIS_EDGE", 4, ModeCNDN, _dst_eg_cm, 250),

    // @Param: SPD_EDGE mission
    // @DisplayName: Speed edge
    // @Description: Mission speed for Edge
    // @Units: cm
    // @Range: 100 1000
    // @User: Advance
    AP_GROUPINFO("SPD_EDGE", 5, ModeCNDN, _spd_edge_cm, 350),

    // @Param: SPD_AUTO
    // @DisplayName: Speed auto mission
    // @Description: Mission speed for Auto
    // @Units: cm
    // @Range: 100 1000
    // @User: Advance
    AP_GROUPINFO("SPD_AUTO", 6, ModeCNDN, _spd_auto_cm, 500),

    // @Param: RADAR_FLT_HZ
    // @DisplayName: RADAR Filter Herz
    // @Description: Radar low pass filter frequency
    // @Units: Hz
    // @Range: 0.0 1.0
    // @User: Advance
    AP_GROUPINFO("RADAR_HZ", 7, ModeCNDN, _radar_flt_hz, 0.25),

    // @Param: LEVEL_PIN
    // @DisplayName: Level sensor gpio pin
    // @Description: Level sensor gpio pin
    // @Range: 0.0 1.0
    // @User: Advance
    AP_GROUPINFO("LEVEL_PIN", 8, ModeCNDN, _sensor_pin, 59),

    // @Param: AVOID_CM
    // @DisplayName: AVOIDANCE DISTANCE CM
    // @Description: Avoidance distance for break mode
    // @Range: 200 1000
    // @User: Advance
    AP_GROUPINFO("AVOID_CM", 9, ModeCNDN, _avoid_cm, 600),

    AP_GROUPEND
};

class GPIOSensor
{
private:
    GPIOSensor() {
        toTICK.reset(0);
    }
#ifdef HAL_PUMP_SENSOR_PIN
    uint8_t u_pin = HAL_PUMP_SENSOR_PIN;
    uint8_t l_pin = HAL_PUMP_SENSOR_PIN;
#else
    uint8_t u_pin = 0;
    uint8_t l_pin = 0;
#endif
    uint32_t l_ms = 0;
    uint32_t l_cn = 0;
    uint32_t l_en = 0;
    uint32_t l_dt = 0;
    uint32_t m_count = 0;
    uint32_t l_count = 0;
    bool m_init = false;
    bool m_set = false;
    bool m_pin_state = false;

public:
    CNTimeout toTICK;

    static GPIOSensor& get() {
        static GPIOSensor sgpio;
        return sgpio;
    }
 
    void set_pin(uint8_t pin) {
        u_pin = pin;
        if (l_pin != u_pin) {
            l_pin = u_pin;
            m_init = false;
        }
    }

    bool isTimeout(uint32_t now, uint32_t tout) {
        if (l_ms == 0) l_ms = now;
        return (now - l_ms) > tout;
    }

    void resetTimeout(uint32_t now) { l_ms = now; }

    bool stateChanged(bool bSet) {
        if (m_set != bSet) {
            m_set = bSet;
            return true;
        }
        return false;
    }

    uint32_t getPulse() { return AP::rpm()->get_counter(0); }

    void resetCount() { AP::rpm()->reset_counter(0); }

    float getCount() {
        if (u_pin) {
            // ensure we are in input mode
            hal.gpio->pinMode(u_pin, HAL_GPIO_INPUT);
            bool bState = hal.gpio->read(u_pin); // Active Low
            return bState ? 0.0f : 1000.0f;
        }
        return 0.0f;
    }

    float getRPM() {
        return AP::rpm()->get_rpm(0);
    }
};

ModeCNDN::ModeCNDN()
{
    AP_Param::setup_object_defaults(this, var_info);
    u32_runSpray = 0;
    cmd_mode = 0;
    m_bZigZag = false;
    data_buff = NULL;
    toBAT.disable();
    toGUIDED.disable();
}

bool ModeCNDN::init(bool ignore_checks)
{
    init_speed();

    loiter_nav->init_target();
    loiter_nav->clear_pilot_desired_acceleration();

    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    stage = MANUAL;

    // 무장상태에서 자동방재ON 상태로 모드 변경이면 자동방재 계속
    if (AP::arming().is_armed()) {
        if ((copter.init_mode_reason != ModeReason::MISSION_STOP)
            && (copter.init_mode_reason != ModeReason::MISSION_STOPED)
            && (copter.init_mode_reason != ModeReason::MISSION_END)) {
            RC_Channel* cnauto = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_AUTO);
            if (cnauto && cnauto->percent_input() >= 70) {
                logdebug("CHECK RESUME MISSION: %d\n", (int)copter.init_mode_reason);
                if (resume_mission())
                    return true;
            }
        }
    }
    else
    {
        copter.rangefinder_state.enabled = false;
    }

    if (stage == MANUAL) {
        if (isZigZag()) {
            RC_Channel* cnzigzag = rc().find_channel_for_option(RC_Channel::AUX_FUNC::RANGEFINDER);
            uint8_t rtv = (cnzigzag ? cnzigzag->percent_input() : 0);
            if (rtv >= 40 && rtv <= 60) {
                stage = PREPARE_ABLINE;
                return true;
            }
            yaw_deg = degrees(ahrs.yaw);
            auto_yaw.set_fixed_yaw(yaw_deg, 0.0f, 0, false);
        }

        if (copter.init_mode_reason == ModeReason::MISSION_END) {
            gcswarning("자동방제가 종료 되었습니다");
            return_to_manual_control(false);
        } else if (copter.init_mode_reason != ModeReason::MISSION_STOP) {
            // initialise waypoint state
            gcsdebug("[CNDN] %s MODE INITIALIZED.", isZigZag()?"AB": "AREA");
            return_to_manual_control(false);
        }
    }
    return true;
}

void ModeCNDN::init_speed()
{
    wp_nav->wp_and_spline_init();
}

void ModeCNDN::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    uint32_t now = AP_HAL::millis();
    switch (stage) {
        case SET_AUTO: {
            manual_control();
            if (is_disarmed_or_landed() || !motors->get_interlock()) {
                return_to_manual_control(false);
                break;
            }
            copter.rangefinder_state.enabled = true;
            if (toAUTO.isTimeout(now, 1000)) {
                toAUTO.disable();
                copter.wp_nav->set_speed_xy(_spd_auto_cm.get());
                copter.set_mode(Mode::Number::AUTO, ModeReason::MISSION_RESUME);
                stage = MANUAL;
            }
        } break;

        case AUTO: {
            manual_control();
            init_speed();
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            copter.sprayer.run(false);
            pos_control->set_alt_target_to_current_alt();
            wp_nav->wp_and_spline_init();
            stage = SET_AUTO;
            toAUTO.reset(now);
        } break;

        case PREPARE_AUTO: {
            auto_control();
            if (cmd_mode == 2) {
                // Enable RangeFinder
                if (!copter.rangefinder_state.enabled && copter.rangefinder_state.alt_healthy)
                    copter.rangefinder_state.enabled = true;

                float yw = auto_yaw.yaw();
                float yc = degrees(ahrs.yaw);
                float dy = wrap_360_cd(yw - yc * 1e+2f);
                if (dy >  18000.0f) dy -= 36000.0f;
                if (dy < -18000.0f) dy += 36000.0f;

                if (dy*dy > 10000.0f) toYAW.reset(now);
                if (toYAW.isTimeout(now, 1000)) {
                    toYAW.disable();
                    stage = AUTO;
                    init_speed();
                    //auto_yaw.set_mode(AUTO_YAW_HOLD);
                    copter.sprayer.run(false);
                    pos_control->set_alt_target_to_current_alt();
                    wp_nav->wp_and_spline_init();
                    stage = SET_AUTO;
                    toAUTO.reset(now);
                }
            }
        } break;

        case PREPARE_ABLINE: {
            manual_control();
            stage = MANUAL;
            copter.set_mode(Mode::Number::ZIGZAG, ModeReason::RC_COMMAND);
        } break;

        default:
            manual_control();
        break;
    }
}

bool ModeCNDN::set_destination(const Vector3f &destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool yaw_relative)
{
    // ensure we are in position control mode
    pos_control_start();

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc))
    {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, yaw_relative);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    return true;
}

bool ModeCNDN::getResume(CNMIS& cms) {
    float alt_cm = 0.0f;
    if (wp_nav->get_terrain_alt(alt_cm)) {
        cms.misAlt = alt_cm;
        cms.misFrame = Location::AltFrame::ABOVE_TERRAIN;
    } else {
        cms.misFrame = Location::AltFrame::ABOVE_HOME;
        if (!copter.current_loc.get_alt_cm(cms.misFrame, cms.misAlt))
            cms.misAlt = _mission_alt_cm.get();
    }

    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    cms.addNew = true;
    cms.edge = 0;
    cms.yaw_deg = degrees(ahrs.yaw);
    cms.repl_idx = cms.jump_idx = 0;
    cms.curr_idx = AP::mission()->get_current_nav_index();
    uint16_t resumeIdx = 0;
    if (hasResume(resumeIdx) && AP::mission()->read_cmd_from_storage(resumeIdx+7, cmd) && cmd.id == MAV_CMD_DO_JUMP) {
        if (cms.curr_idx >= resumeIdx)
            cms.curr_idx = cmd.content.jump.target;
        cms.addNew = false;
        cms.repl_idx = resumeIdx;
        cms.jump_idx = resumeIdx + 7;
    }

    for(uint16_t i=1; i<AP::mission()->num_commands(); i++) {
        if (AP::mission()->read_cmd_from_storage(i, cmd)) {
            switch(cmd.id) {
                case MAV_CMD_NAV_WAYPOINT:
                    if (i > cms.curr_idx) break;
                    cms.misAlt = cmd.content.location.alt;
                    cms.misFrame = cmd.content.location.get_alt_frame();
                break;

                case MAV_CMD_CONDITION_YAW:
                    if (i > cms.curr_idx) break;
                    cms.yaw_deg = cmd.content.yaw.angle_deg;
                break;

                case MAV_CMD_DO_CHANGE_SPEED:
                    if (i > cms.curr_idx) break;
                    cms.spdcm = ((cms.edge == 4)?_spd_edge_cm.get():_spd_auto_cm.get()) * 1e-2f;
                break;

                case MAV_CMD_DO_SET_RELAY:
                    if (i > cms.curr_idx) break;
                    if (cmd.content.relay.num == 254) {
                        switch (cmd.content.relay.state) {
                            case 0: case 1: cms.spryr = cmd.content.relay.state; break;
                            case 3: case 4: cms.edge = cmd.content.relay.state; break;
                        }
                    }
                break;
            }
        }
    }
    return !cms.addNew;
}

void ModeCNDN::setResume(CNMIS& cms, bool bRemote) {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    if (cms.addNew) {
        cmd.id = MAV_CMD_NAV_TAKEOFF;
        cmd.p1 = 2;
        cmd.content.location = copter.current_loc;
        cmd.content.location.set_alt_cm(_mission_alt_cm.get(), cms.misFrame);
        AP::mission()->add_cmd(cmd);
        cms.repl_idx = cmd.index;

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.relay.num = 254;
        cmd.content.relay.state = bRemote ? 10 : 0; // 원격이면 높이 재 설정
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_CONDITION_YAW;
        cmd.p1 = 3;
        cmd.content.yaw.angle_deg = cms.yaw_deg;
        cmd.content.yaw.turn_rate_dps = 0;
        cmd.content.yaw.direction = 0;
        cmd.content.yaw.relative_angle = 0;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = copter.current_loc;
        cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
        AP::mission()->add_cmd(cmd);
        //cms.repl_idx = cmd.index;

        cmd.id = MAV_CMD_DO_CHANGE_SPEED;
        cmd.p1 = 0;
        cmd.content.speed.speed_type = 0;
        cmd.content.speed.throttle_pct = 0;
        cmd.content.speed.target_ms = cms.spdcm;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.relay.num = 254;
        cmd.content.relay.state = cms.edge;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.relay.num = 254;
        cmd.content.relay.state = cms.spryr;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_JUMP;
        cmd.p1 = 1;
        cmd.content.jump.target = cms.curr_idx;
        cmd.content.jump.num_times = -1;
        AP::mission()->add_cmd(cmd);
        cms.jump_idx = cmd.index;
        cms.addNew = false;
    } else {
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+0, cmd) && cmd.id == MAV_CMD_NAV_TAKEOFF) {
            cmd.content.location = copter.current_loc;
            cmd.content.location.set_alt_cm(_mission_alt_cm.get(), cms.misFrame);
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+2, cmd) && cmd.id == MAV_CMD_CONDITION_YAW) {
            cmd.content.yaw.angle_deg = cms.yaw_deg;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+3, cmd) && cmd.id == MAV_CMD_NAV_WAYPOINT) {
            cmd.content.location = copter.current_loc;
            cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+4, cmd) && cmd.id == MAV_CMD_DO_CHANGE_SPEED) {
            cmd.content.speed.target_ms = cms.spdcm;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+5, cmd) && cmd.id == MAV_CMD_DO_SET_RELAY) {
            cmd.content.relay.state = cms.edge;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+6, cmd) && cmd.id == MAV_CMD_DO_SET_RELAY) {
            cmd.content.relay.state = cms.spryr;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.jump_idx, cmd) && cmd.id == MAV_CMD_DO_JUMP) {
            cmd.content.jump.target = cms.curr_idx;
            cmd.content.jump.num_times = -1;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
    }
}

bool ModeCNDN::hasResume(uint16_t &resumeIdx) {
    if (!isOwnMission()) return false;

    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    uint16_t ncmds = AP::mission()->num_commands();
    for (uint16_t i=1; i<ncmds; i++) {
        if (AP::mission()->read_cmd_from_storage(i+0, cmd) && cmd.id == MAV_CMD_NAV_TAKEOFF && cmd.p1 == 2) {
            if (!AP::mission()->read_cmd_from_storage(i+1, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY) continue;
            if (!AP::mission()->read_cmd_from_storage(i+2, cmd) || cmd.id != MAV_CMD_CONDITION_YAW) continue;
            if (!AP::mission()->read_cmd_from_storage(i+3, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT) continue;
            if (i+4 > ncmds || !AP::mission()->read_cmd_from_storage(i+4, cmd) || cmd.id != MAV_CMD_DO_CHANGE_SPEED) return false;
            if (i+5 > ncmds || !AP::mission()->read_cmd_from_storage(i+5, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY) return false;
            if (i+7 > ncmds || !AP::mission()->read_cmd_from_storage(i+7, cmd) || cmd.id != MAV_CMD_DO_JUMP) return false;
            resumeIdx = i;
            return true;
        }
    }
    return false;
}

// save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
void ModeCNDN::mission_command(uint8_t dest_num)
{
    bool bControlled = copter.flightmode == &copter.mode_auto || copter.flightmode == &copter.mode_zigzag;

    if (bControlled) { // 속도/분사 제어
        float mss, prr;
        switch (dest_num) {
            case 6: // CNDN_SPD_UP
                if (copter.flightmode == &copter.mode_auto && edge_mode) {
                    mss = _spd_edge_cm.get() + 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() + 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("속도 증가: %0.2f m/s", mss * 1e-2f);
            break;

            case 7: // CNDN_SPD_DN
                if (copter.flightmode == &copter.mode_auto && edge_mode) {
                    mss = _spd_edge_cm.get() - 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() - 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("속도 감소: %0.2f m/s", mss * 1e-2f);
            break;

#if SPRAYER_ENABLED == ENABLED
            case 8: // CNDN_SPR_UP
                prr = copter.sprayer.inc_pump_rate(+2.5f);
                gcsdebug("분무량 증가: %0.1f%%", prr);
            break;
            case 9: // CNDN_SPR_DN
                prr = copter.sprayer.inc_pump_rate(-2.5f);
                gcsdebug("분무량 감소: %0.1f%%", prr);
            break;
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
            case 0: case 1: case 2:
                if (copter.flightmode == &copter.mode_zigzag)
                    copter.mode_zigzag.save_or_move_to_destination(dest_num);
            break;
#endif
        }
        return;
    }

    switch (dest_num) {
        case 20: {
            Location loc(copter.current_loc);
            Location home(AP::ahrs().get_home());
            int32_t yaws = wrap_180_cd(ahrs.yaw_sensor);
            gcs().send_cndn_trigger(home, loc, 0, 0, 2, yaws);
        } return;
    }

    if (dest_num > 2)
        return;

    // handle state machine changes
    // if (copter.flightmode != &copter.mode_cndn) {
    //     // 씨엔디엔 모드가 아닐 때
    //     return;
    // }

    switch (stage) {
    case MANUAL:
        if (_method.get() == 0)
            break;

        if (dest_num > 0) {
            if (copter.flightmode != &copter.mode_cndn)
                break;

            cmd_mode = dest_num;
            init_speed();
            Vector3f stopping_point;
            wp_nav->get_wp_stopping_point(stopping_point);
            wp_nav->set_wp_destination(stopping_point, false);

            Location loc(copter.current_loc);
            Location home(AP::ahrs().get_home());
            int32_t yaws = wrap_180_cd(ahrs.yaw_sensor);
            gcs().send_cndn_trigger(home, loc, _dst_eg_cm.get(), _spray_width_cm.get(), m_bZigZag?1:0, yaws);
            gcsdebug("[방제검색] %d,%d,%d", (int)loc.lat, (int)loc.lng, (int)yaws);

            copter.rangefinder_state.alt_cm_filt.set_cutoff_frequency(_radar_flt_hz.get());
            float alt_cm = 0.0f;
            if (wp_nav->get_terrain_alt(alt_cm)) {
                copter.surface_tracking.set_target_alt_cm(alt_cm);
            }
        } else {
            copter.rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
        }
        break;

    case PREPARE_AUTO:
        if (_method.get() == 0){
            init_speed();
            return_to_manual_control(false);
            return;
        }

        if (dest_num == 2)
            cmd_mode = dest_num;

        if (dest_num == 0) {
            init_speed();
            return_to_manual_control(false);
            return;
        }
        break;

    // case AUTO:
    // case PREPARE_ABLINE:
    // case FINISHED:
    default:
        if (dest_num == 0) {
            init_speed();
            return_to_manual_control(false);
            return;
        }
        break;
    }
}

// return manual control to the pilot
void ModeCNDN::return_to_manual_control(bool maintain_target)
{
    cmd_mode = 0;

    copter.rangefinder_state.enabled = false;

    if (stage != MANUAL) {
        stage = MANUAL;
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest);
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }
}

bool ModeCNDN::processAB()
{
    data_wpos = 0;
    uint16_t nCMDs = *(uint16_t*)(data_buff+data_wpos); data_wpos += 2;

    float alt_cm = 0.0f;
    int32_t misAlt = _mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;
    int8_t mValue = _method.get();
    if (mValue == 2 || mValue == 4) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm))
            misAlt = altCm;

        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt = alt_cm);
        }
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt);
        }
    }
    //logdebug("method %d, mission_alt: %d, alt_cm: %0.0f\n", _method.get(), misAlt, alt_cm);

    int nCmds = 0, nWays = 0;
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    Vector2f locA, locB;
    uint8_t lr = 0;
    for(int i=data_wpos; i < data_size; ) {
        switch (data_buff[i++]) {
            case MAV_CMD_DO_SET_PARAMETER: {
                uint8_t len = ((uint8_t*)data_buff)[i++];
                i += len;
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_ROI: {
                i += 4;
                i += 4;
                nCmds ++;
            } break;

            case MAV_CMD_NAV_WAYPOINT: {
                // create edge navigation
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.location.lat = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.lng = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.alt = ((uint32_t*)(data_buff+i))[0]; i += 4;
                Vector2f v2;
                switch (nWays) {
                    case 0:
                        if (cmd.content.location.get_vector_xy_from_origin_NE(locA))
                            nWays = nWays;

                        if (cmd.content.location.get_vector_xy_from_origin_NE(locA)) {
                            cmd.content.location.set_alt_cm(misAlt, misFrame);
                            v2 = locA;
                        } else {
                            logdebug("failed %d\n", nWays);
                        }
                    break;

                    case 1:
                        if (cmd.content.location.get_vector_xy_from_origin_NE(locB))
                            nWays = nWays;

                        if (cmd.content.location.get_vector_xy_from_origin_NE(locB)) {
                            cmd.content.location.set_alt_cm(misAlt, misFrame);
                            v2 = locB;
                        } else {
                            logdebug("failed %d\n", nWays);
                        }
                    break;
                }

                logdebug("W%d: %d, %d (%0.3f, %0.3f)\n", nWays, cmd.content.location.lat, cmd.content.location.lng, v2.x, v2.y);
                nWays ++;
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_RELAY: {
                cmd.id = MAV_CMD_DO_SET_RELAY;
                cmd.p1 = 0;
                cmd.content.relay.num = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.relay.state = ((uint8_t*)(data_buff+i))[0]; i += 1;
                lr = cmd.content.relay.state;
                nCmds ++;
            } break;
        }
    }

    if (nCmds == nCMDs) {
        copter.mode_zigzag.processArea(locA, locB, lr != 0);
        resumeLoc.zero();
        if (AP::arming().is_armed()) {
            last_yaw_deg = copter.mode_zigzag.cms.yaw_deg;
            auto_yaw.set_fixed_yaw(last_yaw_deg, 0.0f, 0, false);
            stage = PREPARE_AUTO;
            toYAW.reset(AP_HAL::millis());
            toAUTO.reset(AP_HAL::millis());
        }
        return true;
    } else {
        gcsdebug("[CNDN] Create mission failed(%d/%d)", nCmds, nCMDs);
        return_to_manual_control(false);
    }
    return false;
}

bool ModeCNDN::isOwnMission() {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);
#if (0)
    if (!AP::mission()->read_cmd_from_storage(2, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY || cmd.cmd.content.relay.num != 255 || (cmd.content.relay.state & 0xF0) != 0x10) {
        return false;
    }
#else
    if (!AP::mission()->read_cmd_from_storage(0, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT || cmd.p1 != 1) return false;
    if (!AP::mission()->read_cmd_from_storage(1, cmd) || cmd.id != MAV_CMD_NAV_TAKEOFF) return false;
    if (!AP::mission()->read_cmd_from_storage(2, cmd) || cmd.id != MAV_CMD_CONDITION_YAW) return false;
    if (!AP::mission()->read_cmd_from_storage(3, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT) return false;
    if (!AP::mission()->read_cmd_from_storage(4, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY) return false;
    if (!AP::mission()->read_cmd_from_storage(5, cmd) || cmd.id != MAV_CMD_DO_CHANGE_SPEED) return false;
#endif
    return true;
}

bool ModeCNDN::processArea()
{
    bool bRet = false;
    // parse data and create mission
    data_wpos = 0;
    uint16_t nCMDs = *(uint16_t*)(data_buff+data_wpos); data_wpos += 2;
    bool bRemoteArea = false;
    uint8_t cmdBuff[64];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);
    int nCmds = 0;

    float alt_cm = 0.0f;
    int32_t misAlt = _mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;
    int8_t mValue = _method.get();
    if (mValue == 2 || mValue == 4) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm))
            misAlt = altCm;

        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt = alt_cm);
        }
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt);
        }
    }
    //logdebug("method %d, mission_alt: %d, alt_cm: %0.0f, %d\n", _method.get(), misAlt, alt_cm, misFrame);

    bool get_yaw = false;
    //int nCmdWayPoint = 0;

#define PARSE_MC 1

    for(int i=data_wpos; i<data_size;) {
        uint8_t cm = (uint8_t)data_buff[i++];
        switch (cm) {
            case MAV_CMD_DO_SET_PARAMETER: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_DO_SET_PARAMETER:%d", i-1);
                //return false;
#endif
                uint8_t len = (uint8_t)(data_buff[i]); i += 1;
                // char buff[256];
                // strncpy(buff, data_buff+i, len);
                // buff[len] = 0;
                //gcsdebug("[지적]%d/%d/%d,%s", i, len, lIDX, buff);
                i += len;
                nCmds ++;
            } break;

            case MAV_CMD_NAV_TAKEOFF: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_NAV_TAKEOFF:%d", i-1);
                //return false;
#endif
                cmd.id = MAV_CMD_NAV_TAKEOFF;
                cmd.p1 = 0;
                cmd.content.location = copter.current_loc;
                cmd.content.location = AP::ahrs().get_home();
                cmd.content.location.set_alt_cm(misAlt, misFrame);
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                i += 4;
                nCmds ++;
            } break;

            case MAV_CMD_DO_CHANGE_SPEED: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_DO_CHANGE_SPEED:%d", i-1);
                //return false;
#endif
                uint8_t typ = ((uint8_t*)(data_buff+i))[0]; i += 1;
                uint8_t spd = ((uint8_t*)(data_buff+i))[0]; i += 1;
                CN_UNUSED(typ);

                cmd.id = MAV_CMD_DO_SET_RELAY;
                cmd.p1 = 0;
                cmd.content.relay.num = 254;
                cmd.content.relay.state = spd ? 4 : 3;
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                cmd.id = MAV_CMD_DO_CHANGE_SPEED;
                cmd.content.speed.speed_type = 0;
                cmd.content.speed.throttle_pct = 0;
                switch (spd) {
                    case 0: // normal speed
                        cmd.content.speed.target_ms = _spd_auto_cm.get() * 1e-2;
                    break;

                    default: // edge speed
                        cmd.content.speed.target_ms = _spd_edge_cm.get() * 1e-2;
                    break;
                }
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                nCmds ++;
            } break;

            case MAV_CMD_NAV_WAYPOINT: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_NAV_WAYPOINT:%d", i-1);
                //return false;
#endif
                // create edge navigation
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.location.lat = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.lng = ((uint32_t*)(data_buff+i))[0]; i += 4; i+= 4;
                cmd.content.location.set_alt_cm(misAlt, misFrame);
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
                // if (nCmdWayPoint == 0)
                //     nCmdWayPoint = AP::mission()->num_commands();
#endif
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_RELAY: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_DO_SET_RELAY:%d", i-1);
                //return false;
#endif
                cmd.id = MAV_CMD_DO_SET_RELAY;
                cmd.p1 = 0;
                cmd.content.relay.num   = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.relay.state = ((uint8_t*)(data_buff+i))[0]; i += 1;
                if (cmd.content.relay.num == 255 && cmd.content.relay.state == 10) {
                    //logdebug("PROCESS REMOTE AREA(%d)\n", cmd.content.relay.state);
                    cmd.content.relay.state = 0;
                    bRemoteArea = true;
                }
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_ROI: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_DO_SET_ROI:%d", i-1);
                //return false;
#endif

#if (PARSE_MC)
                // cmd.id = MAV_CMD_DO_SET_ROI;
                // cmd.p1 = 0;
                // cmd.content.location.lat = ((uint32_t*)(data_buff))[0]; i += 4;
                // cmd.content.location.lng = ((uint32_t*)(data_buff))[0]; i += 4;
                // cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
                // AP::mission()->add_cmd(cmd);
#endif
                i += 4;
                i += 4;
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_SERVO: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_DO_SET_SERVO:%d", i-1);
                //return false;
#endif
                cmd.id = MAV_CMD_DO_SET_SERVO;
                cmd.p1 = 0;
                cmd.content.servo.channel = ((uint8_t *)(data_buff+i))[0]; i += 1;
                cmd.content.servo.pwm     = ((uint16_t*)(data_buff+i))[0]; i += 2;
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                nCmds ++;
            } break;

            case MAV_CMD_CONDITION_YAW: {
#if !(PARSE_MC)
                gcsdebug("MAV_CMD_CONDITION_YAW:%d", i-1);
                //return false;
#endif
                uint32_t yaw_cd = 0;
                memcpy(&yaw_cd, data_buff+i, sizeof(yaw_cd)); i += 4;

                cmd.id = MAV_CMD_CONDITION_YAW;
                cmd.p1 = 1;
                cmd.content.yaw.angle_deg = yaw_cd * 1e-2f;
                cmd.content.yaw.turn_rate_dps = 0;
                cmd.content.yaw.direction = 0;
                cmd.content.yaw.relative_angle = 0;
#if (PARSE_MC)
                AP::mission()->add_cmd(cmd);
#endif
                if (!get_yaw) {
                    get_yaw = true;
                    last_yaw_deg = yaw_cd * 1e-2f;
                }
                nCmds ++;
            } break;

            default: {
                gcsdebug("[CNDN] MISSION ERROR(%d/%d)", cm, i);
                i = data_size;
            } break;
        }
    }

    if (nCMDs == nCmds) {
        // create edge navigation
        if (AP::arming().is_armed()) {
            auto_yaw.set_fixed_yaw(last_yaw_deg, 0.0f, 0, false);
            stage = PREPARE_AUTO;
            toYAW.reset(AP_HAL::millis());
            toAUTO.reset(AP_HAL::millis());
        }
        if (bRemoteArea) {
#if (PARSE_MC)
            CNMIS cms = {0};
            AP::mission()->set_current_cmd(2);
            getResume(cms);
            setResume(cms, true);
#endif
            gcsdebug("[CNDN] 원격방제 생성(%d/%d)", nCmds, nCMDs);
        } else {
            gcsdebug("[CNDN] 지적방제 생성(%d/%d)", nCmds, nCMDs);
        }
        bRet = true;
    } else {
        gcsdebug("[CNDN] 생성 실패(%d/%d)", nCmds, nCMDs);
        return_to_manual_control(false);
    }
    return bRet;
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_CNDN_CONNECT: {
            mavlink_cndn_connect_t packet;
            mavlink_msg_cndn_connect_decode(&msg, &packet);
            gcsdebug("[MC] CONNECTED(No. %d)", int(packet.type));
        } break;

        case MAVLINK_MSG_ID_CNDN_DETECT: {
#if (USE_CNDN)
            if (copter.flightmode != &copter.mode_cndn && copter.flightmode != &copter.mode_loiter) {
                gcswarning("[E:500]방제모드에서 실행하세요");
                return;
            }
#endif
            mavlink_cndn_detect_t packet;
            mavlink_msg_cndn_detect_decode(&msg, &packet);
            gcsdebug("[CNDN:%d/%d] RECEIVING(%d bytes)",msg.sysid,msg.compid,int(packet.result));

            data_wpos = 0;
            data_size = packet.result;
            if (data_buff != NULL) {
                delete[] data_buff;
                data_buff = NULL;
            }

            if (data_size > 0) {
                data_sysid = msg.sysid;
                data_cmpid = msg.compid;
                data_buff = new uint8_t[data_size];
                gcs().send_cndn_request(msg.sysid, msg.compid, 1, MIN(data_size, 120), 0);
            }
        } break;

        case MAVLINK_MSG_ID_CNDN_DATA: {
            if (data_sysid != msg.sysid || data_cmpid != msg.compid) break;
            mavlink_cndn_data_t packet;
            mavlink_msg_cndn_data_decode(&msg, &packet);
            if (packet.size == 0 || packet.sess != 1) { data_wpos = 0; break; }
            memcpy((void*)(data_buff+packet.offset), packet.data, packet.size);
            data_wpos = packet.offset + packet.size;
            gcsdebug("[CNDN:%d/%d] DATA %d/%d", msg.sysid, msg.compid, packet.offset, packet.size);
            if (data_wpos < data_size) {
                uint16_t dlen = data_size - data_wpos;
                gcs().send_cndn_request(msg.sysid, msg.compid, 1, MIN(dlen, 120), data_wpos);
            } else {
                if (isZigZag()) {
                    processAB();
                    gcsinfo("[MC] ZIGZAG CREATED");
                    resumeLoc.zero();
                } else {
                    processArea();
                    gcsinfo("[MC] MISSION CREATED");
                    resumeLoc.zero();
                }
            }
        } break;

        case MAVLINK_MSG_ID_CNDN_REQUEST:
        break;

        case MAVLINK_MSG_ID_CNDN_F_OPEN:
        case MAVLINK_MSG_ID_CNDN_F_CLOSE:
        case MAVLINK_MSG_ID_CNDN_F_READ:
        case MAVLINK_MSG_ID_CNDN_F_DATA:
        case MAVLINK_MSG_ID_CNDN_F_RESULT:
        break;

        case MAVLINK_MSG_ID_CNDN_REMOTE: {
            mavlink_cndn_remote_t packet;
            mavlink_msg_cndn_remote_decode(&msg, &packet);
            logdebug("CNDN REMOTE: %d\n", packet.command);
            switch (packet.command) {
                case 1: // remote command trigger
                {
#if (USE_CNDN)
                    if (copter.flightmode != &copter.mode_cndn && copter.flightmode != &copter.mode_loiter) {
                        gcswarning("[E:500]방제모드에서 실행하세요");
                        return;
                    }
#endif
                    uint32_t *p = (uint32_t *)(packet.param);
                    uint32_t lat = p[0];
                    uint32_t lng = p[1];

                    Location loc(lat, lng, 3.0f, Location::AltFrame::ABOVE_HOME);
                    Location home(loc);
                    int32_t yaws = wrap_180_cd(ahrs.yaw_sensor);
                    gcs().send_cndn_trigger(home, loc, _dst_eg_cm.get(), _spray_width_cm.get(), (m_bZigZag?1:0)+16, yaws);
                    gcsdebug("[원격방제검색] %d,%d,%d", (int)loc.lat, (int)loc.lng, (int)yaws);
                } break;
            }
        } break;

        case MAVLINK_MSG_ID_CNDN_F_SESS: {
            // mavlink_cndn_request_t packet;
            // mavlink_msg_cndn_request_decode(&msg, &packet);
        } break;

        case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
            mavlink_named_value_int_t packet;
            mavlink_msg_named_value_int_decode(&msg, &packet);
            if (strncmp(packet.name,"FLOWCOUNT",9) == 0) {
                if (packet.value == 0) GPIOSensor::get().resetCount();
                gcs().send_named_float("FLOWCOUNT", GPIOSensor::get().getPulse()*1.0f);
            } else if (strncmp(packet.name,"FLOWLEVEL",9) == 0) {
                if (packet.value == 0) {
                    // mission break
#if SPRAYER_ENABLED == ENABLED
                    if (copter.sprayer.is_active()) {
                        copter.sprayer.set_empty(true);
                        AP_Notify::flags.sprayer_empty = true;
                    }
                    copter.sprayer.test_pump(false);
#endif
                }
            }
        } break;
    }
}

void ModeCNDN::pos_control_start()
{
    // initialise waypoint and spline controller
    init_speed();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void ModeCNDN::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    float target_climb_rate = 0.0f;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // call z-axis position controller (wp_nav should have already updated its alt target)
        // get pilot desired climb rate
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

    float roll_target = wp_nav->get_roll();
    float pitch_target = wp_nav->get_pitch();

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(roll_target, pitch_target, copter.aparm.angle_max);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
#endif

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_target, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_target, pitch_target, auto_yaw.yaw(), true);
    }

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

void ModeCNDN::manual_control()
{
    if (isZigZag()) {
        zigzag_manual();
    } else {
        copter.mode_loiter.run();
    }
}

bool ModeCNDN::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        reach_wp_time_ms = 0;
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > CNDN_WP_RADIUS_CM) {
        reach_wp_time_ms = 0;
        return false;
    }

    // wait at least one second
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0)
        reach_wp_time_ms = now;

    return ((now - reach_wp_time_ms) > 1000);
}

void ModeCNDN::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle) {
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

void ModeCNDN::do_set_relay(const AP_Mission::Mission_Command& cmd) {
    logdebug("CMD(%d) RELAY SET %d:%d\n", cmd.index, cmd.content.relay.num, cmd.content.relay.state);
    if (cmd.content.relay.num == 255) {
        switch (cmd.content.relay.state) {
            case 0:
                AP::mission()->truncate(cmd.index + 1);
                copter.mode_auto.mission.stop();
                copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_END);
            break;

            case 1: case 2:
                copter.mode_zigzag.turnZigZag(cmd.content.relay.state);
            break;
        }
    } else if (cmd.content.relay.num == 254) {
        switch (cmd.content.relay.state) {
            case 3:
            case 4:
                edge_mode = (cmd.content.relay.state == 4);
            break;

            case 0:
            case 1: {
                bool bRun = (cmd.content.relay.state == 1);
                u32_runSpray = bRun ? 1 : 0;
#if SPRAYER_ENABLED == ENABLED
                copter.sprayer.run(bRun);
#endif
            } break;

            case 10: {
                bool bRun = (cmd.content.relay.state == 1);
                u32_runSpray = bRun ? 1 : 0;
#if SPRAYER_ENABLED == ENABLED
                copter.sprayer.run(bRun);
#endif
                // set alt at current
                float alt_cm = 0.0f;
                int32_t misAlt = _mission_alt_cm.get();
                Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;
                logdebug("R:10:%d\n", 0);
                int8_t mValue = _method.get();
                if (mValue == 2 || mValue == 4) {
                    int32_t altCm = 0;
                    logdebug("R:10:%d\n", 1);
                    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, altCm))
                    {
                        misAlt = inertial_nav.get_altitude();
                        misFrame = copter.current_loc.get_alt_frame();
                        logdebug("R:10:%d, %0.2d\n", misAlt, altCm);
                    }
                    if (wp_nav->get_terrain_alt(alt_cm)) {
                        misFrame = Location::AltFrame::ABOVE_TERRAIN;
                        copter.surface_tracking.set_target_alt_cm(misAlt = alt_cm);
                        logdebug("R:10:%d\n", 3);
                    }
                } else if (copter.rangefinder_state.alt_healthy) {
                    logdebug("R:10:%d\n", 4);
                    misFrame = Location::AltFrame::ABOVE_TERRAIN;
                    if (wp_nav->get_terrain_alt(alt_cm)) {
                        misFrame = Location::AltFrame::ABOVE_TERRAIN;
                        copter.surface_tracking.set_target_alt_cm(misAlt);
                        logdebug("R:10:%d\n", 5);
                    }
                }
                logdebug("NAV_WAYPOINT misAlt: %d, misFrame: %d\n", misAlt, (int)misFrame);
                for(uint16_t i=0; i<AP::mission()->num_commands(); i++) {
                    uint8_t cmdBuff[32];
                    AP_Mission::Mission_Command &xmd = *((AP_Mission::Mission_Command*)cmdBuff);
                    if (AP::mission()->read_cmd_from_storage(i, xmd)) {
                        switch(xmd.id) {
                            case MAV_CMD_NAV_TAKEOFF:
                            case MAV_CMD_NAV_WAYPOINT:
                                xmd.content.location.set_alt_cm(misAlt, misFrame);
                                AP::mission()->replace_cmd(xmd.index, xmd);
                            break;
                        }
                    }
                }
            } break;
        }
    }
}

void ModeCNDN::stop_mission() {
    if (copter.mode_auto.mission.state() == AP_Mission::mission_state::MISSION_RUNNING) {
        CNMIS cms = {0};
        copter.mode_auto.mission.stop();
        copter.sprayer.run(false);
        if (copter.mode_zigzag.isOwnMission()) {
            copter.mode_zigzag.getResume();
            copter.mode_zigzag.setResume();
            logdebug("setResume ZigZag(%d).\n", 0);
        } else if (isOwnMission()) {
            logdebug("setResume Auto(%d).\n", 0);
            getResume(cms);
            setResume(cms);
        } else {
            logdebug("setResume failed(%d).\n", 0);
        }
    }
}

bool ModeCNDN::resume_mission() {
    if (isZigZag()) {
        if (copter.mode_zigzag.isOwnMission() && copter.mode_zigzag.resume_mission()) {
            copter.rangefinder_state.enabled = true;
            copter.sprayer.run(false);
            stage = AUTO;
            return true;
        }
        return false;
    }

    uint16_t resumeIndex = 0;
    if (!hasResume(resumeIndex)) {
        return false;
    }

    if (AP::mission()->set_current_cmd(resumeIndex)) {
        copter.rangefinder_state.enabled = true;
        copter.sprayer.run(false);
        stage = AUTO;
        return true;
    }
    return false;
}

void ModeCNDN::inject() {
    uint32_t now = AP_HAL::millis();
#if SPRAYER_ENABLED == ENABLED
    copter.sprayer.set_fullspray(is_disarmed_or_landed() ? 1 : 0);
    // RC_Channel* cnfull = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_SPR_FF);
    // if (cnfull && cnfull->norm_input() >= 0.8f)
    //     copter.sprayer.set_fullspray(true);

    if (copter.sprayer.is_manual()) {
        RC_Channel* cnfull = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_PUMP);
        if (cnfull) {
            int pcts = cnfull->percent_input();
            copter.sprayer.set_manual_speed((pcts * 500.0f) / 100.0f);
        }
    }

    copter.sprayer.set_spreader(_method == 3 || _method == 4);

    if (!copter.sprayer.is_spreader()) {
        float ss_count = 0;
        if (_sensor_pin.get() == 59) {
            GPIOSensor::get().set_pin(_sensor_pin.get());
            ss_count = GPIOSensor::get().getCount();
        } else {
            ss_count = GPIOSensor::get().getRPM();
        }
        bool bNotEmpty = copter.sprayer.test_sensor(ss_count);
        if (!copter.sprayer.running() || !copter.sprayer.is_test_empty() || !copter.sprayer.is_active() || bNotEmpty)
            GPIOSensor::get().resetTimeout(now);

        if (bNotEmpty) {
            //AP_Notify::flags.sprayer_empty = false;
            copter.sprayer.test_pump(false);
        } else if (GPIOSensor::get().isTimeout(now, 2000)) {
            if (copter.sprayer.is_active()) {
                copter.sprayer.set_empty(true);
                AP_Notify::flags.sprayer_empty = true;
            }
            copter.sprayer.test_pump(false);
        }

        if (GPIOSensor::get().stateChanged(AP_Notify::flags.sprayer_empty)) {
            if (AP_Notify::flags.sprayer_empty)
                gcswarning("농약이 없습니다");
        }
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // if (toDBG.isTimeout(now, 1000)) {
    //     gcsdebug("RF1-h:%d,ms:%d,cm:%d", copter.rangefinder_state.alt_healthy?1:0, copter.rangefinder_state.last_healthy_ms, copter.rangefinder_state.alt_cm);
    // }
#endif

    if (copter.motors->armed()) {
        if (AP_Notify::flags.failsafe_battery || AP_Notify::flags.sprayer_empty) {
            toBAT.reset(now);
            switch (copter.control_mode) {
                case Mode::Number::AUTO:
                    if (copter.mode_auto.mission.state() == AP_Mission::mission_state::MISSION_RUNNING) {
                        if (resumeLoc.is_zero()) {
                            resumeLoc = copter.current_loc;
                            resumeLoc.alt = inertial_nav.get_altitude() + _take_alt_cm.get();
                        } else {
                            resumeLoc.lat = copter.current_loc.lat;
                            resumeLoc.lng = copter.current_loc.lng;
                            resumeLoc.alt = inertial_nav.get_altitude() + _take_alt_cm.get();
                        }
                        loiter_nav->init_target();
                        loiter_nav->clear_pilot_desired_acceleration();
                        toGUIDED.reset(AP_HAL::millis());
                        copter.mode_cndn.initMissionResume();
                        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_STOP);
                    }
                break;

                case Mode::Number::LOITER:
                    if (toGUIDED.isTimeout(AP_HAL::millis(), 5000)) {
                        toGUIDED.reset(AP_HAL::millis());
                        copter.set_mode(Mode::Number::GUIDED, ModeReason::MISSION_STOPED);
                    }
                break;

                case Mode::Number::GUIDED:
                    if (copter.init_mode_reason != ModeReason::MISSION_STOPED) {
                        toGUIDED.disable();
                        break;
                    }
                    if (toGUIDED.isTimeout(AP_HAL::millis(), 200)) {
                        toGUIDED.disable();
                        Location targetLoc = copter.current_loc;
                        targetLoc.alt += _take_alt_cm.get();
                        copter.mode_guided.set_destination(targetLoc, false, 0.0f, false, 0.0f, false);
                    }
                break;

                default:
                break;
            }
        }
#endif
    } else {
        if (AP_Notify::flags.failsafe_battery) {
            if (toBAT.isTimeout(now, 1000)) {
                AP_BattMonitor::BatteryFailsafe type = AP::battery().check_battery();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                float volt = AP::battery().voltage();
                logdebug("Battery: %0.2f V, type: %d\n", volt, (int)type);
#endif
                if (type == AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_None) {
                    AP::battery().reset_remaining(0xffff, 100.0f);
                    AP::battery().read();
                    if (!AP_Notify::flags.failsafe_battery) {
                        if (g2.proximity.get_status() != AP_Proximity::Status::Good) {
                            // proximity reinitializing
                            g2.proximity.init();
                        }
                    }
                }
            }
        }
    }
}

void ModeCNDN::inject_25hz() {
}

void ModeCNDN::inject_50hz() {
    if (copter.motors->armed()) {
#if AC_AVOID_ENABLED == ENABLED
        if (copter.control_mode == Mode::Number::AUTO && copter.mode_auto.mission.state() == AP_Mission::mission_state::MISSION_RUNNING) {
            float angle, distance;
            if (copter.avoid.enabled() && copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle, distance)) {
 #if 0
                if (toDBG.isTimeout(now, 500))
                    gcsdebug("FR: %.2f-%0.2f", angle, distance);
#endif
                if (distance <= _avoid_cm.get() * 0.01f) {
                    if (resumeLoc.is_zero()) {
                        resumeLoc = copter.current_loc;
                        resumeLoc.alt = inertial_nav.get_altitude();
                    } else {
                        resumeLoc.lat = copter.current_loc.lat;
                        resumeLoc.lng = copter.current_loc.lng;
                        resumeLoc.alt = inertial_nav.get_altitude();
                    }
                    loiter_nav->init_target();
                    loiter_nav->clear_pilot_desired_acceleration();
                    copter.set_mode(Mode::Number::BRAKE, ModeReason::MISSION_STOP);
                }
            }
        }
    }
}

void ModeCNDN::inject_400hz() {
}

void ModeCNDN::initMissionResume() {
    if (copter.init_mode_reason == ModeReason::MISSION_STOP) {
        m_missionReset = millis();
    }
}

bool ModeCNDN::hoverMissionResume() {
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }

    if (m_missionReset != 0) {
        if ((millis() - m_missionReset) >= 5000) {
            m_missionReset = 0;
        }
    } else {
        float alte = (resumeLoc.alt - inertial_nav.get_altitude()) * 1.0f;
        if (fabs(alte) < 1.0f) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
            return false;
        }
        target_climb_rate = constrain_float(alte, -get_pilot_speed_dn(), g.pilot_speed_up) * 0.5f;
        pos_control->add_takeoff_climb_rate(target_climb_rate, G_Dt);
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    loiter_nav->update();

    // call z-axis position controller (wp_nav should have already updated its alt target)
    pos_control->update_z_controller();

    //loiter_nav->update();
    target_roll = loiter_nav->get_roll();
    target_pitch = loiter_nav->get_pitch();

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif
    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, auto_yaw.yaw(), true);
    }
    return true;
}

void ModeCNDN::zigzag_manual() {
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
        float aroll = fabs(target_roll), apitch = fabs(target_pitch);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        switch (m_lockStick) {
            case 0: {
                auto_yaw.set_fixed_yaw(yaw_deg, 0.0f, 0, false);
                if (aroll > 500.0f || apitch > 500.0f) {
                    if (aroll > apitch) {
                        m_lockStick = 2;
                        Location loc = copter.current_loc;
                        loc.offset_bearing(yaw_deg + 90 * (target_roll/aroll), _spray_width_cm.get() * 1e-2f);
                        wp_nav->set_wp_destination(loc);
                    } else {
                        m_lockStick = 1;
                    }
                }
                target_roll = target_pitch = 0.0f;
                loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
            } break;

            case 1: {
                target_roll = 0.0f;
                target_yaw_rate = 0.0f;
                if (apitch < 200.0f && aroll < 200.0f) {
                    m_lockStick = 0;
                }
                // process pilot's roll and pitch input
                loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
            } break;

            case 2: {
                target_yaw_rate = 0.0f;
                if (reached_destination() && (apitch < 200.0f && aroll < 200.0f)) {
                    m_lockStick = 0;
                }
            } break;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
        m_lockStick = 0;
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms();
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        if (copter.mode_loiter.do_precision_loiter()) {
            copter.mode_loiter.precision_loiter_xy();
        }
#endif

        if (m_lockStick == 2) {
            // run waypoint and z-axis position controller
            copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
            // call attitude controller
            if (auto_yaw.mode() == AUTO_YAW_HOLD) {
                // roll & pitch from waypoint controller, yaw rate from pilot
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
            } else {
                // roll, pitch from waypoint controller, yaw heading from auto_heading()
                attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
            }
        } else {
            // run loiter controller
            loiter_nav->update();
            // call attitude controller
            if (auto_yaw.mode() == AUTO_YAW_HOLD) {
                attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
            } else {
                attitude_control->input_euler_angle_roll_pitch_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), auto_yaw.yaw(), true);
            }
        }

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

void ModeCNDN::do_aux_function_sprayer(const uint8_t ch_flag)
{
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return;
    }

    RC_Channel* cnauto = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_AUTO);
    bool bMission = (cnauto && cnauto->norm_input() >= 0.9f);

    sprayer->active((ch_flag == HIGH) || (ch_flag == MIDDLE));

    if (bMission) {
        sprayer->run((ch_flag == HIGH) || (u32_runSpray && (ch_flag == MIDDLE)));
    } else {
        sprayer->run(ch_flag >= MIDDLE);
    }
    sprayer->manual_pump(ch_flag == HIGH);
    if (AP_Notify::flags.sprayer_empty && ch_flag == LOW) {
        AP_Notify::flags.sprayer_empty = false;
        sprayer->set_empty(false);
    }
    // if we are disarmed the pilot must want to test the pump
    sprayer->test_pump((ch_flag == MIDDLE) && !hal.util->get_soft_armed());
}


bool ModeZigZag::getResume() {
    float alt_cm = 0.0f;
    if (wp_nav->get_terrain_alt(alt_cm)) {
        cms.misAlt = alt_cm;
        cms.misFrame = Location::AltFrame::ABOVE_TERRAIN;
    } else {
        cms.misFrame = Location::AltFrame::ABOVE_HOME;
        if (!copter.current_loc.get_alt_cm(cms.misFrame, cms.misAlt))
            cms.misAlt = copter.mode_cndn._mission_alt_cm.get();
    }
    cms.curr_idx = AP::mission()->get_current_nav_index();
    if (!isOwnMission()) {
        cms.addNew = true;
        return false;
    }

    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    cms.edge = 0;
    cms.spdcm = copter.mode_cndn._spd_auto_cm.get() * 1e-2f;
    cms.yaw_deg = ahrs.yaw;
    cms.repl_idx = cms.jump_idx = 0;
    uint16_t resumeIdx = 0;
    cms.addNew = true;

    if (hasResume(resumeIdx) && AP::mission()->read_cmd_from_storage(resumeIdx+6, cmd) && cmd.id == MAV_CMD_DO_JUMP) {
        if (cms.curr_idx >= resumeIdx)
            cms.curr_idx = cmd.content.jump.target;
        cms.repl_idx = resumeIdx;
        cms.jump_idx = resumeIdx + 6;
        cms.addNew = false;
    }

    for(uint16_t i=1; i<AP::mission()->num_commands(); i++) {
        if (AP::mission()->read_cmd_from_storage(i, cmd)) {
            switch(cmd.id) {
                case MAV_CMD_NAV_WAYPOINT:
                    if (i > cms.curr_idx) break;
                    cms.misAlt = cmd.content.location.alt;
                    cms.misFrame = cmd.content.location.get_alt_frame();
                break;

                case MAV_CMD_CONDITION_YAW:
                    if (i > cms.curr_idx) break;
                    cms.yaw_deg = cmd.content.yaw.angle_deg;
                break;

                case MAV_CMD_DO_CHANGE_SPEED:
                    if (i > cms.curr_idx) break;
                    cms.spdcm = ((cms.edge == 4)?copter.mode_cndn._spd_edge_cm.get():copter.mode_cndn._spd_auto_cm.get()) * 1e-2f;
                break;

                case MAV_CMD_DO_SET_RELAY:
                    if (i > cms.curr_idx) break;
                    if (cmd.content.relay.num == 254) {
                        switch (cmd.content.relay.state) {
                            case 0: case 1: cms.spryr = cmd.content.relay.state; break;
                            case 3: case 4: cms.edge = cmd.content.relay.state; break;
                        }
                    }
                break;
            }
        }
    }
    return !cms.addNew;
}

void ModeZigZag::setResume() {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    if (cms.addNew) {
        logdebug("SET RESUME POINT ZIGZAG: %d\n", 0);

        cmd.id = MAV_CMD_NAV_TAKEOFF;
        cmd.p1 = 2;
        cmd.content.location = copter.current_loc;
        cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
        AP::mission()->add_cmd(cmd);
        cms.repl_idx = cmd.index;

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.location = Location();
        cmd.content.relay.num = 254;
        cmd.content.relay.state = 0;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_CONDITION_YAW;
        cmd.p1 = 0;
        cmd.content.yaw.angle_deg = cms.yaw_deg;
        cmd.content.yaw.turn_rate_dps = 0;
        cmd.content.yaw.direction = 0;
        cmd.content.yaw.relative_angle = 0;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 1;
        cmd.content.location = copter.current_loc;
        cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
        AP::mission()->add_cmd(cmd);
        //cms.repl_idx = cmd.index;

        cmd.id = MAV_CMD_DO_CHANGE_SPEED;
        cmd.p1 = 0;
        cmd.content.speed.speed_type = 0;
        cmd.content.speed.throttle_pct = 0;
        cmd.content.speed.target_ms = cms.spdcm;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.location = Location();
        cmd.content.relay.num = 254;
        cmd.content.relay.state = cms.spryr;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_JUMP;
        cmd.p1 = 1;
        cmd.content.jump.target = cms.curr_idx;
        cmd.content.jump.num_times = -1;
        AP::mission()->add_cmd(cmd);
        cms.jump_idx = cmd.index;
        cms.addNew = false;
    } else {
        logdebug("SET RESUME POINT ZIGZAG: %d\n", 1);
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+0, cmd) && cmd.id==MAV_CMD_NAV_TAKEOFF) {
            cmd.content.location = copter.current_loc;
            cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+2, cmd) && cmd.id==MAV_CMD_CONDITION_YAW) {
            cmd.content.yaw.angle_deg = cms.yaw_deg;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+3, cmd) && cmd.id==MAV_CMD_NAV_WAYPOINT) {
            cmd.content.location = copter.current_loc;
            cmd.content.location.set_alt_cm(cms.misAlt, cms.misFrame);
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+4, cmd) && cmd.id==MAV_CMD_DO_CHANGE_SPEED) {
            cmd.content.speed.target_ms = cms.spdcm;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.repl_idx+5, cmd) && cmd.id==MAV_CMD_DO_SET_RELAY) {
            cmd.content.relay.state = cms.spryr;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
        if (AP::mission()->read_cmd_from_storage(cms.jump_idx, cmd) && cmd.id==MAV_CMD_DO_JUMP) {
            cmd.content.jump.target = cms.curr_idx;
            AP::mission()->replace_cmd(cmd.index, cmd);
        }
    }
}

bool ModeZigZag::hasResume(uint16_t &resumeIdx) {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    uint16_t ncmds = AP::mission()->num_commands();
    for(uint16_t i=1; i<ncmds; i++) {
        if (AP::mission()->read_cmd_from_storage(i, cmd) && cmd.id == MAV_CMD_NAV_TAKEOFF && cmd.p1 == 2) {
            if (!AP::mission()->read_cmd_from_storage(i+1, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY) continue;
            if (!AP::mission()->read_cmd_from_storage(i+2, cmd) || cmd.id != MAV_CMD_CONDITION_YAW) continue;
            if (i+3 > ncmds || !AP::mission()->read_cmd_from_storage(i+3, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT) return false;
            if (i+4 > ncmds || !AP::mission()->read_cmd_from_storage(i+4, cmd) || cmd.id != MAV_CMD_DO_CHANGE_SPEED) return false;
            if (i+6 > ncmds || !AP::mission()->read_cmd_from_storage(i+6, cmd) || cmd.id != MAV_CMD_DO_JUMP) return false;
            resumeIdx = i;
            return true;
        }
    }
    return false;
}

bool ModeZigZag::isOwnMission() {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

#if (0)
    if (!AP::mission()->read_cmd_from_storage(2, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY || cmd.cmd.content.relay.num != 255 || (cmd.content.relay.state & 0xF0) != 0x20) {
        return false;
    }
#else
    if (!AP::mission()->read_cmd_from_storage(0, cmd) || cmd.id != MAV_CMD_NAV_WAYPOINT || cmd.p1 != 2) return false;
    if (!AP::mission()->read_cmd_from_storage(1, cmd) || cmd.id != MAV_CMD_NAV_TAKEOFF) return false;
    if (!AP::mission()->read_cmd_from_storage(2, cmd) || cmd.id != MAV_CMD_CONDITION_YAW) return false;
    if (!AP::mission()->read_cmd_from_storage(3, cmd) || cmd.id != MAV_CMD_DO_CHANGE_SPEED) return false;
    if (!AP::mission()->read_cmd_from_storage(4, cmd) || cmd.id != MAV_CMD_DO_SET_RELAY) return false;
    if (!AP::mission()->read_cmd_from_storage(14, cmd) || cmd.id != MAV_CMD_DO_JUMP) return false;
#endif
    return true;
}

bool ModeZigZag::resume_mission() {
    uint16_t resumeIdx = 0;
    if (hasResume(resumeIdx) && AP::mission()->set_current_cmd(resumeIdx)) {
        copter.sprayer.run(false);
        return true;
    }
    return false;
}

bool ModeZigZag::processArea(Vector2f& dstA,Vector2f& dstB, bool bLeftRight) {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    float alt_cm = 0.0f;
    int32_t misAlt = copter.mode_cndn._mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;
    if (copter.mode_cndn._method.get() == 2) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm))
            misAlt = altCm;

        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt = alt_cm);
        }
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt);
        }
    }

    float dist = (dstB - dstA).length();
    if (dist < 1e-5f) {
        gcsdebug("Distance too near(%.4f).\n", dist);
        return false;
    }

    Vector2f delta = (dstB - dstA);
    float ang = degrees(atan2f(delta.y, delta.x)) + 180.0f;
    cms.yaw_deg = wrap_360(ang);

    logdebug("[지적AB] method %d, mission_alt:%d, alt_cm:%0.0f, dx:%0.2f, dy:%0.2f, deg:%0.2f\n", copter.mode_cndn._method.get(), misAlt, alt_cm,  delta.x, delta.y, ang);

    float dir = bLeftRight ? -90.0f : +90.0f;
    Location dloc_A(Vector3f(dstA.x, dstA.y, misAlt));
    Location dloc_B(Vector3f(dstB.x, dstB.y, misAlt));
    dloc_A.set_alt_cm(misAlt, misFrame);
    dloc_B.set_alt_cm(misAlt, misFrame);
    // Vector2f dA = dstA + direct;    // in NEU frame in cm relative to ekf origin
    // Vector2f dB = dstB + direct;    // in NEU frame in cm relative to ekf origin
    // Location dloc_AA(Vector3f(dA.x, dA.y, misAlt));
    // Location dloc_BA(Vector3f(dB.x, dB.y, misAlt));
    Location dloc_AA(dloc_A);
    Location dloc_BA(dloc_B);
    dloc_AA.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
    dloc_BA.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);

    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 2;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = bLeftRight ? 0 : 1;
    cmd.content.location = AP::ahrs().get_home();
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 0;
    cmd.content.yaw.angle_deg = ang;
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.speed.speed_type = 0;
    cmd.content.speed.throttle_pct = 0;
    cmd.content.speed.target_ms = copter.mode_cndn._spd_auto_cm.get() * 1e-2f;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 0;    // SPRAYER OFF
    AP::mission()->add_cmd(cmd);

    // 5
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_B;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 1;    // SPRAYER ON
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 255;
    cmd.content.relay.state = 1;    // ZIGZAG SET NEXT
    AP::mission()->add_cmd(cmd);

    // 8
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_A;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 0;    // SPRAYER OFF
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    // 10
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_AA;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 1;    // SPRAYER ON
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 255;
    cmd.content.relay.state = 2;    // ZIGZAG SET NEXT
    AP::mission()->add_cmd(cmd);

    // 13
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_BA;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_JUMP;
    cmd.p1 = 0;
    cmd.content.jump.target = 5;
    cmd.content.jump.num_times = -1;
    AP::mission()->add_cmd(cmd);

    return true;
}

bool ModeZigZag::processAB(Vector2f& dstA,Vector2f& dstB, bool bLeftRight) {
    uint8_t cmdBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);

    float alt_cm = 0.0f;
    int32_t misAlt = copter.mode_cndn._mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;
    if (copter.mode_cndn._method.get() == 2) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm))
            misAlt = altCm;

        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt = alt_cm);
        }
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
        if (wp_nav->get_terrain_alt(alt_cm)) {
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            copter.surface_tracking.set_target_alt_cm(misAlt);
        }
    }

    Vector2f delta = (dstB - dstA).normalized();

    float ang = wrap_360(-degrees(atan2f(-delta.y, delta.x)));
    cms.yaw_deg = ang;

    float dir = bLeftRight ? -90.0f : +90.0f;
    Location dloc_A(Vector3f(dstA.x, dstA.y, misAlt));
    Location dloc_B(Vector3f(dstB.x, dstB.y, misAlt));
    dloc_A.set_alt_cm(misAlt, misFrame);
    dloc_B.set_alt_cm(misAlt, misFrame);

    Location dloc_AA(dloc_A);
    Location dloc_BA(dloc_B);
    dloc_AA.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
    dloc_BA.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);


    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 2;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);

    logdebug("[일반AB] ang: %0.2f, %0.2f, %0.2f %s, p1: %d\n", ang, dir, ang+dir, bLeftRight? "LF":"RT", cmd.p1);

    cmd.id = MAV_CMD_NAV_TAKEOFF;
    cmd.p1 = bLeftRight ? 1 : 0;
    cmd.content.location = AP::ahrs().get_home();
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_CONDITION_YAW;
    cmd.p1 = 0;
    cmd.content.yaw.angle_deg = ang;
    cmd.content.yaw.turn_rate_dps = 0;
    cmd.content.yaw.direction = 0;
    cmd.content.yaw.relative_angle = 0;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_CHANGE_SPEED;
    cmd.p1 = 0;
    cmd.content.speed.speed_type = 0;
    cmd.content.speed.throttle_pct = 0;
    cmd.content.speed.target_ms = copter.mode_cndn._spd_auto_cm.get() * 1e-2f;
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 0;    // SPRAYER OFF
    AP::mission()->add_cmd(cmd);

    // 5
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_B;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 1;    // SPRAYER ON
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 255;
    cmd.content.relay.state = 1;    // ZIGZAG SET NEXT
    AP::mission()->add_cmd(cmd);

    // 8
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_A;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 0;    // SPRAYER OFF
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    // 10
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_AA;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 254;
    cmd.content.relay.state = 1;    // SPRAYER ON
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_SET_RELAY;
    cmd.p1 = 0;
    cmd.content.relay.num = 255;
    cmd.content.relay.state = 2;    // ZIGZAG SET NEXT
    AP::mission()->add_cmd(cmd);

    // 13
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 1;
    cmd.content.location = dloc_BA;
    cmd.content.location.set_alt_cm(misAlt, misFrame);
    AP::mission()->add_cmd(cmd);

    cmd.id = MAV_CMD_DO_JUMP;
    cmd.p1 = 0;
    cmd.content.jump.target = 5;
    cmd.content.jump.num_times = -1;
    AP::mission()->add_cmd(cmd);

    return true;
}

void ModeZigZag::turnZigZag(uint8_t state) {
    Vector2f dA, dB;
    uint8_t cmdBuff[32];
    uint8_t cmdABuff[32];
    uint8_t cmdBBuff[32];
    AP_Mission::Mission_Command &cmd = *((AP_Mission::Mission_Command*)cmdBuff);
    AP_Mission::Mission_Command &cmdA = *((AP_Mission::Mission_Command*)cmdABuff);
    AP_Mission::Mission_Command &cmdB = *((AP_Mission::Mission_Command*)cmdBBuff);

    switch (state) {
        case 1: //
        // 후반부 작업
        if (AP::mission()->read_cmd_from_storage(1, cmd) && cmd.id == MAV_CMD_NAV_TAKEOFF) {
            float dir = cmd.p1 ? -90.0f : +90.0f;
            if (AP::mission()->read_cmd_from_storage(2, cmd) && cmd.id == MAV_CMD_CONDITION_YAW) {
                float ang = cmd.content.yaw.angle_deg;
                if (AP::mission()->read_cmd_from_storage(5, cmdB) && cmdB.id == MAV_CMD_NAV_WAYPOINT) {
                    if (AP::mission()->read_cmd_from_storage(8, cmdA) && cmdA.id == MAV_CMD_NAV_WAYPOINT) {
                        cmdA.content.location.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
                        cmdB.content.location.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
                        cmdA.index = 10; AP::mission()->replace_cmd(cmdA.index, cmdA);
                        cmdB.index = 13; AP::mission()->replace_cmd(cmdB.index, cmdB);
                    }
                }
            }
        }
        break;

        case 2: //
        // 전반부 작업
        if (AP::mission()->read_cmd_from_storage(1, cmd) && cmd.id == MAV_CMD_NAV_TAKEOFF) {
            float dir = cmd.p1 ? -90.0f : +90.0f;
            if (AP::mission()->read_cmd_from_storage(2, cmd) && cmd.id == MAV_CMD_CONDITION_YAW) {
                float ang = cmd.content.yaw.angle_deg;
                if (AP::mission()->read_cmd_from_storage(10, cmdA) && cmdA.id == MAV_CMD_NAV_WAYPOINT) {
                    if (AP::mission()->read_cmd_from_storage(13, cmdB) && cmdB.id == MAV_CMD_NAV_WAYPOINT) {
                        cmdA.content.location.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
                        cmdB.content.location.offset_bearing(ang+dir, copter.mode_cndn._spray_width_cm.get() * 1e-2f);
                        cmdA.index = 8; AP::mission()->replace_cmd(cmdA.index, cmdA);
                        cmdB.index = 5; AP::mission()->replace_cmd(cmdB.index, cmdB);
                    }
                }
            }
        }
        break;
    }
}

#endif