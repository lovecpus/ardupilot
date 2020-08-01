#include "Copter.h"
#include <stdio.h>
#include <../libraries/AP_RangeFinder/AP_RangeFinder_ETRI.h>

#if MODE_CNDN_ENABLED == ENABLED

const AP_Param::GroupInfo ModeCNDN::var_info[] = {
    // @Param: METHOD
    // @DisplayName: Mode using method
    // @Description: Mode using method of CNDN & ETRI Mission computer
    // @Values: 0: Disable, 1: All enable, 2: Take picture only, 2: Edge follow only, 3: Take picture after Edge following
    // @User: Standard
    AP_GROUPINFO_FLAGS("METHOD", 0, ModeCNDN, _method, 2, AP_PARAM_FLAG_ENABLE),

    // @Param: TAKE_ALT
    // @DisplayName: Take picture altitute
    // @Description: Altitute of take picture
    // @Units: cm
    // @Range: 100 2000
    // @User: Standard
    AP_GROUPINFO("TAKEOFF_ALT", 1, ModeCNDN, _take_alt_cm, 300),

    // @Param: MISSION_ALT
    // @DisplayName: Mission altitute
    // @Description: Altitute of mission planning
    // @Units: cm
    // @Range: 200 1000
    // @User: Standard
    AP_GROUPINFO("MISSION_ALT", 2, ModeCNDN, _mission_alt_cm, 300),

    // @Param: SPRAY_WIDTH
    // @DisplayName: Spray width
    // @Description: Mission planning width of spraying
    // @Units: cm
    // @Range: 3000 8000
    // @User: Standard
    AP_GROUPINFO("SPRAY_WIDTH", 3, ModeCNDN, _spray_width_cm, 400),

    // @Param: DIS_EDGE
    // @DisplayName: Distance Edge
    // @Description: Distance from Edge
    // @Units: cms
    // @Range: 100 800
    // @User: Standard
    AP_GROUPINFO("DIS_EDGE", 4, ModeCNDN, _dst_eg_cm, 400),

    // @Param: SPD_EDGE mission
    // @DisplayName: Speed edge
    // @Description: Mission speed for Edge
    // @Units: cm
    // @Range: 100 1000
    // @User: Standard
    AP_GROUPINFO("SPD_EDGE", 5, ModeCNDN, _spd_edge_cm, 350),

    // @Param: SPD_AUTO
    // @DisplayName: Speed auto mission
    // @Description: Mission speed for Auto
    // @Units: cm
    // @Range: 100 1000
    // @User: Standard
    AP_GROUPINFO("SPD_AUTO", 6, ModeCNDN, _spd_auto_cm, 500),

    // @Param: RADAR_FLT_HZ
    // @DisplayName: RADAR Filter Herz
    // @Description: Radar low pass filter frequency
    // @Units: Hz
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("RADAR_HZ", 7, ModeCNDN, _radar_flt_hz, 0.25),

    AP_GROUPEND
};

ModeCNDN::ModeCNDN()
{
    AP_Param::setup_object_defaults(this, var_info);
    cmd_mode = 0;
    m_bZigZag = false;
}

bool ModeCNDN::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    init_speed();

    loiter_nav->init_target();
    loiter_nav->clear_pilot_desired_acceleration();

    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    wp_nav->set_wp_destination(stopping_point, false);

#ifdef USE_CNDN_RNG        
    copter.rangefinder_state.enabled = false;
#endif

    last_yaw_ms = 0;

    if (copter.init_mode_reason != ModeReason::MISSION_STOP && copter.init_mode_reason != ModeReason::MISSION_END) {
        RC_Channel* cnauto = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_AUTO);
        if (cnauto && cnauto->norm_input() >= 0.8f)
            resume_mission();
    }

    if (copter.init_mode_reason == ModeReason::MISSION_END) {
        gcsinfo("[CNDN] MISSION COMPLETE.");
        return_to_manual_control(false);
    } else if (copter.init_mode_reason != ModeReason::MISSION_STOP) {
        // initialise waypoint state
        gcsdebug("[CNDN] %s MODE INITIALIZED.", isZigZag()?"AB": "AREA");
        return_to_manual_control(false);
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

    switch (stage) {
        case AUTO:
        case RETURN_AUTO: {
            // if vehicle has reached destination switch to PREPARE_FINISH
            auto_control();
            if (reached_destination()) {
                stage = PREPARE_FINISH;
                last_yaw_ms = 0;
                last_yaw_cd = copter.initial_armed_bearing;
                AP_Notify::events.waypoint_complete = 1;

                gcsinfo("[CNDN] PREPARE FINISH.");

                Vector3f tpos;
                AP_Mission *_mission = AP::mission();
                uint16_t nCmds = _mission->num_commands();
                for (uint16_t i=1; i < nCmds; i++) {
                    AP_Mission::Mission_Command cmd;
                    if (_mission->read_cmd_from_storage(i, cmd)) {
                        if (cmd.id != MAV_CMD_NAV_WAYPOINT) continue;
                        if (cmd.content.location.get_vector_from_origin_NEU(tpos)) {
                            const Vector3f wp_dest = wp_nav->get_wp_destination();
                            tpos.z = wp_dest.z;
                            wp_nav->set_wp_destination(tpos, false);
                        } else {
                            return_to_manual_control(false);
                        }
                        break;
                    }
                }
                auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
            }
        } break;

        case WAY_A:
        case WAY_B: {
#ifdef USE_CNDN_RNG        
            // Enable RangeFinder
            if (!copter.rangefinder_state.enabled && copter.rangefinder.has_orientation(ROTATION_PITCH_270))
                copter.rangefinder_state.enabled = true;
#endif
            auto_control();
            uint32_t now = AP_HAL::millis();
            if (reached_destination()) {
                if (last_yaw_ms == 0)
                    last_yaw_ms = now;

                if ((now - last_yaw_ms) > 500) {
                    last_yaw_ms = now;
                    float dy = last_yaw_cd - ahrs.yaw_sensor;
                    if (dy*dy < 1000.0f) {
                        stage = FINISHED;
                        auto_yaw.set_mode(AUTO_YAW_HOLD);
                        AP_Notify::events.mission_complete = 1;
                        gcsinfo("[CNDN] FINISHING.");
                    }
                }
            }
        } break;

        case PREPARE_AUTO:
        case PREPARE_FINISH: {
            auto_control();
            uint32_t now = AP_HAL::millis();
            if (stage == PREPARE_AUTO) {
                if (cmd_mode == 2) {
#ifdef USE_CNDN_RNG        
                    // Enable RangeFinder
                    if (!copter.rangefinder_state.enabled && copter.rangefinder.has_orientation(ROTATION_PITCH_270))
                        copter.rangefinder_state.enabled = true;
#endif
                    if (last_yaw_ms == 0)
                        last_yaw_ms = now;

                    if ((now - last_yaw_ms) > 1000) {
                        last_yaw_ms = now;
                        float dy = last_yaw_cd - ahrs.yaw_sensor;
                        if (dy*dy < 1000.0f) {
                            stage = AUTO;
                            init_speed();
                            auto_yaw.set_mode(AUTO_YAW_HOLD);
                            copter.sprayer.run(false);
                            copter.set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
                            gcsinfo("[CNDN] GO WITH MISSIONS.");
                        }
                    }
                }
            } else if (stage == PREPARE_FINISH) {
                if (reached_destination()) {
                    if (last_yaw_ms == 0)
                        last_yaw_ms = now;

                    if ((now - last_yaw_ms) > 500) {
                        last_yaw_ms = now;
                        float dy = last_yaw_cd - ahrs.yaw_sensor;
                        if (dy*dy < 1000.0f) {
                            stage = FINISHED;
                            auto_yaw.set_mode(AUTO_YAW_HOLD);
                            AP_Notify::events.mission_complete = 1;
                            gcsinfo("[CNDN] FINISHING.");
                        }
                    }
                }
            }
        } break;

        case MANUAL:
        case FINISHED:
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

bool ModeCNDN::getResume(Mode::CNMIS& dat) {
    AP_Mission::Mission_Command cmd;

    dat.misFrame = Location::AltFrame::ABOVE_HOME;
    if (!copter.current_loc.get_alt_cm(dat.misFrame, dat.misAlt))
        dat.misAlt = _mission_alt_cm.get();
#ifdef USE_CNDN_RNG
    if (copter.rangefinder_state.alt_healthy) {
        dat.misAlt = copter.rangefinder_state.alt_cm_filt.get();
        dat.misFrame = Location::AltFrame::ABOVE_TERRAIN;
    }
#endif

    dat.repl_idx = dat.jump_idx = 0;
    if (dat.curr_idx == 0)
        dat.curr_idx = AP::mission()->get_current_nav_cmd().index;

    if (dat.curr_idx == AP_MISSION_CMD_INDEX_NONE) {
        for(uint16_t i=0; i<AP::mission()->num_commands(); i++) {
            if (AP::mission()->read_cmd_from_storage(i, cmd) && cmd.id == MAV_CMD_DO_JUMP) {
                dat.curr_idx = cmd.content.jump.target;
            }
        }
    }

    for(uint16_t i=0; i<AP::mission()->num_commands(); i++) {
        if (AP::mission()->read_cmd_from_storage(i, cmd)) {
            switch(cmd.id) {
                case MAV_CMD_NAV_WAYPOINT:
                    if (i > dat.curr_idx) {
                        if (cmd.p1 == 3) dat.repl_idx = i;
                    } else {
                        dat.loctg = cmd.content.location;
                    }
                break;

                case MAV_CMD_CONDITION_YAW:
                    if (i >= dat.curr_idx) break;
                    dat.yawcd = cmd.content.yaw.angle_deg;
                break;

                case MAV_CMD_DO_CHANGE_SPEED:
                    if (i >= dat.curr_idx) break;
                    dat.spdcm = cmd.content.speed.target_ms;
                break;

                case MAV_CMD_DO_SET_RELAY:
                    if (i >= dat.curr_idx) break;
                    if (cmd.content.relay.num == 254) {
                        switch (cmd.content.relay.state) {
                            case 0: case 1: dat.spryr = cmd.content.relay.state; break;
                            case 3: case 4: dat.edge = cmd.content.relay.state; break;
                        }
                    }
                break;
                case MAV_CMD_DO_JUMP:
                    if (i <= dat.curr_idx) break;
                    dat.jump_idx = i;
                break;
            }
        }
    }
    dat.addNew = !(dat.repl_idx && dat.jump_idx);
    return !dat.addNew;
}

void ModeCNDN::setResume(Mode::CNMIS& dat) {
    AP_Mission::Mission_Command cmd;

    if (dat.addNew) {
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.p1 = 3;
        cmd.content.location = copter.current_loc;
        cmd.content.location.set_alt_cm(dat.misAlt, dat.misFrame);
        AP::mission()->add_cmd(cmd);
        dat.repl_idx = cmd.index;

        cmd.id = MAV_CMD_DO_CHANGE_SPEED;
        cmd.p1 = 0;
        cmd.content.speed.speed_type = 0;
        cmd.content.speed.throttle_pct = 0;
        cmd.content.speed.target_ms = dat.spdcm;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.location = Location();
        cmd.content.relay.num = 254;
        cmd.content.relay.state = dat.edge;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_CONDITION_YAW;
        cmd.p1 = 0;
        cmd.content.yaw.angle_deg = dat.yawcd;
        cmd.content.yaw.turn_rate_dps = 0;
        cmd.content.yaw.direction = 0;
        cmd.content.yaw.relative_angle = 0;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_SET_RELAY;
        cmd.p1 = 0;
        cmd.content.location = Location();
        cmd.content.relay.num = 254;
        cmd.content.relay.state = dat.spryr;
        AP::mission()->add_cmd(cmd);

        cmd.id = MAV_CMD_DO_JUMP;
        cmd.p1 = 0;
        cmd.content.jump.target = dat.curr_idx;
        cmd.content.jump.num_times = -1;
        AP::mission()->add_cmd(cmd);
        dat.jump_idx = cmd.index;
        dat.addNew = !(dat.repl_idx && dat.jump_idx);

        gcsinfo("[CNDN] ADD RESUME POSITION.");
    } else {
        for(uint16_t i=dat.repl_idx; i<dat.jump_idx+1; i++) {
            if (AP::mission()->read_cmd_from_storage(i, cmd)) {
                switch (cmd.id) {
                    case MAV_CMD_NAV_WAYPOINT:
                        cmd.content.jump.target = dat.curr_idx;
                        cmd.content.jump.num_times = 5;
                        cmd.content.location = copter.current_loc;
                        cmd.content.location.set_alt_cm(dat.misAlt, dat.misFrame);
                        AP::mission()->replace_cmd(i, cmd);
                    break;
                    case MAV_CMD_CONDITION_YAW:
                        cmd.content.yaw.angle_deg = dat.yawcd;
                        AP::mission()->replace_cmd(i, cmd);
                    break;
                    case MAV_CMD_DO_CHANGE_SPEED:
                        cmd.content.speed.target_ms = dat.spdcm;
                        AP::mission()->replace_cmd(i, cmd);
                    break;
                    case MAV_CMD_DO_SET_RELAY:
                        if (cmd.content.relay.num != 254) break;
                        switch (cmd.content.relay.state) {
                            case 0: case 1: cmd.content.relay.state = dat.spryr; break;
                            case 3: case 4: cmd.content.relay.state = dat.edge; break;
                        }
                        AP::mission()->replace_cmd(i, cmd);
                    break;
                    case MAV_CMD_DO_JUMP:
                        cmd.content.jump.target = dat.curr_idx;
                        cmd.content.jump.num_times = -1;
                        AP::mission()->replace_cmd(i, cmd);
                    break;
                }
            }
        }
        gcsinfo("[CNDN] REPLACE RESUME POSITION.");
    }
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
                    mss = MAX(100, MIN(1500, mss));
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() + 50.0f;
                    mss = MAX(100, MIN(1500, mss));
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("[CNDN] Change Speed to :%0.2f m/s", mss * 1e-2);
            break;

            case 7: // CNDN_SPD_DN
                if (copter.flightmode == &copter.mode_auto && edge_mode) {
                    mss = _spd_edge_cm.get() - 50.0f;
                    mss = MAX(100, MIN(1500, mss));
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() - 50.0f;
                    mss = MAX(100, MIN(1500, mss));
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("[CNDN] Change Speed to :%0.2f m/s", mss * 1e-2);
            break;

#if SPRAYER_ENABLED == ENABLED
            case 8: // CNDN_SPR_UP
                prr = copter.sprayer.inc_pump_rate(+2.5);
                gcsdebug("[CNDN] Sprayer pump rate %0.1f%%", prr);
            break;
            case 9: // CNDN_SPR_DN
                prr = copter.sprayer.inc_pump_rate(-2.5);
                gcsdebug("[CNDN] Sprayer pump rate %0.1f%%", prr);
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
    } else if (copter.flightmode != &copter.mode_cndn) {
        // 씨엔디엔 모드가 아닐 때
        return;
    }

    if (dest_num > 2)
        return;

    // handle state machine changes
    switch (stage) {
    case MANUAL:
        if (_method.get() == 0)
            break;

        if (dest_num > 0) {
            cmd_mode = dest_num;

            init_speed();
            Vector3f stopping_point;
            wp_nav->get_wp_stopping_point(stopping_point);
            wp_nav->set_wp_destination(stopping_point, false);

            Location loc(copter.current_loc);
            Location home(AP::ahrs().get_home());
            gcs().send_cndn_trigger(home, loc, _dst_eg_cm.get(), _spray_width_cm.get(), m_bZigZag?1:0, (int16_t)ahrs.yaw_sensor);
            gcsdebug("[CNDN] TRIGGER SEND.[%d,%d]", (int)loc.lat, (int)loc.lng);
            copter.rangefinder_state.alt_cm_filt.set_cutoff_frequency(_radar_flt_hz.get());
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

    case AUTO:
    case RETURN_AUTO:
    case PREPARE_FINISH:
    case FINISHED:
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
#ifdef USE_CNDN_RNG        
    copter.rangefinder_state.enabled = false;
#endif    
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
        gcsinfo("[CNDN] MANUAL CONTROL");
    }
}

void ModeCNDN::processAB()
{
#if 0    
    data_wpos = 0;
    uint16_t nCMDs = *(uint16_t*)(data_buff+data_wpos); data_wpos += 2;
    gcsdebug("[CNDN] PROCESS AB(%d commands)", int(nCMDs));

    //Location homeLoc = AP::ahrs().get_home();
    int32_t misAlt = _mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;

    if (_method.get() == 2) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm)) {
            misAlt = altCm;
        }
        misFrame = Location::AltFrame::ABOVE_HOME;
#ifdef USE_CNDN_RNG
        if (copter.rangefinder_state.alt_healthy) {
            misAlt = copter.rangefinder_state.alt_cm_filt.get();
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            gcsdebug("[CNDN] ALT Using terrain: %d", (int)misAlt);
        } else {
            gcsdebug("[CNDN] ALT Using current: %d/%d", (int)misAlt, (int)altCm);
        }
#else        
        gcsdebug("[CNDN] ALT Using current: %d/%d", (int)misAlt, (int)altCm);
#endif        
#ifdef USE_CNDN_RNG
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
#endif
    }
#endif
}

void ModeCNDN::processArea()
{
    // parse data and create mission
    data_wpos = 0;
    uint16_t nCMDs = *(uint16_t*)(data_buff+data_wpos); data_wpos += 2;

    AP_Mission::Mission_Command cmd;

    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);
    bool get_yaw = false;
    int nCmds = 0;
    //Location homeLoc = AP::ahrs().get_home();
    int32_t misAlt = _mission_alt_cm.get();
    Location::AltFrame misFrame = Location::AltFrame::ABOVE_HOME;

    if (_method.get() == 2) {
        int32_t altCm = 0;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altCm)) {
            misAlt = altCm;
        }
        misFrame = Location::AltFrame::ABOVE_HOME;
#ifdef USE_CNDN_RNG
        if (copter.rangefinder_state.alt_healthy) {
            misAlt = copter.rangefinder_state.alt_cm_filt.get();
            misFrame = Location::AltFrame::ABOVE_TERRAIN;
            gcsdebug("[CNDN] ALT Using terrain: %d", (int)misAlt);
        } else {
            gcsdebug("[CNDN] ALT Using current: %d/%d", (int)misAlt, (int)altCm);
        }
#else        
        gcsdebug("[CNDN] ALT Using current: %d/%d", (int)misAlt, (int)altCm);
#endif        
#ifdef USE_CNDN_RNG
    } else if (copter.rangefinder_state.alt_healthy) {
        misFrame = Location::AltFrame::ABOVE_TERRAIN;
#endif
    }

    for(int i=data_wpos; i < data_size; ) {
        switch ((uint8_t)data_buff[i++]) {
            case MAV_CMD_NAV_TAKEOFF: {
                cmd.id = MAV_CMD_NAV_TAKEOFF;
                cmd.p1 = 0;
                cmd.content.location = AP::ahrs().get_home();
                cmd.content.location.set_alt_cm(misAlt, misFrame);
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;

            case MAV_CMD_DO_CHANGE_SPEED: {
                uint8_t typ = ((uint8_t*)(data_buff+i))[0]; i += 1;
                uint8_t spd = ((uint8_t*)(data_buff+i))[0]; i += 1;
                CN_UNUSED(typ);

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
                AP::mission()->add_cmd(cmd);
                nCmds ++;

                cmd.id = MAV_CMD_DO_SET_RELAY;
                cmd.p1 = 0;
                cmd.content.location = Location();
                cmd.content.relay.num = 254;
                cmd.content.relay.state = spd ? 4 : 3;
                AP::mission()->add_cmd(cmd);
            } break;

            case MAV_CMD_NAV_WAYPOINT: {
                // create edge navigation
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.location.lat = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.lng = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.set_alt_cm(misAlt, misFrame);
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;

            case MAV_CMD_CONDITION_YAW: {
                uint32_t yaw_cd = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.id = MAV_CMD_CONDITION_YAW;
                cmd.p1 = 1;
                cmd.content.yaw.angle_deg = yaw_cd / 100;
                cmd.content.yaw.turn_rate_dps = 0;
                cmd.content.yaw.direction = 0;
                cmd.content.yaw.relative_angle = 0;
                AP::mission()->add_cmd(cmd);
                if (!get_yaw) {
                    get_yaw = true;
                    last_yaw_cd = yaw_cd;
                }
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_RELAY: {
                cmd.id = MAV_CMD_DO_SET_RELAY;
                cmd.p1 = 0;
                cmd.content.location = Location();
                cmd.content.relay.num = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.relay.state = ((uint8_t*)(data_buff+i))[0]; i += 1;
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_ROI: {
                cmd.id = MAV_CMD_DO_SET_ROI;
                cmd.p1 = 0;
                cmd.content.location.lat = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.lng = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.set_alt_cm(_mission_alt_cm.get(), Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;

            case MAV_CMD_DO_SET_SERVO: {
                cmd.id = MAV_CMD_DO_SET_SERVO;
                cmd.p1 = 0;
                cmd.content.location = Location();
                cmd.content.servo.channel = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.servo.pwm = ((uint16_t*)(data_buff+i))[0]; i += 2;
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;
        }
    }

    if (nCMDs == nCmds) {
        // create edge navigation
        if (AP::arming().is_armed()) {
            auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
            stage = PREPARE_AUTO;
        }
    } else {
        gcsdebug("[CNDN] No edge detected. %d/%d", nCmds, nCMDs);
        return_to_manual_control(false);
    }
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CNDN_CONNECT: {
        mavlink_cndn_connect_t packet;
        mavlink_msg_cndn_connect_decode(&msg, &packet);
        gcsdebug("[CNDN] MC CONNECTED(No. %d)", int(packet.type));
    } break;

    case MAVLINK_MSG_ID_CNDN_DETECT: {
        if (copter.flightmode != &copter.mode_cndn) 
            return;

        mavlink_cndn_detect_t packet;
        mavlink_msg_cndn_detect_decode(&msg, &packet);
        gcsdebug("[CNDN] RECEIVED DETECT(%d bytes)", int(packet.result));

        data_size = packet.result;
        if (data_buff != NULL) {
            delete[] data_buff;
            data_buff = NULL;
        }
        if (data_size > 0) {
            data_buff = new char[data_size];
            data_wpos = 0;
            gcs().send_cndn_request(1, MIN(data_size, 120), 0);
        }
    } break;

    case MAVLINK_MSG_ID_CNDN_DATA: {
        if (copter.flightmode != &copter.mode_cndn) 
            return;

        mavlink_cndn_data_t packet;
        mavlink_msg_cndn_data_decode(&msg, &packet);
        if (packet.size == 0) {
            data_wpos = 0;
            break;
        }

        memcpy((void*)(data_buff+packet.offset), packet.data, packet.size);
        data_wpos = packet.offset + packet.size;
        if (data_wpos < data_size) {
            uint16_t dlen = data_size - data_wpos;
            gcs().send_cndn_request(1, MIN(dlen, 120), data_wpos);
        } else {
            processArea();
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
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // call z-axis position controller (wp_nav should have already updated its alt target)
    pos_control->update_z_controller();

    float roll_target = wp_nav->get_roll();
    float pitch_target = wp_nav->get_pitch();

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(roll_target, pitch_target, copter.aparm.angle_max);
#endif

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_target, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_target, pitch_target, auto_yaw.yaw(), true);
    }

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

void ModeCNDN::manual_control()
{
    copter.mode_loiter.run();
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

void ModeCNDN::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

void ModeCNDN::do_set_relay(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.relay.num == 255) {
        AP::mission()->truncate(cmd.index + 1);
        copter.mode_auto.mission.stop();
        copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_END);
    } else if (cmd.content.relay.num == 254) {
        switch (cmd.content.relay.state) {
            case 3:
            case 4:
                edge_mode = (cmd.content.relay.state == 4);
                logdebug("[CNDN] SPEED %s.\n", edge_mode?"EDGE":"NORMAL");
            break;

            case 0:
            case 1: {
                bool bRun = (cmd.content.relay.state == 1);
                logdebug("[CNDN] SPRAYER %s.\n", bRun?"RUN":"STOP");
#if SPRAYER_ENABLED == ENABLED
                copter.sprayer.run(bRun);
#endif
            } break;
        }
    }
}

void ModeCNDN::stop_mission() {
    if (copter.mode_auto.mission.state() == AP_Mission::mission_state::MISSION_RUNNING) {
        CNMIS cms = {0};
        copter.mode_auto.mission.stop();
        copter.sprayer.run(false);
        getResume(cms);
        setResume(cms);
    }
}

void ModeCNDN::resume_mission() {
    CNMIS cms;
    if (!getResume(cms)) return;

    gcsdebug("[CNDN] FOUND RESUME(%d, %d)", cms.repl_idx, cms.jump_idx);
    if (AP::mission()->set_current_cmd(cms.repl_idx)) {
        gcsdebug("[CNDN] SET CURRENT %d.", cms.repl_idx);
    }
    last_yaw_cd = cms.yawcd * 100.0f;

    if (!is_disarmed_or_landed()) {
        cmd_mode = 2;
        stage = PREPARE_AUTO;
        copter.sprayer.run(false);
        auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
        gcsinfo("[CNDN] PREPARE AUTO.");
    }
}

#if defined(HAL_PUMP_SENSOR_PIN)
class GPIOSensor
{
private:
    GPIOSensor() {
        hal.gpio->pinMode(HAL_PUMP_SENSOR_PIN, HAL_GPIO_INPUT);
    }
    uint8_t last = 0;
    uint32_t l_ms = 0;
    bool m_set = false;

public:
    static GPIOSensor& get() {
        static GPIOSensor sgpio;
        return sgpio;
    }

    uint8_t read() {
        return hal.gpio->read(HAL_PUMP_SENSOR_PIN);
    }

    bool isTimeout(uint32_t tout) {
        uint32_t now = AP_HAL::millis();
        if (l_ms == 0) l_ms = now;
        return (now - l_ms) > tout;
    }

    bool changed() {
        uint8_t curr = read();
        if (last != curr) {
            last = curr;
            l_ms = AP_HAL::millis();
            return true;
        }
        return false;
    }

    void resetTimeout() {
        l_ms = AP_HAL::millis();
    }

    bool stateChanged(bool bSet) {
        if (m_set != bSet) {
            m_set = bSet;
            return true;
        }
        return false;
    }
};
#endif

void ModeCNDN::inject() {
#if SPRAYER_ENABLED == ENABLED
    copter.sprayer.set_fullspray(is_disarmed_or_landed() ? 1 : 0);
    RC_Channel* cnfull = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_SPR_FF);
    if (cnfull && cnfull->norm_input() >= 0.8f)
        copter.sprayer.set_fullspray(true);

    if (copter.sprayer.is_manual()) {
        cnfull = rc().find_channel_for_option(RC_Channel::AUX_FUNC::CNDN_PUMP);
        if (cnfull) {
            int pcts = cnfull->percent_input();
            copter.sprayer.set_manual_speed((pcts * 500.0f) / 100.0f);
        }
    }

#if defined(HAL_PUMP_SENSOR_PIN)
    if (!copter.sprayer.spraying()) {
        GPIOSensor::get().resetTimeout();
    }

    if (GPIOSensor::get().changed()) {
        AP_Notify::flags.sprayer_empty = false;
        copter.sprayer.test_pump(false);
    } else if (copter.sprayer.spraying() && GPIOSensor::get().isTimeout(600)) {
        AP_Notify::flags.sprayer_empty = true;
        copter.sprayer.test_pump(false);
    }

    if (GPIOSensor::get().stateChanged(AP_Notify::flags.sprayer_empty)) {
        if (AP_Notify::flags.sprayer_empty)
            gcsinfo("[CNDN] SPRAYER TANK EMPTY");
    }
#endif

#else
    AP_Notify::flags.sprayer_empty = false;
#endif

    if (copter.motors->armed() && (AP_Notify::flags.failsafe_battery || AP_Notify::flags.sprayer_empty)) {
        switch (copter.control_mode) {
            case Mode::Number::AUTO:
                stop_mission();
                copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_STOP);
            break;
            case Mode::Number::ZIGZAG:
                stop_mission();
                copter.set_mode(Mode::Number::CNDN2, ModeReason::MISSION_STOP);
            break;

            default:
            break;
        }
    }
}

#endif