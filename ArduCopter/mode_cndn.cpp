#include "Copter.h"
#include <stdio.h>
#include <../libraries/AP_RangeFinder/AP_RangeFinder_ETRI.h>

#if MODE_CNDN_ENABLED == ENABLED

#define USE_ETRI DISABLED

int degNE(const Vector2f& pp) {
    Vector2f npos = pp.normalized();
    Vector2f nort(1, 0);
    int rd = (int)(nort.angle(npos) * 180 / M_PI);
    if (npos.y<0) rd = 360 - rd;
    return rd;
}

int degNE(const Vector2f& p1, const Vector2f& p2) {
    return degNE(p1-p2);
}

Vector3f locNEU(float latf, float lngf, float altf) {
    Vector3f pos;
    int32_t lat = latf * 1e7f;
    int32_t lng = lngf * 1e7f;
    const Location lc {lat,lng,(int)(altf*100),Location::AltFrame::ABSOLUTE,};
    if (lc.check_latlng() && lc.get_vector_from_origin_NEU(pos))
        return pos;
    return pos;
}

bool lineIntersection(const Vector3f& a,const Vector3f& b,const Vector3f& c,const Vector3f& d, Vector2f &o) {
	Vector3f dmc(d - c);

	Vector3f bma(b - a);
	float det = bma.x*dmc.y - bma.y*dmc.x;
	if (fabsf(det) < 1e-9f)
		return false;

	Vector3f cma(c-a);
	float cdt = cma.x*dmc.y - cma.y*dmc.x;

    Vector3f oo(a + (bma * cdt/det));
    o.x = oo.x;
    o.y = oo.y;
    return true;
}

const AP_Param::GroupInfo ModeCNDN::var_info[] = {
    // @Param: METHOD
    // @DisplayName: Mode using method
    // @Description: Mode using method of CNDN & ETRI Mission computer
    // @Values: 0: Disable, 1: All enable, 2: Take picture only, 2: Edge follow only, 3: Take picture after Edge following
    // @User: Standard
    AP_GROUPINFO_FLAGS("METHOD", 0, ModeCNDN, _method, 3, AP_PARAM_FLAG_ENABLE),

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

    AP_GROUPEND
};

ModeCNDN::ModeCNDN()
{
    AP_Param::setup_object_defaults(this, var_info);
    cmd_mode = 0;
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
    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    init_speed();
    copter.rangefinder_state.enabled = false;

    if (stage != RETURN_AUTO) {
        // initialise waypoint state
        stage = MANUAL;
        last_yaw_ms = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MODE INITIALIZED.");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MISSION COMPLETE.");
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

                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] PREPARE FINISH.");

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

        case PREPARE_AUTO:
        case PREPARE_FINISH: {
            auto_control();
            uint32_t now = AP_HAL::millis();
            if (stage == PREPARE_AUTO) {
                if (cmd_mode == 2) {
                    // Enable RangeFinder
                    if (!copter.rangefinder_state.enabled && copter.rangefinder.has_orientation(ROTATION_PITCH_270))
                        copter.rangefinder_state.enabled = true;

                    if (last_yaw_ms == 0)
                        last_yaw_ms = now;

                    if ((now - last_yaw_ms) > 500) {
                        last_yaw_ms = now;
                        float dy = last_yaw_cd - ahrs.yaw_sensor;
                        if (dy*dy < 1000.0f) {
                            stage = AUTO;
                            init_speed();
                            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] GO WITH MISSIONS.");
                            copter.set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
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
                            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] FINISHING.");
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

#if 0
void ModeCNDN::live_log(const char *fmt, ...)
{
    uint32_t now = AP_HAL::millis();
    if (live_logt_ms == 0)
        live_logt_ms = now;

    if ((now - live_logt_ms) < 200)
        return;

    live_logt_ms = now;

    va_list args;
    char buff[128];
    va_start(args, fmt);
    vsprintf(buff, fmt, args);
    va_end(args);
    gcs().send_text(MAV_SEVERITY_INFO, "%s", buff);
}
#endif

// save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
void ModeCNDN::mission_command(uint8_t dest_num)
{
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
            gcs().send_cndn_trigger(home, loc, _dst_eg_cm.get(), _spray_width_cm.get());
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] TRIGGER SEND.[%u,%u]", loc.lat, loc.lng);
        }
        break;

    case PREPARE_AUTO:
        if (_method.get() == 0)
            break;
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
        gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MANUAL CONTROL");
    }
}

void ModeCNDN::processArea()
{
    // parse data and create mission
    data_wpos = 0;
    uint16_t nCMDs = *(uint16_t*)(data_buff+data_wpos); data_wpos += 2;
    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] DATA COMPLETE(%d commands)", int(nCMDs));

    AP_Mission::Mission_Command cmd;

    AP::mission()->reset();
    AP::mission()->clear();

    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.p1 = 0;
    cmd.content.location = AP::ahrs().get_home();
    AP::mission()->add_cmd(cmd);
    bool get_yaw = false;
    int nCmds = 0;
    for(int i=data_wpos; i < data_size; ) {
        switch ((uint8_t)data_buff[i++]) {
            case MAV_CMD_NAV_TAKEOFF: {
                cmd.id = MAV_CMD_NAV_TAKEOFF;
                cmd.p1 = 0;
                cmd.content.location = AP::ahrs().get_home();
                uint32_t alt = ((uint32_t*)(data_buff+i))[0]; i += 4;
                alt = _take_alt_cm.get();
                cmd.content.location.set_alt_cm(alt, Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);
                nCmds ++;
            } break;

            case MAV_CMD_NAV_WAYPOINT: {
                // create edge navigation
                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = ((uint8_t*)(data_buff+i))[0]; i += 1;
                cmd.content.location.lat = ((uint32_t*)(data_buff+i))[0]; i += 4;
                cmd.content.location.lng = ((uint32_t*)(data_buff+i))[0]; i += 4;
                uint16_t alt = ((uint16_t*)(data_buff+i))[0]; i += 2;
                cmd.content.location.set_alt_cm(alt, Location::AltFrame::ABOVE_HOME);
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
        if (AP::arming().is_armed()) {
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MOVE TO START POINT.");
            auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
            stage = PREPARE_AUTO;
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] No edge detected. %d/%d", nCmds, nCMDs);
        return_to_manual_control(false);
    }
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CNDN_CONNECT: {
        mavlink_cndn_connect_t packet;
        mavlink_msg_cndn_connect_decode(&msg, &packet);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MC CONNECTED(No. %d)", int(packet.type));
    } break;

    case MAVLINK_MSG_ID_CNDN_DETECT: {
        if (copter.flightmode != &copter.mode_cndn) 
            return;

        mavlink_cndn_detect_t packet;
        mavlink_msg_cndn_detect_decode(&msg, &packet);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] RECEIVED DETECT(%d bytes)", int(packet.result));

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

void ModeCNDN::return_to_mode()
{
    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] RELAY TO CNDN.");
    stage = RETURN_AUTO;
    copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_END);
}

void ModeCNDN::inject()
{
/*
    if (copter.flightmode != this)
    {
        const float ofs_north = cosf(radians(ahrs.yaw_sensor)) * channel_pitch->get_control_in() * 0.001f;
        const float ofs_east  = sinf(radians(ahrs.yaw_sensor)) * channel_pitch->get_control_in() * 0.001f;
        AP::gps().set_offset_cm(ofs_north, ofs_east);
    }
    else
    {
        AP::gps().set_offset_cm(0, 0);
    }
*/
}

#endif