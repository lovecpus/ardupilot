#include "Copter.h"
#include <stdio.h>
#include <../libraries/AP_RangeFinder/AP_RangeFinder_ETRI.h>

#if MODE_CNDN_ENABLED == ENABLED

AP_RangeFinder_ETRI *rf_rt = nullptr;
AP_RangeFinder_ETRI *rf_lf = nullptr;

int degNE(const Vector2f& pp)
{
    Vector2f npos = pp.normalized();
    Vector2f nort(1, 0);
    int rd = (int)(nort.angle(npos) * 180 / M_PI);
    if (npos.y<0) rd = 360 - rd;
    return rd;
}

int degNE(const Vector2f& p1, const Vector2f& p2)
{
    return degNE(p1-p2);
}

Vector3f locNEU(float latf, float lngf, float altf)
{
    Vector3f pos;
    int32_t lat = latf * 1e7f;
    int32_t lng = lngf * 1e7f;
    const Location lc {lat,lng,(int)(altf*100),Location::AltFrame::ABSOLUTE,};
    if (lc.check_latlng() && lc.get_vector_from_origin_NEU(pos))
        return pos;
    return pos;
}

bool inside(const CNAREA& area, const Location& loc)
{
    int cross = 0;
    Vector2f vp[4], cp(loc.lat/1e7, loc.lng/1e7);
    vp[0] = Vector2f(area.latitude1, area.longitude1);
    vp[1] = Vector2f(area.latitude2, area.longitude2);
    vp[2] = Vector2f(area.latitude3, area.longitude3);
    vp[3] = Vector2f(area.latitude4, area.longitude4);

    for(uint16_t i=0; i<4; i++)
    {
        int j=(i + 1) % 4;
        if ((vp[i].y > cp.y) != (vp[j].y > cp.y))
        {
            double aX = (vp[j].x-vp[i].x)*(cp.y-vp[i].y)/(vp[j].y-vp[i].y)+vp[i].x;
            if (cp.x < aX)
                cross ++;
        }
    }
    return (cross % 2) > 0;
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
    // @Range: 2000 4000
    // @User: Standard
    AP_GROUPINFO("TAKE_ALT", 1, ModeCNDN, _take_alt_cm, 1500),

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

    // @Param: ACC_XY
    // @DisplayName: Acceleration xy
    // @Description: ground speed acceleration
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("ACC_XY", 4, ModeCNDN, _acc_xy_cms, 200),

    // @Param: SPD_XY
    // @DisplayName: Speed xy
    // @Description: ground speed
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("SPD_XY", 5, ModeCNDN, _spd_xy_cmss, 400),

    // @Param: SPD_UP
    // @DisplayName: Z Speed Up
    // @Description: z speed for up
    // @Units: cms
    // @Range: 50 1000
    // @User: Standard
    AP_GROUPINFO("SPD_UP", 6, ModeCNDN, _spd_up_cmss, 200),

    // @Param: SPD_DN
    // @DisplayName: Z Speed Down
    // @Description: z speed for down
    // @Units: cms
    // @Range: 30 500
    // @User: Standard
    AP_GROUPINFO("SPD_DN", 7, ModeCNDN, _spd_dn_cmss, 100),

    // @Param: SPD_EDGE
    // @DisplayName: Edge Speed
    // @Description: ground Speed xy for Edge
    // @Units: cms
    // @Range: 50 2000
    // @User: Standard
    AP_GROUPINFO("SPD_EDGE", 8, ModeCNDN, _spd_eg_cmss, 300),

    // @Param: DIS_EDGE
    // @DisplayName: Distance Edge
    // @Description: Distance from Edge
    // @Units: cms
    // @Range: 100 800
    // @User: Standard
    AP_GROUPINFO("DIS_EDGE", 9, ModeCNDN, _dst_eg_cm, 200),

    AP_GROUPEND
};

ModeCNDN::ModeCNDN()
{
    AP_Param::setup_object_defaults(this, var_info);

// #if !CNDN_PARAMS
//         _method.set(3);
//         _take_alt_cm.set(1500);
//         _mission_alt_cm.set(300);
//         _spray_width_cm.set(400);
//         _acc_xy_cms.set(200);
//         _spd_xy_cmss.set(500);
//         _spd_up_cmss.set(200);
//         _spd_dn_cmss.set(100);        
//         _spd_eg_cmss.set(200);
// #endif

#if defined(SIM_LOCATION)
    if (vecAreas.empty())
    {
        CNAREA area;
        // area0
        area.latitude1  = 37.2842096f;
        area.longitude1 = 126.8735343f;
        area.latitude2  = 37.2836760f;
        area.longitude2 = 126.8727310f;
        area.latitude3  = 37.2839001f;
        area.longitude3 = 126.8725044f;
        area.latitude4  = 37.2844283f;
        area.longitude4 = 126.8733077f;
        vecAreas.push_back(area);

        // area1
        area.latitude1  = 36.1111974f;
        area.longitude1 = 127.5232490f;
        area.latitude2  = 36.1112895f;
        area.longitude2 = 127.5240063f;
        area.latitude3  = 36.1110848f;
        area.longitude3 = 127.5240575f;
        area.latitude4  = 36.1109903f;
        area.longitude4 = 127.5232995f;
        vecAreas.push_back(area);

        // area2
    	area.latitude1  = 36.11093940f;	
        area.longitude1 = 127.52330410f;
        area.latitude2  = 36.11103260f;	
        area.longitude2 = 127.52406820f;
        area.latitude3  = 36.11094920f;	
        area.longitude3 = 127.52408800f;
        area.latitude4  = 36.11084030f;	
        area.longitude4 = 127.52330820f;
        vecAreas.push_back(area);

        // area3
        area.latitude1  = 36.11078770f;
        area.longitude1 = 127.52330480f;
        area.latitude2  = 36.11089500f;
        area.longitude2 = 127.52409070f;
        area.latitude3  = 36.11080720f;
        area.longitude3 = 127.52410950f;
        area.latitude4  = 36.11068910f;
        area.longitude4 = 127.52329010f;
        vecAreas.push_back(area);

        // area4
        area.latitude1  = 36.11126150f;
        area.longitude1 = 127.52323980f;
        area.latitude2  = 36.11146870f;
        area.longitude2 = 127.52319220f;
        area.latitude3  = 36.11156560f;
        area.longitude3 = 127.52393580f;
        area.latitude4  = 36.11134240f;
        area.longitude4 = 127.52398680f;
        vecAreas.push_back(area);
    }
#endif

}

bool ModeCNDN::init(bool ignore_checks)
{
    if (!copter.failsafe.radio)
    {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    }
    else
    {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z())
    {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    if (stage != RETURN_AUTO)
    {
        // initialise waypoint state
        stage = MANUAL;
        b_position_target = false;
        last_yaw_ms = 0;

        dest_A.zero();
        dest_B.zero();

        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MODE INITIALIZED.");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MISSION COMPLETE.");
    }

    init_speed();

    return true;
}

void ModeCNDN::init_speed()
{
    wp_nav->set_speed_xy(_spd_xy_cmss.get()*1.0f);
    wp_nav->set_speed_up(_spd_up_cmss.get()*1.0f);
    wp_nav->set_speed_down(_spd_dn_cmss.get()*1.0f);
    wp_nav->wp_and_spline_init();
    pos_control->set_max_accel_xy(_acc_xy_cms.get()*1.0f);
    pos_control->calc_leash_length_xy();
}

void ModeCNDN::run()
{
    if (rf_rt == nullptr) rf_rt = (AP_RangeFinder_ETRI *)AP::rangefinder()->get_backend(1);
    if (rf_lf == nullptr) rf_lf = (AP_RangeFinder_ETRI *)AP::rangefinder()->get_backend(2);
    if (rf_rt != nullptr && rf_rt->type() != RangeFinder::RangeFinder_Type::RangeFinder_TYPE_ETRI) rf_rt = nullptr;
    if (rf_lf != nullptr && rf_lf->type() != RangeFinder::RangeFinder_Type::RangeFinder_TYPE_ETRI) rf_lf = nullptr;
    if (stage != EDGE_FOLLOW && rf_rt!=nullptr && rf_lf != nullptr){
        rf_rt->set_distance(50.0f);
        rf_lf->set_distance(50.0f);
    }

    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-_spd_dn_cmss.get()*1.0f, _spd_up_cmss.get()*1.0f);
    pos_control->set_max_accel_z(g.pilot_accel_z);
    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -_spd_dn_cmss.get()*1.0f, _spd_up_cmss.get()*1.0f);

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (stage)
    {
    case RETURN_AUTO:
    case AUTO:
    {
        // if vehicle has reached destination switch to PREPARE_FINISH
        auto_control();
        if (reached_destination())
        {
            stage = PREPARE_FINISH;
            last_yaw_ms = 0;
            last_yaw_cd = copter.initial_armed_bearing;
            AP_Notify::events.waypoint_complete = 1;
            b_position_target_reached = false;
            b_position_target = false;

            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] PREPARE FINISH.");

            const Vector3f tpos(vecRects.back().x, vecRects.back().y, _mission_alt_cm.get() * 1.0f);
            wp_nav->set_wp_destination(tpos, false);
            auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
        }
    } break;

    case PREPARE_AUTO:
    case PREPARE_FINISH:
    {
        uint32_t now = AP_HAL::millis();
        auto_control();
        if (reached_destination())
        {
            if (last_yaw_ms == 0)
                last_yaw_ms = now;

            if ((now - last_yaw_ms) > 500)
            {
                last_yaw_ms = now;
                float dy = last_yaw_cd - ahrs.yaw_sensor;
                if (dy*dy < 1000.0f)
                {
                    if (stage == PREPARE_FINISH)
                    {
                        stage = FINISHED;
                        auto_yaw.set_mode(AUTO_YAW_HOLD);
                        AP_Notify::events.mission_complete = 1;
                        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] FINISHING.");
                    }
                    else if (stage == PREPARE_AUTO)
                    {
                        stage = AUTO;
                        init_speed();
                        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] GO WITH MISSIONS.");
                        copter.set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
                    }
                    
                }
            }
        }
    } break;

    case EDGE_FOLLOW:
        auto_control();
        if (reached_destination())
        {
            if (!vecPoints.empty())
            {
                const Vector3f tpos(vecPoints.front().x, vecPoints.front().y, _mission_alt_cm.get() * 1.0f);
                wp_nav->set_wp_destination(tpos, false);
                auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                vecPoints.pop_front();
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] GO TO NEXT EDGE.");
            }
            else
            {
                stage = PREPARE_AUTO;
                int rd = -999;
                if (vecRects.size() > 2)
                {
                    rd = degNE(vecRects[1], vecRects[0]);
                    last_yaw_cd = rd * 100.0f;
                    auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
                }
                gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
            }
        }
        break;

    case MOVE_TO_EDGE:
        auto_control();
        if (reached_destination())
        {
            stage = EDGE_FOLLOW;
            // speed control for edge
            init_speed();
            pos_control->set_max_speed_xy(_spd_eg_cmss.get()*1.0f);
            pos_control->calc_leash_length_xy();

            if (!vecPoints.empty())
            {
                const Vector3f tpos(vecPoints.front().x, vecPoints.front().y, _mission_alt_cm.get() * 1.0f);
                wp_nav->set_wp_destination(tpos, false);
                auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                vecPoints.pop_front();
            }
        }
        break;

    case TAKE_PICTURE:
    {
        auto_control();
        bool bReach = b_position_target_reached;
        b_position_target_reached = reached_destination();
        if (bReach != b_position_target_reached && b_position_target_reached)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SPT COMPLETE. TAKE PICTURE.");
        }
    } break;

    case PREPARE_FOLLOW:
    case FINISHED:
    case MANUAL:
        manual_control();
        break;
    }
}

bool ModeCNDN::set_destination(const Vector3f &destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool yaw_relative)
{
    // ensure we are in position control mode
    if (stage != TAKE_PICTURE)
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
    b_position_target_reached = false;

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
    case MANUAL: {
        if (_method.get() == 0)
            break;

        if (dest_num > 0) {
            init_speed();

            Vector3f stopping_point;
            wp_nav->get_wp_stopping_point(stopping_point);
            wp_nav->set_wp_destination(stopping_point, false);

            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SIGNAL TO ETRI-MC.");
            gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
            // set to position control mode
            if (_method.get() == 2) {
                if (!vecRects.empty()) {
                    vecPoints.resize(vecRects.size());
                    std::copy(vecRects.begin(), vecRects.end(), vecPoints.begin());
                }

                if (!vecPoints.empty()) {
                    Vector3f hpos(vecPoints.front().x, vecPoints.front().y, _mission_alt_cm.get() * 1.0f);

                    wp_nav->set_wp_destination(hpos, false);
                    last_yaw_cd = degNE(vecPoints[1], vecPoints[0]) * 100.0f;
                    auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
                    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MOVE TO START POINT.");
                    vecPoints.pop_front();
                    stage = MOVE_TO_EDGE;
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] NO DETECTED EDGES.");
                    return_to_manual_control(false);
                }
            } else {
                stage = TAKE_PICTURE;
            }
        }
    } break;

    case PREPARE_FOLLOW:
        if (dest_num == 2)
        {
            wp_nav->wp_and_spline_init();
            if (!vecPoints.empty())
            {
                Vector3f hpos(vecPoints.front().x, vecPoints.front().y, _mission_alt_cm.get() * 1.0f);
                wp_nav->set_wp_destination(hpos, false);
                last_yaw_cd = degNE(vecPoints[1], vecPoints[0]) * 100.0f;
                auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MOVE TO START POINT.");
                vecPoints.pop_front();
                stage = MOVE_TO_EDGE;
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] NO DETECTED EDGES.");
                return_to_manual_control(false);
            }
        }
    case TAKE_PICTURE:
    case EDGE_FOLLOW:
    case MOVE_TO_EDGE:
    case PREPARE_AUTO:
    case AUTO:
    case RETURN_AUTO:
    case PREPARE_FINISH:
    case FINISHED:
    default:
    {
        if (dest_num == 0)
        {
            wp_nav->wp_and_spline_init();
            return_to_manual_control(false);
            return;
        }
    } break;
    }
}

// return manual control to the pilot
void ModeCNDN::return_to_manual_control(bool maintain_target)
{
    if (stage != MANUAL)
    {
        stage = MANUAL;
        b_position_target = false;
        b_position_target_reached = false;
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target)
        {
            const Vector3f wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest);
            if (wp_nav->origin_and_destination_are_terrain_alt())
            {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        }
        else
        {
            loiter_nav->init_target();
        }
        auto_yaw.set_mode(AUTO_YAW_HOLD);
        gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
        gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MANUAL CONTROL");
    }
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_ACK:
    {
        mavlink_command_ack_t packet;
        mavlink_msg_command_ack_decode(&msg, &packet);
        gcs().send_text(MAV_SEVERITY_INFO, "[ETRI] COMMAND_ACK(%d)", int(packet.command));
    } break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode())
            break;

        Vector3f cpos = inertial_nav.get_position();

        bool bTargeted = false;
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_LOCAL_NED)
        {
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED)
            {
                packet.x = packet.y = 0.0f;
                packet.z = _take_alt_cm.get() * -0.01f;
                packet.z += cpos.z * 0.01f;
            }
            bTargeted = true;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
            packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED)
        {
            break;
        }

        bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore)
        {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED)
            {
                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED)
            {
                pos_vector += cpos;
            }
            else
            {
                // convert from alt-above-home to alt-above-ekf-origin
                if (!AP::ahrs().home_is_set())
                {
                    break;
                }
                Location origin;
                pos_vector.z += AP::ahrs().get_home().alt;
                if (copter.ahrs.get_origin(origin))
                {
                    pos_vector.z -= origin.alt;
                }
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore)
        {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED)
            {
                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore)
        {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore)
        {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && vel_ignore && acc_ignore)
        {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
            if (bTargeted)
            {
                b_position_target = true;
                //gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SPT ON %0.3f,%0.3f,%0.3f", pos_vector.x, pos_vector.y, pos_vector.z);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] SPT ON.");
            }
        }
    } break;

    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
    {
        mavlink_named_value_int_t packet;
        mavlink_msg_named_value_int_decode(&msg, &packet);
        if (strncasecmp(packet.name, "CNDN_VALUE", 10) == 0)
        {
            switch (packet.value)
            {
            case 0:
                return_to_manual_control(false);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : return_to_manual_control.", int(packet.value));
                break;

            case 1:
                stage = TAKE_PICTURE;
                gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : IMAGE_START_CAPTURE.", int(packet.value));
                break;

            case 2:
                b_position_target_reached = true;
                gcs().send_message(MSG_POSITION_TARGET_LOCAL_NED);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : POSITION_TARGET_LOCAL_NED.", int(packet.value));
                break;

            case 3:
                gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : VIDEO_START_CAPTURE.", int(packet.value));
                break;

            case 4:
                gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : VIDEO_STOP_CAPTURE.", int(packet.value));
                break;

            case 5:
                gcs().send_local_position_ned(0.0f,0.0f,0.0f,0.02f,0.0f,0.0001f);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : LOCAL_POSITION_NED.", int(packet.value));
                break;
            }
        }
    }
    break;

    case MAVLINK_MSG_ID_CAMERA_TRIGGER:
        if (stage == TAKE_PICTURE)
        {
            stage = PREPARE_FOLLOW;
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            gcs().send_text(MAV_SEVERITY_INFO, "[ETRI] CAMERA_TRIGGERED, Prepare to EDGE FOLLOW.");
            AP_Notify::events.waypoint_complete = 1;

#if 0
            Vector3f fd1(-1,-1,0); fd1.normalize();
            Location l0(Vector3f(vecRects[0].x+fd1.x, vecRects[0].y+fd1.y, _mission_alt_cm.get()*1.0f));
            Location l1(Vector3f(vecRects[1].x+fd1.x, vecRects[1].y+fd1.y, _mission_alt_cm.get()*1.0f));
            Location l2(Vector3f(vecRects[2].x+fd1.x, vecRects[2].y+fd1.y, _mission_alt_cm.get()*1.0f));
            Location l3(Vector3f(vecRects[3].x+fd1.x, vecRects[3].y+fd1.y, _mission_alt_cm.get()*1.0f));

            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] 1 %ld; %ld;", l0.lat, l0.lng);
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] 2 %ld; %ld;", l1.lat, l1.lng);
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] 3 %ld; %ld;", l2.lat, l2.lng);
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] 4 %ld; %ld;", l3.lat, l3.lng);
#endif            

            AP_Mission::Mission_Command cmd;

            AP::mission()->reset();
            AP::mission()->clear();

            cmd.id = MAV_CMD_NAV_WAYPOINT;
            cmd.p1 = 0;
            cmd.content.location = AP::ahrs().get_home();
            AP::mission()->add_cmd(cmd);

            Vector2f vd1 = (vecRects[3] - vecRects[0]); // step vector
            Vector2f vd2 = (vecRects[2] - vecRects[1]); // step vector
            Vector2f p1(vecRects[0]),p2(vecRects[1]),p3,p4;
            float ldir = vd2.length();
            vd1.normalize();
            vd2.normalize();

            float lw_cm = _spray_width_cm.get()*1.0f;
            Vector2f step1(vd1 * lw_cm);
            Vector2f step2(vd2 * lw_cm);

            p1 += step1;
            p2 += step2;

            float ll_cm = lw_cm;

            for (float l = 0.0f; l < ldir - lw_cm; l += lw_cm * 2.0f)
            {
                p4 = p1 + step1;
                p3 = p2 + step2;

                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = 1;
                cmd.content.location = Location(Vector3f(p1.x, p1.y, _mission_alt_cm.get()*1.0f));
                cmd.content.location.set_alt_cm(300, Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);

                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = 1;
                cmd.content.location = Location(Vector3f(p2.x, p2.y, _mission_alt_cm.get()*1.0f));
                cmd.content.location.set_alt_cm(300, Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);

                ll_cm += lw_cm;

                if (ll_cm > ldir - lw_cm)
                    break;

                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = 1;
                cmd.content.location = Location(Vector3f(p3.x, p3.y, _mission_alt_cm.get()*1.0f));
                cmd.content.location.set_alt_cm(300, Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);

                cmd.id = MAV_CMD_NAV_WAYPOINT;
                cmd.p1 = 1;
                cmd.content.location = Location(Vector3f(p4.x, p4.y, _mission_alt_cm.get()*1.0f));
                cmd.content.location.set_alt_cm(300, Location::AltFrame::ABOVE_HOME);
                AP::mission()->add_cmd(cmd);

                p1 = p4 + step1;
                p2 = p3 + step2;

                ll_cm += lw_cm;
            }

            // mission finish command.
            cmd.id = MAV_CMD_DO_SET_RELAY;
            cmd.p1 = 0;
            cmd.content.location = Location();
            cmd.content.relay.num = 255;
            cmd.content.relay.state = 1;
            AP::mission()->add_cmd(cmd);

            if (_method.get() == 3)
            {
                init_speed();
                if (!vecPoints.empty())
                {
                    Vector3f hpos(vecPoints.front().x, vecPoints.front().y, _mission_alt_cm.get() * 1.0f);
                    wp_nav->set_wp_destination(hpos, false);
                    last_yaw_cd = degNE(vecPoints[1], vecPoints[0]) * 100.0f;
                    auto_yaw.set_fixed_yaw(last_yaw_cd * 0.01f, 0.0f, 0, false);
                    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] MOVE TO START POINT.");
                    vecPoints.pop_front();
                    stage = MOVE_TO_EDGE;
                }
                else
                {
                    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] No edge detected.");
                    return_to_manual_control(false);
                }
            }
        }
        break;

    case MAVLINK_MSG_ID_ETRI_PADDY_EDGE_GPS_INFORMATION:
        if (stage == TAKE_PICTURE)
        {
            mavlink_etri_paddy_edge_gps_information_t packet;
            mavlink_msg_etri_paddy_edge_gps_information_decode(&msg, &packet);

            wp_nav->wp_and_spline_init();

#if defined(SIM_LOCATION)
            packet.edge_count = 0;
            Location loc(copter.current_loc);
            for(uint16_t i=0; i<vecAreas.size(); i++)
            {
                CNAREA& area = vecAreas[i];
                if (!inside(area, loc))
                    continue;
                packet.latitude1  = area.latitude1 ;
                packet.longitude1 = area.longitude1;
                packet.latitude2  = area.latitude2 ;
                packet.longitude2 = area.longitude2;
                packet.latitude3  = area.latitude3 ;
                packet.longitude3 = area.longitude3;
                packet.latitude4  = area.latitude4 ;
                packet.longitude4 = area.longitude4;
                packet.edge_count = 4;
                break;
            }
#endif
            edge_count = packet.edge_count;

            for (int i = 0; i < 10; i++)
                edge_points[i].zero();

            gcs().send_text(MAV_SEVERITY_INFO, "[ETRI] GPS_INFO(%d points) received.", packet.edge_count);
            if (edge_count > 0)
            {
                edge_points[0].x = packet.latitude1;
                edge_points[0].y = packet.longitude1;
            }
            if (edge_count > 1)
            {
                edge_points[1].x = packet.latitude2;
                edge_points[1].y =  packet.longitude2;
            }
            if (edge_count > 2)
            {
                edge_points[2].x = packet.latitude3;
                edge_points[2].y = packet.longitude3;
            }
            if (edge_count > 3)
            {
                edge_points[3].x = packet.latitude4;
                edge_points[3].y = packet.longitude4;
            }
            if (edge_count > 4)
            {
                edge_points[4].x = packet.latitude5;
                edge_points[4].y = packet.longitude5;
            }
            if (edge_count > 5)
            {
                edge_points[5].x = packet.latitude6;
                edge_points[5].y = packet.longitude6;
            }
            if (edge_count > 6)
            {
                edge_points[6].x = packet.latitude7;
                edge_points[6].y = packet.longitude7;
            }
            if (edge_count > 7)
            {
                edge_points[7].x = packet.latitude8;
                edge_points[7].y = packet.longitude8;
            }
            if (edge_count > 8)
            {
                edge_points[8].x = packet.latitude9;
                edge_points[8].y = packet.longitude9;
            }
            if (edge_count > 9)
            {
                edge_points[9].x = packet.latitude10;
                edge_points[9].y = packet.longitude10;
            }

            vecRects.clear();
            vecPoints.clear();

            if (edge_count > 0) {
                Vector3f hpos;
                if (!ahrs.get_home().get_vector_from_origin_NEU(hpos))
                    return;
                Vector2f cpos(hpos.x, hpos.y);

                // GEO to NEU
                Vector3f pcm; // position (North, East, Up coordinates) in centimeters
                for (int i = 0; i < edge_count; i++)
                {
                    Vector2f& pos = edge_points[i];
                    pcm = locNEU(pos.x, pos.y, 3.0f);
                    vecRects.push_back(Vector2f(pcm.x, pcm.y));
                }

                float minlen = (vecRects.front()-cpos).length();
                Vector2f apos = vecRects.front();
                vecPoints.push_back(apos);
                for (int i = 1; i < (int)vecRects.size(); i++)
                {
                    if ((vecRects[i]-cpos).length() < minlen)
                    {
                        apos = vecRects[i];
                        minlen = (apos-cpos).length();
                    }
                    vecPoints.push_back(vecRects[i]);
                }

                for(int i = 0; i < (int)vecPoints.size(); i++)
                {
                    if ((vecPoints.front()-apos).length() <= 0.001f)
                        break;
                    cpos = vecPoints.front();
                    vecPoints.pop_front();
                    vecPoints.push_back(cpos);
                }

                vecPoints.pop_front();
                if ((vecPoints.front()-apos).length() < (vecPoints.back()-apos).length())
                    std::reverse(vecPoints.begin(), vecPoints.end());
                vecPoints.push_front(apos);
                vecPoints.push_back(apos);

                vecRects.resize(vecPoints.size());
                std::copy(vecPoints.begin(), vecPoints.end(), vecRects.begin());
            }
        }
        break;

    case MAVLINK_MSG_ID_ETRI_PADDY_DETECT_COMMAND:
        break;

    case MAVLINK_MSG_ID_ETRI_DRONE_PADDY_DISTANCE:
    {
        mavlink_etri_drone_paddy_distance_t packet;
        mavlink_msg_etri_drone_paddy_distance_decode(&msg, &packet);
        //gcs().send_text(MAV_SEVERITY_INFO, "[MAV] ETRI_DISTANCE (%0.3f)", packet.distance);
    }
    break;
    }
}

void ModeCNDN::pos_control_start()
{
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

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
    if (!copter.failsafe.radio)
    {
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

    // control edge following to attitute controller

    if (stage == EDGE_FOLLOW) {

        uint32_t now = AP_HAL::millis();
        if (edge_time_ms == 0)
            edge_time_ms = now;

        float fv = rc().channel(5)->norm_input();
        float ferrv = /*_dst_eg_cm.get() * 0.01f - */fv * 5.0f;

        if (rf_rt != nullptr && rf_lf != nullptr)
        {
            if (ferrv > 0.0f){
                rf_lf->set_distance(5.0f - fabsf(ferrv));
                rf_rt->set_distance(50.0f);
            }else if (ferrv < 0.0f){
                rf_lf->set_distance(50.0f);
                rf_rt->set_distance(5.0f - fabsf(ferrv));
            }else{
                rf_lf->set_distance(50.0f);
                rf_rt->set_distance(50.0f);
            }

            if (now - edge_time_ms > 250){   // 4Hz
                edge_time_ms = now;
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] %0.1f/%0.1f=>%0.4f", fv, ferrv, roll_target);
            }
        }
    }

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_target, target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_target, pitch_target, auto_yaw.yaw(), true);
    }

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok)
    {
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
    if (!wp_nav->reached_wp_destination())
    {
        reach_wp_time_ms = 0;
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > CNDN_WP_RADIUS_CM)
    {
        reach_wp_time_ms = 0;
        return false;
    }

    // check height to destination
    if (stage == TAKE_PICTURE)
    {
        const Vector3f cpos = inertial_nav.get_position();
        Vector3f tpos = wp_nav->get_wp_destination() - cpos;
        float fz = sqrtf(tpos.x*tpos.x+tpos.y*tpos.y+tpos.z*tpos.z);

        if (fz > CNDN_WP_RADIUS_CM)
        {
            reach_wp_time_ms = 0;
            return false;
        }

        if (!b_position_target)
        {
            reach_wp_time_ms = 0;
            return false;
        }
    }
    // wait at least one second
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0)
        reach_wp_time_ms = now;

    return ((now - reach_wp_time_ms) > 1000);
}

bool ModeCNDN::calculate_next_dest(uint8_t position_num, bool use_wpnav_alt, Vector3f &next_dest, bool &terrain_alt) const
{
    // sanity check position_num
    if (position_num > 1)
    {
        return false;
    }

    // define start_pos as either A or B depending upon position_num
    Vector2f start_pos = position_num == 0 ? dest_A : dest_B;

    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (!is_positive(AB_diff.length_squared()))
    {
        return false;
    }

    // get distance from vehicle to start_pos
    const Vector3f curr_pos = inertial_nav.get_position();
    const Vector2f curr_pos2d = Vector2f(curr_pos.x, curr_pos.y);
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared())
    {
        scalar = veh_to_start_pos.length() / AB_diff.length();
    }

    // create a line perpendicular to AB but originating at start_pos
    Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
    Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);

    // find the closest point on the perpendicular line
    const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
    next_dest.x = closest2d.x;
    next_dest.y = closest2d.y;

    if (use_wpnav_alt)
    {
        // get altitude target from waypoint controller
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest.z = wp_nav->get_wp_destination().z;
    }
    else
    {
        // if we have a downward facing range finder then use terrain altitude targets
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used();
        if (terrain_alt)
        {
            if (!copter.surface_tracking.get_target_alt_cm(next_dest.z))
            {
                next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
            }
        }
        else
        {
            next_dest.z = pos_control->is_active_z() ? pos_control->get_alt_target() : curr_pos.z;
        }
    }

    return true;
}

void ModeCNDN::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw)
    {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    }
    else if (use_yaw_rate)
    {
        auto_yaw.set_rate(yaw_rate_cds);
    }
}

void ModeCNDN::return_to_mode()
{
    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] RELAY TO CNDN.");
    stage = RETURN_AUTO;
    copter.set_mode(Mode::Number::CNDN, ModeReason::MISSION_END);
}

#endif