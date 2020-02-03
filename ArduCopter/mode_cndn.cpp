#include "Copter.h"
#include <stdio.h>

#if MODE_CNDN_ENABLED == ENABLED

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

    // initialise waypoint state
    stage = MANUAL;
    b_position_target = false;

    dest_A.zero();
    dest_B.zero();

    gcs().send_text(MAV_SEVERITY_INFO, "CNDN mode initialzied.");

    return true;
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

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    // if (is_disarmed_or_landed() || !motors->get_interlock() ) {
    //     zero_throttle_and_relax_ac(copter.is_tradheli() && motors->get_interlock());
    //     return;
    // }

    switch (stage)
    {
    case AUTO:
        // if vehicle has reached destination switch to manual control
        if (reached_destination())
        {
            stage = FINISHED;
            AP_Notify::events.waypoint_complete = 1;
            b_position_target_reached = false;
            b_position_target = false;
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] FINISHING Stages.");
        }
        else
        {
            auto_control();
        }
        break;

    case EDGE_FOLLOW:
        auto_control();
        if (reached_destination())
        {
            if (!vecPoints.empty())
            {
                    const Vector3f tpos(vecPoints.front().x, vecPoints.front().y, wayHeight * 100.0f);
                    const Vector3f epos = wp_nav->get_wp_destination();
                    wp_nav->set_wp_destination(tpos, false);
                    auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                    auto_yaw.set_rate(4500.0f);
                    const Vector3f rpos(tpos-epos), npos(1.0f,0.0f,0.0f);
                    vecPoints.pop_front();
                    gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] Move to next EDGE.");
            }
            else
            {
                stage = AUTO;
                // calculating waypoints
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] Change to AUTO stage.");
            }
        }
        break;

    case MOVE_TO_EDGE:
        auto_control();
        if (reached_destination())
        {
            stage = EDGE_FOLLOW;
            if (!vecPoints.empty())
            {
                const Vector3f tpos(vecPoints.front().x, vecPoints.front().y, wayHeight * 100.0f);
                wp_nav->set_wp_destination(tpos, false);
                auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                auto_yaw.set_rate(4500.0f);
                gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                vecPoints.pop_front();
            }
            gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] Change to EDGE_FOLLOW.");
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
        manual_control();
        break;

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
    {
        pos_control_start();
    }

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

#if defined(_DEBUG)
void ModeCNDN::live_log(const char *fmt, ...)
{
    uint32_t now = AP_HAL::millis();
    if (reach_wp_logt_ms == 0)
        reach_wp_logt_ms = now;

    if ((now - reach_wp_logt_ms) < 500)
        return;

    reach_wp_logt_ms = now;

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
    switch (stage)
    {
    case MANUAL:
    {
        if (dest_num > 0)
        {
            pos_control_start();
            gcs().send_text(MAV_SEVERITY_INFO, "send image start capture to ETRI-MC");
            gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
            // set to position control mode
            stage = TAKE_PICTURE;
        }
    }
    break;

    case PREPARE_FOLLOW:
    {
        if (dest_num == 2)
        {
            vecPoints.clear();
            if (edge_count > 0)
            {
                Vector3f hpos;

                if (!ahrs.get_home().get_vector_from_origin_NEU(hpos))
                    return;
                Vector2f cpos(hpos.x, hpos.y);

                if (edge_count > 0)
                {
                    float minlen = (edge_points[0]-cpos).length();
                    Vector2f apos = edge_points[0];
                    vecPoints.push_back(apos);
                    for (int i = 1; i < edge_count; i++)
                    {
                        if ((edge_points[i]-cpos).length() < minlen)
                        {
                            apos = edge_points[i];
                            minlen = (apos-cpos).length();
                        }
                        vecPoints.push_back(edge_points[i]);
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
                }

                if (!vecPoints.empty())
                {
                    hpos = Vector3f(vecPoints.front().x, vecPoints.front().y, wayHeight * 100.0f);
                    wp_nav->set_wp_destination(hpos, false);
                    gcs().send_text(MAV_SEVERITY_INFO, "EFS: %0.6f,%0.6f,%0.6f.", hpos.x, hpos.y, hpos.z);
                    vecPoints.pop_front();
                    stage = MOVE_TO_EDGE;
                }
            }
            else
            {
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] No edge detected.");
                pos_control_start();
                return_to_manual_control(false);
            }
            return;
        }
    }
    case TAKE_PICTURE:
    case EDGE_FOLLOW:
    case MOVE_TO_EDGE:
    case AUTO:
    case FINISHED:
    {
        if (dest_num == 0)
        {
            pos_control_start();
            return_to_manual_control(false);
            return;
        }
    }
    break;
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
        gcs().send_text(MAV_SEVERITY_INFO, "CNDN: manual control");
    }
}

#define SIM_LOCATION 1
void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
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
        gcs().send_text(MAV_SEVERITY_INFO, "[MAV] SPT(%d) %0.3f,%0.3f,%0.3f", packet.coordinate_frame, packet.x, packet.y, packet.z);
        if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_LOCAL_NED)
        {
#if defined(SIM_LOCATION)
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED)
            {
                packet.x = 0.0f;
                packet.y = 0.0f;
                packet.z = -25.0f;
                packet.z += cpos.z * 0.01f;
            }
#endif
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
                gcs().send_text(MAV_SEVERITY_INFO, "[MAV] SPT ON %0.3f,%0.3f,%0.3f", pos_vector.x, pos_vector.y, pos_vector.z);
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
            // case 0:
            //     return_to_manual_control(false);
            //     gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : return_to_manual_control.", packet.value);
            //     break;

            // case 1:
            //     gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
            //     gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %d : IMAGE_START_CAPTURE.", packet.value);
            //     break;

            case 2:
                b_position_target_reached = true;
                gcs().send_message(MSG_POSITION_TARGET_LOCAL_NED);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %ld : POSITION_TARGET_LOCAL_NED.", packet.value);
                break;

            case 3:
                gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %ld : VIDEO_START_CAPTURE.", packet.value);
                break;

            case 4:
                gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "[CNDN] VALUE %ld : VIDEO_STOP_CAPTURE.", packet.value);
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
            gcs().send_text(MAV_SEVERITY_INFO, "CAMERA_TRIGGER received, prepare to EDGE FOLLOW.");
        }
        break;

    case MAVLINK_MSG_ID_ETRI_PADDY_EDGE_GPS_INFORMATION:
        if (stage == TAKE_PICTURE)
        {
            mavlink_etri_paddy_edge_gps_information_t packet;
            mavlink_msg_etri_paddy_edge_gps_information_decode(&msg, &packet);

            wp_nav->wp_and_spline_init();

#if defined(SIM_LOCATION)
            packet.latitude1  = 37.2842096f;
            packet.longitude1 = 126.8735343f;
            packet.latitude2  = 37.2836760f;
            packet.longitude2 = 126.8727310f;
            packet.latitude3  = 37.2839001f;
            packet.longitude3 = 126.8725044f;
            packet.latitude4  = 37.2844283f;
            packet.longitude4 = 126.8733077f;
#endif

            edge_count = packet.edge_count;

            for (int i = 0; i < 10; i++)
                edge_points[i].zero();

            gcs().send_text(MAV_SEVERITY_INFO, "ETRI_GPS_INFO(%d points) received.", packet.edge_count);
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

            // GEO to NEU
            Vector3f pos_neu_cm; // position (North, East, Up coordinates) in centimeters
            for (int i = 0; i < edge_count; i++)
            {
                Vector2f& pos = edge_points[i];
                int32_t lat = pos.x * 1e7f;
                int32_t lng = pos.y * 1e7f;
                if (!check_latlng(lat, lng))
                    continue;
                const Location loc {lat,lng,300,Location::AltFrame::ABSOLUTE,};
                if (!loc.get_vector_from_origin_NEU(pos_neu_cm))
                    continue;
                pos.x = pos_neu_cm.x;
                pos.y = pos_neu_cm.y;
            }
        }
        break;

    case MAVLINK_MSG_ID_ETRI_PADDY_DETECT_COMMAND:
        break;

    case MAVLINK_MSG_ID_ETRI_DRONE_PADDY_DISTANCE:
    {
        mavlink_etri_drone_paddy_distance_t packet;
        mavlink_msg_etri_drone_paddy_distance_decode(&msg, &packet);
        gcs().send_text(MAV_SEVERITY_INFO, "[MAV] ETRI_DISTANCE (%0.3f)", packet.distance);
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

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
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

#endif