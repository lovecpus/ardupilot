#if MODE_CNDN_ENABLED == ENABLED
/*
* Init and run calls for CNDN flight mode
*/
#define CNDN_WP_RADIUS_CM 200
#define CASE_CNDN_AUX_INIT()  case AUX_FUNC::CNDN: case AUX_FUNC::CNDN_ETRI:

#define CASE_CNDN_AUX_FUNC()  case AUX_FUNC::CNDN: \
    do_aux_function_change_mode(Mode::Number::CNDN, ch_flag); \
    break; \
    case AUX_FUNC::CNDN_ETRI: \
    if (copter.flightmode == &copter.mode_cndn) { \
        switch (ch_flag) { \
        case LOW:\
            copter.mode_cndn.mission_command(0);\
            break;\
        case MIDDLE:\
            copter.mode_cndn.mission_command(1);\
            break;\
        case HIGH:\
            copter.mode_cndn.mission_command(2);\
            break;\
    }\
}\
break;

class ModeCNDN : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override
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

        return true;
    }

    void run() override
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
        case EDGE_FOLLOW:
            // if vehicle has reached destination switch to manual control
            if (reached_destination())
            {
                AP_Notify::events.waypoint_complete = 1;
                return_to_manual_control(true);
            }
            else
            {
                auto_control();
            }
            break;

        case TAKE_PICTURE:
            auto_control();
            b_position_target_reached = b_position_target && (stage == TAKE_PICTURE) && reached_destination();
            break;

        case PREPARE_FOLLOW:
            manual_control();
            break;

        case MANUAL:
            manual_control();
            break;
        }
    }

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool is_position_target_reached() override
    {
        return b_position_target_reached;
    }

    bool set_destination(const Vector3f &destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false)
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
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination(destination, false);

        gcs().send_text(MAV_SEVERITY_INFO, "set position target %0.3f,%0.3f,%0.3f", destination.x, destination.y, destination.z);

        b_position_target_reached = false;
        b_position_target = true;

        return true;
    }

    // save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
    void mission_command(uint8_t dest_num)
    {
        // sanity check
        if (dest_num > 4)
        {
            return;
        }

        // get current position as an offset from EKF origin
        //const Vector3f curr_pos = inertial_nav.get_position();

        // handle state machine changes
        switch (stage)
        {

        case MANUAL:
            if (dest_num > 0)
            {
                gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
                gcs().send_text(MAV_SEVERITY_INFO, "send image start capture to ETRI-MC");
            }
            break;

        case TAKE_PICTURE:
        case PREPARE_FOLLOW:
        case EDGE_FOLLOW:
        case AUTO:
            if (dest_num == 0)
            {
                wp_nav->wp_and_spline_init();
                return_to_manual_control(false);
                return;
            }

            if (stage == PREPARE_FOLLOW && dest_num == 2)
            {
                stage = EDGE_FOLLOW;
                if (edge_count > 0)
                {
                    Vector3f stopping_point;
                    wp_nav->get_wp_stopping_point(stopping_point);
                    stopping_point.x = edge_points[0].x;
                    stopping_point.y = edge_points[0].y;
                    // no need to check return status because terrain data is not used
                    wp_nav->set_wp_destination(stopping_point, false);

                    gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Edge follow stage started.");
                }
                return;
            }
            break;
        }
    }

    // return manual control to the pilot
    void return_to_manual_control(bool maintain_target)
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
            gcs().send_text(MAV_SEVERITY_INFO, "CNDN: manual control");
        }
    }

    void handle_message(const mavlink_message_t &msg) override
    {
        switch (msg.msgid)
        {
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
                    gcs().send_text(MAV_SEVERITY_INFO, "CNDN immediatly applied %d : return_to_manual_control.", packet.value);
                    break;

                case 1:
                    gcs().send_command_long(MAV_CMD_IMAGE_START_CAPTURE);
                    gcs().send_text(MAV_SEVERITY_INFO, "CNDN immediatly applied %d : IMAGE_START_CAPTURE.", packet.value);
                    break;

                case 2:
                    b_position_target_reached = true;
                    gcs().send_message(MSG_POSITION_TARGET_LOCAL_NED);
                    gcs().send_text(MAV_SEVERITY_INFO, "CNDN immediatly applied %d : POSITION_TARGET_LOCAL_NED.", packet.value);
                    break;

                case 3:
                    gcs().send_command_long(MAV_CMD_VIDEO_START_CAPTURE);
                    gcs().send_text(MAV_SEVERITY_INFO, "CNDN immediatly applied %d : VIDEO_START_CAPTURE.", packet.value);
                    break;

                case 4:
                    gcs().send_command_long(MAV_CMD_VIDEO_STOP_CAPTURE);
                    gcs().send_text(MAV_SEVERITY_INFO, "CNDN immediatly applied %d : VIDEO_STOP_CAPTURE.", packet.value);
                    break;
                }
            }
        }
        break;

        case MAVLINK_MSG_ID_CAMERA_TRIGGER:
            if (stage == TAKE_PICTURE)
            {
                stage = PREPARE_FOLLOW;
                gcs().send_text(MAV_SEVERITY_INFO, "CAMERA_TRIGGER received, prepare to EDGE FOLLOW.");
            }
            break;

        case MAVLINK_MSG_ID_ETRI_PADDY_EDGE_GPS_INFORMATION:
            if (stage == TAKE_PICTURE)
            {
                mavlink_etri_paddy_edge_gps_information_t packet;
                mavlink_msg_etri_paddy_edge_gps_information_decode(&msg, &packet);
                edge_count = packet.edge_count;

                for (int i = 0; i < 10; i++)
                    edge_points[0].zero();

                gcs().send_text(MAV_SEVERITY_INFO, "ETRI_PADDY_EDGE_GPS_INFORMATION (%d points) received.", packet.edge_count);
                if (edge_count > 0)
                {
                    edge_points[0].x = packet.latitude1;
                    edge_points[0].y = packet.longitude1;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos1 (%0.8f,%0.8f)", edge_points[0].x, edge_points[0].y);
                }
                if (edge_count > 1)
                {
                    edge_points[1].x = packet.latitude2;
                    edge_points[1].y = packet.longitude2;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos2 (%0.8f,%0.8f)", edge_points[1].x, edge_points[1].y);
                }
                if (edge_count > 2)
                {
                    edge_points[2].x = packet.latitude3;
                    edge_points[2].y = packet.longitude3;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos3 (%0.8f,%0.8f)", edge_points[2].x, edge_points[2].y);
                }
                if (edge_count > 3)
                {
                    edge_points[3].x = packet.latitude4;
                    edge_points[3].y = packet.longitude4;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos4 (%0.8f,%0.8f)", edge_points[3].x, edge_points[3].y);
                }
                if (edge_count > 4)
                {
                    edge_points[4].x = packet.latitude5;
                    edge_points[4].y = packet.longitude5;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos5 (%0.8f,%0.8f)", edge_points[4].x, edge_points[4].y);
                }
                if (edge_count > 5)
                {
                    edge_points[5].x = packet.latitude6;
                    edge_points[5].y = packet.longitude6;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos6 (%0.8f,%0.8f)", edge_points[5].x, edge_points[5].y);
                }
                if (edge_count > 6)
                {
                    edge_points[6].x = packet.latitude7;
                    edge_points[6].y = packet.longitude7;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos7 (%0.8f,%0.8f)", edge_points[6].x, edge_points[6].y);
                }
                if (edge_count > 7)
                {
                    edge_points[7].x = packet.latitude8;
                    edge_points[7].y = packet.longitude8;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos8 (%0.8f,%0.8f)", edge_points[7].x, edge_points[7].y);
                }
                if (edge_count > 8)
                {
                    edge_points[8].x = packet.latitude9;
                    edge_points[8].y = packet.longitude9;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos9 (%0.8f,%0.8f)", edge_points[8].x, edge_points[8].y);
                }
                if (edge_count > 9)
                {
                    edge_points[9].x = packet.latitude10;
                    edge_points[9].y = packet.longitude10;
                    gcs().send_text(MAV_SEVERITY_INFO, "pos10 (%0.8f,%0.8f)", edge_points[9].x, edge_points[9].y);
                }
            }
            break;

        case MAVLINK_MSG_ID_ETRI_PADDY_DETECT_COMMAND:
            break;

        case MAVLINK_MSG_ID_ETRI_DRONE_PADDY_DISTANCE:
        {
            mavlink_etri_drone_paddy_distance_t packet;
            mavlink_msg_etri_drone_paddy_distance_decode(&msg, &packet);
            gcs().send_text(MAV_SEVERITY_INFO, "MAVLINK_MSG_ID_ETRI_DRONE_PADDY_DISTANCE (%0.3f)", packet.distance);
        }
        break;
        }
    }

protected:
    const char *name() const override { return "CNDN_ETRI"; }
    const char *name4() const override { return "CNDN"; }

private:
    void pos_control_start()
    {
        // set to position control mode
        stage = TAKE_PICTURE;

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

    void auto_control()
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
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

        // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
        if (!wpnav_ok)
        {
            return_to_manual_control(false);
        }
    }

    void manual_control()
    {
        float target_roll, target_pitch;
        float target_yaw_rate = 0.0f;
        float target_climb_rate = 0.0f;
        float takeoff_climb_rate = 0.0f;

        // initialize vertical speed and acceleration
        pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
        pos_control->set_max_accel_z(g.pilot_accel_z);

        // process pilot inputs unless we are in radio failsafe
        if (!copter.failsafe.radio)
        {
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

            // make sure the climb rate is in the given range, prevent floating point errors
            target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        }
        else
        {
            // clear out pilot desired acceleration in case radio failsafe event occurs and we
            // do not switch to RTL for some reason
            loiter_nav->clear_pilot_desired_acceleration();
        }

        // relax loiter target if we might be landed
        if (copter.ap.land_complete_maybe)
        {
            loiter_nav->soften_for_landing();
        }

        // Loiter State Machine Determination
        AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

        // Loiter State Machine
        switch (loiter_state)
        {

        case AltHold_MotorStopped:

            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
            loiter_nav->init_target();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
            pos_control->update_z_controller();
            break;

        case AltHold_Takeoff:

            // initiate take-off
            if (!takeoff.running())
            {
                takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
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

            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            // FALLTHROUGH

        case AltHold_Landed_Pre_Takeoff:

            loiter_nav->init_target();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
            pos_control->relax_alt_hold_controllers(0.0f); // forces throttle output to go to zero
            pos_control->update_z_controller();
            break;

        case AltHold_Flying:

            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            // run loiter controller
            loiter_nav->update();

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

            // adjust climb rate using rangefinder
            target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            pos_control->update_z_controller();
            break;
        }
    }

    bool reached_destination()
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

        // wait at least one second
        uint32_t now = AP_HAL::millis();
        if (reach_wp_time_ms == 0)
        {
            reach_wp_time_ms = now;
        }
        return ((now - reach_wp_time_ms) > 1000);
    }

    bool calculate_next_dest(uint8_t position_num, bool use_wpnav_alt, Vector3f &next_dest, bool &terrain_alt) const
    {
        // sanity check dest_num
        if (dest_num > 1)
        {
            return false;
        }

        // define start_pos as either A or B depending upon dest_num
        Vector2f start_pos = dest_num == 0 ? dest_A : dest_B;

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

    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
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

    Vector2f dest_A; // in NEU frame in cm relative to ekf origin
    Vector2f dest_B; // in NEU frame in cm relative to ekf origin

    enum cndn_state
    {
        MANUAL,       // pilot toggle the switch to middle position, has manual control
        TAKE_PICTURE, // storing points A and B, pilot has manual control
        PREPARE_FOLLOW,
        EDGE_FOLLOW,
        AUTO, // after A and B defined, pilot toggle the switch from one side to the other, vehicle flies autonomously
    } stage;

    uint32_t reach_wp_time_ms = 0; // time since vehicle reached destination (or zero if not yet reached)
    bool b_position_target = false;
    bool b_position_target_reached = false;
    uint8_t edge_count = 0;
    uint8_t edge_position = 0;
    Vector2f edge_points[10]; // in NEU frame in cm relative to ekf origin
};
#endif
