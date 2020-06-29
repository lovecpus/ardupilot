#include "Copter.h"

#if MODE_ZIGZAG_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 100

// initialise zigzag controller
bool ModeZigZag::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise position_z and desired velocity_z
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise waypoint state
    copter.sprayer.run(false);
    stage = (stage == WAIT_MOVE) ? WAIT_AUTO : MANUAL;

    if (stage == MANUAL) {
        copter.rangefinder_state.enabled = false;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        gcs().send_text(MAV_SEVERITY_INFO, "[AB-LINE] MANUAL CONTROL.");
#endif
    } else if (stage == WAIT_AUTO) {
        copter.rangefinder_state.enabled = true;
        misAlt = copter.mode_cndn._mission_alt_cm.get();
        if (copter.rangefinder_state.alt_healthy)
            misAlt = copter.rangefinder_state.alt_cm_filt.get();

        Vector3f curr_pos = inertial_nav.get_position();
        curr_pos.z = misAlt;
        wp_nav->set_wp_destination(curr_pos, true);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        gcs().send_text(MAV_SEVERITY_INFO, "[AB-LINE] WAIT FOR DIRECTION.");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "[AB-LINE] STAGE(%d)", stage);
#endif        
    }
    return true;
}

// run the zigzag controller
// should be called at 100hz or more
void ModeZigZag::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    switch (stage) {
        case WAIT_AUTO: {
            if (is_disarmed_or_landed() || !motors->get_interlock()) {
                // vehicle should be under manual control when disarmed or landed
                return_to_manual_control(false);
            }
            auto_control();
            // wait vector to move direction
            int16_t rv = (channel_roll->get_control_in() / 1000);
            int16_t arv = (rv != 0) ? (rv / abs(rv)) : 0;
            if (arv > 0) {
                Vector2f delta = (dest_B - dest_A).normalized();
                float dir = 90.0f * M_PI / 180.0f;
                direct.x = cosf(dir)*delta.x - sinf(dir)*delta.y;
                direct.y = sinf(dir)*delta.x + cosf(dir)*delta.y;
                direct *= copter.mode_cndn._spray_width_cm.get();

                stage = AUTO;
                Vector3f dst;
                dst.x = dest_A.x;
                dst.y = dest_A.y;
                dst.z = misAlt;
                copter.rangefinder_state.enabled = true;
                wp_nav->set_speed_xy(copter.mode_cndn._spd_auto_cm.get());
                wp_nav->set_wp_destination(dst, true);
                steps = 0;
            } else if (arv < 0) {
                Vector2f delta = (dest_B - dest_A).normalized();
                float dir = -90.0f * M_PI / 180.0f;
                direct.x = cosf(dir)*delta.x - sinf(dir)*delta.y;
                direct.y = sinf(dir)*delta.x + cosf(dir)*delta.y;
                direct *= copter.mode_cndn._spray_width_cm.get();

                stage = AUTO;
                Vector3f dst;
                dst.x = dest_A.x;
                dst.y = dest_A.y;
                dst.z = misAlt;
                copter.rangefinder_state.enabled = true;
                wp_nav->set_speed_xy(copter.mode_cndn._spd_auto_cm.get());
                wp_nav->set_wp_destination(dst, true);
                steps = 0;
            }
        } break;

        case WAIT_MOVE:
        case MANUAL:
            manual_control();
        break;

        case AUTO:
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            return_to_manual_control(false);
        }

        if (reached_destination()) {
            uint32_t now = AP_HAL::millis();
            // move to next point
            Vector3f dst;
            switch (steps) {
                case 0: { // arrived at A, move to B point with spray
                    if (step_time_ms == 0) step_time_ms = now;
                    if (now - step_time_ms < 1000) break;
                    step_time_ms = 0;

                    dst.x = dest_B.x;
                    dst.y = dest_B.y;
                    dst.z = misAlt;
                    wp_nav->set_wp_destination(dst, true);
                    copter.sprayer.run(true);
                    steps = 1;
                } break;

                case 1: { // arrived at B, move to B step point without spray
                    dest_A += direct;
                    dest_B += direct;

                    dst.x = dest_B.x;
                    dst.y = dest_B.y;
                    dst.z = misAlt;
                    wp_nav->set_wp_destination(dst, true);
                    copter.sprayer.run(false);
                    steps = 2;
                } break;

                case 2: { // arrived at B step, move to A point with spray
                    if (step_time_ms == 0) step_time_ms = now;
                    if (now - step_time_ms < 1000) break;
                    step_time_ms = 0;

                    dst.x = dest_A.x;
                    dst.y = dest_A.y;
                    dst.z = misAlt;
                    wp_nav->set_wp_destination(dst, true);
                    copter.sprayer.run(true);
                    steps = 3;
                } break;

                case 3: { // arrived at A, move to A step point without spray
                    dest_A += direct;
                    dest_B += direct;

                    dst.x = dest_A.x;
                    dst.y = dest_A.y;
                    dst.z = misAlt;
                    wp_nav->set_wp_destination(dst, true);
                    copter.sprayer.run(false);
                    steps = 0;
                } break;
            }
        }
        auto_control();
        break;
    }
}

// save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
void ModeZigZag::save_or_move_to_destination(uint8_t dest_num)
{
    // sanity check
    if (dest_num > 2) {
        return;
    }

    // get current position as an offset from EKF origin
    const Vector3f curr_pos = inertial_nav.get_position();

    // handle state machine changes
    switch (stage) {
        case MANUAL:
            if (dest_num == 1) {
                // store point A
                dest_A.x = curr_pos.x;
                dest_A.y = curr_pos.y;
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: point A stored");
                copter.Log_Write_Event(DATA_ZIGZAG_STORE_A);
            } else if (dest_num == 2) {
                // store point B
                dest_B.x = curr_pos.x;
                dest_B.y = curr_pos.y;
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: point B stored");
                copter.Log_Write_Event(DATA_ZIGZAG_STORE_B);
            } else {
                dest_A.zero();
                dest_B.zero();
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: points cleared");
                stage = MANUAL;
            }

            // if both A and B have been stored advance state
            if (!dest_A.is_zero() && !dest_B.is_zero() && is_positive((dest_B - dest_A).length_squared())) {
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: location successed.");
                stage = WAIT_MOVE;
            }
            break;

        case AUTO:
        case WAIT_MOVE:
        case WAIT_AUTO:
            if (dest_num == 0) {
                dest_A.zero();
                dest_B.zero();
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: points cleared");
                stage = MANUAL;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "AB Line: stage invalid: %d", stage);
            }
            break;
    }
}

// return manual control to the pilot
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL;
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest);
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        gcs().send_text(MAV_SEVERITY_INFO, "AB Line: manual control");
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
void ModeZigZag::auto_control()
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

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
void ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
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
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    switch (althold_state) {

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
        FALLTHROUGH;

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

// return true if vehicle is within a small area around the destination
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // wait at least one second
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) > 1000);
}

// calculate next destination according to vector A-B and current position
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZag::calculate_next_dest(uint8_t dest_num, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const
{
    // sanity check dest_num
    if (dest_num > 1) {
        return false;
    }

    // define start_pos as either A or B depending upon dest_num
    Vector2f start_pos = dest_num == 0 ? dest_A : dest_B;

    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (!is_positive(AB_diff.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos
    const Vector3f curr_pos = inertial_nav.get_position();
    const Vector2f curr_pos2d = Vector2f(curr_pos.x, curr_pos.y);
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff.length();
    }

    // create a line perpendicular to AB but originating at start_pos
    Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
    Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);

    // find the closest point on the perpendicular line
    const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
    next_dest.x = closest2d.x;
    next_dest.y = closest2d.y;

    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest.z = wp_nav->get_wp_destination().z;
    } else {
        // if we have a downward facing range finder then use terrain altitude targets
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used();
        if (terrain_alt) {
            if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
                next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
            }
        } else {
            next_dest.z = pos_control->is_active_z() ? pos_control->get_alt_target() : curr_pos.z;
        }
    }

    return true;
}

#endif // MODE_ZIGZAG_ENABLED == ENABLED
