#if MODE_CNDN_ENABLED == ENABLED
/*
* Init and run calls for CNDN flight mode
*/
#define CNDN_WP_RADIUS_CM 200
#define CASE_CNDN_MODE() case Mode::Number::CNDN: ret = &mode_cndn; break;
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

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool is_position_target_reached() override { return b_position_target_reached; }
    bool set_destination(const Vector3f &destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    // save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
    void mission_command(uint8_t dest_num);
    void return_to_manual_control(bool maintain_target);
    void handle_message(const mavlink_message_t &msg) override;

protected:
    const char *name() const override { return "CNDN_ETRI"; }
    const char *name4() const override { return "CNDN"; }

private:
    void pos_control_start();
    void auto_control();
    void manual_control();
    bool reached_destination();
    bool calculate_next_dest(uint8_t position_num, bool use_wpnav_alt, Vector3f &next_dest, bool &terrain_alt) const;
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
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
    typedef Vector2<double> Vector2D;
    Vector2D edge_points[10]; // in NEU frame in cm relative to ekf origin
};
#endif
