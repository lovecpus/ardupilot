#include <deque>
#include <algorithm>

#define USE_CNDN_RNG
#define CN_UNUSED(_X)  ((_X)=(_X))

#if MODE_CNDN_ENABLED == ENABLED
/*
* Init and run calls for CNDN flight mode
*/
#define CNDN_TONE_STARTUP    { "MFT200L4O4CL8GAL2F", false }

#define CNDN_WP_RADIUS_CM 100
#define CASE_CNDN_MODE() case Mode::Number::CNDN: ret = &mode_cndn; mode_cndn.setZigZag(false);  break; \
    case Mode::Number::CNDN2: ret = &mode_cndn; mode_cndn.setZigZag(true);  break;

#define CASE_CNDN_AUX_INIT()  case AUX_FUNC::CNDN: case AUX_FUNC::CNDN_AUTO:  case AUX_FUNC::CNDN_PUMP: \
        case AUX_FUNC::CNDN_SPD_UP: case AUX_FUNC::CNDN_SPD_DN: case AUX_FUNC::CNDN_SPR_UP: case AUX_FUNC::CNDN_SPR_DN:

#define CASE_CNDN_AUX_FUNC()  case AUX_FUNC::CNDN: \
    do_aux_function_change_mode(Mode::Number::CNDN, ch_flag); \
    break; \
    case AUX_FUNC::CNDN_AUTO: {\
        switch (ch_flag) { \
        case LOW: copter.mode_cndn.mission_command(0); break;\
        case MIDDLE: copter.mode_cndn.mission_command(1); break;\
        case HIGH: copter.mode_cndn.mission_command(2); break;\
        }\
    } break; \
    case AUX_FUNC::CNDN_SPD_UP: if (ch_flag != LOW) copter.mode_cndn.mission_command(6); break;\
    case AUX_FUNC::CNDN_SPD_DN: if (ch_flag != LOW) copter.mode_cndn.mission_command(7); break;\
    case AUX_FUNC::CNDN_SPR_UP: if (ch_flag != LOW) copter.mode_cndn.mission_command(8); break;\
    case AUX_FUNC::CNDN_SPR_DN: if (ch_flag != LOW) copter.mode_cndn.mission_command(9); break;\
    case AUX_FUNC::CNDN_PUMP: break;

#define CNDN_HANDLE_MESSAGE() \
    copter.mode_cndn.handle_message(msg);

#define CNDN_MODE_INJECT()  mode_cndn.inject()
/*
int degNE(const Vector2f& pp);
int degNE(const Vector2f& p1, const Vector2f& p2);
Vector3f locNEU(float latf, float lngf, float altf);
*/
class ModeCNDN : public Mode
{
    friend class ModeZigZag;
public:
    // inherit constructor
    using Mode::Mode;
    ModeCNDN();

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool set_destination(const Vector3f &destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    // save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
    void mission_command(uint8_t dest_num);
    void return_to_manual_control(bool maintain_target);
    void handle_message(const mavlink_message_t &msg) override;
    void do_set_relay(const AP_Mission::Mission_Command& cmd);
    void inject();
    void stop_mission();
    void resume_mission();
    void setZigZag(bool bZigZag) { m_bZigZag = bZigZag; }
    bool isZigZag() { return m_bZigZag; }

    static const struct AP_Param::GroupInfo var_info[];

#if 0
    uint32_t live_logt_ms = 0; // time since vehicle reached destination (or zero if not yet reached)
    void live_log(uint32_t tout, const char *fmt, ...);
#endif

protected:
    const char *name() const override { return "CNDN_AW"; }
    const char *name4() const override { return "CNDN"; }

private:
    void init_speed();
    void pos_control_start();
    void auto_control();
    void manual_control();
    bool reached_destination();
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    void processArea();
    void processAB();

    enum cndn_state
    {
        MANUAL,         // pilot toggle the switch to middle position, has manual control
        PREPARE_AUTO,
        AUTO,
        RETURN_AUTO,
        PREPARE_FINISH,
        FINISHED,
        WAY_A,
        WAY_B,
    } stage;

    uint16_t        data_size = 0;
    uint16_t        data_wpos = 0;
    char*           data_buff = NULL;
    uint32_t        reach_wp_time_ms = 0; // time since vehicle reached destination (or zero if not yet reached)
    float           last_yaw_cd = 0.0f;
    uint32_t        last_yaw_ms = 0;
    uint8_t         cmd_mode;
    bool            edge_mode = false;
    bool            m_bZigZag;

    // parameters
    AP_Int8         _method;                ///< CNDN Method 0: Disable, 1: Take Picture, 2: Edge following and auto mission, 3: Mission 
    AP_Int16        _take_alt_cm;           ///< Takeoff Altitute
    AP_Int16        _mission_alt_cm;        ///< Mission altitute
    AP_Int16        _spray_width_cm;        ///< Spray width cm
    AP_Int16        _dst_eg_cm;             ///< Edge distance cm
    AP_Int16        _spd_edge_cm;           ///< Edge speed cm/s
    AP_Int16        _spd_auto_cm;           ///< Auto speed cm/s
    AP_Float        _radar_flt_hz;          ///< RADAR Lowpass filter apply frequency
};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define CNLOG   1
class CNLog {
private:
    FILE *fp;     
public:
    static CNLog& get(const char *fname) {
        static CNLog logs(fname);
        return logs;
    }

    CNLog(const char *fname) {
        fp = fopen(fname, "at");
    }

    ~CNLog() {
        if (fp)
            fclose(fp);
    }

    int log(const char *form, ...) {
        if (!fp)
            return -1;
        va_list args;
        va_start(args, form);
        return vfprintf(fp, form, args);
    }
};

extern CNLog &cnlog;
#endif

#ifdef CNLOG
#define cndebug(form, args ...) cnlog.log(form "\r\n", ## args)
#else
#define cndebug(form, args ...)
#endif

#endif
