#include <deque>
#include <algorithm>

# define gcsdebug(FORM, ...)    gcs().send_text(MAV_SEVERITY_DEBUG, FORM, __VA_ARGS__)
# define gcsinfo(FORM)          gcs().send_text(MAV_SEVERITY_INFO, FORM)
# define gcswarning(FORM)       gcs().send_text(MAV_SEVERITY_WARNING, FORM)

struct CNMIS {
    float   yaw_deg;
    float   spdcm;
    uint8_t spryr;
    uint8_t edge;
    bool    addNew;
    uint16_t curr_idx;
    uint16_t repl_idx;
    uint16_t jump_idx;
    Location loctg;
    int32_t  misAlt;
    Location::AltFrame misFrame;
};

# define    CNDN_ZIGZAG_RESUME                                              \
    CNMIS cms;                                                              \
    bool getResume();                                                       \
    void setResume();                                                       \
    bool processArea(Vector2f& dstA,Vector2f& dstB, bool bLeftRight);       \
    bool processAB(Vector2f& dstA,Vector2f& dstB, bool bLeftRight);         \
    void turnZigZag(uint8_t state);                                         \
    bool resume_mission();                                                  \
    bool isOwnMission();                                                    \
    bool hasResume(uint16_t &resumeIdx);

#define CN_UNUSED(_X)  ((_X)=(_X))

#if MODE_CNDN_ENABLED == ENABLED
/*
* Init and run calls for CNDN flight mode
*/
#define CNDN_TONE_STARTUP    { "MFT200L4O4CL8GAL2F", false }

#define CNDN_WP_RADIUS_CM 100
#define CASE_CNDN_MODE() case Mode::Number::CNDN: ret = &mode_cndn; mode_cndn.setZigZag(false);  break; \
    case Mode::Number::CNDN2: ret = &mode_cndn; mode_cndn.setZigZag(true); break;

#define CASE_CNDN_AUX_INIT()  case AUX_FUNC::CNDN: case AUX_FUNC::CNDN_AUTO:  case AUX_FUNC::CNDN_PUMP: \
        case AUX_FUNC::CNDN_SPD_UP: case AUX_FUNC::CNDN_SPD_DN: case AUX_FUNC::CNDN_SPR_UP: case AUX_FUNC::CNDN_SPR_DN: \
        case AUX_FUNC::CNDN_SPR_FF:

#define CASE_CNDN_AUX_FUNC()  case AUX_FUNC::CNDN: \
    copter.mode_cndn.mission_command(3 + ch_flag); \
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
    case AUX_FUNC::CNDN_PUMP: break; \
    case AUX_FUNC::CNDN_TRIG02: if (ch_flag == HIGH) copter.mode_cndn.mission_command(20); break;

#define CNDN_HANDLE_MESSAGE() \
    copter.mode_cndn.handle_message(msg);

#define CNDN_MODE_INJECT()  mode_cndn.inject()
#define CNDN_MODE_INJECT25()  mode_cndn.inject_25hz()
#define CNDN_MODE_INJECT50()  mode_cndn.inject_50hz()
#define CNDN_MODE_INJECT400()  mode_cndn.inject_400hz()

/*
int degNE(const Vector2f& pp);
int degNE(const Vector2f& p1, const Vector2f& p2);
Vector3f locNEU(float latf, float lngf, float altf);
*/
// auxillary switch handling (n.b.: we store this as 2-bits!):

class ModeCNDN : public Mode
{
    friend class ModeZigZag;
public:
    enum aux_switch_pos_t : uint8_t {
        LOW,       // indicates auxiliary switch is in the low position (pwm <1200)
        MIDDLE,    // indicates auxiliary switch is in the middle position (pwm >1200, <1800)
        HIGH       // indicates auxiliary switch is in the high position (pwm >1800)
    };
    // inherit constructor
    using Mode::Mode;
    ModeCNDN();

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return false; }
    bool set_destination(const Vector3f &destination, bool use_yaw = false, float yaw_cd = 0.0, bool use_yaw_rate = false, float yaw_rate_cds = 0.0, bool yaw_relative = false);
    // save current position as A (dest_num = 0) or B (dest_num = 1).  If both A and B have been saved move to the one specified
    void mission_command(uint8_t dest_num);
    void return_to_manual_control(bool maintain_target);
    void handle_message(const mavlink_message_t &msg) override;
    void do_set_relay(const AP_Mission::Mission_Command& cmd);
    void inject();
    void inject_25hz();
    void inject_50hz();
    void inject_400hz();
    void stop_mission();
    bool resume_mission();
    void setZigZag(bool bZigZag) { m_bZigZag = bZigZag; }
    bool isZigZag() { return m_bZigZag; }
    void initMissionResume();
    bool hoverMissionResume();
    void do_aux_function_sprayer(const uint8_t ch_flag);

    static const struct AP_Param::GroupInfo var_info[];
    CNTimeout toAUTO, toDBG, toGUIDED;
    uint32_t    u32_runSpray;

    Mode *switch_zigzag();

protected:
    const char *name() const override { return "CNDN_AW"; }
    const char *name4() const override { return "CNDN"; }

private:

    void init_speed();
    void pos_control_start();
    void auto_control();
    void manual_control();
    void zigzag_manual();
    bool reached_destination();
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    bool processArea();
    bool processAB();
    bool getResume(CNMIS& dat);
    void setResume(CNMIS& dat, bool bRemote = false);
    bool isOwnMission();
    bool hasResume(uint16_t &resumeIdx);

   enum cndn_state
    {
        MANUAL,         // pilot toggle the switch to middle position, has manual control
        PREPARE_AUTO,
        AUTO,
        SET_AUTO,
        PREPARE_ABLINE,
        FINISHED,
    } stage;

    float           yaw_deg = 0;
    uint32_t        reach_wp_time_ms = 0; // time since vehicle reached destination (or zero if not yet reached)
    uint8_t*        data_buff = NULL;
    uint16_t        data_size = 0;
    uint16_t        data_wpos = 0;
    uint8_t         data_sysid = 0;
    uint8_t         data_cmpid = 0;
    float           last_yaw_deg = 0.0f;
    bool            edge_mode = false;
    bool            m_bZigZag;
    uint8_t         cmd_mode;
    uint8_t         m_lockStick = 0;
    Vector2f        m_target_pos;
    uint32_t        _rate_dt = 0;
    Location        resumeLoc;
    CNTimeout       toYAW, toBAT;
    uint32_t        m_missionReset = 0;

    // parameters
    AP_Int8         _method;                ///< CNDN Method 0: Disable, 1: Take Picture, 2: Edge following and auto mission, 3: Mission 
    AP_Int16        _take_alt_cm;           ///< Takeoff Altitute
    AP_Int16        _mission_alt_cm;        ///< Mission altitute
    AP_Int16        _spray_width_cm;        ///< Spray width cm
    AP_Int16        _dst_eg_cm;             ///< Edge distance cm
    AP_Int16        _spd_edge_cm;           ///< Edge speed cm/s
    AP_Int16        _spd_auto_cm;           ///< Auto speed cm/s
    AP_Float        _radar_flt_hz;          ///< RADAR Lowpass filter apply frequency
    AP_Int16        _spray_backs;           ///< Sprayer minimum steps
    AP_Int8         _sensor_pin;            ///< CNDN Level sensor gpio pin
    AP_Int16        _avoid_cm;              ///< avoid 10cm
};

#endif
