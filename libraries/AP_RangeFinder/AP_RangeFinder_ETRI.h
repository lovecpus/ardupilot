#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

#define AP_RANGEFINDER_ETRI_TIMEOUT_MS 400


class AP_RangeFinder_ETRI : public AP_RangeFinder_Backend
{
public:
#if RF_ETRI_MAVLINK
    // constructor
    AP_RangeFinder_ETRI(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_paramsm);

    // static detection function
    static bool detect();

    // Get update from mavlink
    void handle_msg(const mavlink_message_t &msg) override;

#else    
    // constructor
    AP_RangeFinder_ETRI(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_paramsm, uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);
#endif

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
#if !RF_ETRI_MAVLINK
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint8_t linebuf[15];
    uint8_t linebuf_len;
#endif    
};
