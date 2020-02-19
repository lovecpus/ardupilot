#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

// Data timeout
#define AP_RANGEFINDER_ETRI_TIMEOUT_MS 2500

class AP_RangeFinder_ETRI : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_ETRI(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    // Get update from source
    void set_distance(float fdist_m);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return sensor_type;
    }

private:
    uint16_t distance_cm;

    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);

    MAV_DISTANCE_SENSOR sensor_type = MAV_DISTANCE_SENSOR_LASER;
};
