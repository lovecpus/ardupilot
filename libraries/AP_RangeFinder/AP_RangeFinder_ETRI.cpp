/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_ETRI.h"
#include <AP_HAL/AP_HAL.h>



extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_ETRI::AP_RangeFinder_ETRI(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    state.last_reading_ms = AP_HAL::millis();
    distance_cm = 0;
}

/*
   detect if a MAVLink rangefinder is connected. We'll detect by
   checking a parameter.
*/
bool AP_RangeFinder_ETRI::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MAVLink,
    // there is an attached MAVLink rangefinder
    return true;
}

/*
   Set the distance based on a MAVLINK message
*/
void AP_RangeFinder_ETRI::set_distance(float fdist_m)
{
    state.last_reading_ms = AP_HAL::millis();
    distance_cm = fdist_m * 100.0f;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_ETRI::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_ETRI_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
        state.distance_cm = 0;
    } else {
        state.distance_cm = distance_cm;
        update_status();
    }
}
