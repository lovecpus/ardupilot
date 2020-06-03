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
AP_RangeFinder_ETRI::AP_RangeFinder_ETRI(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(57600);//serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/*
   detect if a CNDN rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_ETRI::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// distance returned in reading_cm, signal_ok is set to true if sensor reports a strong signal
bool AP_RangeFinder_ETRI::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }
    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 'E' or '1', add to buffer
        if (linebuf_len == 0) {
            if ((c == 'E') || (c == '1')) {
                linebuf[linebuf_len++] = c;
            }
        } else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is ':', add it to buffer
            // if not clear the buffer
            if (c == ':') {
                linebuf[linebuf_len++] = c;
            } else {
                linebuf_len = 0;
            }
        } else if (c == '\n') {
            if (linebuf[0] == '1') {
                linebuf[linebuf_len] = 0;
                uint16_t dist = (uint16_t)strtol((const char *)(linebuf+2),0,0);
                if (dist <= max_distance_cm()) {
                    count ++;
                    sum_cm += dist;
                } else {
                    count_out_of_range ++;
                }
            }
            // clear buffer
            linebuf_len = 0;
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;
            if (linebuf_len > 14) {
                linebuf_len = 0;
            }
        }
    }

    if (count > 0) {
        // return average distance of readings
        reading_cm = (uint16_t)(sum_cm / count);
        return true;
    }
#if 0
    return true;
#else
    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_cm = max_distance_cm() + 100.0f;
        return true;
    }

    // no readings so return false
    return false;
#endif    
}
/*
   update the state of the sensor
*/
void AP_RangeFinder_ETRI::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
