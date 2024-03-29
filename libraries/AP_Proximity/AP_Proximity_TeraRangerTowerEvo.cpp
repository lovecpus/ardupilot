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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_TeraRangerTowerEvo.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_TeraRangerTowerEvo::AP_Proximity_TeraRangerTowerEvo(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar360, 0));
    }
    _last_request_sent_ms = AP_HAL::millis();
}

// detect if a TeraRanger Tower proximity sensor is connected by looking for a configured serial port
bool AP_Proximity_TeraRangerTowerEvo::detect()
{
    return (AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Lidar360, 0) != nullptr);
}

// update the state of the sensor
void AP_Proximity_TeraRangerTowerEvo::update(void)
{
    if (uart == nullptr) {
        return;
    }

    //initialize the sensor
    if(_current_init_state != InitState::InitState_Finished)
    {
        initialise_modes();
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_TRTOWER_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_TeraRangerTowerEvo::distance_max() const
{
    return 60.0f;
}
float AP_Proximity_TeraRangerTowerEvo::distance_min() const
{
    return 0.50f;
}

void AP_Proximity_TeraRangerTowerEvo::initialise_modes()
{
    if((AP_HAL::millis() - _last_request_sent_ms) < _mode_request_delay) {
        return;
    }

    if (_current_init_state == InitState_Printout) {
        set_mode(BINARY_MODE, 4);
    } else if (_current_init_state == InitState_Sequence) {
        //set tower mode - 4 sensors are triggered at once with 90 deg angle between each sensor
        set_mode(TOWER_MODE, 4);
    } else if (_current_init_state == InitState_Rate) {
        //set update rate of the sensor.
        set_mode(REFRESH_100_HZ, 5);
    } else if (_current_init_state == InitState_StreamStart) {
        set_mode(ACTIVATE_STREAM, 5);
    }
}

void AP_Proximity_TeraRangerTowerEvo::set_mode(const uint8_t *c, int length)
{
    uart->write(c, length);
    _last_request_sent_ms = AP_HAL::millis();
}

// check for replies from sensor, returns true if at least one message was processed
bool AP_Proximity_TeraRangerTowerEvo::read_sensor_data()
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t message_count = 0;
    int16_t nbytes = uart->available();

    if(_current_init_state != InitState_Finished && nbytes == 4) {

        //Increment _current_init_state only when we receive 4 ack bytes
        switch (_current_init_state) {
            case InitState_Printout:
                _current_init_state = InitState_Sequence;
                break;
            case InitState_Sequence:
                _current_init_state = InitState_Rate;
                break;
            case InitState_Rate:
                _current_init_state = InitState_StreamStart;
                break;
            case InitState_StreamStart:
                _current_init_state = InitState_Finished;
                break;
            case InitState_Finished:
                break;
        }
    }

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T' ) {
            buffer_count = 0;
        }
        buffer[buffer_count++] = c;

        // we should always read 19 bytes THxxxxxxxxxxxxxxxxMC
        if (buffer_count >= 20){
            buffer_count = 0;

            //check if message has right CRC
            if (crc_crc8(buffer, 19) == buffer[19]){
                update_sector_data(0,   UINT16_VALUE(buffer[2],  buffer[3]));   // d1
                update_sector_data(45,  UINT16_VALUE(buffer[4],  buffer[5]));   // d2
                update_sector_data(90,  UINT16_VALUE(buffer[6],  buffer[7]));   // d3
                update_sector_data(135, UINT16_VALUE(buffer[8],  buffer[9]));   // d4
                update_sector_data(180, UINT16_VALUE(buffer[10], buffer[11]));  // d5
                update_sector_data(225, UINT16_VALUE(buffer[12], buffer[13]));  // d6
                update_sector_data(270, UINT16_VALUE(buffer[14], buffer[15]));  // d7
                update_sector_data(315, UINT16_VALUE(buffer[16], buffer[17]));  // d8

                message_count++;
            }
        }
    }
    return (message_count > 0);
}

// process reply
void AP_Proximity_TeraRangerTowerEvo::update_sector_data(int16_t angle_deg, uint16_t distance_cm)
{
    uint8_t sector;
    if (convert_angle_to_sector(angle_deg, sector)) {
        _angle[sector] = angle_deg;
        _distance[sector] = ((float) distance_cm) / 1000;

        //check for target too far, target too close and sensor not connected
        _distance_valid[sector] = distance_cm != 0xffff && distance_cm != 0x0000 && distance_cm != 0x0001;
        _last_distance_received_ms = AP_HAL::millis();
        // update boundary used for avoidance
        update_boundary_for_sector(sector, true);
    }
}
