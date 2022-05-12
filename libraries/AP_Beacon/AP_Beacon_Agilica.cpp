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
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

#include <ctype.h>
#include <stdio.h>

#include "AP_Beacon_Agilica.h"

//#define AGILICA_LOG_ENABLE

#define AGILICA_MSG_LEN_MIN                  12
#define AGILICA_MSG_SYNC                     0x61    // message sync
#define AGILICA_VEHICLE_POS_MSGID            0x01
#define AGILICA_BEACON_POS_MSGID             0x02
#define AGILICA_BEACON_DISTANCE_MAX          20000 //in cm

extern const AP_HAL::HAL& hal;

AP_Beacon_Agilica::AP_Beacon_Agilica(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Beacon, 0));
    }

#ifdef AGILICA_LOG_ENABLE
    AP::logger().Write("AGLB", "tsMS", "I",
        AP_HAL::millis());  
#endif
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Agilica::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - _last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Agilica::update(void)
{
    // return immediately if not serial port
    if (_uart == nullptr) {
        return;
    }

    // check uart for any incoming messages
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t rcv_byte = _uart->read();
        switch (_parse_state) {
            default:
            case ParseState::Waiting_Sync:
                if (rcv_byte == AGILICA_MSG_SYNC) {
                    _parse_state = ParseState::Waiting_Len;
                }                
                break;
            case ParseState::Waiting_Len:
                if (rcv_byte >= AGILICA_MSG_LEN_MIN && rcv_byte < AGILICA_MSG_BUF_MAX) {
                    _msg_len = rcv_byte;
                    _write_index_buf = 0;
                    _parse_state = ParseState::Waiting_Payload;
                } else {
                    _parse_state = ParseState::Waiting_Sync;
                }
                break;
            case ParseState::Waiting_Payload:
                _msg_buf[_write_index_buf++] = rcv_byte;
                if (_write_index_buf == _msg_len) {
#ifdef AGILICA_LOG_ENABLE
                    AP::logger().Write("AGLB", "tsMS", "II",
                                        AP_HAL::millis(), (uint32_t)_msg_len);
#endif
                    if (verify_checksum()) {
                        uint8_t msgid = _msg_buf[0];
                        uint32_t num_beacon = _msg_buf[1];
#ifdef AGILICA_LOG_ENABLE
                        AP::logger().Write("AGLB", "tsMS,msgLn,msgId,nBcn", "IIII",
                                                AP_HAL::millis(),
                                               (uint32_t)_msg_len,
                                               (uint32_t)msgid,
                                               (uint32_t)num_beacon);
#endif

                        if (msgid == AGILICA_VEHICLE_POS_MSGID) {
                            parse_vehicle_pos_msg(num_beacon);
                            _last_update_ms = AP_HAL::millis();
                        } else if (msgid == AGILICA_BEACON_POS_MSGID) {
                            parse_beacon_pos_msg(num_beacon);
                            _last_update_ms = AP_HAL::millis();
                        } 
                    }
                    _parse_state = ParseState::Waiting_Sync;
                } 
                break;
        }
        
    }
}


bool AP_Beacon_Agilica::verify_checksum()
{
    // verify message checksum
    uint8_t checksum = 0;
    checksum ^= _msg_len;    
    for (uint8_t i = 0; i < _write_index_buf; i++) {
        checksum ^= _msg_buf[i];
    }
    // return if failed checksum check
    if (checksum == 0) {
        return true;
    } 

    return false;
}

void AP_Beacon_Agilica::parse_vehicle_pos_msg(const uint32_t num_beacon)
{   
    uint8_t *p;

    int16_t x = 0;
    p = (uint8_t *)&x;
    *p++ = _msg_buf[2];
    *p = _msg_buf[3];

    int16_t y = 0;
    p = (uint8_t *)&y;
    *p++ = _msg_buf[4];
    *p = _msg_buf[5];

    int16_t z = 0;
    p = (uint8_t *)&z;
    *p++ = _msg_buf[6];
    *p = _msg_buf[7];

    float accuracy_estimate = _msg_buf[8]*0.01f;
#ifdef AGILICA_LOG_ENABLE
    AP::logger().Write("AGLB", "tsMS,vpx,vpx,vpz, vpa", "IhhhB",
    _last_update_ms, x, y, z, _msg_buf[8]);
#endif


    Vector3f pos(Vector3f(x*0.01f, y*0.01f, -z*0.01f));
    set_vehicle_position(pos, accuracy_estimate);



    for (uint8_t i = 9; i < (_write_index_buf - 1); i += 3) {
        uint8_t beacon_id = _msg_buf[i];

        uint16_t beacon_dist = 0;
        p = (uint8_t *)&beacon_dist;
        *p++ = _msg_buf[i + 1];
        *p = _msg_buf[i + 2];        
#ifdef AGILICA_LOG_ENABLE
        AP::logger().Write("AGLB", "bcnId,bcnDst", "BH",
                           beacon_id, beacon_dist);
#endif

        if (beacon_dist <= AGILICA_BEACON_DISTANCE_MAX) {
            set_beacon_distance(beacon_id, beacon_dist*0.01f);
        }
    }
}

void AP_Beacon_Agilica::parse_beacon_pos_msg(const uint32_t num_beacon)
{    

    uint8_t *p;
    for (uint8_t i = 2; i < (_write_index_buf - 1); i += 7) {
        uint8_t beacon_id = _msg_buf[i];
        
        int16_t x = 0;
        p = (uint8_t *)&x;
        *p++ = _msg_buf[i + 1];
        *p = _msg_buf[i + 2];

        int16_t y = 0;
        p = (uint8_t *)&y;
        *p++ = _msg_buf[i + 3];
        *p = _msg_buf[i + 4];

        int16_t z = 0;
        p = (uint8_t *)&z;
        *p++ = _msg_buf[i + 5];
        *p = _msg_buf[i + 6];    
#ifdef AGILICA_LOG_ENABLE
        AP::logger().Write("AGLB", "tsMS,bcnId,bpx,bpx,bpz", "IBhhh",
        _last_update_ms, beacon_id, x, y, z);  
#endif

        Vector3f pos(x * 0.01f, y * 0.01f, -z * 0.01f);
        set_beacon_position(beacon_id, pos);
    }
}