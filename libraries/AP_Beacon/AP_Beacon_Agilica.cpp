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

#define AGILICA_MSG_LEN_MIN                  5
#define AGILICA_MSG_SYNC                     0x61    // message sync
#define AGILICA_VEHICLE_POS_MSGID            0x01
#define AGILICA_BEACON_POS_MSGID             0x02

#ifndef AGILICA_LEGACY
#define AGILICA_BEACON_AUX_MSGID             0x03
#define AGILICA_STATUS_MSGID                 0x04

/* STATUS VALUES */
/* Possible STATUS VALUES in the status message */
#define AGILICA_STATUS_OKAY             0 // all is good
#define AGILICA_STATUS_WAITING_SYNC     1 // waiting for synchronization lock.
#define AGILICA_STATUS_NOT_ENOUGH_ANK   2 // does not receive from enough anchors
#define AGILICA_STATUS_NLOSOFIL         3 // outlier filter is active
#endif

#define AGILICA_BEACON_DISTANCE_MAX          20000 //in cm

extern const AP_HAL::HAL& hal;

#define MM_DEBUG_LEVEL 0

#if MM_DEBUG_LEVEL
  #include <GCS_MAVLink/GCS.h>
  #define Debug(level, fmt, args ...)  do { if (level <= MM_DEBUG_LEVEL) { gcs().send_text(MAV_SEVERITY_INFO, fmt, ## args); } } while (0)
#else
  #define Debug(level, fmt, args ...)
#endif



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
    if (uart == nullptr) {
        return;
    }

    // check uart for any incoming messages
    uint32_t nbytes = MIN(uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t rcv_byte = uart->read();
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
                        } 
#ifndef AGILICA_LEGACY
                        else if (msgid == AGILICA_BEACON_AUX_MSGID) {
                            parse_beacon_aux_msg(num_beacon);
                        }
                        else if (msgid == AGILICA_STATUS_MSGID) {
                            parse_status_msg(num_beacon);
                        }
#endif
                        else if (msgid == AGILICA_BEACON_POS_MSGID) {
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
    int16_t x = ((uint16_t)(_msg_buf[2])) | ((uint16_t)(_msg_buf[3] << 8));
    int16_t y = ((uint16_t)(_msg_buf[4])) | ((uint16_t)(_msg_buf[5] << 8));
    int16_t z = ((uint16_t)(_msg_buf[6])) | ((uint16_t)(_msg_buf[7] << 8));


    float accuracy_estimate = _msg_buf[8]*0.01f;
#ifndef AGILICA_LEGACY
    float xdop = _msg_buf[9]*0.1f;
    /*WARNING: the software fails to compile if there is any unused variable
     *TODO: user need to implement logic on how to use 'xdop'  
     */
    dummySinkXdop(xdop);
#endif

#ifdef AGILICA_LOG_ENABLE
    AP::logger().Write("AGLB", "tsMS,vpx,vpx,vpz, vpa", "IhhhB",
    _last_update_ms, x, y, z, _msg_buf[8]);
#endif


    Vector3f pos(Vector3f(x*0.01f, y*0.01f, -z*0.01f));
    set_vehicle_position(pos, accuracy_estimate);



#ifndef AGILICA_LEGACY
    for (uint8_t i = 10; i < (_write_index_buf - 1); i += 3) {
#else
    for (uint8_t i = 9; i < (_write_index_buf - 1); i += 3) {
#endif
        uint8_t beacon_id = _msg_buf[i];

        uint16_t beacon_dist = ((uint16_t)(_msg_buf[i + 1])) | ((uint16_t)(_msg_buf[ i + 2] << 8));    

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
    for (uint8_t i = 2; i < (_write_index_buf - 1); i += 7) {
        uint8_t beacon_id = _msg_buf[i];        

        int16_t x = ((uint16_t)(_msg_buf[i + 1])) | ((uint16_t)(_msg_buf[i + 2] << 8));
        int16_t y = ((uint16_t)(_msg_buf[i + 3])) | ((uint16_t)(_msg_buf[i + 4] << 8));
        int16_t z = ((uint16_t)(_msg_buf[i + 5])) | ((uint16_t)(_msg_buf[i + 6] << 8));
 

#ifdef AGILICA_LOG_ENABLE      
        AP::logger().Write("AGLB", "tsMS,bcnId,bpx,bpx,bpz", "IBhhh",
        _last_update_ms, beacon_id, x, y, z);          
#endif
      // Debug(2, "Beacon %d is %.2fm, %.2fm, %.2fm", beacon_id, x*0.01f, y*0.01f, z*0.01f);

        Vector3f pos(x * 0.01f, y * 0.01f, -z * 0.01f);
        set_beacon_position(beacon_id, pos);
    }
}

#ifndef AGILICA_LEGACY
void AP_Beacon_Agilica::parse_status_msg(const uint32_t status)
{
    switch (status) {
        case AGILICA_STATUS_WAITING_SYNC:
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "tag,s,WAITING_SYNC");
#endif
            break;
        case AGILICA_STATUS_NOT_ENOUGH_ANK:
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "tag,s,NOT_ENOUGH_ANK");
#endif
            break;
        case AGILICA_STATUS_NLOSOFIL:
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "tag,s,NLOSOFIL");
#endif
            break;
        case AGILICA_STATUS_OKAY:
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "tag,s,OKAY");
#endif
            break;
        default:
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "tag,s,UNKNOWN");
#endif
            break;
    }

}

void AP_Beacon_Agilica::parse_beacon_aux_msg(const uint32_t num_beacon)
{
    for (uint8_t i = 2; i < (_write_index_buf - 1); i += 2) {        
        uint8_t ankId = _msg_buf[i];
        int8_t rssi = _msg_buf[i+1];
        /*WARNING: the software fails to compile if there is any unused variable
        *TODO: user need to implement logic on how to use 'ankId' and 'rssi'  
        */
        dummySinkBeaconAux(ankId, rssi);
    }
}

void AP_Beacon_Agilica::dummySinkXdop(const float xdop)
{
    

}

void AP_Beacon_Agilica::dummySinkBeaconAux(const uint8_t ankId, const int8_t rssi)
{
#ifdef AGILICA_LOG_ENABLE 
            AP::logger().Write("AGLB", "ank,i,s","hh", (int)ankId, (int)rssi);
#endif
}
#endif