#pragma once

#include "AP_Beacon_Backend.h"

#define AGILICA_MSG_BUF_MAX      (255)

/* Will also parse status and aux beacon messages when line '#define AGILICA_LEGACY' is commented out  */
//#define AGILICA_LEGACY

class AP_Beacon_Agilica : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_Agilica(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update the state of the sensor
    void update() override;    

private:
    // parse vehicle pos message 
    void parse_vehicle_pos_msg(const uint32_t num_beacon);
#ifndef AGILICA_LEGACY
    // parse beacon aux msg
    void parse_beacon_aux_msg(const uint32_t num_beacon);
    //parse status msg
    void parse_status_msg(const uint32_t status);
    
    void dummySinkXdop(const float xdop);
    void dummySinkBeaconAux(const uint8_t ankId, const int8_t rssi);    
#endif
    // parse beacon pos message 
    void parse_beacon_pos_msg(const uint32_t num_beacon);
    //verify message checksum
    bool verify_checksum();



    AP_HAL::UARTDriver *_uart = nullptr;

    enum class ParseState : uint8_t {
        Waiting_Sync = 0,       // waiting for msg sync
        Waiting_Len = 1,        // waiting for message len
        Waiting_Payload = 2     // waiting for message payload
    };
    
    ParseState _parse_state = ParseState::Waiting_Sync;

    // members
    uint8_t _msg_buf[AGILICA_MSG_BUF_MAX];      // buffer to hold most recent message from tag
    uint8_t  _write_index_buf = 0;              // write index for the msg_buf
    uint16_t _msg_len;                          // received message length    
    uint32_t _last_update_ms;                   // last time we receive data from tag
};
