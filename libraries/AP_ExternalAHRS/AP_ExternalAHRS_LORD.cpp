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
/*
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_LORD.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

using LORDpacketData_t = AP_ExternalAHRS_LORD::LORDpacketData_t;

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}

bool AP_ExternalAHRS_LORD::check_uart() {
    if (!port_opened)
        return false;
    WITH_SEMAPHORE(state.sem);

    uint8_t pkt[] = { 0x75, 0x65, 0x80, 0x2a, 0x0e, 0x04, 0x3e, 0x49, 0x56, 0x65, 0xbb, 0x24, 0x12, 0xc0, 0xbf, 0x7a, 0xa0, 0x1d, 0x0e, 0x05, 0xbb, 0xc7, 0x35, 0x1d, 0xbb, 0x22, 0xce, 0x02, 0x3b, 0x0e, 0xf6, 0x1a, 0x0e, 0x12, 0x40, 0x5c, 0x1a, 0xb0, 0x20, 0xc4, 0x9b, 0xa6, 0x00, 0x00, 0x00, 0x06, 0x1d, 0x37 };
    LORDpacketData_t data = processLORDPacket(pkt);

    state.accel = data.accel;
    state.gyro = data.accel;
    state.have_quaternion = false;
    state.have_velocity = false;
    state.have_location = false;

    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel = state.accel;
    ins.gyro = state.gyro;
    AP::ins().handle_external(ins);

    return true;

}

void AP_ExternalAHRS_LORD::update_thread() {
    if (!port_opened) {
        // open port in the thread
        port_opened = true;
        uart->begin(baudrate);
    }

    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

int8_t AP_ExternalAHRS_LORD::get_port(void) const
{
    return -1;
};

bool AP_ExternalAHRS_LORD::healthy(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::initialised(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    return true;
}

void AP_ExternalAHRS_LORD::get_filter_status(nav_filter_status &status) const
{
    return;
}

void AP_ExternalAHRS_LORD::send_status_report(mavlink_channel_t chan) const
{
    return;
}

LORDpacketData_t AP_ExternalAHRS_LORD::processLORDPacket(const uint8_t* pkt) {
    uint8_t pktDesc = pkt[2];
    switch (pktDesc) {
        case 0x80:
            return insData(pkt);
        default:
            LORDpacketData_t nullPacket = { {-999, -999, -999}, {-999, -999, -999} };
            return nullPacket;
    }
}

LORDpacketData_t AP_ExternalAHRS_LORD::insData(const uint8_t* pkt) {
    LORDpacketData_t data;
    uint8_t payloadLen = pkt[3];
    for (uint8_t i = 4; i < payloadLen; i += pkt[i]) {
        uint8_t fieldDesc = pkt[i+1];
        switch (fieldDesc) {
            case 04:
                data.accel = populateVector3f(pkt, i, 9.8);
                break;
            case 05:
                data.gyro = populateVector3f(pkt, i, 1);
                break;
        }
    }
    return data;
}

Vector3f AP_ExternalAHRS_LORD::populateVector3f(const uint8_t* pkt, uint8_t offset, float multiplier = 1) {
    Vector3f data;
    uint32_t tmp[3];
    for (uint8_t j = 0; j < 3; j++) {
        tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    }
    data.x = (*reinterpret_cast<float*>( &tmp[0] )) * multiplier;
    data.y = (*reinterpret_cast<float*>( &tmp[1] )) * multiplier;
    data.z = (*reinterpret_cast<float*>( &tmp[2] )) * multiplier;
    return data;
}

uint64_t AP_ExternalAHRS_LORD::get8ByteField(const uint8_t* pkt, uint8_t offset) {
    uint64_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 32 | get4ByteField(pkt,offset + 4 * i);
    return res;
}

uint32_t AP_ExternalAHRS_LORD::get4ByteField(const uint8_t* pkt, uint8_t offset) {
    uint32_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 16 | get2ByteField(pkt, offset + 2 * i);
    return res;
}

uint16_t AP_ExternalAHRS_LORD::get2ByteField(const uint8_t* pkt,uint8_t offset) {
    uint16_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 8 | pkt[offset + i];
    return res;
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED