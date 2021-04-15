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

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start external AHRS update thread");
    }

}

void AP_ExternalAHRS_LORD::update_thread() {
    while (true) {
        check_uart();
        hal.scheduler->delay(10);
        hal.console->printf("In the update thread\n");
    }
}

bool AP_ExternalAHRS_LORD::check_uart() {
    return true;
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

#endif  // HAL_EXTERNAL_AHRS_ENABLED