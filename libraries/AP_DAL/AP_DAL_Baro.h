#pragma once

#include <AP_Baro/AP_Baro.h>

#include <AP_Logger/LogStructure.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_DAL_Baro {
public:
    // methods so we look like AP_Baro:
    uint8_t get_primary() const {
        return _RBRH.primary;
    }
    uint8_t num_instances() const {
        return _RBRH.num_instances;
    }
    bool healthy(uint8_t sensor_id) const {
        return _RBRI[sensor_id].healthy;
    }
    uint32_t get_last_update(uint8_t sensor_id) const {
        return _RBRI[sensor_id].last_update_ms;
    }
    uint32_t get_last_update() const {
        return get_last_update(get_primary());
    }
    float get_altitude(uint8_t sensor_id) const {
        return _RBRI[sensor_id].altitude;
    }
    float get_altitude() const {
        return get_altitude(get_primary());
    }

    // update_calibration is a no-op in Replay as it simply modifies the data
    // which we'll be logging for input to the EKF.
    void update_calibration();

    // methods for being part of AP_DAL:
    AP_DAL_Baro();

    void start_frame(const uint64_t time_us);

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    void handle_message(const log_RBRH &msg) {
        _RBRH = msg;
    }
    void handle_message(const log_RBRI &msg) {
        _RBRI[msg.instance] = msg;
    }
#endif

private:

    struct log_RBRH _RBRH {
        LOG_PACKET_HEADER_INIT(LOG_RBRH_MSG),
    };
    struct log_RBRI _RBRI[BARO_MAX_INSTANCES];

    uint32_t _last_logged_update_ms[BARO_MAX_INSTANCES];
};
