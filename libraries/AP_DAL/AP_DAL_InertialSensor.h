#pragma once

#include <AP_InertialSensor/AP_InertialSensor.h>

#include <AP_Logger/LogStructure.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_DAL_InertialSensor {
public:

    // InertialSensor-like methods:

    // return time in microseconds of last update() call
    uint32_t get_last_update_usec(void) const { return _RISH.last_update_usec; }

    // return the selected loop rate at which samples are made available
    uint16_t get_loop_rate_hz(void) const { return _RISH.loop_rate_hz; }


    const Vector3f &get_imu_pos_offset(uint8_t instance) const {
        return _pos[instance];
    }

    // accel stuff
    uint8_t get_accel_count(void) const { return _RISH.accel_count; }
    uint8_t get_primary_accel(void) const { return _RISH.primary_accel; };

    bool use_accel(uint8_t instance) const { return _RISI[instance].use_accel; }
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity) const {
        delta_velocity = _delta_velocity[i];
        return _RISI[i].get_delta_velocity_ret;
    }
    float get_delta_velocity_dt(uint8_t i) const {
        return _RISI[i].delta_velocity_dt;
    }

    // gyro stuff
    uint8_t get_gyro_count(void) const { return _RISH.gyro_count; }
    uint8_t get_primary_gyro(void) const { return _RISH.primary_gyro; };

    bool use_gyro(uint8_t instance) const { return _RISJ[instance].use_gyro; }
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro() const { return _gyro[_primary_gyro]; }
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle) const {
        delta_angle = _delta_angle[i];
        return _RISJ[i].get_delta_angle_ret;
    }
    float get_delta_angle_dt(uint8_t i) const { return _RISJ[i].delta_angle_dt; }

    // return the main loop delta_t in seconds
    float get_loop_delta_t(void) const { return _RISH.loop_delta_t; }

    // AP_DAL methods:
    AP_DAL_InertialSensor();

    void start_frame(const uint64_t time_us);

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    void handle_message(const log_RISH &msg) {
        _RISH = msg;
    }
    void handle_message(const log_RISI &msg) {
        _RISI[msg.instance] = msg;
    }
    void handle_message(const log_RISJ &msg) {
        _RISJ[msg.instance] = msg;
    }
#endif

private:

    struct log_RISH _RISH {
        LOG_PACKET_HEADER_INIT(LOG_RISH_MSG),
    };

    struct log_RISI _RISI[INS_MAX_INSTANCES];
    struct log_RISJ _RISJ[INS_MAX_INSTANCES];

    uint8_t _primary_gyro;
    Vector3f _gyro[INS_MAX_INSTANCES];

    Vector3f _accel[INS_MAX_INSTANCES];

    Vector3f _delta_velocity[INS_MAX_INSTANCES];
    Vector3f _delta_angle[INS_MAX_INSTANCES];

    Vector3f _pos[INS_MAX_INSTANCES];

    void log_header(uint64_t time_us);
    void log_instance(uint64_t time_us, uint8_t i);
};