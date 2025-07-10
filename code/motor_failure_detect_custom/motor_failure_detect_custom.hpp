/**
 * @file motor_failure_detect_custom.hpp
 * @brief Custom motor failure detection for PX4 autopilot
 *
 * This module provides real-time detection of motor failures through:
 * - ESC status monitoring
 * - Attitude error analysis
 * - Current measurement analysis
 * - Actuator control monitoring
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/param.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/WelfordMeanVector.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <parameters/param.h>

#include <atomic>
#include <cmath>
#include <csignal>
#include <stdlib.h>

extern "C" __EXPORT int motor_failure_detect_custom_main(int argc, char *argv[]);

/**
 * @brief Parameters for motor failure detection
 */
struct motor_failure_params {
    param_t ekf2_mot_f;  ///< Parameter handle for motor failure flag
};

/**
 * @brief Motor Failure Detection Class
 */
class MotorFailureDetect : public ModuleBase<MotorFailureDetect> {
public:
    MotorFailureDetect();
    ~MotorFailureDetect() override;

    /** @brief Module initialization and task spawn */
    static int task_spawn(int argc, char *argv[]);
    static MotorFailureDetect *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    void run() override;

private:
    // Task management
    bool should_exit() { return _task_should_exit.load(); }
    void set_should_exit() { _task_should_exit.store(true); }
    std::atomic<bool> _task_should_exit{false};

    // uORB subscriptions
    uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    // Constants
    static constexpr useconds_t UPDATE_INTERVAL = 8000;           ///< 125Hz update rate
    static constexpr float CURRENT_THRESHOLD_DIFF = 3.5f;        ///< Amperes threshold
    static constexpr float SUDDEN_CHANGE_THRESHOLD = 0.03f;      ///< Seconds
    static constexpr float VARIANCE_THRESHOLD = 0.5f;            ///< Current variance threshold
    static constexpr float ACTUATOR_FAILURE_THRESHOLD = 0.08f;   ///< Actuator control threshold

    // Error thresholds (configurable based on flight mode)
    float ROLL_ERROR_THRESHOLD = 0.524f;    ///< Roll error threshold (rad)
    float PITCH_ERROR_THRESHOLD = 0.524f;   ///< Pitch error threshold (rad)
    float YAW_ERROR_THRESHOLD = 0.05f;      ///< Yaw error threshold (rad)

    // Status tracking
    esc_status_s _esc_status{};
    uint64_t _last_update_time{0};
    uint32_t _update_counter{0};
    float _update_rate{0.0f};
    bool _failure_detected{false};
    int8_t _failed_motor_index{-1};
    uint8_t _current_flight_mode_index{255};
    float angular_velocity_z{0.0f};

    // Failure flags
    bool roll_fail_flag{false};
    bool pitch_fail_flag{false};
    bool esc_fail_flag{false};
    bool _motor_failure_detected{false};
    int mot_f_flag{0};
    int _ekf2_mot_f{0};

    // Current monitoring
    uint64_t _last_current_check_time{0};
    matrix::Vector<float, 4> _previous_currents;
    matrix::Vector<float, 4> _curr_currents;
    matrix::Vector<float, 4> _current_diff_mat;
    matrix::Vector<float, 4> _curr_esc_failure;
    matrix::Vector<float, 10> _avg_current_mat;
    float _moving_avg_current{0.0f};

    // Historical data
    matrix::Matrix<float, 4, 10> _current_history;
    matrix::Vector<float, 4> _variance_matrix;
    matrix::Vector<float, 4> _previous_actuator_controls;
    bool _first_actuator_read{true};

    // Parameters
    struct motor_failure_params _params{};

    // Helper functions
    static const char* get_flight_mode_name(uint8_t nav_state, uint8_t &mode_index);
    void print_esc_status();
    void check_esc_failure();
    void check_motor_failure();
    void checkRYPError();
    void publish_motor_failure();
    void enableLandMode();

    // Current analysis methods
    void updateAverageCurrentMatrix(matrix::Vector<float, 4>& curr_currents,
                                  matrix::Vector<float, 10>& avg_current_mat);
    float computeOverallAverage(const matrix::Vector<float, 10>& avg_current_mat);
    void updateCurrentHistory(matrix::Vector<float, 4>& curr_currents);
    void calculateVarianceMatrix();
};
