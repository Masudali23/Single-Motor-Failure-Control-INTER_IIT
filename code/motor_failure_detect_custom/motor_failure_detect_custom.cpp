/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "motor_failure_detect_custom.hpp"

MotorFailureDetect::MotorFailureDetect()
{
    _params.ekf2_mot_f = param_find("EKF2_MOT_F");

}

MotorFailureDetect::~MotorFailureDetect()
{
    // Empty destructor - no cleanup needed
}

int
MotorFailureDetect::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}

MotorFailureDetect *MotorFailureDetect::instantiate(int argc, char *argv[])
{
    MotorFailureDetect *instance = new MotorFailureDetect();

    if (instance == nullptr) {
        PX4_ERR("Failed to allocate MotorFailureDetect object");
    }

    return instance;
}

int MotorFailureDetect::task_spawn(int argc, char *argv[])
{
    int task_id = px4_task_spawn_cmd("motor_failure_detect_custom",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT,
                                    2048,
                                    (px4_main_t)&run_trampoline,
                                    (char *const *)argv);

    if (task_id < 0) {
        return -errno;
    }

    return 0;
}

// Get the flight mode name based on the navigation state
const char*
MotorFailureDetect::get_flight_mode_name(uint8_t nav_state, uint8_t &mode_index)
{
	mode_index = nav_state;
	switch (nav_state) {
		case 0: return "Manual";
		case 1: return "Altitude";
		case 2: return "Position";
		case 3: return "Mission";
		case 4: return "Hold";
		case 5: return "Return";
		case 10: return "Acro";
		case 15: return "Stabilized";
		case 14: return "Offboard";
		case 17: return "Takeoff";
		case 19: return "Follow Me";
		case 20: return "Land";
		default: return "Unknown";
	}
}

void
MotorFailureDetect::publish_motor_failure()
{
    // Log the start of the motor failure detection process
    PX4_INFO("Publishing motor failure detector!");

    // Find the EKF2_MOT_F parameter
    _params.ekf2_mot_f = param_find("EKF2_MOT_F");

    // Initialize the failure detector status structure
    failure_detector_status_s failsafe_detector = {};
    memset(&failsafe_detector, 0, sizeof(failsafe_detector));
    failsafe_detector.timestamp = hrt_absolute_time();
    failsafe_detector.fd_motor = true;

    // Retrieve the EKF2_MOT_F parameter value
    int32_t failed_motor = 0;
    if (param_get(_params.ekf2_mot_f, &failed_motor) == PX4_OK) {
        PX4_INFO("EKF2_MOT_F parameter is now: %d", failed_motor);

        // Set the motor failure mask based on the failed motor index
        switch (failed_motor) {
            case 1:
                failsafe_detector.motor_failure_mask = 0b00000001;
                break;
            case 2:
                failsafe_detector.motor_failure_mask = 0b00000010;
                break;
            case 3:
                failsafe_detector.motor_failure_mask = 0b00000100;
                break;
            case 4:
                failsafe_detector.motor_failure_mask = 0b00001000;
                break;
            default:
                PX4_ERR("NO MOTOR FAILED");
                return;
        }
    } else {
        PX4_ERR("Failed to get EKF2_MOT_F parameter");
        return;
    }

    // Advertise the failure detector status
    orb_advert_t failsafe_detector_pub = orb_advertise(ORB_ID(failure_detector_status), &failsafe_detector);
    if (failsafe_detector_pub == nullptr) {
        PX4_ERR("Failed to advertise failsafe detector topic!");
        return;
    }

    // Publish the failure detector status
    if (orb_publish(ORB_ID(failure_detector_status), failsafe_detector_pub, &failsafe_detector) != PX4_OK) {
        PX4_ERR("Failed to publish failsafe_detector message!");
    } else {
        PX4_INFO("Motor failure detector published successfully!");
    }
}

void
MotorFailureDetect:: enableLandMode() // Enable LAND mode
{
    // Initialize the vehicle_command topic
    struct vehicle_command_s cmd = {};
    cmd.timestamp = hrt_absolute_time();  // Get the current time
    cmd.param1 = 0;                       // No additional parameters required for LAND
    cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
    cmd.target_system = 1;                // Target the current vehicle system
    cmd.target_component = 1;             // Target the autopilot component
    cmd.source_system = 1;                // Source system ID (your module)
    cmd.source_component = 1;             // Source component ID (your module)
    cmd.from_external = false;            // Indicate this is not an external command

    // Advertise and publish the command
    orb_advert_t cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    if (cmd_pub != nullptr) {
        orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);
    }
}


void
MotorFailureDetect::print_esc_status()
{
	// Print ESC information for each ESC
	for (unsigned i = 0; i < _esc_status.esc_count; i++) {
		const esc_report_s &esc = _esc_status.esc[i];

		PX4_INFO("ESC %d:", i);
		PX4_INFO("  Temperature: %.1f°C", (double)esc.esc_temperature);
		PX4_INFO("  Voltage: %.2fV", (double)esc.esc_voltage);
		PX4_INFO("  Current: %.2fA", (double)esc.esc_current);
		PX4_INFO("  RPM: %d", esc.esc_rpm);
		PX4_INFO("  Status Flags: 0x%08x", esc.esc_state);
	}

	PX4_INFO("Update rate: %.1f Hz", (double)_update_rate);
}

int
MotorFailureDetect::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("ESC monitoring module that tracks ESC status");

	PRINT_MODULE_USAGE_NAME("motor_failure_detect_custom", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}


void
MotorFailureDetect::checkRYPError()
{
	vehicle_attitude_s attitude{};
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Update subscriptions
	_attitude_sub.update(&attitude);
	_attitude_setpoint_sub.update(&attitude_setpoint);

	vehicle_angular_velocity_s angular_velocity;

	if (vehicle_angular_velocity_sub.update(&angular_velocity)) {
		angular_velocity_z = angular_velocity.xyz[2];
	}

	// Convert quaternion to Euler angles for actual attitude
	const matrix::Quatf q(attitude.q);
	const matrix::Eulerf euler(q);

	// Convert quaternion to Euler angles for setpoint
	const matrix::Quatf q_sp(
		attitude_setpoint.q_d[0],
		attitude_setpoint.q_d[1],
		attitude_setpoint.q_d[2],
		attitude_setpoint.q_d[3]
	);
	const matrix::Eulerf euler_sp(q_sp);

	// Calculate errors (wrapped to [-pi, pi])
	float roll_error = matrix::wrap_pi(euler_sp.phi() - euler.phi());
	float pitch_error = matrix::wrap_pi(euler_sp.theta() - euler.theta());
	// float yaw_error = matrix::wrap_pi(euler_sp.psi() - euler.psi());


	// Check if errors exceed thresholds for roll, pitch, and yaw
	if (fabsf(roll_error) > ROLL_ERROR_THRESHOLD) {
		PX4_WARN("Roll failure detected!");
		roll_fail_flag = 1;
	}else roll_fail_flag = 0;

	if (fabsf(pitch_error) > PITCH_ERROR_THRESHOLD) {
		PX4_WARN("Pitch failure detected!");
		pitch_fail_flag = 1;
	}else pitch_fail_flag = 0;




	// Persistent motor failure flags
	static int motor_failure_flag = 0;

	// If a motor failure has already been detected, print it and return
	if (motor_failure_flag != 0|| esc_fail_flag) {
		switch (motor_failure_flag) {
		case 1:
			PX4_ERR("Motor 1 failure detected!");
			break;
		case 2:
			PX4_ERR("Motor 2 failure detected!");
			break;
		case 3:
			PX4_ERR("Motor 3 failure detected!");
			break;
		case 4:
			PX4_ERR("Motor 4 failure detected!");
			break;
		}
		// publish_motor_failure_flag();
		publish_motor_failure();
		return;
	}

	if (_failed_motor_index == -1) {

	if (roll_fail_flag || pitch_fail_flag) {
		// Motor failure detection cases
		if (roll_error < (-ROLL_ERROR_THRESHOLD/3) &&
			pitch_error > (PITCH_ERROR_THRESHOLD/3)) {
			PX4_ERR("Motor 1 failure detected!");
			motor_failure_flag = 1;
			_failed_motor_index=1;
			if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {
				int32_t ekf_value = 1;
				if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
					_ekf2_mot_f = true;  // Set local flag to prevent repeated parameter sets
					int32_t verify_value = 0;
					if (param_get(_params.ekf2_mot_f, &verify_value) == PX4_OK) {
						PX4_INFO("EKF2_MOT_F parameter is now: %d", verify_value);
					}
				}
                publish_motor_failure();
				}
		}
		else if (roll_error > (ROLL_ERROR_THRESHOLD/3) &&
			pitch_error < (-PITCH_ERROR_THRESHOLD/3)) {
			PX4_ERR("Motor 2 failure detected!");
			motor_failure_flag = 2;
			_failed_motor_index=2;
			if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {
				int32_t ekf_value = 2;
				if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
					_ekf2_mot_f = true;
					int32_t verify_value = 0;
					if (param_get(_params.ekf2_mot_f, &verify_value) == PX4_OK) {
						PX4_INFO("EKF2_MOT_F parameter is now: %d", verify_value);
					}
				}
                publish_motor_failure();
				}
		}
		else if (roll_error > (ROLL_ERROR_THRESHOLD/3) &&
			pitch_error > (PITCH_ERROR_THRESHOLD/3)) {
			PX4_ERR("Motor 3 failure detected!");
			motor_failure_flag = 3;
			_failed_motor_index=3;
			if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {
				int32_t ekf_value = 3;
				if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
					_ekf2_mot_f = true;
					int32_t verify_value = 0;
					if (param_get(_params.ekf2_mot_f, &verify_value) == PX4_OK) {
						PX4_INFO("EKF2_MOT_F parameter is now: %d", verify_value);
					}
				}
                publish_motor_failure();
				}
		}
		else if (roll_error < (-ROLL_ERROR_THRESHOLD/3) &&
			pitch_error < (-PITCH_ERROR_THRESHOLD/3)) {
			PX4_ERR("Motor 4 failure detected!");
			motor_failure_flag = 4;
			_failed_motor_index=4;
			if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {
				int32_t ekf_value = 4;
				if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
					_ekf2_mot_f = true;
					int32_t verify_value = 0;
					if (param_get(_params.ekf2_mot_f, &verify_value) == PX4_OK) {
						PX4_INFO("EKF2_MOT_F parameter is now: %d", verify_value);
					}
				}
                publish_motor_failure();
				}
		}
	}
	}
}

void MotorFailureDetect::check_esc_failure()
{
    // Check if four ESCs are available
    if (_esc_status.esc_count != 4) {
        PX4_INFO("4 Motors are not online");
        return;
    }

    // Get the current time
    int now = hrt_absolute_time();
    // Calculate the time difference since the last check
    const float dt = (now - _last_current_check_time) / 1e6f;  // Convert to seconds
    PX4_INFO("%d", now);

    // Check if the sudden change threshold is met
    if (dt > SUDDEN_CHANGE_THRESHOLD) {
        // Update current and failure values for each ESC
        for (unsigned i = 0; i < _esc_status.esc_count; i++) {
            _curr_currents(i) = _esc_status.esc[i].esc_current;
            _curr_esc_failure(i) = _esc_status.esc[i].failures;
        }

        // Update the current history and calculate the difference matrix
        updateCurrentHistory(_curr_currents);
        _current_diff_mat = _curr_currents - _previous_currents;

        // Store the current values as previous for the next iteration
        for (unsigned i = 0; i < _esc_status.esc_count; i++) {
            _previous_currents(i) = _esc_status.esc[i].esc_current;
        }

        // Update the last current check time
        _last_current_check_time = now;
    }

    // Update the average current matrix and compute the overall average
    updateAverageCurrentMatrix(_curr_currents, _avg_current_mat);
    _moving_avg_current = computeOverallAverage(_avg_current_mat);

    // Check if the moving average current is below the threshold
    if (static_cast<double>(_moving_avg_current) < 8.5) {
        // Check for ESC failure based on variance
        for (unsigned i = 0; i < 4; i++) {
            esc_fail_flag = static_cast<double>(_variance_matrix(i)) > 0.5;
        }
    } else {
        esc_fail_flag = 0;
    }

    // Calculate the variance matrix for further analysis
    calculateVarianceMatrix();

    // Check if a motor failure has already been detected
    if (_failed_motor_index != -1) {
        // Identify the failed motor based on current thresholds
        for (unsigned i = 0; i < _esc_status.esc_count; i++) {
            if (static_cast<double>(_esc_status.esc[i].esc_current) < 1.5 || static_cast<double>(_esc_status.esc[i].esc_current) >= 15.8) {
                _failed_motor_index = i + 1;
            }
        }
    }
}



void
MotorFailureDetect::updateAverageCurrentMatrix(matrix::Vector<float, 4>& curr_currents, matrix::Vector<float, 10>& avg_current_mat)
{
    // Step 1: Calculate the average of the current readings
    float average_current = 0.0f;
    for (int i = 0; i < 4; ++i) {
        average_current += curr_currents(i);
    }
    average_current /= 4.0f;

    // Step 2: Shift elements in _avg_current_mat to the left
    for (int i = 0; i < 9; ++i) {
        avg_current_mat(i) = avg_current_mat(i + 1);
    }
    // Insert the new average current at the end
    avg_current_mat(9) = average_current;
}


float
MotorFailureDetect::computeOverallAverage(const matrix::Vector<float, 10>& avg_current_mat)
{
    float sum = 0.0f;
    for (int i = 0; i < 10; ++i) {
        sum += avg_current_mat(i);
    }
    return sum / 10.0f;
}


void
MotorFailureDetect::updateCurrentHistory(matrix::Vector<float, 4>& curr_currents)
{
    // Shift elements in _current_history left for each motor
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 9; ++j) {
            _current_history(i, j) = _current_history(i, j + 1);
        }
        // Add the latest current reading to the end of the history for each motor
        _current_history(i, 9) = curr_currents(i);
    }
}


void
MotorFailureDetect::calculateVarianceMatrix()
{
    for (int i = 0; i < 4; ++i)
    {
	float mean = 0.0f;

	// Calculate mean for the current motor
	for (int j = 0; j < 10; ++j) {
	mean += _current_history(i, j);
	}
	mean /= 10.0f;

	// Calculate variance for the current motor
	float variance = 0.0f;
	for (int j = 0; j < 10; ++j) {
	float diff = _current_history(i, j) - mean;
	variance += diff * diff;  // Use multiplication instead of std::pow
	}
	variance /= 10.0f;

	_variance_matrix(i) = variance;
    }
}
void
MotorFailureDetect::check_motor_failure()
{
    actuator_motors_s actuator_motors{};
    if (_actuator_motors_sub.update(&actuator_motors)) {

        // Log the start of motor failure check
        PX4_ERR("#################################");

        // If motor failure flag is already set, publish the failure and return
        if (mot_f_flag) {
            publish_motor_failure();
            return;
        }

        // Check each motor pair for failure
        for (unsigned i = 0; i < 4; i += 2) {
            float first_value = actuator_motors.control[i];
            float second_value = actuator_motors.control[i + 1];
            float diff = first_value - second_value;

            // Check for failure condition where the difference is significant and negative
            if (fabsf(diff) > ACTUATOR_FAILURE_THRESHOLD && diff < 0) {
                PX4_WARN("Potential motor %d failure detected!", i + 2);

                // Set EKF2_MOT_F parameter if not already set
                if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {

                    int32_t ekf_value = i + 2;
                    if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
                        _ekf2_mot_f = true;
                        PX4_INFO("EKF2_MOT_F enabled due to motor %d failure", ekf_value);
                        mot_f_flag = 1;
                    }
                }
                publish_motor_failure();
                enableLandMode();
                break;
            }

            // Check for failure condition where the difference is significant and positive
            else if (fabsf(diff) > ACTUATOR_FAILURE_THRESHOLD && diff > 0) {
                PX4_WARN("Potential motor %d failure detected!", i + 1);

                // Set EKF2_MOT_F parameter if not already set
                if (_params.ekf2_mot_f != PARAM_INVALID && !_ekf2_mot_f) {
                    int32_t ekf_value = i + 1;
                    if (param_set(_params.ekf2_mot_f, &ekf_value) == PX4_OK) {
                        _ekf2_mot_f = true;
                        PX4_INFO("EKF2_MOT_F enabled due to motor %d failure", ekf_value);
                        mot_f_flag = 1;
                    }
                }
                publish_motor_failure();
                enableLandMode();
                break;
            }
        }
    }
}
void MotorFailureDetect::run()
{
    PX4_INFO("Motor failure detection module started");

    // Initialize timestamps
    _last_update_time = hrt_absolute_time();
    _last_current_check_time = hrt_absolute_time();

    // Subscribe to vehicle_status topic
    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    vehicle_status_s vehicle_status{};

    while (!should_exit()) {
        // Update ESC status
        if (_esc_status_sub.update(&_esc_status)) {
            _update_counter++;
            PX4_ERR("STRTED ---------------->");
            const uint64_t now = hrt_absolute_time();
            const float dt = (now - _last_update_time) / 1e6f;

            // Check for vehicle status updates
            if (orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status) == PX4_OK) {
                uint8_t nav_state = vehicle_status.nav_state;

                // Determine the current flight mode
                const char *mode_str = "UNKNOWN";

                switch (nav_state) {
                    case vehicle_status_s::NAVIGATION_STATE_MANUAL:
                        mode_str = "MANUAL";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
                        mode_str = "ALTCTL";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_POSCTL:
                        mode_str = "POSCTL";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
                        mode_str = "AUTO_MISSION";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
                        mode_str = "AUTO_LOITER";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
                        mode_str = "AUTO_RTL";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_ACRO:
                        mode_str = "ACRO";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_STAB:
                        mode_str = "STABILIZED";
                        break;
                    case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
                        mode_str = "OFFBOARD";
                        break;
                    default:
                        mode_str = "UNKNOWN";
                        break;
                }

                // Update flight mode index
                if (_vehicle_status_sub.update(&vehicle_status)) {
                    uint8_t flight_mode_index;
                    get_flight_mode_name(vehicle_status.nav_state, flight_mode_index);
                    _current_flight_mode_index = flight_mode_index;
                }

                // Check for specific flight modes and update thresholds
                if (_current_flight_mode_index == 17) {
                    PX4_INFO("Takeoff Mode");
                    checkRYPError();
                } else if (strcmp(mode_str, "MANUAL") == 0) {
                    PX4_INFO("Manual Mode");
                    ROLL_ERROR_THRESHOLD = 0.656f;
                    PITCH_ERROR_THRESHOLD = 0.656f;
                    YAW_ERROR_THRESHOLD = 0.05f;
                    checkRYPError();
                    check_motor_failure();
                } else if (strcmp(mode_str, "ACRO") == 0) {
                    PX4_INFO("ACRO Mode");
                    ROLL_ERROR_THRESHOLD = 0.768f;
                    PITCH_ERROR_THRESHOLD = 0.768f;
                    YAW_ERROR_THRESHOLD = 0.05f;
                    checkRYPError();
                    check_motor_failure();
                } else {
                    PX4_INFO("Other Mode");
                    ROLL_ERROR_THRESHOLD = 0.524f;
                    PITCH_ERROR_THRESHOLD = 0.524f;
                    YAW_ERROR_THRESHOLD = 0.05f;
                    check_esc_failure();
                    checkRYPError();
                    check_motor_failure();
                }
            }

            // Update the update rate every second
            if (dt > 1.0f) {
                _update_rate = _update_counter / dt;
                _update_counter = 0;
                _last_update_time = now;

                if (_failure_detected) {
                    PX4_INFO("FAILURE DETECTED !!!!!!!!!");
                }
            }
        }
        px4_usleep(UPDATE_INTERVAL); // Set update interval at 125 Hz
    }
    PX4_INFO("Motor Failure Detection Module Exiting");
}

int motor_failure_detect_custom_main(int argc, char *argv[])
{
    	return MotorFailureDetect::main(argc, argv);
}

