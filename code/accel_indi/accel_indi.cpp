/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

/**
 * @file accel_indi_main.c
 * Getting the desired acceleration in inertial (NED) frame
 *
 * @brief A PX4 module that subscribes to vehicle_local_position_setpoint
 * and publishes the desired input for Fault Tolerant Control

 */

#include "accel_indi.hpp"

AccelINDI::AccelINDI()
{
}

AccelINDI::~AccelINDI()
{
}
void AccelINDI::publish_pa_estimates()
{
    float_t a_des[3] = {_pos_setpoint.acceleration[0], _pos_setpoint.acceleration[1], _pos_setpoint.acceleration[2]};
    const float_t g_ = 9.81;
    matrix::Vector3f g_vect(0,0,-g_);
    matrix::Vector3f a_des_vec(a_des[0], a_des[1], a_des[2]);
    a_des_vec = a_des_vec - g_vect;

    PX4_INFO("a_des before filtering : [%.4f, %.4f, %.4f]",(double)a_des[0], (double)a_des[1], (double)a_des[2]);
    a_des_vec(0) = math::constrain(a_des[0], -4.0f, 4.0f);
    a_des_vec(1) = math::constrain(a_des[1], -4.0f, 4.0f);
    // a_des_vec(2) = math::constrain(a_des[2], g_- 4.0f, g_+ 4.0f);

    PX4_INFO("a_des : [%.4f, %.4f, %.4f]",(double)a_des_vec(0), (double)a_des_vec(1), (double)a_des_vec(2));

    _custom_pa_msg.n_b[0] = 0.0544988f;
    _custom_pa_msg.n_b[1] = -0.0838444f;
    _custom_pa_msg.n_b[2] = 0.994987f;


    matrix::Vector3f a_des_normalized = a_des_vec.normalized();

    _custom_pa_msg.n_des_i[0] = a_des_normalized(0);
    _custom_pa_msg.n_des_i[1] = a_des_normalized(1);
    _custom_pa_msg.n_des_i[2] = a_des_normalized(2);

    matrix::Quatf q(_attitude.q[0], _attitude.q[1], _attitude.q[2], _attitude.q[3]);
    // matrix::Vector3f z_body = -a_des_vec.normalized();

    // float theta = std::acos((-g_vect.dot(z_body)) / g_);
    // PX4_INFO("z_body : [%.4f, %.4f, %.4f]",(double)z_body(0), (double)z_body(1), (double)z_body(2));

    // const float theta_1 = 1; // 60 degrees in radians
    // float limited_theta = math::min(theta, theta_1);
    _custom_pa_msg.thrust_design = _pos_setpoint.thrust[2];
    // _custom_pa_msg.thrust_design = math::constrain(thrust_design / 0.90f, -1.0f, 1.0f);
    _custom_pa_msg.timestamp = hrt_absolute_time();
    matrix::Quatf q_inverse = q.inversed();
    // q.rotate(a_des_normalized);
    matrix::Dcmf dcm(q_inverse);
    matrix::Vector3f n_des_b_tf = dcm * a_des_normalized;
    // matrix::Quatf q_vector(0.0f, a_des_normalized(0), a_des_normalized(1), a_des_normalized(2));
    // matrix::Vector3f n_des_b_tf = q.inversed() * a_des_normalized;
    // matrix::Quatf q_rotated = q * q_vector * q_inverse;
    // matrix::Quatf q_rotated = q * q_vector;
    // matrix::Vector3f n_des_b_tf(q_rotated(1), q_rotated(2), q_rotated(3));
    _custom_pa_msg.n_des_b[0] = n_des_b_tf(0);
    _custom_pa_msg.n_des_b[1] = n_des_b_tf(1);
    _custom_pa_msg.n_des_b[2] = n_des_b_tf(2);
    // _custom_pa_msg.n_des_b[0] = a_des_normalized(0);
    // _custom_pa_msg.n_des_b[1] = a_des_normalized(1);
    // _custom_pa_msg.n_des_b[2] = a_des_normalized(2);

    _estimated_pa_pub.publish(_custom_pa_msg);

    // PX4_INFO("Published custom_topic: timestamp = %llu, n_b = [%.4f, %.4f, %.4f], n_des_i = [%.4f, %.4f, %.4f], n_des_b = [%.4f, %.4f, %.4f]",
    //          (unsigned long long)_custom_pa_msg.timestamp,
    //          (double)_custom_pa_msg.n_b[0], (double)_custom_pa_msg.n_b[1], (double)_custom_pa_msg.n_b[2],
    //          (double)_custom_pa_msg.n_des_i[0], (double)_custom_pa_msg.n_des_i[1], (double)_custom_pa_msg.n_des_i[2],
    //          (double)_custom_pa_msg.n_des_b[0], (double)_custom_pa_msg.n_des_b[1], (double)_custom_pa_msg.n_des_b[2]);

    PX4_INFO("n_b : [%.4f, %.4f, %.4f]",(double)_custom_pa_msg.n_b[0], (double)_custom_pa_msg.n_b[1], (double)_custom_pa_msg.n_b[2]);
    PX4_INFO("n_des_i : [%.4f, %.4f, %.4f]",(double)_custom_pa_msg.n_des_i[0], (double)_custom_pa_msg.n_des_i[1], (double)_custom_pa_msg.n_des_i[2]);
    PX4_INFO("n_des_b : [%.4f, %.4f, %.4f]",(double)_custom_pa_msg.n_des_b[0], (double)_custom_pa_msg.n_des_b[1], (double)_custom_pa_msg.n_des_b[2]);


}

void AccelINDI::print_acceleration_setpoint()
{
    PX4_INFO("Acceleration Setpoint (NED):\tX: %.4f m/s²\tY: %.4f m/s²\tZ: %.4f m/s²",
             (double)_pos_setpoint.acceleration[0],
             (double)_pos_setpoint.acceleration[1],
             (double)_pos_setpoint.acceleration[2]);
}

void AccelINDI::print_actuator_signals()
{
    if (_actuator_controls_sub.update(&_actuator_controls)) {
        PX4_INFO("Actuator Controls:");
        for (int i = 0; i < 8; i++) {
            PX4_INFO("Control[%d]: %f", i, (double)_actuator_controls.control[i]);
        }
    }
}

void AccelINDI::print_hover_thrust_estimate()
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
    // with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
    const float thrust_ned_z = _pos_setpoint.acceleration[2]*(_hover_thrust.hover_thrust/CONSTANTS_ONE_G) - _hover_thrust.hover_thrust;
    PX4_INFO("Hover Thrust Estimate:\tValue: %.4f\tVariance: %.4f",
             (double)_hover_thrust.hover_thrust,
             (double)_hover_thrust.hover_thrust_var);
    PX4_INFO("Calculated Thrust in Z(NED):\tValue: %.4f",
             (double)thrust_ned_z);
}

int AccelINDI::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION("Module to monitor and print the acceleration setpoint in the NED frame");

    PRINT_MODULE_USAGE_NAME("accel_indi", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");

    return 0;
}

int AccelINDI::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}

AccelINDI *AccelINDI::instantiate(int argc, char *argv[])
{
    AccelINDI *instance = new AccelINDI();

    if (instance == nullptr) {
        PX4_ERR("Failed to allocate AccelINDI object");
    }

    return instance;
}

int AccelINDI::task_spawn(int argc, char *argv[])
{
    int task_id = px4_task_spawn_cmd("accel_indi",
                                     SCHED_DEFAULT,
                                     SCHED_PRIORITY_DEFAULT,
                                     2048,
                                     (px4_main_t)&run_trampoline,
                                     (char *const *)argv);

    if (task_id < 0) {
        task_id = -1;
        return -errno;
    }

    return 0;
}

void AccelINDI::run()
{
    PX4_INFO("Acceleration INDI module started");

    _last_update_time = hrt_absolute_time();

    while (!should_exit()) {
        if (_pos_setpoint_sub.update(&_pos_setpoint) || _attitude_sub.update(&_attitude) ) {
            publish_pa_estimates();
        }
        // print_actuator_signals();
        px4_usleep(200);
    }

    PX4_INFO("Acceleration INDI module exiting");
}

int accel_indi_main(int argc, char *argv[])
{
    return AccelINDI::main(argc, argv);
}
