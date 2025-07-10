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
 * and prints the desired acceleration in the NED frame.
 *
 * @note Author: Rohan Jena <rohan.jena.eee22@itbhu.ac.in>
 */


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/estimatedpa.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <atomic>
#include <geo/geo.h>
#include <matrix/matrix/math.hpp>


extern "C" __EXPORT int accel_indi_main(int argc, char *argv[]);

class AccelINDI : public ModuleBase<AccelINDI>
{
public:
    AccelINDI();
    ~AccelINDI() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static AccelINDI *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase */
    void run() override;

private:
    bool should_exit() { return _task_should_exit.load(); }
    void set_should_exit() { _task_should_exit.store(true); }

    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _pos_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
    uORB::Subscription _hover_thrust_sub{ORB_ID(hover_thrust_estimate)};
    uORB::Publication<estimatedpa_s> _estimated_pa_pub{ORB_ID(estimatedpa)};
    uORB::Subscription _actuator_controls_sub{ORB_ID(actuator_motors)};
    estimatedpa_s _custom_pa_msg{};
    vehicle_attitude_s _attitude{};
    vehicle_local_position_setpoint_s _pos_setpoint{};
    hover_thrust_estimate_s _hover_thrust{};
    actuator_motors_s _actuator_controls{};
    uint64_t _last_update_time{0};
    uint32_t _update_counter{0};
    float _update_rate{0.0f};

    void print_acceleration_setpoint();
    void print_hover_thrust_estimate();
    void publish_pa_estimates();
    void print_actuator_signals();

    std::atomic<bool> _task_should_exit{false};
};
