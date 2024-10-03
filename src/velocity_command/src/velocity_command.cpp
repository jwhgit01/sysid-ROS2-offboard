/****************************************************************************
 *
 * Copyright 2024 Jeremy W. Hopwood. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Velocity commands from Offboard Mode
 * @file velocity_command.cpp
 * @author Jeremy W. Hopwood <jeremyhopwood@vt.edu>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/input_rc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Eigen>
#include <vector>
#include <string>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class VelocityCommand : public rclcpp::Node
{
public:
	VelocityCommand() : Node("velocity_command")
	{
		// Publishers
		_offboard_control_mode = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		_trajectory_setpoint = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

		// QOS (related to the subsribers?)
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// Subscribers
		_vehicle_attitude = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
		[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
			const float quat_array[4] = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
			const Eigen::Quaternionf quat(quat_array);
			_R_IB = quat.toRotationMatrix();
		});
		_input_rc = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos,
		[this](const px4_msgs::msg::InputRc::UniquePtr msg) {
			_mod_1 = getSwitchState(msg->values[10]);
        	_mod_2 = getSwitchState(msg->values[11]);
		});

		auto timer_callback = [this]() -> void {

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publishOffboardControlMode();
			publishTrajectorySetpoint();

		};

		// Publish at a 10Hz rate
		_timer = this->create_wall_timer(100ms, timer_callback);

		// Debug mode
		this->get_logger().set_level(rclcpp::Logger::Level::Debug);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr _timer;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr _trajectory_setpoint;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr _vehicle_attitude;
	rclcpp::Subscription<InputRc>::SharedPtr _input_rc;

	// Attitude rotation matrix
	Eigen::Matrix3f _R_IB;

	// RC switch states
 	uint8_t _mod_1;
  	uint8_t _mod_2;

  	// Lookup table for body velocity reference
	const float V = 10.0;
	const Eigen::Vector3f lookupTable[3][3] = {
		//               Mod 1 = 0,                 Mod 1 = 1,                 Mod 1 = 2 
		{Eigen::Vector3f(-V,-V,-V), Eigen::Vector3f(+V,-V,-V), Eigen::Vector3f(-V,+V,-V)},  // Mod 2 = 0
		{Eigen::Vector3f(+V,+V,-V), Eigen::Vector3f(-V,-V,0.0),Eigen::Vector3f(+V,-V,0.0)}, // Mod 2 = 1
		{Eigen::Vector3f(-V,+V,0.0),Eigen::Vector3f(+V,+V,0.0),Eigen::Vector3f(0.0,0.0,0.0)}// Mod 2 = 2
	};

	void publishTrajectorySetpoint()
	{
		// Initialize setpoint
		TrajectorySetpoint msg{};

		// Convert body velocity commands to the NED frame
		const Eigen::Vector3f velocity_body_m_s = lookupTable[_mod_1][_mod_2];
		RCLCPP_DEBUG_STREAM(this->get_logger(), "velocity_body_m_s:\n" << velocity_body_m_s);
    	const Eigen::Vector3f velocity_ned_m_s = _R_IB*velocity_body_m_s;

    	// Populate TrajectorySetpoint message
		msg.velocity = {velocity_ned_m_s[0], velocity_ned_m_s[1], velocity_ned_m_s[2]};
		msg.yawspeed = 0.0;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		// Publish
		_trajectory_setpoint->publish(msg);
	}

	void publishOffboardControlMode()
	{
		OffboardControlMode msg{};
		msg.position = false;
		msg.velocity = true;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		_offboard_control_mode->publish(msg);
	}

	// Function to read RC switch state
	uint8_t getSwitchState(uint16_t value) {
		if (value > 1666) {
			return 2;
		} else if (value > 1333) {
			return 1;
		} else {
			return 0;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting velocity_command node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelocityCommand>());
	rclcpp::shutdown();
	return 0;
}
