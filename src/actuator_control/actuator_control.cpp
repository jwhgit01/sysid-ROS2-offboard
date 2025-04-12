/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
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
 * @brief Direct actuator control using offboard flight mode
 * @file actuator_control.cpp
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <map>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("actuator_control")
	{
		// Create publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 1);
		actuator_servos_publisher_ = this->create_publisher<ActuatorServos>("/fmu/in/actuator_servos", 1);
		actuator_motors_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 1);

		// Create subscribers
		rmw_qos_profile_t qos_profile_rc = rmw_qos_profile_sensor_data;
		auto qos_rc = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_rc.history, 1), qos_profile_rc);
		input_rc_subscriber_ = this->create_subscription<px4_msgs::msg::InputRc>("/fmu/out/input_rc", qos_rc,
			[this](const px4_msgs::msg::InputRc::UniquePtr msg) {
				PTI_PWM = msg->values[7];
				amp = 1.0*(msg->values[8]-988)/1025.0; // [988,2013] --> [0, 1]
				prop_amp = 0.5*(msg->values[11]-988)/1025.0; // [988,2013] --> [0, 1]
			});
		rmw_qos_profile_t qos_profile_mcs = rmw_qos_profile_sensor_data;
		auto qos_mcs = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_mcs.history, 1), qos_profile_mcs);
		manual_control_setpoint_subscriber_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>("/fmu/out/manual_control_setpoint", qos_mcs,
			[this](const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
				da = msg->roll;
				de = msg->pitch;
				dr = msg->yaw;
				dt = msg->throttle;
			});
		
		// Load CSV
		load_data();
		if (InputSignal.count(1)<1) {
			RCLCPP_ERROR(get_logger(), "Error loading CSV: %s", ms_file.c_str());
		} else {
			RCLCPP_INFO(get_logger(), "Successfully loaded CSV: %s", ms_file.c_str());
		}

		auto timer_callback = [this]() -> void {

			// Publish messages
			// Note: offboard_control_mode needs to be paired with publish_actuators
			publish_offboard_control_mode();
			publish_actuators();
		};
		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	// Declare publishers and subscribers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<ActuatorServos>::SharedPtr actuator_servos_publisher_;
	rclcpp::Publisher<ActuatorMotors>::SharedPtr actuator_motors_publisher_;
	rclcpp::Subscription<InputRc>::SharedPtr input_rc_subscriber_;
	rclcpp::Subscription<ManualControlSetpoint>::SharedPtr manual_control_setpoint_subscriber_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	// CSV info and map
	const std::string ms_file = "/home/uav/src/sysid-ROS2-offboard/src/signals/ms_aeroprop_T30_f005-075-2_100hz.csv";
	const int T_ms = 30;
	const int fs = 100; // Hz
	std::map<int,std::vector<float>> InputSignal;
	uint64_t t0 = 0;
	
	// Amplitude knob andd PTI logic
	double amp = 0.0;
	double prop_amp = 0.0;
	uint16_t PTI_PWM;
	bool PTI = false;

	// Stick positions
	double da, de, dr, dt;

	void publish_offboard_control_mode();
	void publish_actuators();

	void load_data();
};

/**
 * @brief Publish the offboard control mode.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.thrust_and_torque = false;
	msg.direct_actuator = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish actuator controls
 */
void OffboardControl::publish_actuators()
{
	// Create messages
	ActuatorServos msg_servos{};
	ActuatorMotors msg_motors{};

	// Populate with manual control inputs (ASW-17)
	msg_servos.control[0] = -da;
	msg_servos.control[1] = -da;
	msg_servos.control[2] = -de;
	msg_servos.control[3] = dr;
	msg_servos.control[4] = -1.0; // flaps
	msg_servos.control[5] = -1.0; // flaps
	msg_motors.control[0] = 0.5*(dt + 0.99); // map stick [-1,1] to [0,1)

	// If we are not in PTI mode, and the PTI switch was engaged, get the initial time.
	if (!PTI && PTI_PWM > 1500) {
		RCLCPP_INFO(get_logger(), "PTI On");
		PTI =  true;
		t0 = this->get_clock()->now().nanoseconds() / 1000; // microseconds
	}
	
	// If we are in PTI mode, send excited actuator controls
	if (PTI) { 

		// Compute the time index
		uint64_t t1 = this->get_clock()->now().nanoseconds() / 1000; // microseconds
		int time_idx = (int)( (t1-t0)/(1000000/fs) ) % (T_ms*fs); // TODO: fix this

		// Get the input excitation vector from the map
		std::vector<float> input = InputSignal[time_idx];
		
		// Add excitation to the manual control inputs
		msg_servos.control[0] += amp*input[0];
		msg_servos.control[1] += amp*input[1];
		msg_servos.control[2] += amp*input[2];
		msg_servos.control[3] += amp*input[3];
		msg_servos.control[4] += amp*input[4];
		msg_servos.control[5] += amp*input[5];
		msg_motors.control[0] += prop_amp*input[6];

		// If the PTI switch has been set to LOW, set exit from PTI mode
		if ( PTI_PWM <= 1500 ) {
			RCLCPP_INFO(get_logger(), "PTI Off");
			PTI = false;
		}
	}

	// Set the timestamp and publish
	uint64_t t = this->get_clock()->now().nanoseconds() / 1000;
	msg_servos.timestamp = t;
	msg_servos.timestamp = t;
	actuator_servos_publisher_->publish(msg_servos);
	actuator_motors_publisher_->publish(msg_motors);
}

void OffboardControl::load_data()
{
	// Initialize and open the CSV file
	std::ifstream indata;
	indata.open(ms_file);
	std::string line;

	// Loop through each line (i.e. time index)
	int tidx = 0;
	while (getline(indata, line)) {
		// initialization for this line
		std::vector<float> values;
		std::stringstream lineStream(line);
		std::string cell;

		// Get the first "cell" as the time index
		std::getline(lineStream, cell, ',');
		tidx = stoi(cell);
		
		// The rest of the line is the vector of values
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(stof(cell));
		}
		
		// Assign the vector of values to the integer time key
		InputSignal[tidx] = values;
	}
}


int main(int argc, char *argv[])
{
	std::cout << "Starting actuator control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

