///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//               2020, Nathan Tsoi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// NOTE: The contents of this file have been taken largely from the ros_control wiki tutorials

#pragma once


// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

// NaN
#include <limits>

// ostringstream
#include <sstream>

const char* WHEEL_NAMES[] = {"front_left", "front_right", "rear_left", "rear_right"};

template <unsigned int NUM_JOINTS = 4>
class DifferentialDriveSimController : public hardware_interface::RobotHW
{
public:
  DifferentialDriveSimController()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &DifferentialDriveSimController::start_callback, this))
  , stop_srv_(nh_.advertiseService("stop", &DifferentialDriveSimController::stop_callback, this))
  {
    // Intialize raw data
    std::fill_n(pos_, NUM_JOINTS, 0.0);
    std::fill_n(vel_, NUM_JOINTS, 0.0);
    std::fill_n(eff_, NUM_JOINTS, 0.0);
    std::fill_n(cmd_, NUM_JOINTS, 0.0);

    // Connect and register the joint state and velocity interface
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
      std::ostringstream os;
      os << WHEEL_NAMES[i] << "_wheel";

      hardware_interface::JointStateHandle state_handle(os.str(), &pos_[i], &vel_[i], &eff_[i]);
      jnt_state_interface_.registerHandle(state_handle);

      hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle(os.str()), &cmd_[i]);
      jnt_vel_interface_.registerHandle(vel_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_vel_interface_);

    left_cmd = nh_.advertise<std_msgs::Float64>("wheel_left_joint_cmd", 10);
    right_cmd = nh_.advertise<std_msgs::Float64>("wheel_right_joint_cmd", 10);

    left_pos = nh_.subscribe("wheel_left_joint_pos", 10, &DifferentialDriveSimController::pos_left_callback, this);
    right_pos = nh_.subscribe("wheel_right_joint_pos", 10, &DifferentialDriveSimController::pos_right_callback, this);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.1);}

  void pos_left_callback(const std_msgs::Float64::ConstPtr& msg) {
    pos_[0] = msg->data;
  }

  void pos_right_callback(const std_msgs::Float64::ConstPtr& msg) {
    pos_[1] = msg->data;
  }

  void read()
  {
    // Read the joint state of the robot into the hardware interface
    if (running_)
    {
      for (unsigned int i = 0; i < NUM_JOINTS; ++i)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        //pos_[i] += vel_[i]*getPeriod().toSec(); // update position
        vel_[i] = cmd_[i]; // might add smoothing here later
      }
    }
    else
    {
      std::fill_n(pos_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
      std::fill_n(vel_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
    }
  }

  void write() {
    //ROS_WARN_STREAM("write: left: " << cmd_[0] << ", right: " << cmd_[1] << std::endl);
    std_msgs::Float64 left;
    left.data = cmd_[0];
    left_cmd.publish(left);
    std_msgs::Float64 right;
    right.data = cmd_[1];
    right_cmd.publish(right);
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[NUM_JOINTS];
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;

  ros::Publisher left_cmd;
  ros::Publisher right_cmd;

  ros::Subscriber left_pos;
  ros::Subscriber right_pos;
};
