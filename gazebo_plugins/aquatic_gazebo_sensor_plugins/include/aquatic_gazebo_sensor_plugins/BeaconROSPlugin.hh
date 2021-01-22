// Copyright (c) 2020 Ingeniarius, Lda Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __UUV_BEACON_ROS_PLUGIN_HH__
#define __UUV_BEACON_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <aquatic_gazebo_sensor_plugins/ROSBaseModelPlugin.hh>
#include "SensorBeacon.pb.h"
#include <aquatic_gazebo_sensor_plugins_msgs/Beacon.h>

namespace gazebo
{
  class BeaconROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: BeaconROSPlugin();

    /// \brief Class destructor
    public: ~BeaconROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    // NEW IMPLEMENTATIONS

    /// \brief Update callback Trigger
    protected: virtual void OnTrigger(const std_msgs::Bool::ConstPtr &_msg);

    /// \brief Run callback
    protected: void Run();

    /// \brief ROS Subscriber for transporting input messages
    protected: ros::Subscriber rosSensorInputSub;

    // SDF PARAMETERS to C++ VARIABLES

    /// \brief Emitting Wave Pressure Frequency
    protected: double operating_frequency;

    /// \brief Sound Pressure Level (SPL)
    protected: double spl;

    /// \brief Beacon Beam Pattern
    protected: double beam_pattern;

    /// \brief Beacon Mode (Passive or Active)
    protected: std::string operational_mode;

  };
}

#endif // __UUV_BEACON_ROS_PLUGIN_HH__
