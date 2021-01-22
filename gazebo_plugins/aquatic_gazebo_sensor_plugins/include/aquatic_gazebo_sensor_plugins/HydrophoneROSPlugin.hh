// Copyright (c) 2020 Ingeniarius Authors.
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

#ifndef __UUV_HYDROPHONE_ROS_PLUGIN_HH__
#define __UUV_HYDROPHONE_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <aquatic_gazebo_sensor_plugins/ROSBaseModelPlugin.hh>
#include "SensorHydrophone.pb.h"
#include <aquatic_gazebo_sensor_plugins_msgs/Hydrophone.h>
#include <aquatic_gazebo_sensor_plugins_msgs/Beacon.h>
#include <std_msgs/Float64.h>

#define PI 3.14159265

namespace gazebo
{
  class HydrophoneROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: HydrophoneROSPlugin();

    /// \brief Class destructor
    public: ~HydrophoneROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// UNDER CONSTRUCTION

    /// \brief ROS Subscriber for transporting input messages
    protected: ros::Subscriber rosSensorInputSub;

    /// \brief ROS publisher for transporting measurement messages.
    protected: ros::Publisher SensorLevelOutputPub;

    /// \brief Gazebo's subscriber for transporting measurement messages
    protected: transport::SubscriberPtr gazeboSensorInputSub;

    /// \brief Update callback from simulator.
    protected: virtual void InputCallBack(const aquatic_gazebo_sensor_plugins_msgs::Beacon::ConstPtr &_msg);

    // SDF PARAMETERS to C++ VARIABLES

    /// \brief Acoustic Source
    protected: std::string beacon_parent;

    /// \brief Sensor sensitivity (re 1V/uPa)
    protected: double sensitivity;

    /// \brief Pre-amplificator Gain
    protected: double pre_ampGain;

    /// \brief Hydrophone Cut Off Frequency
    protected: double cut_off_frequency;

    /// \brief Acoustic Source
    protected: std::string sound_speed_model;

    /// \brief Geometrical Spreading Loss Model
    protected: std::string spreading_model;

    /// \brief Geometrical Spreading Loss Model
    protected: std::string operational_mode;

    /// \brief Pointer to the Beacon model (Acoustic Source). TESTING
    protected: physics::ModelPtr beacon_model;

    /// \brief Pointer to the Beacon link (Acoustic Source). TESTING
    protected: physics::LinkPtr beacon_link;


    // OUTPUT VARIABLES

    /// \brief Sound Wave Initial Time [ms]
    protected: common::Time initial_time;

    /// \brief Sound Wave Path time [ms]
    protected: double TOF;

    /// \brief Standard Deviation for Time
    protected: double stdDev;

    /// \brief Impulse Signal
    protected: bool pulse;

    /// \brief Periods Impulse Wave [n]
    protected: int wave_period;

    /// \brief Receiver Level [dB]
    protected: double RL;

    /// \brief Propagation Loss [dB]
    protected: double TF;

    /// \brief Noise Loss [dB]
    protected: double NL;

    /// \brief 7.3 Ocean Turbulence  [dB] re: 1 uPa, 1Hz, f in Hz
    protected: double NL_turbulence[2];

    /// \brief 7.4 Shipping Noise 10Hz to 10kHz [Sadowski et al] Table 7.1 - SIPPING LEVELS 1-7
    protected: double NL_shipping[2];

    /// \brief 7.5 Wave Noise mid-hundreds Hz to 50kHz [Sadowski et al] Table 7.2 - SEASTATES LEVELS 0-6
    protected: double NL_wave[2];

    /// \brief 7.7 Rain Noise max at 1kHz but goes to 100kHz [Sadowski et al] Table 7.3 - L, M, H, EH
    protected: double NL_rain[2];

    /// \brief 7.6 Thermal Noise greater than 50kHz [Mellan]
    protected: double NL_thermal[2];


    // FUNCTIONS

    /// \brief Compute Sound Speed for a give T, S, pH and Depth (Z)
    protected: double SoundSpeed(std::string sound_speed_model, double T, double S, double Z);

    /// \brief Compute Absorption Loss Coeficient for a give T, S, and Depth (Z)
    protected: double Absorption(int f, double c, double T, double S, double pH, double Z);

    /// \brief Compute Ray Range per Layer [m]
    protected: double RayRangeByLayer(double zs, double zr, double z1, double z2, double theta);

    /// \brief Compute Transmission/Propagation or absortion Loss [dB]
    protected: double PropagationLoss(std::string spreading_model, double R, double H, double alpha_R);

    /// \brief Compute Path Time [s]
    protected: double PathTime(double R, double c);

    /// \brief Compute Receiver Level [dB]
    protected: double ReceiverLevel(double SL, double TL);

    /// \brief Compute Noise [dB]
    protected: double Noise();

    /// \brief Compute Signal-to-Noise-Ratio [dB]
    protected: double SignalToNoiseRatio(double SL, double TL, double NL, double DI);

    /// \brief Compute Pressure Level [uPa]
    protected: double PressureLevel(double RL);

    /// \brief Compute Signal Level [V]
    protected: double SignalLevel(double SPL, double Gain, double Sensitivity);

    /// \brief Compute Signal Level Output [ dB, uPa or V]
    protected: double LevelConversion(double A);

    /// \brief Compute Continuos Wave (CW)
    protected: double ContinuosWave(double A, double w, double t);

  };
}

#endif // __UUV_HYDROPHONE_ROS_PLUGIN_HH__
