// Copyright (c) 2016 The UUV Simulator Authors.
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

#include <aquatic_gazebo_sensor_plugins/BeaconROSPlugin.hh>

namespace gazebo{

/////////////////////////////////////////////////
BeaconROSPlugin::BeaconROSPlugin() : ROSBaseModelPlugin(){

}
/////////////////////////////////////////////////
BeaconROSPlugin::~BeaconROSPlugin(){

}
/////////////////////////////////////////////////
void BeaconROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

  ROSBaseModelPlugin::Load(_model, _sdf);

  /* SDF PARAMETERS to C++ VARIABLES */
  GetSDFParam<double>(_sdf, "operating_frequency", this->operating_frequency, 10000.0);
  GetSDFParam<double>(_sdf, "spl", this->spl, 183.0);
  GetSDFParam<double>(_sdf, "beam_pattern", this->beam_pattern, 180.0);
  GetSDFParam<std::string>(_sdf, "operational_mode", this->operational_mode, "passive");

  /* ROS Input Topic THE TRIGGER */
  this->rosSensorInputSub = this->rosNode->subscribe<std_msgs::Bool>(
    "/rexrov2/beacon/trigger", 1,
      boost::bind(&BeaconROSPlugin::OnTrigger, this, _1));
        // this->robotNamespace + "/bluerov2/beacon/trigger" this->robotNamespace +
        // this->sensorOutputTopic = beacon ... this->robotNamespace = /bluerov2/bluerov2/trigger

  /* Gazebo output Msg */
  if (this->gazeboMsgEnabled){

    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<aquatic_gazebo_sensor_msgs::msgs::Beacon>("~/" +
        this->sensorOutputTopic);
  }else{
    /* ROS Output Topic -> ALTERNATIVE: FIX TO GAZEBO TOPICS */
    this->rosSensorOutputPub =
      this->rosNode->advertise<aquatic_gazebo_sensor_plugins_msgs::Beacon>(
        this->sensorOutputTopic, 1);
  }
}
/////////////////////////////////////////////////
bool BeaconROSPlugin::OnUpdate(const common::UpdateInfo& _info){

  /* Publish sensor state */
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  if (!this->operational_mode.compare("passive")) {

    /* Run @ Update rate [Hz] */
    Run();
  }

  /* Read the current simulation time */
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastMeasurementTime = this->world->SimTime();
  #else
    this->lastMeasurementTime = this->world->GetSimTime();
  #endif
    return true;
}
/////////////////////////////////////////////////
void BeaconROSPlugin::OnTrigger(const std_msgs::Bool::ConstPtr &_msg){

  /* Run @ Trigger */
  Run();

  //return(_msg->data);
}
/////////////////////////////////////////////////
void BeaconROSPlugin::Run(){

  /* Initial Sound Pressure Wave Time [s, ns] */
  common::Time initial_time;

  /* Read the current simulation time */
#if GAZEBO_MAJOR_VERSION >= 8
  initial_time = this->world->SimTime();
#else
  initial_time = this->world->GetSimTime();
#endif

  /* Using the world pose wrt Gazebo's ENU reference frame */
  ignition::math::Vector3d beacon_pos;

  /* Get Beacon Pose */
#if GAZEBO_MAJOR_VERSION >= 8
  beacon_pos = this->link->WorldPose().Pos();
#else
  beacon_pos = this->link->GetWorldPose().Ign().Pos();
#endif

  if (this->gazeboMsgEnabled){

    /* Gazebo Beacon message */
    aquatic_gazebo_sensor_msgs::msgs::Beacon gazeboMsg;

    /* Building Beacon Gazebo message */
    gazebo::msgs::Time* _time = new gazebo::msgs::Time();
    _time->set_sec(initial_time.sec);
    _time->set_nsec(initial_time.nsec);

    gazebo::msgs::Vector3d* pos = new gazebo::msgs::Vector3d();
    pos->set_x(beacon_pos.X());
    pos->set_y(beacon_pos.Y());
    pos->set_z(beacon_pos.Z());

    gazeboMsg.set_allocated_initial_time(_time);
    gazeboMsg.set_allocated_initial_pose(pos);
    gazeboMsg.set_operating_frequency(operating_frequency);
    gazeboMsg.set_spl(spl);
    gazeboMsg.set_beam_pattern(beam_pattern);

    /* Publish GAZEBO Beacon Message */
    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }else{

    /* Beacon ROS message - ALTERNATIVE */
    aquatic_gazebo_sensor_plugins_msgs::Beacon rosMsg;

    /* Get the Header */
    rosMsg.header.stamp.sec  = initial_time.sec;; //_info.simTime.sec; initial_time.sec;
    rosMsg.header.stamp.nsec = initial_time.nsec;; //_info.simTime.nsec; initial_time.nsec;
    rosMsg.header.frame_id = this->link->GetName();

    /* Building ROS Beacon message */

    /* TIME */
    //rosMsg.initial_time.sec = initial_time.sec;
    //rosMsg.initial_time.nsec = initial_time.nsec;
    // Pulse TEST
    if (this->beam_pattern == 180.0) {
      this->beam_pattern = 0.0;
    }else{
      this->beam_pattern = 180.0;
    }

    /* POSE */
    rosMsg.initial_pose.x = beacon_pos.X();
    rosMsg.initial_pose.y = beacon_pos.Y();
    rosMsg.initial_pose.z = beacon_pos.Z();

    /* BEACON PARAMETERS */
    rosMsg.operating_frequency = this->operating_frequency;
    rosMsg.SPL = this->spl;
    rosMsg.beam_pattern = this->beam_pattern;

    /* Publish ROS Beacon message */
    this->rosSensorOutputPub.publish(rosMsg);
  }
}
/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(BeaconROSPlugin)
}
