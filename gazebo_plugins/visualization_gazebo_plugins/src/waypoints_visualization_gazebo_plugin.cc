/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * 

*/

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <cmath>
#include <gazebo/common/Console.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Model.hh>

#include "vrx_gazebo/waypoint_markers.hh"
#include "waypoints_visualization_plugin/waypoints_visualization_gazebo_plugin.hh"

/////////////////////////////////////////////////
WaypointsVisualizationPlugin::WaypointsVisualizationPlugin()
  : waypointMarkers("waypoint_marker")
{
  gzmsg << "Wayfinding scoring plugin loaded" << std::endl;
  this->timer.Stop();
  this->timer.Reset();
}

void WaypointsVisualizationPlugin::OnTargetMsg(const geometry_msgs::PoseArray& msg){
  for (int i = 0; i < msg.poses.size(); i++){
    this->waypointMarkers.DrawMarker(i, msg.poses[i].position.x, msg.poses[i].position.y, 1, std::to_string(i));
  }
}

/////////////////////////////////////////////////
void WaypointsVisualizationPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);
  this->sdf = _sdf;

  // Setup ROS node and publishers, subscribers
  this->rosNode.reset(new ros::NodeHandle());

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&WaypointsVisualizationPlugin::Update, this));

  //define waypoints_topic 
  const std::string waypoints_topic = _sdf->Get<std::string>("waypoints_topic");
  this->targetSub = this->rosNode->subscribe(waypoints_topic, 1000,
                                          &WaypointsVisualizationPlugin::OnTargetMsg, this);

  this->waypointMarkers.Load(_sdf->GetElement("markers"));

}

//////////////////////////////////////////////////
void WaypointsVisualizationPlugin::Update()
{
  // The vehicle might not be ready yet, let's try to get it.
  if (!this->vehicleModel)
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      this->vehicleModel = this->world->ModelByName(this->vehicleName);
    #else
      this->vehicleModel = this->world->GetModel(this->vehicleName);
    #endif
    if (!this->vehicleModel)
      return;
  }

  #if GAZEBO_MAJOR_VERSION >= 8
    const auto robotPose = this->vehicleModel->WorldPose();
  #else
    const auto robotPose = this->vehicleModel->GetWorldPose().Ign();
  #endif

  // Publish at 1 Hz.
  if (this->timer.GetElapsed() >= gazebo::common::Time(1.0))
  {
    this->timer.Reset();
    this->timer.Start();
  }

}

//////////////////////////////////////////////////
void WaypointsVisualizationPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
}


//////////////////////////////////////////////////
void WaypointsVisualizationPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
  this->timer.Start();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(WaypointsVisualizationPlugin)
