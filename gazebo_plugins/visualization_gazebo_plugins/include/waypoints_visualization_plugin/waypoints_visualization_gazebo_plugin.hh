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
*/

#ifndef WAYPOINTS_VISUALIZATION_PLUGIN_HH_
#define WAYPOINTS_VISUALIZATION_PLUGIN_HH_

#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/scoring_plugin.hh"
#include "vrx_gazebo/waypoint_markers.hh"

#include <geometry_msgs/PoseArray.h> 

/// \brief A plugin for computing the score of the wayfinding navigation task.
/// This plugin derives from the generic ScoringPlugin class. 
///
/// This plugin provides gazebo visualization for a series of poses from a topic
///
/// This plugin requires the following SDF parameter:
/// <markers>: Optional parameter to enable visualization markers. Check the
/// WaypointMarkers class for SDF documentation.
class WaypointsVisualizationPlugin : public ScoringPlugin
{
  /// \brief Constructor.
  public: WaypointsVisualizationPlugin();

  // Documentation inherited.
  public: void Load(gazebo::physics::WorldPtr _world,
                    sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  // Documentation inherited.
  private: void OnReady() override;

  // Documentation inherited.
  private: void OnRunning() override;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to the sdf plugin element.
  private: sdf::ElementPtr sdf;

  /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// Subscribe to target poses
  private: ros::Subscriber targetSub;
  private: void OnTargetMsg(const geometry_msgs::PoseArray& msg);

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  private: gazebo::common::Timer timer;

  /// \brief Waypoint visualization markers.
  private: WaypointMarkers waypointMarkers;
};

#endif
