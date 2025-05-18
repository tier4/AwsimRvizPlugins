// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_

#include <memory>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "rclcpp/qos.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class EditableEnumProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace awsim_rviz_plugins
{
/** @brief Accumulates and displays the pose from a geometry_msgs::PoseStamped message. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC NpcSpawnerStatus : public
  rviz_common::Display
{
  Q_OBJECT

public:
  NpcSpawnerStatus();

  ~NpcSpawnerStatus() override;
  void onInitialize() override;
  void reset() override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateStatus();
  void updateTopic();

private:
  rviz_common::properties::EditableEnumProperty * shape_property_;
  rviz_common::properties::FloatProperty * shaft_length_property_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::QoS qos_profile;
  rviz_common::properties::RosTopicProperty * topic_property_name;
  rviz_common::properties::RosTopicProperty * topic_property_velo;
  rviz_common::properties::QosProfileProperty * qos_profile_property_name;
  rviz_common::properties::QosProfileProperty * qos_profile_property_velo;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_name;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velo;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
