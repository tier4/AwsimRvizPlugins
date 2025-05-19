// Copyright (c) 2008, Willow Garage, Inc.
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


#include "npc_spawner/npc_spawner_status.hpp"

#include <memory>

#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace awsim_rviz_plugins
{

NpcSpawnerStatus::NpcSpawnerStatus() : rviz_ros_node_(), qos_profile(5)
{
  topic_property_name = new rviz_common::properties::RosTopicProperty(
    "Topic (NPC Type)", "/awsim/awsim_rviz_plugin/npc_spawner/npc_name",
    "std_msgs::msg::String", "topic name", this, SLOT(updateNameTopic()));
  topic_property_velo = new rviz_common::properties::RosTopicProperty(
    "Topic (Velocity)", "/awsim/awsim_rviz_plugin/npc_spawner/npc_velocity",
    "std_msgs::msg::Float32", "topic name", this, SLOT(updateVelocityTopic()));

  qos_profile_property_name = new rviz_common::properties::QosProfileProperty(topic_property_name, qos_profile);
  qos_profile_property_velo = new rviz_common::properties::QosProfileProperty(topic_property_velo, qos_profile);

  shape_property_ = new rviz_common::properties::EditableEnumProperty(
    "NPC Type", "Hatchback", "Type of NPC which spawn to AWSIM.", this, SLOT(updateName()));
  shape_property_->addOption("Hatchback");
  shape_property_->addOption("SmallCar");
  shape_property_->addOption("Taxi");
  shape_property_->addOption("Truck_2t");
  shape_property_->addOption("Van");

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Velocity [km/h]", 1, "Velocity of NPC which spawn to AWSIM.",
    this, SLOT(updateVelocity()));
}

void NpcSpawnerStatus::onInitialize()
{
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  topic_property_name->initialize(rviz_ros_node_);
  topic_property_velo->initialize(rviz_ros_node_);
  updateNameTopic();
  updateVelocityTopic();
}

void NpcSpawnerStatus::reset()
{

}

NpcSpawnerStatus::~NpcSpawnerStatus() = default;

void NpcSpawnerStatus::onEnable()
{
  
}

void NpcSpawnerStatus::onDisable()
{
  
}

void NpcSpawnerStatus::updateName()
{
  std_msgs::msg::String name;
  name.data = shape_property_->getStdString();

  publisher_name->publish(name);
}

void NpcSpawnerStatus::updateVelocity()
{
  std_msgs::msg::Float32 velocity;
  velocity.data = shaft_length_property_->getFloat();

  publisher_velo->publish(velocity);
}

void NpcSpawnerStatus::updateNameTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_name = raw_node->template create_publisher<std_msgs::msg::String>(topic_property_name->getStdString(), qos_profile);
  clock_ = raw_node->get_clock();
}

void NpcSpawnerStatus::updateVelocityTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_velo = raw_node->template create_publisher<std_msgs::msg::Float32>(topic_property_velo->getStdString(), qos_profile);
  clock_ = raw_node->get_clock();
}
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(awsim_rviz_plugins::NpcSpawnerStatus, rviz_common::Display)
