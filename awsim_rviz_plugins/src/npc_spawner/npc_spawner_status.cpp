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
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "/awsim/awsim_rviz_plugin/npc_spawner/status",
    "autoware_vehicle_msgs/msg/VelocityReport", "topic name", this, SLOT(updateTopic()));

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(topic_property_, qos_profile);

  shape_property_ = new rviz_common::properties::EditableEnumProperty(
    "NPC Type", "Hatchback", "Type of NPC which spawn to AWSIM.", this, SLOT(updateStatus()));
  shape_property_->addOption("Hatchback");
  shape_property_->addOption("SmallCar");
  shape_property_->addOption("Taxi");
  shape_property_->addOption("Truck_2t");
  shape_property_->addOption("Van");

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Velocity", 1, "Velocity of NPC which spawn to AWSIM.",
    this, SLOT(updateStatus()));
}

void NpcSpawnerStatus::onInitialize()
{
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  topic_property_->initialize(rviz_ros_node_);
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

void NpcSpawnerStatus::updateTopic()
{

}

void NpcSpawnerStatus::updateStatus()
{

}
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(awsim_rviz_plugins::NpcSpawnerStatus, rviz_common::Display)
