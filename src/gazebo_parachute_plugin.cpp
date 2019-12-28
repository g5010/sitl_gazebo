/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 /**
  * @brief Parachute Plugin
  *
  * This plugin simulates parachute deployment
  *
  * @author Jaeyoung Lim <jaeyoung@auterion.com>
  */

#include "gazebo_parachute_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(ParachutePlugin)

ParachutePlugin::ParachutePlugin() : ModelPlugin()
{
}

ParachutePlugin::~ParachutePlugin()
{
  _updateConnection->~Connection();
}

void ParachutePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_parachute_plugin] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", trigger_sub_topic_, trigger_sub_topic_);


  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ParachutePlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &ParachutePlugin::VelocityCallback, this);
}

void ParachutePlugin::OnUpdate(const common::UpdateInfo&){

    physics::ModelPtr parachute_model = world_->ModelByName("parachute_small");

    if(!attach_parachute_ && parachute_model){

      const ignition::math::Pose3d uavPose = model_->WorldPose();
      parachute_model->SetWorldPose(ignition::math::Pose3d(uavPose.Pos().X(), uavPose.Pos().Y(), uavPose.Pos().Z()+0.3, 0, 0, 0));        // or use uavPose.ros.GetYaw() ?

      gazebo::physics::JointPtr parachute_joint = world_->Physics()->CreateJoint("fixed", model_);
      parachute_joint->SetName("parachute_joint");
      
      // Attach parachute to base_link
      gazebo::physics::LinkPtr base_link = model_->GetLink("base_link");
      gazebo::physics::LinkPtr parachute_link = parachute_model->GetLink("chute");
      parachute_joint->Attach(base_link, parachute_link);

      // load the joint, and set up its anchor point
      parachute_joint->Load(base_link, parachute_link, ignition::math::Pose3d(0, 0, 0.3, 0, 0, 0));

      attach_parachute_ = true;
    }
}

void ParachutePlugin::TriggerCallback(const boost::shared_ptr<const msgs::Int> &_msg){
  gzwarn << "Parachute trigger callback: " << _msg->data() << "\n";
  
  LoadParachute();
}

void ParachutePlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void ParachutePlugin::LoadParachute(){
  // Don't create duplicate paracutes
  if(physics::ModelPtr parachute_model = world_->ModelByName("parachute_small")) return;
  // Insert parachute model
  world_->InsertModelFile("model://parachute_small");

  msgs::Int request;
  request.set_data(0);
  
}
} // namespace gazebo