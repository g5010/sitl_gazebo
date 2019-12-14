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

#ifndef _GAZEBO_PARACHUTE_PLUGIN_HH_
#define _GAZEBO_PARACHUTE_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Odometry.pb.h>

namespace gazebo
{

class GAZEBO_VISIBLE ParachutePlugin : public ModelPlugin
{
public:
  ParachutePlugin();
  virtual ~ParachutePlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  void getSdfParams(sdf::ElementPtr sdf);

private:
  void LoadParachute();
  void TriggerCallback(const boost::shared_ptr<const msgs::Int> &_msg);


  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr _updateConnection;

  bool attach_parachute_ = false;

  const std::string trigger_sub_topic_ = "~/video_stream";

  transport::NodePtr node_handle_;
  transport::PublisherPtr  cmd_kill_pub_;
  transport::SubscriberPtr trigger_sub_;

};     // class GAZEBO_VISIBLE ParachutePlugin
}      // namespace gazebo
#endif // _GAZEBO_PARACHUTE_PLUGIN_HH_
