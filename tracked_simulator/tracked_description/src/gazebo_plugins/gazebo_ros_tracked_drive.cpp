/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

/*
    Youge Su(yougesu@163.com)
    add ros->gazebo twist command transport
*/

#include <algorithm>
#include <assert.h>
#include <iostream>
#include <math.h>
#include <gazebo_plugins/gazebo_ros_tracked_drive.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include <ignition/math/Vector3.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

#include <ros/ros.h>

namespace gazebo
{

GazeboRosTrackedDrive::GazeboRosTrackedDrive() {}

// Destructor
GazeboRosTrackedDrive::~GazeboRosTrackedDrive() 
{
	FiniChild();
}

// Load the controller
void GazeboRosTrackedDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TrackedDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    this->node_ = transport::NodePtr(new transport::Node());
    this->world_ = this->parent->GetWorld()->Name();
    this->node_->Init(world_);

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( pub_topic_, "pubTopic", "/cmd_vel" );
    pub_topic_ = "/gazebo/" + this->world_ + "/" + this->parent->GetName() + pub_topic_;

    gazebo_ros_->getParameterBoolean ( legacy_mode_, "legacyMode", true );
    twistPub_ = node_->Advertise<msgs::Twist>(pub_topic_);
    
    if (!_sdf->HasElement("legacyMode"))
    {
      ROS_ERROR_NAMED("diff_drive", "GazeboRosTrackedDrive Plugin missing <legacyMode>, defaults to true\n"
	       "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
	       "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
	       "To fix an old package you have to exchange left wheel by the right wheel.\n"
	       "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
	       "just set <legacyMode> to true.\n"
      );
    }

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );

    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;

    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent->GetWorld()->SimTime();
    #else
        last_update_time_ = parent->GetWorld()->GetSimTime();
    #endif

    x_ = 0;
    rot_ = 0;
    alive_ = true;
    
    // subscribe ROS node sent command
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosTrackedDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("tracked_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosTrackedDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosTrackedDrive::UpdateChild, this ) );

    twistPub_->WaitForConnection();
}

void GazeboRosTrackedDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  x_ = 0;
  rot_ = 0;
}

// Update the controller
void GazeboRosTrackedDrive::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        setWheelVelocities();
        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void GazeboRosTrackedDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosTrackedDrive::setWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    double vr = x_;
    double va = -1.0 * rot_;
    msgs::Twist msg;
    msgs::Set(msg.mutable_linear(), ignition::math::Vector3d(vr, 0, 0));
    msgs::Set(msg.mutable_angular(), ignition::math::Vector3d(0, 0, va));
    twistPub_->Publish(msg, true);
}

void GazeboRosTrackedDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosTrackedDrive::QueueThread()
{
    static const double timeout = 0.01;
    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosTrackedDrive )
}
