//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ais_gazebo_plugins/gazebo_ros_presence.h>

/*  modified by mfernandezcarmona@lincoln.ac.uk
 * a simple presence detector based on gazebo_ros_sonar by aforecopyrighted
 * */


namespace gazebo {

GazeboRosPresence::GazeboRosPresence()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPresence::~GazeboRosPresence()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosPresence::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosPresence requires a Ray Sensor as its parent");
    return;
  }
  
  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);

  // default parameters
  namespace_.clear();
  topic_ = "sonar";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();


  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  //Publish to iot_update
  this->iot_pub_ = node_handle_->advertise<diagnostic_msgs::KeyValue>(
    std::string("/openhab_set"), 1);
  
  this->update_status_msg_.key = std::string(this->topic_);

  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosPresence::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void GazeboRosPresence::Reset()
{
  updateTimer.Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPresence::Update()
{
  bool doPublish=false;
  double newRange;
  int num_ranges;
  std::vector<double> newRanges;
  
  
  common::Time now_time = world->GetSimTime();
  
  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  //int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();

  //do we have previous history?
  if (this->storedRanges.size()==0){
	  this->event_time = world->GetSimTime(); 
	  this->update_status_msg_.value = std::string("CLOSED");
      this->iot_pub_.publish(this->update_status_msg_);
	  sensor_->GetRanges(this->storedRanges);
	  
      //ROS_INFO("Init sending");
  	
	return; 
  } 	  
  
  // not first time: lets track changes and send a yes to the world
  sensor_->GetRanges(newRanges);
  num_ranges=newRanges.size();	
  int diffCount=0;
  for(int i = 0; i < num_ranges; ++i) {
		if ((newRanges[i]!=storedRanges[i])&&(!doPublish)) {
			if (abs(newRanges[i]-storedRanges[i])>0.01){
				doPublish=true;
				this->update_status_msg_.value = std::string("OPEN");
				this->event_time = world->GetSimTime();  
				this->iot_pub_.publish(this->update_status_msg_);
				//ROS_INFO("Change detected %1.2f!=%1.2f, sending",newRanges[i],storedRanges[i]);
			}
		}
  }
  storedRanges=newRanges;
  if (doPublish) {
			return;
  }
	
 // not first time, no new movement detected, after 20 seconds is a off again
 		//ROS_INFO("%d secs from last detection",(now_time-this->event_time).sec);
 		//ROS_INFO("Message was %s,%d",this->update_status_msg_.value.c_str(),this->update_status_msg_.value.compare("On"));
 if  (((now_time-this->event_time).sec>10)&&(this->update_status_msg_.value.compare("OPEN")==0))  {
		this->update_status_msg_.value = std::string("CLOSED");
		this->iot_pub_.publish(this->update_status_msg_);
		//ROS_INFO("Forgetting movement..., sending off");
 }
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosPresence)

} // namespace gazebo
