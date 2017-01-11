#include "ros/ros.h"
// iot_set uses this kind of msg
#include "diagnostic_msgs/KeyValue.h"

// an example for ambient data
#include <env_sensor/Environment.h>


#include <sstream>
#include <string>

        
ros::Publisher iot_set_pub;
        
/**
 * This node subscribes to Robot environment sensor topics in ros and sets the state
 * of their corresponding openHAB items in consecuence. 
 * This way we synchronize both, and led openHAB handle all ambient data.
 * See env_sensor project for getting the data into ros
 */
 
 
 /* 
  * 
  * 
  * */



template <typename T>
  std::string NumberToString ( T Number )
  {
     std::ostringstream ss;
     ss << Number;
     return ss.str();
  }
   
 
 void environmentInfoCallback(const env_sensor::Environment::ConstPtr& msg)
{
   ROS_DEBUG("New environmental data");
   diagnostic_msgs::KeyValue response;

  response.key="Env_cO";
  response.value=NumberToString<float>(msg->cO);
  iot_set_pub.publish(response);
  
  response.key="Env_particleCount";
  response.value=NumberToString<float>(msg->particleCount);
  iot_set_pub.publish(response);
  
  response.key="Env_vOCResistance";
  response.value=NumberToString<float>(msg->vOCResistance);
  iot_set_pub.publish(response);
  
  response.key="Env_temp";
  response.value=NumberToString<float>(msg->temp);
  iot_set_pub.publish(response);
  
  response.key="Env_humidity";
  response.value=NumberToString<float>(msg->humidity);
  iot_set_pub.publish(response);
  
  response.key="Env_light";
  response.value=NumberToString<float>(msg->light);
  iot_set_pub.publish(response);
  
  response.key="Env_timestamp";
  response.value=NumberToString<uint32_t>(msg->header.stamp.sec);
  iot_set_pub.publish(response);
}
 
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "amb2openhab");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("environment_sensor", 1000, environmentInfoCallback);
  
  iot_set_pub = n.advertise<diagnostic_msgs::KeyValue >("openhab_set", 1000);
  ros::spin();

  return 0;
}


