// Copyright (c) 2022, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SIM_ENV_MAP_PUBLISHER_H_
#define SIM_ENV_MAP_PUBLISHER_H_

#include <ros/ros.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

namespace sim_env_map_publisher
{
class SimEnvMapPublisher
{
public:
  SimEnvMapPublisher()
    : nh_(""), priv_nh_("~") {
    //priv_nh_.getParam("vicon_topic", vicon_topic_);
    //priv_nh_.getParam("vicon_covariance", vicon_covariance_);

    env_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sim_env_in_map", 1000);
    env_map_sub_ = nh_.subscribe("/simulator/markers", 10, &SimEnvMapPublisher::pub_callback, this, ros::TransportHints().tcpNoDelay());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Publisher env_map_pub_;
  ros::Subscriber env_map_sub_;

  void pub_callback(const visualization_msgs::MarkerArray::ConstPtr msg) {
    auto new_msg = visualization_msgs::MarkerArray();
    //new_msg.header = msg->header;
    //new_msg.pose.pose.position.x = msg->transform.translation.x;
    //new_msg.pose.pose.position.y = msg->transform.translation.y;
    //new_msg.pose.pose.position.z = msg->transform.translation.z;
    //new_msg.pose.pose.orientation = msg->transform.rotation;

    new_msg = *msg;

    for (int i = 0; i < msg-> markers.size(); i++) {
      new_msg.markers[i].header.frame_id = "map";
    }

    //ROS_INFO("Publishing converted vicon data");
    env_map_pub_.publish(new_msg);
  }


};
}  // namespace sim_env_map_publisher

#endif  // SIM_ENV_MAP_PUBLISHER_H_
