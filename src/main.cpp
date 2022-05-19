#include <ros/ros.h>
#include "sim_env_map_publisher/sim_env_map_publisher.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "sim_env_map_publisher");
  auto sim_env_map_publisher_node = sim_env_map_publisher::SimEnvMapPublisher();
  ros::spin();
  ros::shutdown();
  return 0;
}
