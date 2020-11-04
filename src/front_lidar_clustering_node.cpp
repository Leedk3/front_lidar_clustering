#include "front_lidar_clustering/front_lidar_clustering.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "front_lidar_clustering");
  LidarClustering planner;
  
  ros::Rate loop_rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    planner.run();
    loop_rate.sleep();
  }

  return 0;
}
