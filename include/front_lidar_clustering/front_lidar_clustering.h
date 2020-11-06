/*
 * front_lidar_clusterings.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */
#ifndef FRONT_CLUSTER_H
#define FRONT_CLUSTER_H

// headers in ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <autoware_msgs/Centroids.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

class LidarClustering
{
public:
  LidarClustering();
  ~LidarClustering();

  void init();
  void run();
  void CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg);
  void CallbackAckermann(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
  void projectPointCloud();
  void filteringCloseToPath();
  void resetMemory();
  void groundRemoval();
  void segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);
  
private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher PubOutlierFilteredPoints;
  ros::Publisher PubGroundPoints, PubGroundRemovedPoints;
  ros::Publisher PubBoundingBoxes;
  ros::Subscriber SubPoints;
  ros::Subscriber SubVehicleState;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_OutlierFilteredCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_RawSensorCloud_ptr;
  pcl::PointCloud<PointXYZIR>::Ptr m_laserCloudInRing;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_GroundRemovedCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_ColoredClusteredCloud;


  bool bPredictivePath;
  bool bVehicleState;
  
  std::string m_PointsTopicName;
  double m_NeglectableArea;  
  double m_tooCloseArea;
  double m_CloseToPredictivePath;
  double m_radius;
  double m_WheelBase;
  double m_ClusteringDistance;
  double m_MinClusterSize;
  double m_MaxClusterSize;  
  bool m_useCloudRing;

  geometry_msgs::Point point_prev;
  std_msgs::Header m_velodyne_header;
  std::vector<geometry_msgs::Point> m_PredictivePoints;

  // VLP-16
  const int N_SCAN = 16;
  const int Horizon_SCAN = 1800;
  const float ang_res_x = 0.2;
  const float ang_res_y = 2.0;
  const float ang_bottom = 15.0+0.1;
  const int groundScanInd = 5; //7

  const float sensorMinimumRange = 1.0;
  const float sensorMountAngle = 0.0;
  const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
  const int segmentValidPointNum = 5;
  const int segmentValidLineNum = 3;
  const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
  const float segmentAlphaY = ang_res_y / 180.0 * M_PI;
  
  const int edgeFeatureNum = 2;
  const int surfFeatureNum = 4;
  const int sectionsTotal = 6;
  const float edgeThreshold = 0.1;
  const float surfThreshold = 0.1;
  const float nearestFeatureSearchSqDist = 25;
};


#endif  // FRONT_CLUSTER_H
