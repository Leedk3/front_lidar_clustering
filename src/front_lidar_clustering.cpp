/*
 * front_lidar_clustering.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */

#include "front_lidar_clustering/front_lidar_clustering.h"

// #else

#include <string>
#include <assert.h>

// Constructor
LidarClustering::LidarClustering() : nh_(""), private_nh_("~"), bPredictivePath(false)
{
  private_nh_.param("/front_lidar_clustering_node/points_topic_name",        m_PointsTopicName, std::string("/velodyne_points"));
  private_nh_.param("/front_lidar_clustering_node/neglectable_area",         m_NeglectableArea, double(50));
  private_nh_.param("/front_lidar_clustering_node/too_close_area",           m_tooCloseArea, double(0.2));
  private_nh_.param("/front_lidar_clustering_node/close_to_predictive_path", m_CloseToPredictivePath, double(5));
  private_nh_.param("/front_lidar_clustering_node/wheel_base",               m_WheelBase, double(1.5));
  private_nh_.param("/front_lidar_clustering_node/use_cloud_ring",           m_useCloudRing, bool(true));
  private_nh_.param("/front_lidar_clustering_node/clustering_distance",      m_ClusteringDistance, double(0.5));
  private_nh_.param("/front_lidar_clustering_node/min_clustering_size",      m_MinClusterSize, double(20));
  private_nh_.param("/front_lidar_clustering_node/max_clustering_size",      m_MaxClusterSize, double(100000));

  

  PubOutlierFilteredPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/OutlierFiltered", 1,true);
  PubGroundPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/Ground", 1,true);
  PubGroundRemovedPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud/GroundRemoved", 1,true);
  PubBoundingBoxes = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/BoundingBoxArray/OutlierFiltered", 1,true);

  SubPoints = nh_.subscribe(m_PointsTopicName, 1, &LidarClustering::CallbackPoints, this);
  SubVehicleState = nh_.subscribe("/Ackermann/veh_state", 10, &LidarClustering::CallbackAckermann,this);

  init();
}

LidarClustering::~LidarClustering(){}

void LidarClustering::init()
{
  ROS_INFO("initialize voronoi planner");
}

void LidarClustering::run()
{

}


void LidarClustering::CallbackAckermann(const ackermann_msgs::AckermannDriveStampedConstPtr& msg) 
{    
  m_PredictivePoints.clear();
  double range = M_PI / 1.5;
  double increment = 0.01;
  bVehicleState = true;
  double steering_angle_rad = msg->drive.steering_angle * M_PI / 180;
  if (steering_angle_rad == 0)
  {
      m_radius = DBL_MAX; 
  }
  else
  {
      m_radius = m_WheelBase / tan(steering_angle_rad);    
  }

  for (double i = 0; i < range; i += increment)
  {
    // calc a point of circumference
    geometry_msgs::Point p;
    if (m_radius == DBL_MAX)
    {
      p.y = 0;
      p.x = i * 20; //Just for visualization
    }
    else
    {
      if (m_radius > 0)
      {
          p.x = m_radius * sin(i);
          p.y = - m_radius * cos(i) + m_radius;
      }
      else
      {
          p.x = -m_radius * sin(i);
          p.y = - m_radius * cos(i) + m_radius;
      }

    }        
    m_PredictivePoints.push_back(p);
  }
}

void LidarClustering::resetMemory()
{
  //reset variables
  m_OutlierFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_RawSensorCloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());
  m_fullCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_GroundRemovedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_ColoredClusteredCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

  m_fullCloud->points.resize(N_SCAN*Horizon_SCAN);
  
}

void LidarClustering::CallbackPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  resetMemory();

  // receive pointcloud msg
  pcl::fromROSMsg(*msg, *m_RawSensorCloud_ptr);
  m_velodyne_header = msg->header;
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*m_RawSensorCloud_ptr, *m_RawSensorCloud_ptr, indices);
  
  // have "ring" channel in the cloud
  if (m_useCloudRing == true){
    pcl::fromROSMsg(*msg, *m_laserCloudInRing);
    if (m_laserCloudInRing->is_dense == false) {
        ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        ros::shutdown();
    }  
  }

  autoware_msgs::Centroids centroids;
  autoware_msgs::CloudClusterArray cloud_clusters;

  projectPointCloud();
  filteringCloseToPath();
  groundRemoval();
  // segmentByDistance(m_OutlierFilteredCloud);

}

void LidarClustering::projectPointCloud()
{
  // range image projection
  float verticalAngle, horizonAngle, range;
  size_t rowIdn, columnIdn, index, cloudSize; 
  pcl::PointXYZI thisPoint;

  cv::Mat rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

  cloudSize = m_RawSensorCloud_ptr->points.size();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    thisPoint.x = m_RawSensorCloud_ptr->points[i].x;
    thisPoint.y = m_RawSensorCloud_ptr->points[i].y;
    thisPoint.z = m_RawSensorCloud_ptr->points[i].z;
    // find the row and column index in the iamge for this point
    if (m_useCloudRing == true){
        rowIdn = m_laserCloudInRing->points[i].ring;
    }
    else{
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
    }
    if (rowIdn < 0 || rowIdn >= N_SCAN)
        continue;

    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
        continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
    if (range < sensorMinimumRange)
        continue;
    
    rangeMat.at<float>(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    index = columnIdn  + rowIdn * Horizon_SCAN;
    m_fullCloud->points[index] = thisPoint;
    m_fullCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
  }
}

void LidarClustering::filteringCloseToPath()
{
  pcl::PointIndices::Ptr outlier_indices(new pcl::PointIndices);
  if(!bVehicleState)
  {
    return;
    ROS_WARN("No vehicle state. Check a '/Ackermann/veh_state' topic.");
  }
    
  for(unsigned int i=0; i< m_fullCloud->points.size(); i++)
  {
    pcl::PointXYZI points_raw;
    points_raw.x= m_fullCloud->points[i].x;
    points_raw.y= m_fullCloud->points[i].y;
    points_raw.z= m_fullCloud->points[i].z;
    points_raw.intensity = m_fullCloud->points[i].intensity;

    double distance2point = sqrt(pow(points_raw.x,2) + 
                                 pow(points_raw.y,2) + 
                                 pow(points_raw.z,2) );
    if (distance2point > m_NeglectableArea || distance2point < m_tooCloseArea || points_raw.x < 0)
    {
      continue;
    }

    point_prev.x = 0, point_prev.y = 0, point_prev.z = 0;

    for (auto point : m_PredictivePoints)
    {
      double slope;
      if (point.x - point_prev.x == 0)
      {
          slope = 0;
      }
      else
      {
          slope = (point.y - point_prev.y) / 
                  (point.x - point_prev.x);                        
      }
      double bias = point.y - slope * point.x;
      double normal_distance = fabs(slope * points_raw.x - points_raw.y + bias) /
                        sqrt(pow(slope,2) + 1);

      //Filtering the points close to the predictive path
      if (normal_distance < m_CloseToPredictivePath && points_raw.x > point_prev.x && points_raw.x < point.x)
      {
        outlier_indices->indices.push_back(i);
      }
      point_prev = point;
    }

  }
  
  m_OutlierFilteredCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_fullCloud);
  extract.setIndices(outlier_indices);
  extract.setNegative(false);//true removes the indices, false leaves only the indices
  extract.filter(*m_OutlierFilteredCloud);

  sensor_msgs::PointCloud2 OutlierFilteredCloudMsg;
  pcl::toROSMsg(*m_OutlierFilteredCloud, OutlierFilteredCloudMsg);
  OutlierFilteredCloudMsg.header = m_velodyne_header;
  PubOutlierFilteredPoints.publish(OutlierFilteredCloudMsg);  
}


void LidarClustering::groundRemoval()
{
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  size_t lowerInd, upperInd;
  float diffX, diffY, diffZ, angle;
  cv::Mat groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  // groundMat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < Horizon_SCAN; ++j)
  {
    for (size_t i = 0; i < groundScanInd; ++i)
    {
      lowerInd = j + ( i )*Horizon_SCAN;
      upperInd = j + (i+1)*Horizon_SCAN;

      if (m_fullCloud->points[lowerInd].intensity == -1 ||
          m_fullCloud->points[upperInd].intensity == -1)
      {
        // no info to check, invalid points
        groundMat.at<int8_t>(i,j) = -1;
        continue;
      }
          
      diffX = m_fullCloud->points[upperInd].x - m_fullCloud->points[lowerInd].x;
      diffY = m_fullCloud->points[upperInd].y - m_fullCloud->points[lowerInd].y;
      diffZ = m_fullCloud->points[upperInd].z - m_fullCloud->points[lowerInd].z;

      angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

      if (abs(angle - sensorMountAngle) <= 3)
      {
        groundMat.at<int8_t>(i,j) = 1;
        groundMat.at<int8_t>(i+1,j) = 1;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i <= groundScanInd; ++i)
  {
    for (size_t j = 0; j < Horizon_SCAN; ++j)
    {
      if (groundMat.at<int8_t>(i,j) == 1)
        groundCloud->push_back(m_fullCloud->points[j + i*Horizon_SCAN]);
        ground_indices->indices.push_back(j + i*Horizon_SCAN);
    }
  }
  
  m_GroundRemovedCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(m_fullCloud);
  extract.setIndices(ground_indices);
  extract.setNegative(true);//true removes the indices, false leaves only the indices
  extract.filter(*m_GroundRemovedCloud);

  sensor_msgs::PointCloud2 GroundCloudMsg;
  pcl::toROSMsg(*groundCloud, GroundCloudMsg);
  GroundCloudMsg.header = m_velodyne_header;
  PubGroundPoints.publish(GroundCloudMsg);  

  sensor_msgs::PointCloud2 GroundRemovedCloudMsg;
  pcl::toROSMsg(*m_GroundRemovedCloud, GroundRemovedCloudMsg);
  GroundRemovedCloudMsg.header = m_velodyne_header;
  PubGroundRemovedPoints.publish(GroundRemovedCloudMsg);  
}


void LidarClustering::segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZI current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    cloud_ptr->points.push_back(current_point);
  }

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  // create 2d pc
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
 // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++)
  {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;
  ec.setClusterTolerance(m_ClusteringDistance);  //
  ec.setMinClusterSize(m_MinClusterSize);
  ec.setMaxClusterSize(m_MinClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  autoware_msgs::CloudClusterArray ClusterArrayMsg;
  jsk_recognition_msgs::BoundingBoxArray BoundingBoxArayMsg;

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  { 
    autoware_msgs::CloudCluster ClusterMsgTmp;
    sensor_msgs::PointCloud2 CloudMsgTmp;
    jsk_recognition_msgs::BoundingBox BoundingBoxTmp;
    
    float average_x = 0, average_y = 0, average_z = 0;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      // fill new colored cluster point by point
      pcl::PointXYZI point;
      point.x = in_cloud_ptr->points[*pit].x;
      point.y = in_cloud_ptr->points[*pit].y;
      point.z = in_cloud_ptr->points[*pit].z;
      point.intensity = in_cloud_ptr->points[*pit].intensity;
      cluster_ptr->points.push_back(point);

      average_x += point.x;
      average_y += point.y;
      average_z += point.z;
      if (point.x < min_x)
        min_x = point.x;
      if (point.y < min_y)
        min_y = point.y;
      if (point.z < min_z)
        min_z = point.z;
      if (point.x > max_x)
        max_x = point.x;
      if (point.y > max_y)
        max_y = point.y;
      if (point.z > max_z)
        max_z = point.z;
    }
    if (it->indices.size() > 0)
    {
      average_x /= it->indices.size();
      average_y /= it->indices.size();
      average_z /= it->indices.size();
    }

    pcl::toROSMsg(*cluster_ptr, CloudMsgTmp);
    ClusterMsgTmp.header = m_velodyne_header;
    ClusterMsgTmp.cloud = CloudMsgTmp;
    ClusterMsgTmp.centroid_point.point.x = average_x;
    ClusterMsgTmp.centroid_point.point.y = average_y;
    ClusterMsgTmp.centroid_point.point.z = average_z;

    // calculate bounding box
    double length = max_x - min_x;
    double width = max_y - min_y;
    double height = max_z - min_z;

    ClusterMsgTmp.bounding_box.header = m_velodyne_header;

    ClusterMsgTmp.bounding_box.pose.position.x = min_x + length / 2;
    ClusterMsgTmp.bounding_box.pose.position.y = min_y + width / 2;
    ClusterMsgTmp.bounding_box.pose.position.z = min_z + height / 2;

    ClusterMsgTmp.bounding_box.dimensions.x = ((length < 0) ? -1 * length : length);
    ClusterMsgTmp.bounding_box.dimensions.y = ((width < 0) ? -1 * width : width);
    ClusterMsgTmp.bounding_box.dimensions.z = ((height < 0) ? -1 * height : height);

    BoundingBoxTmp = ClusterMsgTmp.bounding_box;
    ClusterArrayMsg.clusters.push_back(ClusterMsgTmp);
    BoundingBoxArayMsg.boxes.push_back(BoundingBoxTmp);
  }
  ClusterArrayMsg.header = m_velodyne_header;
  BoundingBoxArayMsg.header = m_velodyne_header;
  PubBoundingBoxes.publish(BoundingBoxArayMsg);
}
