#ifndef ROAD_PCL_HPP
#define ROAD_PCL_HPP
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

class PCLNode{
    private:
    ros::Publisher pclPublisher;
    ros::Subscriber pclSubscriber;
    double leafSize;
    int maxIter;
    std::string topicName;
    double distanceThresh;
    public:
    PCLNode(ros::NodeHandle &nh);
    void calbak(const sensor_msgs::PointCloud2ConstPtr &cloud);

};

















#endif