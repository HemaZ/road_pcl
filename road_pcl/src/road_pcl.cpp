
#include "road_pcl.hpp"

PCLNode::PCLNode(ros::NodeHandle &nh){
    std::cout << "Creating ROS Node..." << std::endl;
    ros::param::param<std::string>("~topicName", topicName, "/pointcloud");
    ros::param::param<int>("~maxIter", maxIter, 1000);
    ros::param::param<double>("~leafSize", leafSize, 0.01);
    ros::param::param<double>("~distanceThresh", distanceThresh, 0.1);

    pclSubscriber =  nh.subscribe(topicName, 1, &PCLNode::calbak, this);
    pclPublisher = nh.advertise<sensor_msgs::PointCloud2>("/roadpcl", 1);
}

void PCLNode::calbak(const sensor_msgs::PointCloud2ConstPtr &input){

    std::cout << "Received PointCloud" << std::endl;
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vGrid;
    vGrid.setInputCloud(cloud.makeShared());
    vGrid.setLeafSize(leafSize, leafSize, leafSize);
    vGrid.filter(*filteredCloud);

    pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMaxIterations(maxIter);
    seg.setDistanceThreshold(distanceThresh);
    seg.setInputCloud(filteredCloud);
    seg.segment(*inliers, *coefficents);


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roadCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*roadCloud);

    sensor_msgs::PointCloud2 outmsg;
    pcl::toROSMsg(*roadCloud, outmsg);
    pclPublisher.publish(outmsg);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pcl_road");
    ros::NodeHandle nh;
    PCLNode node(nh);
    ros::spin();

}