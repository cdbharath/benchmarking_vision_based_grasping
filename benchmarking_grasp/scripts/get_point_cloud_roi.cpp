#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/improc.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class GetPointCloudROI{
    public:
        GetPointCloudROI(ros::NodeHandle& nh);
        void ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr applyPassthroughFilter(
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
        void on_trackbar(int, void*);

    private:
        ros::NodeHandle n;
        ros::Subscriber pt_cloud_sub;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::visualization::PCLVisualizer::Ptr viewer;
};

GetPointCloudROI::on_trackbar(int, void*){
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZ> (cloud);
    viewer->spinOnce ();
    ros::spinOnce();
}

GetPointCloudROI::GetPointCloudROI(ros::NodeHandle& nh): n(nh){
    // std::string pc_topic;
    // n.getParam("point_cloud", pc_topic);
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pt_cloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, &GetPointCloudROI::ptCloudCallback, this);
    viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("Point Cloud"));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);

    int init_percent = 0;
    namedWindow("Adjust ROI", WINDOW_AUTOSIZE);
    createTrackbar("Trackbar", "Adjust ROI", &init_percent, 100, on_trackbar);

    waitKey(0);

    // while (!viewer->wasStopped ()){
    //     viewer->removeAllPointClouds();
    //     viewer->addPointCloud<pcl::PointXYZ> (cloud);
    //     viewer->spinOnce ();
    //     ros::spinOnce();
    // }
}

void GetPointCloudROI::ptCloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud){
    pcl::fromROSMsg(*in_cloud, *cloud);
}

// pcl::PointCloud<pcl::PointXYZ>::Ptr GetPointCloudROI::applyPassthroughFilter(
//     pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud (cloud);
//     pass.setFilterFieldName ("x");
//     pass.setFilterLimits (x_min, x_max);
//     pass.filter (*cloud_filtered);

//     pass.setInputCloud (cloud_filtered);
//     pass.setFilterFieldName ("y");
//     pass.setFilterLimits (y_min, y_max);
//     pass.filter (*cloud_filtered);

//     pass.setInputCloud (cloud_filtered);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (z_min, z_max);
//     pass.filter (*cloud_filtered);

//     return cloud_filtered;
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "get_pc_roi");
    ros::NodeHandle n;
    GetPointCloudROI get_pc_roi(n);

    ros::spin();
    return 0;
}