// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

#ifndef MERGE_MAPS_CLASS_H_
#define MERGE_MAPS_CLASS_H_

//some generically useful stuff to include...
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <random>
#include <unordered_set>
#include <numeric>
#include <chrono>

#include <dirent.h>

#include "semantic_labeller/grid_subsampling/grid_subsampling.h"
#include "semantic_labeller/himmelsbach/himmelsbach.h"
#include "semantic_labeller/pointmap/pointmap.h"
#include "semantic_labeller/polar_processing/polar_processing.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
//#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_sequencer.h"

////message types used in this example code;  include more message types, as needed
//#include <std_msgs/Bool.h>
//#include <std_msgs/Float32.h>

// define a class, including a constructor, member variables and member functions
class merge_maps_class
{
public:
    merge_maps_class(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    // params config

    // global map
    PointMap buffer_map;

    // params config
    std::string map_dir;
    std::string save_dir;
    float map_dl;
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

//    // some objects to support subscriber, and publisher
//    ros::Subscriber velo_sub_; //these will be set up within the class constructor, hiding these ugly details
//    ros::Subscriber kf_ref_sub_; // subscribe to kf_ref topic
//    ros::Subscriber slam_map_sub_; // subscribe to kf_ref topic
////    message_filters::Subscriber<sensor_msgs::PointCloud2> velo_sub_; // subscribe to velodyne_points topic
////    message_filters::TimeSequencer<sensor_msgs::PointCloud2> velo_time_seq; //time sequencer

//    ros::Publisher map_pub_, map_ground_pub_, latent_pub_;
//    sensor_msgs::PointCloud2 map_msg, map_ground_msg, latent_msg;
//
//    // transforms
//    tf2_ros::Buffer kf_ref_to_map_buffer, velo_to_map_buffer;
//    tf2_ros::TransformListener kf_ref_to_map_listener, velo_to_map_listener;
////    tf::StampedTransform kf_ref_to_map_transform, velo_to_map_transform;
//    geometry_msgs::TransformStamped kf_ref_to_map_transform, velo_to_map_transform;
//
//    // PCL point cloud for publishing ROS messages
//    pcl::PointCloud<pcl::PointXYZ>::Ptr map_publish, map_ground_publish, latent_publish;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void getParams();

//    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
//    void kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

//    vector<PointXYZ> extract_negative(std::vector<PointXYZ> &cloud, std::vector<int> &indices);
//
//    void extract_negative(std::vector<bool> &flag, std::vector<int> &indices);
//
//    vector<PointXYZ> extract_ground_himmelsbach(vector<PointXYZ> &cloud);
//
//    void extract_ground_himmelsbach(vector<PointXYZ> &cloud, vector<bool> &flag);
//
//    void moveToPCLPtr(vector<PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &PCL_cloud_ptr, vector<bool> &flag, bool update);
//
//    vector<PointXYZ>
//    extract_ground(vector<PointXYZ> &points, vector<PointXYZ> &normals, float angle_vertical_thresh = M_PI / 6, float dist_thresh = 0.1,
//                   int max_iter = 200, bool mode_2D = false);

    std::string configureDataPath(const char keyString[5], unsigned int seq, uint64_t sec);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match