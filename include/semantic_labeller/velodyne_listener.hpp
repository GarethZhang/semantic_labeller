// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

#ifndef VELODYNE_LISTENER_CLASS_H_
#define VELODYNE_LISTENER_CLASS_H_

//some generically useful stuff to include...
#include <string>
#include <vector>
#include <sstream>
#include "inttypes.h"
//#include "semantic_labeller/semantic_labeller.hpp"

#include "ros/ros.h" //ALWAYS need to include this
#include "sensor_msgs/PointCloud2.h" // For Velodyne point cloud
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/correspondence_estimation.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl_ros/transforms.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_sequencer.h"
#include <pcl/registration/icp.h>


////message types used in this example code;  include more message types, as needed
//#include <std_msgs/Bool.h>
//#include <std_msgs/Float32.h>

// define a class, including a constructor, member variables and member functions
class velodyne_listener_class
{
public:
    velodyne_listener_class(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    // some objects to support subscriber, and publisher
//    ros::Subscriber velo_sub_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber kf_ref_sub_, slam_map_sub_; // subscribe to kf_ref topic
    message_filters::Subscriber<sensor_msgs::PointCloud2> velo_sub_; // subscribe to velodyne_points topic
    message_filters::TimeSequencer<sensor_msgs::PointCloud2> velo_time_seq; //time sequencer

    message_filters::Subscriber<sensor_msgs::PointCloud2> velo_undistort_sub_; // subscribe to velodyne_points topic
    message_filters::TimeSequencer<sensor_msgs::PointCloud2> velo_undistort_time_seq; //time sequencer

    ros::Publisher velo_filtered_pub_, velo_undistort_filtered_pub_;
    sensor_msgs::PointCloud2 velo_filtered_msg, velo_undistort_filtered_msg;

    std::ostringstream velo_ss_, kf_ref_ss_, velo_filtered_ss_;
    std::string velo_str_, kf_ref_str_, velo_filtere_str_;

    tf::TransformListener kf_ref_to_velo_listener, velo_to_map_listener, velo_undistort_to_map_listener;
    tf::StampedTransform kf_ref_to_velo_transform, velo_to_map_transform, velo_undistort_to_map_transform;
    uint32_t kf_ref_seq_, velo_seq_;
    uint64_t kf_ref_nsec_; // keep track of latest timestamp for submap
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_kf_ref, slam_map, cur_sub_slam_map, cur_sub_slam_map_undistort; // keep track of last submap
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    // params config
    std::string save_dir;
    std::string submap_frame, submap_topic, slamMap_frame, slamMap_topic, velodyne_frame, velodyne_topic;
    bool save_to_ply;
    double max_distance;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void getParams();

    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void velodyneUndistortCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void slamMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void PointCloudXYZItoXYZ (const pcl::PointCloud<pcl::PointXYZI>& in,
                              pcl::PointCloud<pcl::PointXYZ>& out);

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match