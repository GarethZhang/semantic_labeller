// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

#ifndef VELODYNE_LISTENER_CLASS_H_
#define VELODYNE_LISTENER_CLASS_H_

//some generically useful stuff to include...
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <random>
#include <unordered_set>
#include <numeric>

#include "grid_subsampling/grid_subsampling.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/time_sequencer.h"

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
    ros::Subscriber kf_ref_sub_; // subscribe to kf_ref topic
    ros::Subscriber slam_map_sub_; // subscribe to kf_ref topic
    message_filters::Subscriber<sensor_msgs::PointCloud2> velo_sub_; // subscribe to velodyne_points topic
    message_filters::TimeSequencer<sensor_msgs::PointCloud2> velo_time_seq; //time sequencer

    // params config
    std::string save_dir;
    std::string submap_frame, submap_topic, velodyne_frame, velodyne_topic;
    bool save_to_ply;
    double max_distance;

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();

    void getParams();

    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match