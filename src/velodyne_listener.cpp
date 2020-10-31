//example_ros_class.cpp:
//wsn, Feb 2015
//illustrates how to use classes to make ROS nodes
// constructor can do the initialization work, including setting up subscribers, publishers
// can use member variables to pass data from subscribers to other member functions

// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo exampleMinimalPubTopic
// rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0


// this header incorporates all the necessary  files and defines the class "velodyne_listener_class"
#include "velodyne_listener.h"
#include <chrono>
#include <thread>

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
velodyne_listener_class::velodyne_listener_class(ros::NodeHandle *nodehandle) :
    nh_(*nodehandle),
    velo_sub_(nh_, "/velodyne_points", 10),
    velo_time_seq(velo_sub_, ros::Duration(0.1), ros::Duration(0.01), 10)
    { // constructor
    ROS_INFO("in class constructor of velodyne_listener_class");
    getParams();
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    //initialize variables here, as needed

    // can also do tests/waits to make sure all required topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &velodyne_listener_class::subscriberCallback is a pointer to a member function of velodyne_listener_class
// "this" keyword is required, to refer to the current instance of velodyne_listener_class
void velodyne_listener_class::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
//    velo_sub_ = nh_.subscribe("/velodyne_points", 1, &velodyne_listener_class::velodyneCallback, this);
    kf_ref_sub_ = nh_.subscribe(submap_topic, 10, &velodyne_listener_class::kfRefCallback, this);
    velo_time_seq.registerCallback(&velodyne_listener_class::velodyneCallback, this);

    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void velodyne_listener_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
//    pub_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true);
    //add more publishers, as needed
    // note: COULD make pub_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"
}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);

    nh_.param<double>("max_distance", max_distance, 1.0);

    nh_.param<std::string>("submap_frame", submap_frame, "/kf_ref");
    nh_.param<std::string>("submap_topic", submap_topic, "/lidar_keyframe_slam/kfRef_cloud");
    nh_.param<std::string>("velodyne_frame", velodyne_frame, "/velodyne");
    nh_.param<std::string>("velodyne_topic", velodyne_topic, "/velodyne_points");
}

int main(int argc, char **argv) {
    // ROS set-ups:
    ros::init(argc, argv, "semantic_labeller_node"); //node name

    ros::NodeHandle nh("~"); // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type velodyne_listener_class");
    velodyne_listener_class velodyne_listener_class(
            &nh);  //instantiate an velodyne_listener_class object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}
