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
#include "semantic_labeller/velodyne_listener.hpp"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
velodyne_listener_class::velodyne_listener_class(ros::NodeHandle *nodehandle) :
    nh_(*nodehandle),
    last_kf_ref (new pcl::PointCloud<pcl::PointXYZI>),
    slam_map(new pcl::PointCloud<pcl::PointXYZI>),
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
    slam_map_sub_ = nh_.subscribe(slamMap_topic, 10, &velodyne_listener_class::slamMapCallback, this);
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

    // update global variable
    kf_ref_seq_ = msg->header.seq;
    kf_ref_nsec_ = msg->header.stamp.toNSec();

    // initialize PCL point cloud (ROS)
    auto *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*msg, *cloud);

    // initialize PCL point cloud (PCL)
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(*cloud, *point_cloud);

    // update global variable to store latest submap
    *last_kf_ref = *point_cloud;

//    // save out transformed point cloud named after the seq number
//    kf_ref_seq_ = msg->header.seq;
//    kf_ref_ss_.str(""); kf_ref_ss_.clear();
//    kf_ref_ss_ << save_dir << "kf_ref_" << kf_ref_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
//    kf_ref_str_ = kf_ref_ss_.str();
//    ROS_INFO("SAVING KF_REF FILE");
//    pcl::io::savePLYFileBinary(kf_ref_str_, *point_cloud);

}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::slamMapCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    // initialize PCL point cloud (ROS)
    auto *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*msg, *cloud);

    // initialize PCL point cloud (PCL)
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(*cloud, *point_cloud);

    // update global variable to store latest submap
    *slam_map = *point_cloud;

    velo_seq_ = msg->header.seq;
    velo_ss_.str(""); velo_ss_.clear();
    velo_ss_ << save_dir << "slamMap_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
    velo_str_ = velo_ss_.str();
    ROS_INFO("SAVING SLAM MAP FILE");
    pcl::io::savePLYFileBinary(velo_str_, *slam_map);
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    // initialize PCL point cloud (ROS)
    auto *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL(*msg, *cloud);

    // initialize PCL point cloud (PCL)
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(*cloud, *point_cloud);

    // transform the point cloud when /tf becomes available
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_out (new pcl::PointCloud<pcl::PointXYZI>);

    bool available = false;
    while (!available){
        try{
            kf_ref_to_velo_listener.lookupTransform("map", "velodyne",
                                                    ros::Time(0), kf_ref_to_velo_transform);
            available = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    pcl_ros::transformPointCloud (*point_cloud, *point_cloud_out, kf_ref_to_velo_transform);
    point_cloud_out->header.frame_id = "map";

    // check if slam_map has got the full_map
    if (!slam_map->empty()){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        velodyne_listener_class::PointCloudXYZItoXYZ(*point_cloud_out, *cloud_in);
        velodyne_listener_class::PointCloudXYZItoXYZ(*slam_map, *cloud_out);
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        est.setInputSource(cloud_in);
        est.setInputTarget(cloud_out);

        pcl::Correspondences all_correspondences;
        est.determineCorrespondences(all_correspondences, max_distance);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ p;
        int nr_correspondences = (int)all_correspondences.size();
        int index_query;
        for (int i = 0; i < nr_correspondences; ++i){
            index_query = all_correspondences[i].index_query;
            p.x = cloud_in->points[index_query].x;
            p.y = cloud_in->points[index_query].y;
            p.z = cloud_in->points[index_query].z;
            cloud_filtered->push_back(p);
        }

        // save out transformed point cloud named after the seq number
        if (save_to_ply){
            velo_seq_ = msg->header.seq;
            velo_ss_.str(""); velo_ss_.clear();
            velo_ss_ << save_dir << "velo_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
            velo_str_ = velo_ss_.str();
            ROS_INFO("SAVING VELO FILE");
            pcl::io::savePLYFileBinary(velo_str_, *point_cloud_out);

            velo_filtered_ss_.str(""); velo_filtered_ss_.clear();
            velo_filtered_ss_ << save_dir << "velo_filtered_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
            velo_filtere_str_ = velo_filtered_ss_.str();
            ROS_INFO("SAVING VELO FILTERED FILE");
            pcl::io::savePLYFileBinary(velo_filtere_str_, *cloud_filtered);
        }

    }
    else{
        ROS_INFO("SLAM MAP EMPTY");
    }

}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);

    nh_.param<double>("max_distance", max_distance, 1.0);

    nh_.param<std::string>("submap_frame", submap_frame, "/kf_ref");
    nh_.param<std::string>("submap_topic", submap_topic, "/lidar_keyframe_slam/kfRef_cloud");
    nh_.param<std::string>("slamMap_topic", slamMap_topic, "/lidar_keyframe_slam/slamMap_cloud");
    nh_.param<std::string>("velodyne_frame", velodyne_frame, "/velodyne");
    nh_.param<std::string>("velodyne_topic", velodyne_topic, "/velodyne_points");
}

void velodyne_listener_class::PointCloudXYZItoXYZ(const pcl::PointCloud<pcl::PointXYZI> &in,
                                                  pcl::PointCloud<pcl::PointXYZ> &out) {
    out.width   = in.width;
    out.height  = in.height;
    for (const auto &point : in.points)
    {
        pcl::PointXYZ p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        out.points.push_back (p);
    }
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
