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
    map_publish(new pcl::PointCloud<pcl::PointXYZ>),
    velo_sub_(nh_, "/velodyne_points", 10),
    velo_time_seq(velo_sub_, ros::Duration(0.0), ros::Duration(0.01), 10)
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
//    velo_time_seq.registerCallback(&velodyne_listener_class::velodyneCallback, this);

    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void velodyne_listener_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
//    pub_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true);
    latent_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velo_latent", 1, true);
    //add more publishers, as needed
    // note: COULD make pub_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    bool available = false;
    if (!available){
        try{
            kf_ref_to_map_listener.lookupTransform(slamMap_frame, submap_frame,
                                                msg->header.stamp, kf_ref_to_map_transform);
            available = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }
    Eigen::Affine3d eigenTr;
    tf::transformTFToEigen(kf_ref_to_map_transform, eigenTr);

    // Get the number of points
    size_t N = (size_t)(msg->width * msg->height);

    // Loop over points and copy in vector container. Do the filtering if necessary
    vector<PointXYZ> kf_ref_velo;
    vector<PointXYZ> f_pts;
    f_pts.reserve(N);
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z)
    {
        // Add all points to the vector container
        kf_ref_velo.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
    }

    // create a copy of the point cloud vector
    vector<PointXYZ> kf_ref_velo_transformed(kf_ref_velo);

    // Matrix for original/aligned data (Shallow copy of parts of the points vector)
    Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> kf_ref_velo_mat((float*)kf_ref_velo.data(), 3, N);
    Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> kf_ref_velo_transformed_mat((float*)kf_ref_velo_transformed.data(), 3, N);

    // Apply initial transformation
    Eigen::Matrix3f R_init = (eigenTr.matrix().block(0, 0, 3, 3)).cast<float>();
    Eigen::Vector3f T_init = (eigenTr.matrix().block(0, 3, 3, 1)).cast<float>();
    kf_ref_velo_transformed_mat = (R_init * kf_ref_velo_mat).colwise() + T_init;

    // Publish ros messages
    pcl::PointXYZ p;
    int nr_correspondences = kf_ref_velo_transformed.size();
    for (int i = 0; i < nr_correspondences; ++i){
        p.x = kf_ref_velo_transformed[i].x;
        p.y = kf_ref_velo_transformed[i].y;
        p.z = kf_ref_velo_transformed[i].z;
        map_publish->push_back(p);
    }

    ROS_INFO("ADDING NEW SUBMAP WITH %lu POINTS", map_publish->size());

    // publish the ROS messages
    pcl::toROSMsg(*map_publish, latent_msg);
    latent_msg.header.frame_id = slamMap_frame;
    latent_msg.header.stamp = msg->header.stamp;
    latent_pub_.publish(latent_msg);
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //////////////////////////////
    // Read point cloud message //
    //////////////////////////////

    // Get the number of points
    size_t N = (size_t)(msg->width * msg->height);

    // Get timestamp
    ros::Time stamp = msg->header.stamp;

    // Loop over points and copy in vector container. Do the filtering if necessary
    PointCloudXYZPtr velo(new PointCloudXYZ ());
    vector<PointXYZ> f_pts;
    f_pts.reserve(N);
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z)
    {
        // Add all points to the vector container
        velo->points.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
    }

    // Remove ground plane using Himmelsbach
    Himmelsbach himmelsbach(velo);
    himmelsbach.set_alpha(alpha);
    himmelsbach.set_tolerance(tolerance);
    himmelsbach.set_thresholds(Tm, Tm_small, Tb, Trmse, Tdprev);
    std::vector<int> inliers;
    himmelsbach.compute_model_and_get_inliers(inliers);
    extract_negative(velo, inliers);

    // Publish ros messages
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_publish (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p;
    int nr_correspondences = velo->size();
    for (int i = 0; i < nr_correspondences; ++i){
        p.x = velo->points[i].x;
        p.y = velo->points[i].y;
        p.z = velo->points[i].z;
        cloud_publish->push_back(p);
    }

    // publish the ROS messages
    pcl::toROSMsg(*cloud_publish, latent_msg);
    latent_msg.header.frame_id = msg->header.frame_id;
    latent_msg.header.stamp = msg->header.stamp;
    latent_pub_.publish(latent_msg);
}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);

    nh_.param<double>("max_distance", max_distance, 1.0);

    nh_.param<float>("alpha", alpha, 1.0);
    nh_.param<float>("tolerance", tolerance, 1.0);
    nh_.param<float>("Tm", Tm, 1.0);
    nh_.param<float>("Tm_small", Tm_small, 1.0);
    nh_.param<float>("Tb", Tb, 1.0);
    nh_.param<float>("Trmse", Trmse, 1.0);
    nh_.param<float>("Tdprev", Tdprev, 1.0);

    nh_.param<std::string>("submap_frame", submap_frame, "/kf_ref");
    nh_.param<std::string>("submap_topic", submap_topic, "/lidar_keyframe_slam/kfRef_cloud");
    nh_.param<std::string>("slamMap_frame", slamMap_frame, "/map");
    nh_.param<std::string>("slamMap_topic", slamMap_topic, "/lidar_keyframe_slam/slamMap_cloud");
    nh_.param<std::string>("velodyne_frame", velodyne_frame, "/velodyne");
    nh_.param<std::string>("velodyne_topic", velodyne_topic, "/velodyne_points");
}

void velodyne_listener_class::extract_negative(PointCloudXYZPtr cloud, std::vector<int> &indices) {
    std::sort(indices.begin(), indices.end());
    std::vector<int> inverse;
    for (uint i = 0; i < cloud->size(); i++) {
        if (std::binary_search(indices.begin(), indices.end(), i))
            continue;
        inverse.push_back(i);
    }
    std::sort(inverse.begin(), inverse.end());
    for (uint i = 0; i < inverse.size(); i++) {
        cloud->points[i] = cloud->points[inverse[i]];
    }
    cloud->resize(inverse.size());
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
