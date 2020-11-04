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
    map_ground_publish(new pcl::PointCloud<pcl::PointXYZ>),
    velo_sub_(nh_, "/velodyne_points", 10),
    velo_time_seq(velo_sub_, ros::Duration(1.0), ros::Duration(0.01), 10)
    { // constructor
    ROS_INFO("in class constructor of velodyne_listener_class");
    getParams();
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    std::ostringstream pose_fname;
    pose_fname.str(""); pose_fname.clear();
    pose_fname << save_dir << "map_poses"<< ".txt";
    pose_file.open (pose_fname.str());

    //initialize variables here, as needed

    // can also do tests/waits to make sure all required topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &velodyne_listener_class::subscriberCallback is a pointer to a member function of velodyne_listener_class
// "this" keyword is required, to refer to the current instance of velodyne_listener_class
void velodyne_listener_class::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers");
//    velo_sub_ = nh_.subscribe("/velodyne_points", 1, &velodyne_listener_class::velodyneCallback, this);
//    kf_ref_sub_ = nh_.subscribe(submap_topic, 10, &velodyne_listener_class::kfRefCallback, this);
    velo_time_seq.registerCallback(&velodyne_listener_class::velodyneCallback, this);

    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void velodyne_listener_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
//    pub_ = nh_.advertise<std_msgs::Float32>("exampleMinimalPubTopic", 1, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/slam_map", 1, true);
    map_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/slam_map_ground", 1, true);
    latent_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velo_latent", 1, true);
    //add more publishers, as needed
    // note: COULD make pub_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::kfRefCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    /////////////////////////////////////////////////////////////
    // Get latest transform between submap frame and map frame //
    /////////////////////////////////////////////////////////////

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
    kf_ref_velo.reserve(N);
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

    //////////////////////////////////////////
    // Preprocess frame and compute normals //
    //////////////////////////////////////////

    // Create a copy of points in polar coordinates
    vector<PointXYZ> polar_pts(kf_ref_velo_transformed);
    cart2pol_(polar_pts);

    // Get lidar angle resolution
    float minTheta, maxTheta;
    float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta, lidar_n_lines);

    // Define the polar neighbors radius in the scaled polar coordinates
    float polar_r = 1.5 * lidar_angle_res;

    // Apply log scale to radius coordinate (in place)
    lidar_log_radius(polar_pts, polar_r, (float)r_scale);

    /////////////////////
    // Compute normals //
    /////////////////////

    // Init result containers
    vector<PointXYZ> normals;
    vector<float> norm_scores;

    // Apply horizontal scaling (to have smaller neighborhoods in horizontal direction)
    lidar_horizontal_scale(polar_pts, (float)h_scale);

    // Do not sub-sample. Use all the indices
    vector<size_t> sub_inds;
    for (int i = 0; i < kf_ref_velo_transformed.size(); ++i)
        sub_inds.push_back(i);

    // Call polar processing function
    extract_lidar_frame_normals(kf_ref_velo_transformed, polar_pts, sub_inds, normals, norm_scores, polar_r);

    // Remove points with a low score
    filter_pointcloud(kf_ref_velo_transformed, norm_scores, 0.1);
    filter_pointcloud(normals, norm_scores, 0.1);
    norm_scores.erase(remove_if(norm_scores.begin(), norm_scores.end(), [](const float s) { return s < 0.1; }), norm_scores.end());

    ////////////////
    // Add to Map //
    ////////////////

    // The update function is called only on subsampled points as the others have no normal
    map.update(kf_ref_velo_transformed, normals, norm_scores);

    // save out map
    std::string save_str = configureDataPath("map", msg->header.seq, msg->header.stamp.toNSec());
    vector<float> features;
    for (auto score:map.scores)
        features.push_back(score);
    for (auto count:map.counts)
        features.push_back(count);
    save_cloud(save_str, map.cloud.pts, map.normals, features);

//    // Publish ros messages
//    moveToPCLPtr(map.cloud.pts, map_publish, false);
//    pcl::toROSMsg(*map_publish, map_msg);
//    map_msg.header.frame_id = slamMap_frame;
//    map_msg.header.stamp = msg->header.stamp;
//    map_pub_.publish(map_msg);
//
//    /////////////////////
//    // Extract Ground  //
//    /////////////////////
//    auto kf_ref_velo_before = kf_ref_velo_transformed.size();
//    vector<PointXYZ> ground = extract_ground_himmelsbach(kf_ref_velo_transformed);
//    ROS_INFO("In KF, keeping %lu / %lu points after removing ground", kf_ref_velo_transformed.size(), kf_ref_velo_before);
//
//    // Publish ros messages
//    moveToPCLPtr(kf_ref_velo_transformed, map_ground_publish, true);
//    pcl::toROSMsg(*map_ground_publish, map_ground_msg);
//    map_ground_msg.header.frame_id = slamMap_frame;
//    map_ground_msg.header.stamp = msg->header.stamp;
//    map_ground_pub_.publish(map_ground_msg);
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //////////////////////////////
    // Read point cloud message //
    //////////////////////////////

//    // Get the number of points
//    size_t N = (size_t)(msg->width * msg->height);
//
//    // Loop over points and copy in vector container. Do the filtering if necessary
//    vector<PointXYZ> velo;
//    velo.reserve(N);
//    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
//         iter_x != iter_x.end();
//         ++iter_x, ++iter_y, ++iter_z)
//    {
//        // Add all points to the vector container
//        velo.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
//    }
//
//    // save out every frame of point cloud
//    std::string save_str = configureDataPath("velo", msg->header.seq, msg->header.stamp.toNSec());
//    save_cloud(save_str, velo);

    if (true){
        try{
            velo_to_map_listener.lookupTransform(slamMap_frame, velodyne_frame,
                                                 msg->header.stamp, velo_to_map_transform);
            ROS_INFO("%lu", msg->header.stamp.toNSec());

            pose_file << msg->header.stamp.toNSec() << " " << velo_to_map_transform.getBasis().getRow(0).x() << " " << velo_to_map_transform.getBasis().getRow(0).y() << " " <<
                      velo_to_map_transform.getBasis().getRow(0).z() << " " << velo_to_map_transform.getBasis().getRow(1).x() << " " <<
                      velo_to_map_transform.getBasis().getRow(1).y() << " " << velo_to_map_transform.getBasis().getRow(1).z() << " " <<
                      velo_to_map_transform.getBasis().getRow(2).x() << " " << velo_to_map_transform.getBasis().getRow(2).y() << " " <<
                      velo_to_map_transform.getBasis().getRow(2).z() << " " << velo_to_map_transform.getOrigin().x() << " " <<
                      velo_to_map_transform.getOrigin().y() << " " << velo_to_map_transform.getOrigin().z() << "\n";
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }
}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);

    nh_.param<double>("max_distance", max_distance, 1.0);

    nh_.param<int>("lidar_n_lines", lidar_n_lines, 1.0);
    nh_.param<double>("r_scale", r_scale, 1.0);
    nh_.param<double>("h_scale", h_scale, 1.0);

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

vector<PointXYZ> velodyne_listener_class::extract_negative(std::vector<PointXYZ> &cloud, std::vector<int> &indices) {
    std::sort(indices.begin(), indices.end());
    std::vector<int> inverse;
    std::vector<PointXYZ> ground;
    for (uint i = 0; i < cloud.size(); i++) {
        if (std::binary_search(indices.begin(), indices.end(), i)){
            ground.push_back(cloud[i]);
            continue;
        }
        inverse.push_back(i);
    }
    std::sort(inverse.begin(), inverse.end());
    for (uint i = 0; i < inverse.size(); i++) {
        cloud[i] = cloud[inverse[i]];
    }
    cloud.resize(inverse.size());
    return ground;
}

vector<PointXYZ> velodyne_listener_class::extract_ground_himmelsbach(vector<PointXYZ> &cloud) {
    Himmelsbach himmelsbach(cloud);
    himmelsbach.set_alpha(alpha);
    himmelsbach.set_tolerance(tolerance);
    himmelsbach.set_thresholds(Tm, Tm_small, Tb, Trmse, Tdprev);
    std::vector<int> inliers;
    himmelsbach.compute_model_and_get_inliers(inliers);
    vector<PointXYZ> ground = extract_negative(cloud, inliers);

    return ground;
}

vector<PointXYZ> velodyne_listener_class::extract_ground(vector<PointXYZ> &points,
                                                vector<PointXYZ> &normals,
                                                float angle_vertical_thresh,
                                                float dist_thresh,
                                                int max_iter,
                                                bool mode_2D)
{

//    // In case of 2D mode, take the minimum height and use vertical normal
//    if (mode_2D)
//    {
//        PointXYZ A = points[0];
//        for (auto& p: points)
//        {
//            if (p.z < A.z)
//                A = p;
//        }
//        return Plane3D(A, PointXYZ(0, 0, 1));
//
//    }

    // Get the points with vertical normal (angle_vertical_thresh should be in radians)
    vector<PointXYZ> valid_points;
    valid_points.reserve(points.size());
    size_t i = 0;
    float cos_thresh = cos(angle_vertical_thresh);
    for (auto& n: normals)
    {
        if (abs(n.z) > cos_thresh)
        {
            valid_points.push_back(points[i]);
        }
        i++;
    }

    // Random generator
    default_random_engine generator;
    uniform_int_distribution<size_t> distribution(0, valid_points.size() - 1);

    // RANSAC loop
    int best_votes = 0;
    Plane3D best_P;
    vector<PointXYZ> best_outliers;
    for (int i = 0; i < max_iter; i++)
    {
        // Draw 3 random points
        unordered_set<size_t> unique_inds;
        vector<PointXYZ> ABC, outliers;
        while (unique_inds.size() < 3)
        {
            size_t ind = distribution(generator);
            unique_inds.insert(ind);
            if (unique_inds.size() > ABC.size())
                ABC.push_back(valid_points[ind]);
        }

        // Get the corresponding plane
        Plane3D candidate_P(ABC[0], ABC[1], ABC[2]);

        // Avoid ill defined planes
        if (candidate_P.u.sq_norm() < 1e-5)
        {
            i--;
            continue;
        }

        // Get the number of votes for this plane
        int votes = candidate_P.in_range(valid_points, dist_thresh, outliers);

        // Save best plane
        if (votes > best_votes)
        {
            best_votes = votes;
            best_P = candidate_P;
            best_outliers = outliers;
        }
    }
    return best_outliers;
}

void velodyne_listener_class::moveToPCLPtr(vector<PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &PCL_cloud_ptr, bool update) {
    if (!update)
        PCL_cloud_ptr->clear();
    pcl::PointXYZ p;
    for (int i = 0; i < cloud.size(); ++i){
        p.x = cloud[i].x;
        p.y = cloud[i].y;
        p.z = cloud[i].z;
        PCL_cloud_ptr->push_back(p);
    }
}

std::string velodyne_listener_class::configureDataPath(const char *keyString, unsigned int seq, uint64_t sec) {
    std::ostringstream dataPath;
    dataPath.str(""); dataPath.clear();
    dataPath << save_dir << keyString << "_" << seq << "_" << sec<< ".ply";
    return dataPath.str();
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
