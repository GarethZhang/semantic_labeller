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
    latent_publish(new pcl::PointCloud<pcl::PointXYZ>),
    velo_sub_(nh_, "/lidar_keyframe_slam/velodyne_points_undistorted", 1000),
    velo_time_seq(velo_sub_, ros::Duration(1.0), ros::Duration(0.01), 1000)
    { // constructor
    ROS_INFO("in class constructor of velodyne_listener_class");
    getParams();
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();

    map.set_map_dl(map_dl);
    buffer_map.set_map_dl(map_dl);

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

    vector<string> clock_str;
    vector<clock_t> t;
    if (show_debug_msg)
    {
        clock_str.reserve(20);
        t.reserve(20);
        clock_str.push_back("tf listener ....... ");
        clock_str.push_back("Pose saving ....... ");
        clock_str.push_back("Process msg ....... ");
        clock_str.push_back("Scan saving ....... ");
        clock_str.push_back("Ground extraction . ");
        clock_str.push_back("Transform scan .... ");
        clock_str.push_back("Polar conversion .. ");
        clock_str.push_back("Grid subsampling .. ");
        clock_str.push_back("Frame normals ..... ");
        clock_str.push_back("Sub-ground ........ ");
        clock_str.push_back("Save buffer ....... ");
        clock_str.push_back("Normals filter .... ");
        clock_str.push_back("Save map .......... ");
        clock_str.push_back("Complete .......... ");
    }
    t.push_back(std::clock());

    //////////////////////////////////////
    // Listen to Velodyne to Map transform
    //////////////////////////////////////
    bool available = false;
    try {
        velo_to_map_listener.lookupTransform(slamMap_frame, velodyne_frame,
                                             msg->header.stamp, velo_to_map_transform);
        ROS_INFO("%lu", msg->header.stamp.toNSec());
        available = true;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    t.push_back(std::clock());

    if (available) {
        // Update frames count
        cur_frame++;
        if (cur_frame < frames_to_skip){
            ROS_INFO("CURRENT FRAME: %d ======> SKIP", cur_frame);
            return;
        }
        else{
            ROS_INFO("CURRENT FRAME: %d ======> PROCEED", cur_frame);
        }

        //////////////////////////
        // Save transforms in .txt
        //////////////////////////
        if (save_velo_to_map)
            pose_file << msg->header.stamp.toNSec() << " " << velo_to_map_transform.getBasis().getRow(0).x() << " "
                      << velo_to_map_transform.getBasis().getRow(0).y() << " " <<
                      velo_to_map_transform.getBasis().getRow(0).z() << " "
                      << velo_to_map_transform.getBasis().getRow(1).x() << " " <<
                      velo_to_map_transform.getBasis().getRow(1).y() << " "
                      << velo_to_map_transform.getBasis().getRow(1).z() << " " <<
                      velo_to_map_transform.getBasis().getRow(2).x() << " "
                      << velo_to_map_transform.getBasis().getRow(2).y() << " " <<
                      velo_to_map_transform.getBasis().getRow(2).z() << " " << velo_to_map_transform.getOrigin().x()
                      << " " <<
                      velo_to_map_transform.getOrigin().y() << " " << velo_to_map_transform.getOrigin().z() << "\n";
        t.push_back(std::clock());

        //////////////////////////////
        // Read point cloud message //
        //////////////////////////////
        // Get the number of points
        size_t N = (size_t) (msg->width * msg->height);

        // Loop over points and copy in vector container. Do the filtering if necessary
        vector<PointXYZ> velo;
        velo.reserve(N);
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
             iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z) {
            // Add all points to the vector container
            velo.push_back(PointXYZ(*iter_x, *iter_y, *iter_z));
        }
        t.push_back(std::clock());

        // save out every frame of point cloud
        if (save_velo) {
            std::string save_str = configureDataPath("velo", msg->header.seq, msg->header.stamp.toNSec());
            save_cloud(save_str, velo);
        }
        t.push_back(std::clock());

        if (save_buffer || save_pointmap){
            ////////////////////////////////////////////
            // Filter out points that are too close   //
            ////////////////////////////////////////////
            auto keep_index = filter_pointcloud(velo, pow(point_distance_thresh, 2));

            //////////////////////////////////////////
            // Ground extraction on velodyne scan   //
            //////////////////////////////////////////

            vector<bool> ground_flag(velo.size(), false);
            extract_ground_himmelsbach(velo, ground_flag);
            int ground_count = 0;

            for (auto && i : ground_flag) {
                if (i) {
                    ground_count++;
                }
            }

//            // Publish ros messages
//            moveToPCLPtr(velo, latent_publish, ground_flag, false);
//            pcl::toROSMsg(*latent_publish, latent_msg);
//            latent_msg.header.frame_id = velodyne_frame;
//            latent_msg.header.stamp = msg->header.stamp;
//            latent_pub_.publish(latent_msg);

//            ROS_INFO("POINTS: %lu | GROUND COUNT: %d", velo.size(), ground_count);
            t.push_back(std::clock());

            // create a copy of the point cloud vector
            vector<PointXYZ> velo_transformed(velo);

            // Matrix for original/aligned data (Shallow copy of parts of the points vector)
            Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> velo_mat((float *) velo.data(), 3, N);
            Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> velo_transformed_mat((float *) velo_transformed.data(), 3, N);

            // Apply initial transformation
            //    transformFromTFTransform(velo_mat, velo_transformed_mat, velo_to_map_transform);
            Eigen::Affine3d eigenTr;
            tf::transformTFToEigen(velo_to_map_transform, eigenTr);
            Eigen::Matrix3f R_init = (eigenTr.matrix().block(0, 0, 3, 3)).cast<float>();
            Eigen::Vector3f T_init = (eigenTr.matrix().block(0, 3, 3, 1)).cast<float>();
            velo_transformed_mat = (R_init * velo_mat).colwise() + T_init;

            filter_pointcloud(velo_transformed, keep_index, 0.5);
            filter_by_value(ground_flag, keep_index, 0.5);

            t.push_back(std::clock());

            //////////////////////////////////////////
            // Preprocess frame and compute normals //
            //////////////////////////////////////////

            // Create a copy of points in polar coordinates
            vector<PointXYZ> polar_pts(velo_transformed);
            cart2pol_(polar_pts);

            // Get lidar angle resolution
            float minTheta, maxTheta;
            float lidar_angle_res = get_lidar_angle_res(polar_pts, minTheta, maxTheta, lidar_n_lines);

            // Define the polar neighbors radius in the scaled polar coordinates
            float polar_r = 1.5 * lidar_angle_res;

            // Apply log scale to radius coordinate (in place)
            lidar_log_radius(polar_pts, polar_r, (float) r_scale);

            t.push_back(std::clock());

            vector<PointXYZ> sub_pts;
            vector<size_t> sub_inds;
            grid_subsampling_centers(velo_transformed, sub_pts, sub_inds, (float) frame_voxel_size);

            t.push_back(std::clock());

            /////////////////////
            // Compute normals //
            /////////////////////

            // Init result containers
            vector<PointXYZ> normals(velo.size(), PointXYZ());
            vector<float> norm_scores(velo.size(), -1.0);

            // Apply horizontal scaling (to have smaller neighborhoods in horizontal direction)
            lidar_horizontal_scale(polar_pts, (float) h_scale);

//            // Call polar processing function
//            extract_lidar_frame_normals(velo_transformed, polar_pts, sub_inds, normals, norm_scores, polar_r);

//            vector<PointXYZ> full_normals;
//            vector<float> full_norm_scores;
//            vector<size_t> full_inds;
//            for (int i = 0; i < velo_transformed.size(); ++i){
//                full_inds.push_back(i);
//            }
//            extract_lidar_frame_normals(velo_transformed, polar_pts, full_inds, full_normals, full_norm_scores, polar_r);
//            std::string buffer_map_save_str = configureDataPath("example_frame_normals", msg->header.seq, msg->header.stamp.toNSec());
//            save_cloud(buffer_map_save_str, velo_transformed, full_normals);

            t.push_back(std::clock());

            // Update sub_ground_flag
            vector<bool> sub_ground_flag;
            for (unsigned long sub_ind : sub_inds){
                sub_ground_flag.push_back(ground_flag[sub_ind]);
            }

            /////////////////////////////
            // Save buffer point cloud //
            /////////////////////////////
            if (save_buffer) {
                buffer_map.update(sub_pts, normals, norm_scores, sub_ground_flag);

                // save out map
                std::string buffer_map_save_str = configureDataPath("buffer_map", msg->header.seq, msg->header.stamp.toNSec());
                vector<float> buffer_map_features;
                for (auto score:buffer_map.scores)
                    buffer_map_features.push_back(score);
                for (auto count:buffer_map.counts)
                    buffer_map_features.push_back(count);
                for (int i = 0; i < buffer_map.ground_scores.size(); ++i){
                    buffer_map_features.push_back(buffer_map.ground_scores[i] / buffer_map.counts[i]);
                }
                ROS_INFO("AFTER UPDATE BUFFER SIZE: %lu", buffer_map.cloud.pts.size());
                if (msg->header.stamp.toSec() >= last_frame_tsec || buffer_map.cloud.pts.size() >= save_every_npoints){
                    ROS_INFO("SAVING BUFFER AND CLEARING OLD MAP");
                    save_cloud(buffer_map_save_str, buffer_map.cloud.pts, buffer_map.normals, buffer_map_features);
//                    buffer_map.clear();
//                    new (&buffer_map) PointMap();
                }
            }
            t.push_back(std::clock());

            long int before_size = sub_pts.size();

//            // Remove points with a low score
//            filter_pointcloud(sub_pts, norm_scores, 0.1);
//            filter_pointcloud(normals, norm_scores, 0.1);
//            filter_by_value(sub_ground_flag, norm_scores, 0.1);
//            norm_scores.erase(remove_if(norm_scores.begin(), norm_scores.end(), [](const float s) { return s < 0.1; }),
//                              norm_scores.end());

            ROS_INFO("BEFORE FILTER: %lu | AFTER: %lu", before_size, sub_pts.size());
            t.push_back(std::clock());
            ////////////////
            // Add to Map //
            ////////////////

            // The update function is called only on subsampled points as the others have no normal
            if (save_pointmap) {
                map.update(sub_pts, normals, norm_scores, sub_ground_flag);

                // save out map
                std::string map_save_str = configureDataPath("map", msg->header.seq, msg->header.stamp.toNSec());
                vector<float> map_features;
                for (auto score:map.scores)
                    map_features.push_back(score);
                for (auto count:map.counts)
                    map_features.push_back(count);
                for (int i = 0; i < map.ground_scores.size(); ++i){
                    map_features.push_back(map.ground_scores[i] / map.counts[i]);
                }
                ROS_INFO("AFTER UPDATE MAP SIZE: %lu", map.cloud.pts.size());
                if (msg->header.stamp.toSec() >= last_frame_tsec || map.cloud.pts.size() >= save_every_npoints){
                    ROS_INFO("SAVING CLOUD AND CLEARING OLD MAP");
                    save_cloud(map_save_str, map.cloud.pts, map.normals, map_features);
//                    map.clear();
//                    new (&map) PointMap();
                }
            }
            t.push_back(std::clock());
        }
        else{
            ROS_INFO("Skip map creation...");
        }
    }
    else{
        ROS_INFO("Transform not ready yet...");
    }
    t.push_back(std::clock());

    ////////////////////////
    // Debugging messages //
    ////////////////////////

    if (show_debug_msg)
    {
        for (size_t i = 0; i < min(t.size() - 1, clock_str.size()); i++)
        {
            double duration = 1000 * (t[i + 1] - t[i]) / (double)CLOCKS_PER_SEC;
            cout << clock_str[i] << duration << " ms" << endl;
        }
        cout << endl
             << "***********************" << endl
             << endl;
    }
}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);
    nh_.param<bool>("show_debug_msg", show_debug_msg, true);

    nh_.param<int>("frames_to_skip", frames_to_skip, 1.0);
    nh_.param<float>("map_dl", map_dl, 1.0);

    nh_.param<double>("max_distance", max_distance, 1.0);
    nh_.param<double>("point_distance_thresh", point_distance_thresh, 1.0);

    nh_.param<bool>("save_velo", save_velo, true);
    nh_.param<bool>("save_velo_to_map", save_velo_to_map, true);
    nh_.param<bool>("save_pointmap", save_pointmap, true);
    nh_.param<bool>("save_buffer", save_buffer, true);
    nh_.param<double>("save_every_npoints", save_every_npoints, 1.0);

    nh_.param<double>("last_frame_tsec", last_frame_tsec, 1.0);

    nh_.param<int>("lidar_n_lines", lidar_n_lines, 1.0);
    nh_.param<double>("r_scale", r_scale, 1.0);
    nh_.param<double>("h_scale", h_scale, 1.0);
    nh_.param<double>("frame_voxel_size", frame_voxel_size, 1.0);

    nh_.param<float>("alpha", alpha, 1.0);
    nh_.param<float>("tolerance", tolerance, 1.0);
    nh_.param<float>("Tm", Tm, 1.0);
    nh_.param<float>("Tm_small", Tm_small, 1.0);
    nh_.param<float>("Tb", Tb, 1.0);
    nh_.param<float>("Trmse", Trmse, 1.0);
    nh_.param<float>("Tdprev", Tdprev, 1.0);
    nh_.param<float>("abs_z", abs_z, 1.0);

    nh_.param<int>("num_bins_small", num_bins_small, 1.0);
    nh_.param<int>("num_bins_large", num_bins_large, 1.0);
    nh_.param<float>("bin_size_small", bin_size_small, 1.0);
    nh_.param<float>("bin_size_large", bin_size_large, 1.0);
    nh_.param<float>("rmin", rmin, 1.0);
    nh_.param<float>("rmax", rmax, 1.0);

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

void velodyne_listener_class::extract_negative(std::vector<bool> &flag, std::vector<int> &indices) {
    std::sort(indices.begin(), indices.end());
    std::vector<int> in_order;
    for (uint i = 0; i < flag.size(); i++) {
        if (std::binary_search(indices.begin(), indices.end(), i)){
            in_order.push_back(i);
        }
    }
    std::sort(in_order.begin(), in_order.end());
    for (uint i = 0; i < in_order.size(); i++) {
        flag[in_order[i]] = true;
    }
}

vector<PointXYZ> velodyne_listener_class::extract_ground_himmelsbach(vector<PointXYZ> &cloud) {
    Himmelsbach himmelsbach(cloud);
    himmelsbach.set_alpha(alpha);
    himmelsbach.set_tolerance(tolerance);
    himmelsbach.set_thresholds(Tm, Tm_small, Tb, Trmse, Tdprev, abs_z);
    std::vector<int> inliers;
    himmelsbach.compute_model_and_get_inliers(inliers);
    vector<PointXYZ> ground = extract_negative(cloud, inliers);

    return ground;
}

void velodyne_listener_class::extract_ground_himmelsbach(vector<PointXYZ> &cloud, vector<bool> &flag) {
    Himmelsbach himmelsbach(cloud);
    himmelsbach.set_alpha(alpha);
    himmelsbach.set_tolerance(tolerance);
    himmelsbach.set_thresholds(Tm, Tm_small, Tb, Trmse, Tdprev, abs_z);
    himmelsbach.set_bins_config(num_bins_small, num_bins_large, bin_size_small, bin_size_large, rmin, rmax);
    std::vector<int> inliers;
    himmelsbach.compute_model_and_get_inliers(inliers);
    extract_negative(flag, inliers);
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

void velodyne_listener_class::moveToPCLPtr(vector<PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &PCL_cloud_ptr, vector<bool> &flag, bool update) {
    if (!update)
        PCL_cloud_ptr->clear();
    pcl::PointXYZ p;
    for (int i = 0; i < cloud.size(); ++i){
        if (flag[i]){
            p.x = cloud[i].x;
            p.y = cloud[i].y;
            p.z = cloud[i].z;
            PCL_cloud_ptr->push_back(p);
        }
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
