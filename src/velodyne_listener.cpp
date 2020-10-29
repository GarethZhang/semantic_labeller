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
    cur_sub_slam_map(new pcl::PointCloud<pcl::PointXYZI>),
    cur_sub_slam_map_undistort(new pcl::PointCloud<pcl::PointXYZI>),
    last_velo_undistort(new pcl::PointCloud<pcl::PointXYZI>),
    coefficients(new pcl::ModelCoefficients),
    velo_sub_(nh_, "/velodyne_points", 10),
    velo_undistort_sub_(nh_, "/lidar_keyframe_slam/velodyne_points_undistorted", 10),
    velo_time_seq(velo_sub_, ros::Duration(0.1), ros::Duration(0.01), 10),
    velo_undistort_time_seq(velo_undistort_sub_, ros::Duration(0.1), ros::Duration(0.01), 10)
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
//    velo_time_seq.registerCallback(&velodyne_listener_class::velodyneCallback, this);
    velo_undistort_time_seq.registerCallback(&velodyne_listener_class::velodyneUndistortCallback, this);

    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void velodyne_listener_class::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    velo_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velo_filtered", 1, true);
    velo_undistort_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velo_undistort_filtered", 1, true);
    latent_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velo_latent", 1, true); // latent one
    ground_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_plane", 1, true); // latent one
    dnm_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_and_moving", 1, true); // latent one
    dbm_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_not_moving", 1, true); // latent one
    static_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/static", 1, true); // latent one
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

    // remove points that have too low intensity
    pcl::PassThrough<pcl::PointXYZI> intensity_pass;
    intensity_pass.setInputCloud(point_cloud);
    intensity_pass.setFilterFieldName("intensity");
    intensity_pass.setFilterLimits(min_intensity, max_intensity);

    // update global variable to store latest submap
    *slam_map = *point_cloud;
    kdtree.setInputCloud (slam_map);
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
    if (!available){
        try{
            velo_to_map_listener.lookupTransform(slamMap_frame, velodyne_frame,
                                                msg->header.stamp, velo_to_map_transform);
            available = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
        }
    }

    // check if slam_map has got the full_map
    if (!slam_map->empty() && available){
        pcl_ros::transformPointCloud (*point_cloud, *point_cloud_out, velo_to_map_transform);
        point_cloud_out->header.frame_id = slamMap_frame;

        // Use radius search in KD-tree to eliminate other regions of slam map
        pcl::PointXYZI search_point;
        search_point.x = velo_to_map_transform.getOrigin().x();
        search_point.y = velo_to_map_transform.getOrigin().y();
        search_point.z = velo_to_map_transform.getOrigin().z();
        search_point.intensity = 0.0;

        double search_radius = 100.0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (slam_map);

        if (kdtree.radiusSearch(search_point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(pointIdxRadiusSearch);
            extract.setIndices(index_ptr);
            extract.setNegative (false);
            extract.filter (*cur_sub_slam_map);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
        velodyne_listener_class::PointCloudXYZItoXYZ(*point_cloud_out, *cloud_in);
        velodyne_listener_class::PointCloudXYZItoXYZ(*cur_sub_slam_map, *cloud_out);
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        est.setInputSource(cloud_in);
        est.setInputTarget(cloud_out);

        pcl::Correspondences all_correspondences;
        est.determineCorrespondences(all_correspondences, scan_to_map_max_d);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI p;
        int nr_correspondences = (int)all_correspondences.size();
        int index_query;
        for (int i = 0; i < nr_correspondences; ++i){
            index_query = all_correspondences[i].index_query;
            p.x = cloud_in->points[index_query].x;
            p.y = cloud_in->points[index_query].y;
            p.z = cloud_in->points[index_query].z;
            p.intensity = point_cloud_out->points[index_query].intensity;
            cloud_filtered->push_back(p);
        }

        // publish the ROS messages
        pcl::toROSMsg(*cloud_filtered, velo_filtered_msg);
        velo_filtered_msg.header.frame_id = "map";
        velo_filtered_msg.header.stamp = msg->header.stamp;
        velo_filtered_pub_.publish(velo_filtered_msg);

        // save out transformed point cloud named after the seq number
        if (save_to_ply){
            velo_seq_ = msg->header.seq;
            velo_ss_.str(""); velo_ss_.clear();
            velo_ss_ << save_dir << "velo_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
            velo_str_ = velo_ss_.str();
            ROS_INFO("SAVING VELO FILE");
            pcl::io::savePLYFileBinary(velo_str_, *point_cloud_out);

            kf_ref_seq_ = msg->header.seq;
            kf_ref_ss_.str(""); kf_ref_ss_.clear();
            kf_ref_ss_ << save_dir << "curSubSlamMap_" << kf_ref_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
            kf_ref_str_ = kf_ref_ss_.str();
            ROS_INFO("SAVING SLAM MAP FILE");
            pcl::io::savePLYFileBinary(kf_ref_str_, *cur_sub_slam_map);

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

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to pub_ (which is a member method)
void velodyne_listener_class::velodyneUndistortCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    // track runtime
//    ros::WallTime start_, end_;
//    start_ = ros::WallTime::now();

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
    if (!available){
        try{
            velo_undistort_to_map_listener.lookupTransform(slamMap_frame, velodyne_frame,
                                                           msg->header.stamp, velo_undistort_to_map_transform);
            available = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
        }
    }

    if (!slam_map->empty() && available){
        pcl_ros::transformPointCloud (*point_cloud, *point_cloud_out, velo_undistort_to_map_transform);
        point_cloud_out->header.frame_id = slamMap_frame;
        point_cloud_out->header.stamp = msg->header.stamp.toNSec();
        point_cloud_out->header.seq = msg->header.seq;

        if (last_velo_undistort->empty()){
            *last_velo_undistort = *point_cloud_out;
        }
        else{
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane_cand_points (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane_points (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr dyna_and_moving_points (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr dyna_no_moving_points (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr static_points (new pcl::PointCloud<pcl::PointXYZI>);

            pcl::PointCloud<pcl::PointXYZI>::Ptr residual_points (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr classified_points (new pcl::PointCloud<pcl::PointXYZI>);

            /////////////////////////
            // Ground plane detection
            /////////////////////////
            // Use radius search in KD-tree to eliminate other regions of slam map
            pcl::PointXYZI search_point;
            search_point.x = velo_undistort_to_map_transform.getOrigin().x();
            search_point.y = velo_undistort_to_map_transform.getOrigin().y();
            search_point.z = velo_undistort_to_map_transform.getOrigin().z();
            search_point.intensity = 0.0;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            pcl::ExtractIndices<pcl::PointXYZI> slam_map_extract;
            slam_map_extract.setInputCloud (slam_map);

            if (kdtree.radiusSearch(search_point, kd_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
            {
                boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(pointIdxRadiusSearch);
                slam_map_extract.setIndices(index_ptr);
                slam_map_extract.setNegative (false);
                slam_map_extract.filter (*cur_sub_slam_map_undistort);
            }

            // Estimate normal
            pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr velo_undistort_tree (new pcl::search::KdTree<pcl::PointXYZI> ());
            pcl::PointCloud<pcl::Normal>::Ptr velo_undistort_normals (new pcl::PointCloud<pcl::Normal>);
            ne.setSearchMethod (velo_undistort_tree);
            ne.setInputCloud (point_cloud_out);
//        ne.setKSearch (k_nearest);
            ne.setRadiusSearch(normal_radius);
            ne.compute (*velo_undistort_normals);

            // Use normals to extract ground plane points
            float angle_vertical_thresh = M_PI * normal_ang_thresh / 180.0;
            float cos_thresh = cos(angle_vertical_thresh);
            for (int normal_i = 0; normal_i < velo_undistort_normals->size(); ++normal_i){
                if (abs(velo_undistort_normals->points[normal_i].normal_z) < cos_thresh){
                    ground_plane_cand_points->push_back(point_cloud_out->points[normal_i]);
                }
            }

//        // Save out the normals
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_latent (new pcl::PointCloud<pcl::PointXYZ>);
//        velodyne_listener_class::PointCloudXYZItoXYZ(*point_cloud_out, *cloud_latent);
//        ROS_INFO("cloud_latent POINTS: %lu", cloud_latent->size());
//        ROS_INFO("velo_undistort_normals POINTS: %lu", velo_undistort_normals->size());
//        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//        pcl::concatenateFields(*cloud_latent, *velo_undistort_normals, *cloud_with_normals);
//        ROS_INFO("cloud_with_normals POINTS: %lu", cloud_with_normals->size());
//        velo_seq_ = msg->header.seq;
//        velo_ss_.str(""); velo_ss_.clear();
//        velo_ss_ << save_dir << "normal_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".pcd";
//        velo_str_ = velo_ss_.str();
//        pcl::io::savePCDFileASCII(velo_str_, *cloud_with_normals);

            // Estimate plane coefficients
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZI> seg;
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold (RANSAC_dist);

            seg.setInputCloud (ground_plane_cand_points);
            seg.setMaxIterations(RANSAC_iter);
            seg.segment (*inliers, *coefficients);

            // Pick points that do not fit in this ground plane
            double point_dist;
            for (auto& point :*point_cloud_out){
                point_dist = pcl::pointToPlaneDistanceSigned(point, coefficients->values[0], coefficients->values[1],
                                                             coefficients->values[2], coefficients->values[3]);
                if (point_dist >= dist_to_plane){
                    residual_points->push_back(point);
                }
                else{
                    ground_plane_points->push_back(point);
                }
            }

            ////////////////////////////
            // Dynamic and moving points
            ////////////////////////////

            // TODO add roslaunch param for distance
            pcl::Correspondences dnm_correspondences = find_correspondence(*residual_points, *last_velo_undistort, cons_scan_max_d);
            std::vector<int> dnm_index_query;
            for (auto & dnm_correspondence : dnm_correspondences)
                dnm_index_query.push_back(dnm_correspondence.index_query);
            *dyna_and_moving_points = extract_points_using_indices(residual_points, dnm_index_query, true);

//            kf_ref_seq_ = msg->header.seq;
//            kf_ref_ss_.str(""); kf_ref_ss_.clear();
//            kf_ref_ss_ << save_dir << "residual_" << kf_ref_seq_ << "_" << msg->header.stamp.toNSec() << ".pcd";
//            kf_ref_str_ = kf_ref_ss_.str();
//            ROS_INFO("SAVING RESIDUAL FILE");
//            pcl::io::savePCDFileASCII(kf_ref_str_, *residual_points);

            *residual_points = extract_points_using_indices(residual_points, dnm_index_query, false);

//            kf_ref_seq_ = msg->header.seq;
//            kf_ref_ss_.str(""); kf_ref_ss_.clear();
//            kf_ref_ss_ << save_dir << "last_velo_" << kf_ref_seq_ << "_" << msg->header.stamp.toNSec() << ".pcd";
//            kf_ref_str_ = kf_ref_ss_.str();
//            ROS_INFO("SAVING LAST VELO FILE");
//            pcl::io::savePCDFileASCII(kf_ref_str_, *last_velo_undistort);

            ////////////////////////////
            // Dynamic not moving points
            ////////////////////////////

            pcl::Correspondences dbm_correspondences = find_correspondence(*residual_points, *cur_sub_slam_map_undistort, scan_to_map_max_d);
            std::vector<int> dbm_index_query;
            for (auto & dbm_correspondence : dbm_correspondences)
                dbm_index_query.push_back(dbm_correspondence.index_query);
            *dyna_no_moving_points = extract_points_using_indices(residual_points, dbm_index_query, true);

            ////////////////
            // Static points
            ////////////////

            *static_points = extract_points_using_indices(residual_points, dbm_index_query, false);

            //////////////////
            // Join all points
            //////////////////
            add_to_points(ground_plane_points, classified_points, 63.0);
            add_to_points(dyna_and_moving_points, classified_points, 127.0);
            add_to_points(dyna_no_moving_points, classified_points, 191.0);
            add_to_points(static_points, classified_points, 255.0);
            ROS_INFO("GROUND PLANE POINTS: %lu | DYNA AND MOVING POINTS: %lu | DYNA NO MOVING POINTS: %lu | STATIC POINTS: %lu | TOTAL POINTS: %lu",
                     ground_plane_points->size(), dyna_and_moving_points->size(), dyna_no_moving_points->size(), static_points->size(), classified_points->size());

            // publish the latent ROS messages
            pcl::toROSMsg(*classified_points, latent_msg);
            latent_msg.header.frame_id = "map";
            latent_msg.header.stamp = msg->header.stamp;
            latent_pub_.publish(latent_msg);

            pcl::toROSMsg(*ground_plane_points, ground_plane_msg);
            ground_plane_msg.header.frame_id = "map";
            ground_plane_msg.header.stamp = msg->header.stamp;
            ground_plane_pub_.publish(ground_plane_msg);

            pcl::toROSMsg(*dyna_and_moving_points, dnm_msg);
            dnm_msg.header.frame_id = "map";
            dnm_msg.header.stamp = msg->header.stamp;
            dnm_pub_.publish(dnm_msg);

            pcl::toROSMsg(*dyna_no_moving_points, dbm_msg);
            dbm_msg.header.frame_id = "map";
            dbm_msg.header.stamp = msg->header.stamp;
            dbm_pub_.publish(dbm_msg);

            pcl::toROSMsg(*static_points, static_msg);
            static_msg.header.frame_id = "map";
            static_msg.header.stamp = msg->header.stamp;
            static_pub_.publish(static_msg);

//            // publish the ROS messages
//            pcl::toROSMsg(*static_points, velo_undistort_filtered_msg);
//            velo_undistort_filtered_msg.header.frame_id = "map";
//            velo_undistort_filtered_msg.header.stamp = msg->header.stamp;
//            velo_undistort_filtered_pub_.publish(velo_undistort_filtered_msg);

            // save out transformed point cloud named after the seq number
            if (save_to_ply){
                velo_seq_ = msg->header.seq;
                velo_ss_.str(""); velo_ss_.clear();
                velo_ss_ << save_dir << "velo_undistort_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
                velo_str_ = velo_ss_.str();
                ROS_INFO("SAVING VELO UNDISTORT FILE");
                pcl::io::savePLYFileBinary(velo_str_, *point_cloud_out);

//                kf_ref_seq_ = msg->header.seq;
//                kf_ref_ss_.str(""); kf_ref_ss_.clear();
//                kf_ref_ss_ << save_dir << "velo_undistort_no_ground_" << kf_ref_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
//                kf_ref_str_ = kf_ref_ss_.str();
//                ROS_INFO("SAVING VELO UNDISTORT NO GROUND FILE");
//                pcl::io::savePLYFileBinary(kf_ref_str_, *point_cloud_no_ground);

                velo_filtered_ss_.str(""); velo_filtered_ss_.clear();
                velo_filtered_ss_ << save_dir << "velo_undistort_filtered_" << velo_seq_ << "_" << msg->header.stamp.toNSec() << ".ply";
                velo_filtere_str_ = velo_filtered_ss_.str();
                ROS_INFO("SAVING VELO UNDISTORT FILTERED FILE");
                pcl::io::savePLYFileBinary(velo_filtere_str_, *static_points);
            }

            //////////////////////////
            // Update global variables
            //////////////////////////
            *last_velo_undistort = *point_cloud_out;
        }
    }
    else{
        if (slam_map->empty())
            ROS_INFO("SUBMAP EMPTY!");
    }
}

void velodyne_listener_class::getParams() {
    nh_.param<std::string>("save_dir", save_dir, "/home/haowei/Desktop/");
    nh_.param<bool>("save_to_ply", save_to_ply, true);

    nh_.param<double>("cons_scan_max_d", cons_scan_max_d, 1.0);
    nh_.param<double>("scan_to_map_max_d", scan_to_map_max_d, 1.0);
    nh_.param<float>("min_intensity", min_intensity, 0.0);
    nh_.param<float>("max_intensity", max_intensity, 255.0);
    nh_.param<double>("RANSAC_dist", RANSAC_dist, 1.0);
    nh_.param<int>("RANSAC_iter", RANSAC_iter, 1000.0);
    nh_.param<double>("normal_radius", normal_radius, 1.0);
    nh_.param<double>("normal_ang_thresh", normal_ang_thresh, 30.0);
    nh_.param<double>("dist_to_plane", dist_to_plane, 30.0);
    nh_.param<int>("k_nearest", k_nearest, 30);
    nh_.param<double>("kd_search_radius", kd_search_radius, 100.0);

    nh_.param<std::string>("submap_frame", submap_frame, "/kf_ref");
    nh_.param<std::string>("submap_topic", submap_topic, "/lidar_keyframe_slam/kfRef_cloud");
    nh_.param<std::string>("slamMap_frame", slamMap_frame, "/map");
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

pcl::Correspondences
velodyne_listener_class::find_correspondence(pcl::PointCloud<pcl::PointXYZI> &cloud_src, pcl::PointCloud<pcl::PointXYZI> &cloud_tgt,
                                             double distance) {
    pcl::Correspondences all_correspondences;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    velodyne_listener_class::PointCloudXYZItoXYZ(cloud_src, *cloud_in);
    velodyne_listener_class::PointCloudXYZItoXYZ(cloud_tgt, *cloud_out);
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
    est.setInputSource(cloud_in);
    est.setInputTarget(cloud_out);
    est.determineCorrespondences(all_correspondences, distance);

    return all_correspondences;
}

pcl::PointCloud<pcl::PointXYZI> &
velodyne_listener_class::extract_points_using_indices(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::vector<int> index_query,
                                                      bool setNegative) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (cloud);

    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index_query);
    extract.setIndices(index_ptr);
    extract.setNegative (setNegative);
    extract.filter (*cloud_out);
    return *cloud_out;
}

void velodyne_listener_class::add_to_points(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out,
                                            float intensity_value) {
    for (auto& point: *cloud_in){
        pcl::PointXYZI p;
        p.x = point.x; p.y = point.y; p.z = point.z; p.intensity = intensity_value;
        cloud_out->push_back(p);
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
