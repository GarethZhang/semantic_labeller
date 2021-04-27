#pragma once

#include <set>
#include <cstdint>
#include <cstdio>
#include <ctime>

#define _USE_MATH_DEFINES
#include <math.h>

#include "nanoflann/nanoflann.hpp"
#include <Eigen/Eigenvalues>

#include "../cloud/cloud.h"

using namespace std;


// KDTree type definition
typedef nanoflann::KDTreeSingleIndexAdaptor< nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3> PointXYZ_KDTree;

void cart2pol_(vector<PointXYZ>& xyz);


void pca_features(vector<PointXYZ>& points,
	vector<float>& eigenvalues,
	vector<PointXYZ>& eigenvectors);


void detect_outliers(vector<PointXYZ>& rtp,
	vector<float>& scores,
	int lidar_n_lines,
	float lidar_angle_res,
	float minTheta,
	int n_pass,
	float threshold);


float get_lidar_angle_res(vector<PointXYZ>& rtp, float& minTheta, float& maxTheta, int lidar_n_lines);


void lidar_log_radius(vector<PointXYZ>& rtp, float polar_r, float r_scale);


void lidar_horizontal_scale(vector<PointXYZ>& rtp, float h_scale);


void extract_features_multi_thread(vector<PointXYZ>& points,
	vector<PointXYZ>& normals,
	vector<float>& planarity,
	vector<float>& linearity,
	int lidar_n_lines,
	float h_scale,
	float r_scale,
	int verbose);


void compare_map_to_frame(vector<PointXYZ>& polar_frame,
	vector<PointXYZ>& polar_map,
	float polar_dl,
	float dr_threshold,
	vector<float>& map_frame_diff,
	vector<int>& valid_mask);


void extract_lidar_frame_normals(vector<PointXYZ>& points,
	vector<PointXYZ>& polar_pts,
	vector<size_t>& query_inds,
	vector<PointXYZ>& normals,
	vector<float>& norm_scores,
	float polar_r);

