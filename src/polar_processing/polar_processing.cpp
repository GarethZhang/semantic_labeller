
#include "polar_processing.h"


void cart2pol_(vector<PointXYZ>& xyz)
{
	// In place modification to carthesian coordinates
	for (auto& p : xyz)
	{

		float rho = sqrt(p.sq_norm());
		float phi = atan2(p.y, p.x);
		float theta = atan2(sqrt(p.x * p.x + p.y * p.y), p.z);
		p.x = rho;
		p.y = theta;
		p.z = phi + M_PI / 2;
	}
}



void pca_features(vector<PointXYZ>& points,
	vector<float>& eigenvalues,
	vector<PointXYZ>& eigenvectors)
{
	// Safe check
	if (points.size() < 4)
		return;


	// Compute PCA
	PointXYZ mean = accumulate(points.begin(), points.end(), PointXYZ());
	mean = mean * (1.0 / points.size());

	// Create centralized data
	for (auto& p : points)
		p -= mean;

	// Create a N by 3 matrix containing the points (same data in memory)
	Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> X_c((float*)points.data(), 3, points.size());

	// Compute covariance matrix
	Eigen::Matrix3f cov(X_c * X_c.transpose() / points.size());

	// Compute pca
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
	es.compute(cov);

	// Convert back to std containers
	eigenvalues = vector<float>(es.eigenvalues().data(), es.eigenvalues().data() + es.eigenvalues().size());
	eigenvectors = vector<PointXYZ>((PointXYZ*)es.eigenvectors().data(), (PointXYZ*)es.eigenvectors().data() + es.eigenvectors().rows());
}


void detect_outliers(vector<PointXYZ>& rtp,
	vector<float>& scores,
	int lidar_n_lines,
	float lidar_angle_res,
	float minTheta,
	int n_pass,
	float threshold)
{
	float theta0 = minTheta - 0.5 * lidar_angle_res;

	// Follow each scan line variables
	for (int pass = 0; pass < n_pass; pass++)
	{
		vector<float> last_r(lidar_n_lines, 0.0);
		vector<size_t> last_i(lidar_n_lines, 0);

		size_t i = 0;
		size_t l = 0;
		for (auto& p : rtp)
		{
			if (scores[i] > -0.5)
			{
				// Get line index
				l = (size_t)floor((p.y - theta0) / lidar_angle_res);

				// Get jumps
				float dl1 = p.x - last_r[l];

				// eliminate jumps 
				if (abs(dl1) > threshold)
				{
					scores[last_i[l]] = -1.0;
					scores[i] = -1.0;
				}

				// Update saved variable
				last_r[l] = p.x;
				last_i[l] = i;
			}
			i++;
		}
	}
}


float get_lidar_angle_res(vector<PointXYZ>& rtp, float& minTheta, float& maxTheta, int lidar_n_lines)
{
	// Find lidar angle resolution automatically
	minTheta = 1000.0f;
	maxTheta = -1000.0f;
	for (size_t i = 1; i < rtp.size() - 1; i++)
	{
		if (rtp[i].y < minTheta)
			minTheta = rtp[i].y;
		if (rtp[i].y > maxTheta)
			maxTheta = rtp[i].y;
	}

	// Get line of scan inds
	return (maxTheta - minTheta) / (float)(lidar_n_lines - 1);
}


void lidar_log_radius(vector<PointXYZ>& rtp, float polar_r, float r_scale)
{
	float r_factor = polar_r / (r_scale * log((1 + polar_r) / (1 - polar_r)));
	for (auto& p : rtp)
		p.x = log(p.x) * r_factor;
}


void lidar_horizontal_scale(vector<PointXYZ>& rtp, float h_scale)
{
	float h_factor = 1 / h_scale;
	for (auto& p : rtp)
		p.z *= h_factor;
}


void extract_features_multi_thread(vector<PointXYZ>& points,
	vector<PointXYZ>& normals,
	vector<float>& planarity,
	vector<float>& linearity,
	int lidar_n_lines,
	float h_scale,
	float r_scale,
	int verbose)
{
	// Initialize variables
	// ********************

	// Number of points
	size_t N = points.size();

	// Result vectors
	normals = vector<PointXYZ>(points.size(), PointXYZ());
	planarity = vector<float>(points.size(), 0.0);
	linearity = vector<float>(points.size(), 0.0);

	// Cloud variable for KDTree
	PointCloud polar_cloud;
	polar_cloud.pts = vector<PointXYZ>(points);

	// Convert points to polar coordinates
	// ***********************************

	// In place modification of the data
	cart2pol_(polar_cloud.pts);

	// Find lidar angle resolution automatically
	float minTheta, maxTheta;
	float lidar_angle_res = get_lidar_angle_res(polar_cloud.pts, minTheta, maxTheta, lidar_n_lines);

	// Apply scaling
	// *************

	// Define search radius in chebichev metric. Vertical angular resolution of HDL32 is 1.33
	// lidar_angle_res = 1.33 * np.pi / 180;
	float polar_r = 1.5 * lidar_angle_res;

	// Apply horizontal and range scales 
	// h_scale = 0.5 (smaller distance for the neighbor in horizontal direction)
	// r_scale = 4.0 (larger dist for neighbors in radius direction). Use log of range so that neighbor radius is proportional to the range
	float h_factor = 1 / h_scale;
	float r_factor = polar_r / (log((1 + polar_r) / (1 - polar_r)) * r_scale);
	for (auto& p : polar_cloud.pts)
	{
		p.z *= h_factor;
		p.x = log(p.x) * r_factor;
	}
	float r2 = polar_r * polar_r;


	// Outlier detection
	// *****************

	detect_outliers(polar_cloud.pts, planarity, lidar_n_lines, lidar_angle_res, minTheta, 2, 0.003);


	// Create KD Tree to search for neighbors
	// **************************************

	// Tree parameters
	nanoflann::KDTreeSingleIndexAdaptorParams tree_params(10 /* max leaf */);

	// Pointer to trees
	PointXYZ_KDTree* index;

	// Build KDTree for the first batch element
	index = new PointXYZ_KDTree(3, polar_cloud, tree_params);
	index->buildIndex();

	// Create search parameters
	nanoflann::SearchParams search_params;
	search_params.sorted = false;

	// Find neighbors and compute features
	// ***********************************

	// Variable for reserving memory
	size_t max_neighbs = 10;

	// Get all features in a parallel loop
// #pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) num_threads(n_thread)
	for (size_t i = 0; i < N; i++)
	{
		if (planarity[i] < -0.5)
		{
			linearity[i] = -1.0f;
			continue;
		}
		vector<pair<size_t, float>> inds_dists;

		// Initial guess of neighbors size
		inds_dists.reserve(max_neighbs);

		// Find neighbors
		float query_pt[3] = { polar_cloud.pts[i].x, polar_cloud.pts[i].y, polar_cloud.pts[i].z };
		size_t n_neighbs = index->radiusSearch(query_pt, r2, inds_dists, search_params);

		// Update max count
		if (n_neighbs > max_neighbs)
		{
			// #pragma omp atomic
			max_neighbs = n_neighbs;
		}

		// Create Eigen matrix of neighbors (we use Eigen for PCA)
		vector<PointXYZ> neighbors;
		neighbors.reserve(n_neighbs);
		for (size_t j = 0; j < n_neighbs; j++)
			neighbors.push_back(points[inds_dists[j].first]);

		// Compute PCA
		vector<float> eigenvalues;
		vector<PointXYZ> eigenvectors;
		pca_features(neighbors, eigenvalues, eigenvectors);

		// Compute normals and score
		if (eigenvalues.size() < 3)
		{
			planarity[i] = -1.0f;
			linearity[i] = -1.0f;
		}
		else
		{
			// Score is 1 - sphericity equivalent to planarity + linearity
			planarity[i] = (eigenvalues[1] - eigenvalues[0]) / (eigenvalues[2] + 1e-9);
			linearity[i] = 1.0f - eigenvalues[1] / (eigenvalues[2] + 1e-9);
			if (eigenvectors[0].dot(points[i]) > 0)
				normals[i] = eigenvectors[0] * -1.0;
			else
				normals[i] = eigenvectors[0];

		}
	}
}



void compare_map_to_frame(vector<PointXYZ>& polar_frame,
	vector<PointXYZ>& polar_map,
	float polar_dl,
	float dr_threshold,
	vector<float>& map_frame_diff,
	vector<int>& valid_mask)
{
	////////////////
	// Parameters //
	////////////////

	float inv_dl = 1.0 / polar_dl;


	///////////////////////////////////////////
	// Get frame smallest radiuses in a grid //
	///////////////////////////////////////////

	// Get grid limits
	PointXYZ minCorner = min_point(polar_frame);
	PointXYZ maxCorner = max_point(polar_frame);
	PointXYZ originCorner = minCorner - PointXYZ(1, 1, 1) * (0.5 * polar_dl);

	// Dimensions of the grid
	size_t grid_n_theta = (size_t)floor((maxCorner.y - originCorner.y) / polar_dl) + 1;
	size_t grid_n_phi = (size_t)floor((maxCorner.z - originCorner.z) / polar_dl) + 1;

	// Initialize variables
	vector<float> grid_radiuses(grid_n_theta * grid_n_phi, -1.0);
	size_t i_theta, i_phi, gridIdx;

	// Compute frame grid indices
	for (auto& p : polar_frame)
	{
		// Position of point in grid
		i_theta = (size_t)floor((p.y - originCorner.y) * inv_dl);
		i_phi = (size_t)floor((p.z - originCorner.z) * inv_dl);
		gridIdx = i_theta + grid_n_theta * i_phi;

		// Update the radius in cell
		if (grid_radiuses[gridIdx] < 0)
			grid_radiuses[gridIdx] = p.x;
		else if (p.x < grid_radiuses[gridIdx])
			grid_radiuses[gridIdx] = p.x;
	}


	/////////////////////////////
	// Compare with map points //
	/////////////////////////////

	// Initialize variables
	size_t i = 0;
	valid_mask = vector<int>(polar_map.size(), 0);
	map_frame_diff = vector<float>(polar_map.size(), 0.0);

	// Compute frame grid indices
	for (auto& p : polar_map)
	{
		// Position of point in grid
		i_theta = (size_t)floor((p.y - originCorner.y) * inv_dl);
		i_phi = (size_t)floor((p.z - originCorner.z) * inv_dl);
		gridIdx = i_theta + grid_n_theta * i_phi;

		// Update the validity and difference
		if (grid_radiuses[gridIdx] > 0)
		{
			float diff = grid_radiuses[gridIdx] - p.x;
			if (diff > - dr_threshold)
			{
				valid_mask[i] = 1;
				map_frame_diff[i] = diff;
			}
		}
		i++;
	}










}


void extract_lidar_frame_normals(vector<PointXYZ>& points,
	vector<PointXYZ>& polar_pts,
	vector<size_t>& query_inds,
	vector<PointXYZ>& normals,
	vector<float>& norm_scores,
	float polar_r)
{

	// Initialize variables
	// ********************

	// Squared search radius
	float r2 = polar_r * polar_r;

	// Result vectors
	normals = vector<PointXYZ>(query_inds.size(), PointXYZ());
	norm_scores = vector<float>(query_inds.size(), 0.0);

	// Cloud variable for KDTree
	PointCloud polar_cloud;
	polar_cloud.pts = polar_pts;

	// Create KD Tree to search for neighbors
	// **************************************

	// Tree parameters
	nanoflann::KDTreeSingleIndexAdaptorParams tree_params(10 /* max leaf */);

	// Pointer to trees
	PointXYZ_KDTree* index;

	// Build KDTree for the first batch element
	index = new PointXYZ_KDTree(3, polar_cloud, tree_params);
	index->buildIndex();

	// Create search parameters
	nanoflann::SearchParams search_params;
	search_params.sorted = false;


	// Find neighbors and compute features
	// ***********************************

	// Variable for reserving memory
	size_t max_neighbs = 10;

	// Get all features in a parallel loop
// #pragma omp parallel for shared(max_neighbs) schedule(dynamic, 10) num_threads(n_thread)
	for (size_t i = 0; i < query_inds.size(); i++)
	{
		// Initial guess of neighbors size
		vector<pair<size_t, float>> inds_dists;
		inds_dists.reserve(max_neighbs);

		// Find neighbors
		size_t query_ind = query_inds[i];
		float query_pt[3] = { polar_pts[query_ind].x, polar_pts[query_ind].y, polar_pts[query_ind].z };
		size_t n_neighbs = index->radiusSearch(query_pt, r2, inds_dists, search_params);

		// Update max count
		if (n_neighbs > max_neighbs)
		{
			// #pragma omp atomic
			max_neighbs = n_neighbs;
		}

		// Create a vector of the neighbors in carthesian coordinates
		vector<PointXYZ> neighbors;
		neighbors.reserve(n_neighbs);
		for (size_t j = 0; j < n_neighbs; j++)
			neighbors.push_back(points[inds_dists[j].first]);

		// Compute PCA
		vector<float> eigenvalues;
		vector<PointXYZ> eigenvectors;
		pca_features(neighbors, eigenvalues, eigenvectors);

		// Compute normals and score
		if (eigenvalues.size() < 3)
		{
			norm_scores[i] = -1.0f;
		}
		else
		{
			// Score is 1 - sphericity equivalent to planarity + linearity
			norm_scores[i] = 1.0f - eigenvalues[0] / (eigenvalues[2] + 1e-9);

			// Orient normal so that it always faces lidar origin
			if (eigenvectors[0].dot(points[i]) > 0)
				normals[i] = eigenvectors[0] * -1.0;
			else
				normals[i] = eigenvectors[0];

		}
	}
}














