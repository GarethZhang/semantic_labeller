//
//
//		0==========================0
//		|    Local feature test    |
//		0==========================0
//
//		version 1.0 : 
//			> 
//
//---------------------------------------------------
//
//		Cloud source :
//		Define usefull Functions/Methods
//
//----------------------------------------------------
//
//		Hugues THOMAS - 10/02/2017
//


#include "himmelsbach.h"


Line::Line(PointCloudXYZPtr pc, std::vector<int> line_set, float m_, float b_) {
    int idx = line_set[0];
    start = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    idx = line_set[line_set.size() - 1];
    end = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    m = m_;
    b = b_;
}

void Himmelsbach::compute_model_and_get_inliers(std::vector<int> &inliers) {
    if (pc->size() == 0)
        return;
    // Sort points into segments
    std::vector<std::vector<int>> segments;
    sort_points_segments(segments);
    // For each segment, extract piecewise line and assign ground labels
    for (std::vector<int> segment : segments) {
        // Sort points into bins
        if (segment.size() == 0)
            continue;
        std::vector<int> bins;
        sort_points_bins(segment, bins);
        std::vector<Line> lines;
        std::vector<int> line_set;
        int c = 0;
        int i = 0;
        for (int idx : bins) {
            if (idx < 0)
                continue;
            float m = 1.0;
            float b = 0.0;
            if (line_set.size() >= 2) {
                float rmse = fitline(line_set, idx, m, b);
                if (fabs(m) <= Tm && (fabs(m) > Tm_small || fabs(b) <= Tb) && rmse <= Trmse) {
                    line_set.push_back(idx);
                } else {
                    fitline(line_set, m, b);
                    lines.push_back(Line(pc, line_set, m, b));
                    c++;
                    line_set.clear();
                    i--;
                }
            } else {
                float dprev = 10000;
                if (lines.size() > 0 && (c - 1) >= 0)
                    dprev = distpointline(lines[c - 1], idx);
                if (dprev <= Tdprev || c == 0 || line_set.size() != 0)
                    line_set.push_back(idx);
            }
        }
        // Assign points as inliers if they are within a treshold of the ground model
        for (int idx : segment) {
            float r = sqrt(pow(pc->points[i].x, 2) + pow(pc->points[i].y, 2));
            // get line that's closest to the candidate point based on distance to endpoints
            int closest = -1;
            float dmin = 10000;
            for (uint i = 0; i < lines.size(); i++) {
                float d1 = fabs(lines[i].start - r);
                float d2 = fabs(lines[i].end - r);
                if (d1 < dmin || d2 < dmin) {
                    dmin = std::min(d1, d2);
                    closest = i;
                }
            }
            if (closest >= 0) {
                if (distpointline(lines[closest], idx) < tolerance)
                    inliers.push_back(idx);
            }
        }
    }
}

void Himmelsbach::sort_points_segments(std::vector<std::vector<int>> &segments) {
    segments.clear();
    int num_segments = ceil((2 * M_PI) / alpha);
    std::vector<int> dummy;
    for (int i = 0; i < num_segments; i++) {
        segments.push_back(dummy);
    }
    for (uint i = 0; i < pc->size(); i++) {
        float angle = atan2(pc->points[i].y, pc->points[i].x);
        if (angle < 0)
            angle = angle + 2 * M_PI;
        int segment = int(angle / alpha);
        segments[segment].push_back(i);
    }
}

void Himmelsbach::sort_points_bins(std::vector<int> segment, std::vector<int> &bins_out) {
    std::vector<std::vector<int>> bins;
    std::vector<int> dummy;
    for (int i = 0; i < (num_bins_small + num_bins_large); i++) {
        bins.push_back(dummy);
    }
    float rsmall = rmin + bin_size_small * num_bins_small;
    for (int idx : segment) {
        float r = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
        int bin = -1;
        if (rmin <= r && r < rsmall)
            bin = (r - rmin) / bin_size_small;
        if (rsmall <= r && r < rmax)
            bin = num_bins_small + (r - rsmall) / bin_size_large;
        if (bin >= 0)
            bins[bin].push_back(idx);
    }
    // The point with the lowest z-coordinate in each bin becomes the representative point
    bins_out = std::vector<int>(num_bins_small + num_bins_large, -1);
    int i = 0;
    for (std::vector<int> bin_points : bins) {
        float zmin = 10000;
        int lowest = -1;
        for (int idx : bin_points) {
            if (pc->points[idx].z < zmin) {
                zmin = pc->points[idx].z;
                lowest = idx;
            }
        }
        bins_out[i] = lowest;
        i++;
    }
}

float Himmelsbach::fitline(std::vector<int> line_set, float &m, float &b) {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(line_set.size(), 2);
    Eigen::VectorXf B = Eigen::VectorXf::Zero(line_set.size());
    for (uint i = 0; i < line_set.size(); i++) {
        int idx = line_set[i];
        A(i, 0) = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
        A(i, 1) = 1;
        B(i) = pc->points[idx].z;
    }
    Eigen::VectorXf x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    m = x(0);
    b = x(1);
    Eigen::VectorXf error = A * x - B;
    return sqrt(error.dot(error) / float(line_set.size()));
}

float Himmelsbach::fitline(std::vector<int> line_set, int idx, float &m, float &b) {
    line_set.push_back(idx);
    return fitline(line_set, m, b);
}

float Himmelsbach::distpointline(Line line, int idx) {
    float r = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    return fabs(pc->points[idx].z - line.m * r - line.b) / sqrt(1 + pow(line.m, 2));
}


