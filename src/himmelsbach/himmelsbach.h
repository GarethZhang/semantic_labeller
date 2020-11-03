//
// Created by haowei on 2020-10-31.
//

#ifndef SEMANTIC_LABELLER_HIMMELSBACH_H
#define SEMANTIC_LABELLER_HIMMELSBACH_H


#include "../cloud/cloud.h"
#include "Eigen/Geometry"

struct Line {
    float m;
    float b;
    float start;
    float end;
    Line(std::vector<PointXYZ> pc, std::vector<int> line_set, float m_, float b_);
};

//* Himmelsbach
/**
* \brief This class uses the (Himmelsbach, 2010) algo to compute a ground plane and get the inliers to that model.
*/
class Himmelsbach {
public:
    explicit Himmelsbach(std::vector<PointXYZ> pc_) : pc(pc_) {}
    void set_alpha(float alpha_) {alpha = alpha_;}
    void set_tolerance(float tolerance_) {tolerance = tolerance_;}
    void set_thresholds(float Tm_, float Tm_small_, float Tb_, float Trmse_, float Tdprev_) {
        Tm = Tm_;
        Tm_small = Tm_small_;
        Tb = Tb_;
        Trmse = Trmse_;
        Tdprev = Tdprev_;
    }

    /*!
       \brief Computes the ground plane model using the Himmelsbach algo and returns the inliers to that model.
       \param inliers [out] The parameter is filled with a vector of indices which correspond to the ground points.
    */
    void compute_model_and_get_inliers(std::vector<int> &inliers);

private:
    std::vector<PointXYZ> pc;
    float alpha = 5 * M_PI / 180;       /*!< Number of segments = (2 * M_PI) / alpha */
    int num_bins_small = 120;
    int num_bins_large = 54;
    float bin_size_small = 0.1;
    float bin_size_large = 2.0;
    float rmin = 2.5;
    float rmax = 119.5;
    float tolerance = 0.25;         /*!< Metric distance from the ground plane model to be considered a ground point */
    float Tm = 15 * M_PI / 180;         /*!< Slopes greater than this value will be considered non-ground. */
    float Tm_small = 5 * M_PI / 180;    /*!< Slopes less than this value will be checked for being a plateau */
    float Tb = -1.5;                /*!< Flat regions that are higher than this value will be considered non-ground */
    float Trmse = 0.1;                  /*!< If the RSME of the line fit exceeds this value, it will be rejected */
    float Tdprev = 0.25;                /*!< Maximum allowed distance between previous line and start of new line */

    /*!
       \brief Sorts points into one of N = (2 * pi) / alpha segments.
       \param segments [out] This parameter will be filled with a vector of segments, each segment consists of a
           vector of points indices.
       \post If a segment does not contain a point, it will be set to an empty vector.
    */
    void sort_points_segments(std::vector<std::vector<int>> &segments);

    /*!
       \brief Sorts points into bins and extracts a representative point for each bin.
       The point with the lowest z-value in each bin becomes the representative point.
       \param segment A vector of point indices for the current segment.
       \param bins [out] This parameter will be set to a vector of indices corresponding to the representative point
            for each bin.
       \post If a bin does not contain a point, the index will be set to -1.
    */
    void sort_points_bins(std::vector<int> segment, std::vector<int> &bins);

    /*!
       \brief Fits a line to a vector of points using Eigen's linear solver.
       \param line_set A vector of point indices which will be used to fit a line.
       \param m [out] The slope of the output line. z = m*r +b
       \param b [out] The z-intercept of the output line. z = m*r + b
    */
    float fitline(std::vector<int> line_set, float&m, float&b);
    float fitline(std::vector<int> line_set, int idx, float&m, float&b);

    /*!
       \brief Computes the distance from a point to a line.
       \param line A line object (z = m*r + b)
       \param idx the index of the point.
       \return returns the Euclidean distance from the point to the line.
    */
    float distpointline(Line line, int idx);
};


#endif //SEMANTIC_LABELLER_HIMMELSBACH_H
