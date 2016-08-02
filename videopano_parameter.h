#ifndef ADOBE_AGT_VIDEOPANO_VIDEOPANO_PARAMETER_H
#define ADOBE_AGT_VIDEOPANO_VIDEOPANO_PARAMETER_H

#include <cstddef>
#include <cmath>

namespace adobe_agt {
namespace videopano {

struct videopano_parameter {
    // Base parameters
    int lm_log_status;                      // Control how many messages are emitted from various nonlinear and SBA solvers
    std::size_t trajectory_min_duration;    // Threshold on minimum trajactory durations
    std::size_t ransac_max_trials;          // Maximum number of trials for all the RANSAC-based algorithms
    std::size_t keyframe_max_iterations;    // Maximum number of iterations used by the nonlinear keyframe reconstruction algorithm
    std::size_t nonkeyframe_max_iterations; // Maximum number of iterations used by the nonlinear non-keyframe reconstruction algorithm
    std::size_t twoview_sba_max_iterations; // Maximum number of iterations in the two-view SBA
    std::size_t nview_sba_max_iterations;   // Maximum number of iterations in the Euclidean multi-view SBA
    double ransac_confidence;     // [ 0,1 ] confidence for all the RANSAC-based algorithms
    double robust_sigma_ratio;    // Ratio for computing robust_sigma
    double ransac_ratio;          // Ratio for computing various ransac ratios
    double twoview_ransac_ratio;  // Ratio for computing twoview_ransac_threshold
    double keyframe_ransac_ratio; // Ratio for computing keyframe_ransac_threshold
    double outlier_ratio;         // Ratio for computing various outlier ratios
    double twoview_outlier_ratio; // Ratio for computing twoview_outlier_threshold
    double nview_outlier_ratio;   // Ratio for computing nview_outlier_threshold

    // Derived parameters
    std::size_t keyframe_frequency;   // Keyframe frequency
    double robust_sigma;              // Parameter for robust cost functions
    double twoview_ransac_threshold;  // Threshold for determining inlier/outliers in the two-view RANSAC algorithm
    double keyframe_ransac_threshold; // Threshold for determining inlier/outliers in the keyframe RANSAC algorithm
    double twoview_outlier_threshold; // Threshold for determining outliers in two-view reconstruction
    double nview_outlier_threshold;   // Threshold for determining outliers after multi-view reconstruction

    videopano_parameter( ) {
        set_default_base_parameters( );
    }

    void set_default_base_parameters( ) {
        lm_log_status = 1;           // Print initial and final errors along with final messages and errors ( if any )
        trajectory_min_duration = 5;
        ransac_max_trials = 1000;
        keyframe_max_iterations = 20;
        nonkeyframe_max_iterations = 20;
        twoview_sba_max_iterations = 20;
        nview_sba_max_iterations = 20;
        ransac_confidence = 0.995;
        robust_sigma_ratio = 0.005;
        ransac_ratio = 0.004;
        twoview_ransac_ratio = ransac_ratio;
        keyframe_ransac_ratio = ransac_ratio;
        outlier_ratio = 0.005;
        twoview_outlier_ratio = outlier_ratio;
        nview_outlier_ratio = outlier_ratio;
    }
    void compute_derived_parameters( std::size_t video_width, std::size_t video_height, std::size_t video_fps ) {
        keyframe_frequency = video_fps;
        double video_diagonal = std::sqrt( static_cast<double>( video_width*video_width+video_height*video_height ) );
        robust_sigma = video_diagonal*robust_sigma_ratio;
        twoview_outlier_threshold = video_diagonal*twoview_outlier_ratio;
        nview_outlier_threshold = video_diagonal*nview_outlier_ratio;
        twoview_ransac_threshold = video_diagonal*twoview_ransac_ratio;
        keyframe_ransac_threshold = video_diagonal*keyframe_ransac_ratio;
    }
};

} // namespace videopano
} // namespace adobe_agt

#endif
