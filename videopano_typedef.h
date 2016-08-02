#ifndef ADOBE_AGT_VIDEOPANO_VIDEOPANO_TYPEDEF_H
#define ADOBE_AGT_VIDEOPANO_VIDEOPANO_TYPEDEF_H

//
/// \author Hailin Jin \n Adobe Systems Incorporated
/// \date 2010-2011 \n Last updated on February 2, 2011
//

#include <math/vector_n.h>
#include <math/matrix_mn.h>
#include <mvg/image_intrinsic.h>
#include <mvg/image_intrinsic_mode.h>
#include <mvg/progress_monitor.h>
#include <mvg/message_reporter.h>
#include <mvg/reconstruction_trajectory.h>
#include <mvg/point_unknown_updater.h>
#include <tracking/tracking_feature.h>
#include <tracking/tracking_trajectory.h>
#include "videopano_reconstruction.h"

namespace adobe_agt {
namespace videopano {

enum Keyframe_Status {
    keyframe_e,
    nonkeyframe_e
};

template <typename Camera_Motion_Type,
          typename Image_Intrinsic_Type>
struct videopano_camera {
    Keyframe_Status status;
    Image_Intrinsic_Type intrinsic;
    Camera_Motion_Type motion;
    videopano_camera( ) {}
    videopano_camera( Keyframe_Status status_in, 
                     const Image_Intrinsic_Type& intrinsic_in,
                     const Camera_Motion_Type& motion_in ) :
        status( status_in ), intrinsic( intrinsic_in ), motion( motion_in ) {}
};

typedef double Data_Type;
typedef math::vector_n<Data_Type, 2> Vector2;
typedef math::vector_n<Data_Type, 3> Vector3;
typedef math::matrix_mn<Data_Type, 3, 3> Matrix33;
typedef Matrix33 Camera_Motion;
typedef mvg::image_intrinsic_fl1_ic_ar<Data_Type> Image_Intrinsic;
typedef mvg::image_intrinsic_mode Image_Intrinsic_Mode;
typedef videopano_camera<Camera_Motion, Image_Intrinsic> Camera;
typedef mvg::point2_unknown_updater_spherical Point_Unknown_Updater;
typedef mvg::void_progress_monitor Progress_Monitor;
typedef mvg::stdcout_message_reporter Message_Reporter;
typedef tracking::tracking_feature<float> Tracking_Feature;
typedef tracking::tracking_trajectory Tracking_Trajectory;
//typedef int Reconstruction_Trajectory_Data_Type;
//typedef reconstructed_trajectory<Reconstruction_Trajectory_Data_Type> Reconstruction_Trajectory;
typedef videopano::videopano_reconstruction<Data_Type, Camera> Reconstruction;

// For rendering only
typedef math::matrix_mn<float, 3, 3> Matrix33_Float;
typedef mvg::image_intrinsic_fl1_ic_ar<float> Image_Intrinsic_Float;

} // namespace videopano
} // namespace adobe_agt

#endif
