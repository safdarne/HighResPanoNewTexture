#ifndef ADOBE_AGT_VIDEOPANO_VIDEOPANO_ALGORITHM_H
#define ADOBE_AGT_VIDEOPANO_VIDEOPANO_ALGORITHM_H

//
/// \author Hailin Jin \n Adobe Systems Incorporated
/// \date 2010-2011 \n Last updated on May 20, 2011
//

#include <cstddef>
#include <iterator>
#include <algorithm>
#include <utility>
#include <vector>
#include <utility/iterator_adaptor.h>
#include <utility/iterator_pair.h>
#include <utility/algorithm_extension.h>
#include <math/vector_n.h>
#include <math/matrix_mn.h>
#include <math/algorithm.h>
#include <tracking/tracking_feature.h>
#include <tracking/tracking_trajectory.h>
#include <tracking/tracking_io.h>
#include <mvg/image_intrinsic.h>
#include <mvg/image_intrinsic_mode.h>
#include <mvg/rotation_est.h>
#include <mvg/progress_monitor.h>
#include <mvg/message_reporter.h>
#include <mvg/reconstruction_trajectory.h>
#include <mvg/rotation3_est.h>
#include <mvg/image_intrinsic_algorithm.h>
#include <mvg/nview_solution_trajectory_reconstruction.h>
#include <mvg/sparse_bundle_adjuster3.h>
#include <mvg/sparse_bundle_adjuster4.h>
#include <mvg/shared_intrinsic.h>
#include <mvg/motion_control_so3_new.h>
#include <mvg/motion_control_so3_robust.h>
#include <mvg/pair_solution2.h>
#include <mvg/point_unknown_updater.h>
#include <mvg/robust_cost_function.h>
#include <mvg/nview_solution_trajectory.h>
#include "videopano_reconstruction.h"
#include "videopano_typedef.h"
#include "videopano_log_message.h"
#include "videopano_parameter.h"

namespace adobe_agt {
namespace videopano {

namespace detail {
template <typename Trajectory_Iterator,
          typename Result_Iterator>
Result_Iterator
compute_corresponding_points( Trajectory_Iterator trajectory_first, Trajectory_Iterator trajectory_last,
                             std::size_t frame1, std::size_t frame2,
                             Result_Iterator result_first ) {
    assert( frame1<=frame2 );
    Trajectory_Iterator trajectory_first0( trajectory_first );
    typedef typename std::iterator_traits<Trajectory_Iterator>::value_type::data_type trajectory_data_type;
    trajectory_data_type frame1l = static_cast<trajectory_data_type>( frame1 );
    trajectory_data_type frame2l = static_cast<trajectory_data_type>( frame2 );
    while ( trajectory_first!=trajectory_last ) {
        if ( trajectory_first->start( ) <=frame1l &&
            trajectory_first->finish( )> frame2l ) {
            *result_first = trajectory_first-trajectory_first0;
            ++result_first;
        }
        ++trajectory_first;
    }
    return result_first;
}

template <typename Reconstruction_Trajectory_Iterator,
          typename Tracking_Trajectory_Iterator,
          typename Result_Iterator>
Result_Iterator
find_candidate_points( std::size_t view_n,
                      std::size_t view_1,
                      std::size_t view_2,
                      Reconstruction_Trajectory_Iterator trajectory_first,
                      Reconstruction_Trajectory_Iterator trajectory_last,
                      Tracking_Trajectory_Iterator tracking_trajectory_first,
                      Result_Iterator result_first ) {
    assert( view_1<view_2 );
    typedef typename std::iterator_traits<Tracking_Trajectory_Iterator>::value_type::data_type
        tracking_trajectory_data_type;
    Reconstruction_Trajectory_Iterator trajectory_first0( trajectory_first );
    //while ( trajectory_first!=trajectory_last &&
    //       trajectory_first->finish( )<=static_cast<trajectory_data_type>( view_1 ) )
    //    ++trajectory_first;
    while ( trajectory_first!=trajectory_last &&
           tracking_trajectory_first[ *trajectory_first ].start( )<=static_cast<tracking_trajectory_data_type>( view_1 ) ) {
        if ( tracking_trajectory_first[ *trajectory_first ].finish( )>static_cast<tracking_trajectory_data_type>( view_2 ) ) {
            Tracking_Trajectory_Iterator reference_trajectory =
                tracking_trajectory_first+*trajectory_first;
            tracking_trajectory_data_type reference_start  = reference_trajectory->start( );
            tracking_trajectory_data_type reference_finish = reference_trajectory->finish( );
            //assert( reference_start <=trajectory_first->start( ) &&
            //       reference_finish>=trajectory_first->finish( ) );
            if ( static_cast<tracking_trajectory_data_type>( view_n )>=reference_start && 
                static_cast<tracking_trajectory_data_type>( view_n )< reference_finish ) {
                *result_first = trajectory_first-trajectory_first0;
                ++result_first;
            }
        }
        ++trajectory_first;
    }
    return result_first;
}
} // namespace detail

/// Find the indices of the trajectories that overlap ( view1, view2 )
/// from [ reference_trajectory_first, reference_trajectory_last )
template <typename Tracking_Trajectory_Iterator,
          typename Result_Iterator>
Result_Iterator
find_overlapping_trajectory( std::size_t view1, std::size_t view2,
                            Tracking_Trajectory_Iterator tracking_trajectory_first,
                            Tracking_Trajectory_Iterator tracking_trajectory_last,
                            Result_Iterator result_first ) {
    typedef typename std::iterator_traits<Tracking_Trajectory_Iterator>::value_type::data_type tracking_trajectory_data_type;
    tracking_trajectory_data_type view1l = static_cast<tracking_trajectory_data_type>( view1 );
    tracking_trajectory_data_type view2l = static_cast<tracking_trajectory_data_type>( view2 );
    assert( view1l!=view2l );
    if ( view1l>view2l ) std::swap( view1l, view2l );
    Tracking_Trajectory_Iterator tracking_trajectory_first0 = tracking_trajectory_first;
    while ( tracking_trajectory_first!=tracking_trajectory_last &&
           tracking_trajectory_first->finish( )<=view1l ) ++tracking_trajectory_first;
    Tracking_Trajectory_Iterator tracking_trajectory = tracking_trajectory_first;
    while ( tracking_trajectory!=tracking_trajectory_last &&
           tracking_trajectory->start( )<=view2l ) ++tracking_trajectory;
    tracking_trajectory_last = tracking_trajectory;
    while ( tracking_trajectory_first!=tracking_trajectory_last ) {
        if ( tracking_trajectory_first->start( )<=view1l &&
            tracking_trajectory_first->finish( )>view2l ) {
            *result_first = tracking_trajectory_first-tracking_trajectory_first0;
            ++result_first;
        }
        ++tracking_trajectory_first;
    }
    return result_first;
}

/// Find trajectories from [ reference_trajectory_first, reference_trajectory_last )
/// that overlap ( view_index_first, view_index_last ) but do not belong to
/// [ trajectory_first, trajectory_last )
/// [ trajectory_first, trajectory_last ) is assumed to be sorted according to reference_index( )
template <typename Reconstruction_Trajectory_Iterator,
          typename Reference_Trajectory_Iterator,
          typename Result_Trajectory_Iterator>
std::pair<Reference_Trajectory_Iterator, Result_Trajectory_Iterator>
find_additional_trajectories( std::size_t view_1, std::size_t view_2,
                             Reconstruction_Trajectory_Iterator trajectory_first,
                             Reconstruction_Trajectory_Iterator trajectory_last,
                             Reference_Trajectory_Iterator reference_trajectory_first0,
                             Reference_Trajectory_Iterator reference_trajectory_first,
                             Reference_Trajectory_Iterator reference_trajectory_last,
                             Result_Trajectory_Iterator result_trajectory_first ) {
    assert( view_1<view_2 );
    //while ( reference_trajectory_first!=reference_trajectory_last &&
    //       reference_trajectory_first->finish( )<=view_1 ) ++reference_trajectory_first;
    typedef typename std::iterator_traits<Reference_Trajectory_Iterator>::value_type::data_type reference_trajectory_data_type;
    while ( reference_trajectory_first!=reference_trajectory_last &&
           reference_trajectory_first->start( )<=static_cast<reference_trajectory_data_type>( view_1 ) ) {
        if ( reference_trajectory_first->finish( )>static_cast<reference_trajectory_data_type>( view_2 ) ) {
            //typedef typename std::iterator_traits<Reconstruction_Trajectory_Iterator>::value_type reconstruction_trajectory_type;
            //typedef typename std::iterator_traits<Result_Trajectory_Iterator>::value_type result_trajectory_type;
            //typedef typename reconstruction_trajectory_type::data_type reconstruction_trajectory_data_type;
            std::size_t index = reference_trajectory_first-reference_trajectory_first0;
            //typedef detail::reference_index_comparator_t<reconstruction_trajectory_type, 
            //                                             std::size_t> reference_index_comparator;
            Reconstruction_Trajectory_Iterator search_result = 
                std::lower_bound( trajectory_first, trajectory_last,
                                 index );//, reference_index_comparator( ) );
            if ( search_result==trajectory_last ||
                *search_result!=index ) {
                // Find a new trajectory
                //result_trajectory_first->start( ) = static_cast<typename result_trajectory_type::data_type>( view_1 );
                //result_trajectory_first->finish( ) = static_cast<typename result_trajectory_type::data_type>( view_2+1 );
                *result_trajectory_first = index;
                ++result_trajectory_first;
            }
        }
        ++reference_trajectory_first;
    }
    return std::make_pair( reference_trajectory_first, result_trajectory_first );
}

namespace {
template <typename Reference_Trajectory_Iterator,
          typename Feature_Iterator>
class reconstruction_meas_copier {
private:
    Reference_Trajectory_Iterator _reference_trajectory_first;
    Feature_Iterator _feature_first;
    std::size_t _feature_size;
public:
    reconstruction_meas_copier( ) {}
    reconstruction_meas_copier( Reference_Trajectory_Iterator reference_trajectory_first,
                               Feature_Iterator feature_first,
                               std::size_t feature_size ) :
        _reference_trajectory_first( reference_trajectory_first ),
        _feature_first( feature_first ),
        _feature_size( feature_size ) {}
    template <typename Trajectory_Iterator,
              typename Result_Iterator>
    void operator( )( Trajectory_Iterator trajectory_first,
                    std::size_t view,
                    Result_Iterator result_first ) const {
        Feature_Iterator feature = _feature_first+view*_feature_size;
        feature += _reference_trajectory_first[ *trajectory_first ].offset( );
        result_first[ 0 ] = feature->x( );
        result_first[ 1 ] = feature->y( );
    }
};
class weight_copier_t {
public:
    weight_copier_t( ) {}
    template <typename Trajectory_Iterator,
              typename Result_Iterator>
    void operator( )( Trajectory_Iterator trajectory_first,
                    std::size_t view,
                    Result_Iterator result_first ) const {
        result_first[ 0 ] = 1.0;
    }
};
template <typename Trajectory_Iterator,
          typename Point_Iterator>
class point_unknown2_copier {
private:
    Trajectory_Iterator _trajectory_first;
    Point_Iterator _point_first;
public:
    point_unknown2_copier( ) {}
    point_unknown2_copier( Trajectory_Iterator trajectory_first, Point_Iterator point_first ) :
        _trajectory_first( trajectory_first ), _point_first( point_first ) {}
    template <typename Result_Iterator>
    void operator( )( Trajectory_Iterator trajectory, Result_Iterator result_first ) const {
        Point_Iterator point = _point_first+( trajectory-_trajectory_first );
        result_first[ 0 ] = point->x( );
        result_first[ 1 ] = point->y( );
    }
};
} // namespace 

template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
bool nview_bundle_adjustment( Reconstruction& recon, 
                             Tracking_Trajectory_Iterator tracking_trajectory_first,
                             Tracking_Feature_Iterator tracking_feature_first,
                             std::size_t tracking_feature_size,
                             Image_Intrinsic& shared_image_intrinsic,
                             Image_Intrinsic_Mode iim,
                             Data_Type bisquare_sigma,
                             std::size_t nview_sba_max_iterations ) {
    typedef mvg::shared_intrinsic<Image_Intrinsic, Image_Intrinsic_Mode> shared_control_type;
    //typedef mvg::robust_cost_function_bisquare_2<Data_Type> Robust_Cost_Function;
    typedef mvg::robust_cost_function_huber_2<Data_Type> Robust_Cost_Function;
    typedef mvg::motion_control_so3_robust<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode, Robust_Cost_Function > motion_control_type;

    std::vector<std::size_t> view_indices;
    std::vector<motion_control_type> motion_controls;
    shared_control_type shared_control( shared_image_intrinsic, iim );

    Reconstruction::camera_const_iterator
        intrinsic_motion_const_first = recon.camera_begin( ),
        intrinsic_motion_const_last  = recon.camera_end( );
    while ( intrinsic_motion_const_first!=intrinsic_motion_const_last ) {
        view_indices.push_back( intrinsic_motion_const_first->first );
        motion_controls.push_back( motion_control_type( intrinsic_motion_const_first->second.intrinsic,
                                                      iim,
                                                      math::make_iterator_2d_n<3>( intrinsic_motion_const_first->second.motion.begin( ) ),
                                                      intrinsic_motion_const_first->first!=recon.reference_index( ),
                                                      Robust_Cost_Function( bisquare_sigma ) ) );
        ++intrinsic_motion_const_first;
    }

    typedef reconstruction_meas_copier<Tracking_Trajectory_Iterator,
                                       Tracking_Feature_Iterator> meas_copier_t;
    typedef point_unknown2_copier<Reconstruction::size_t_const_iterator,
                                  Reconstruction::point2_const_iterator> point_unknown_copier_t;

    // Run sparse bundle adjustment
    // sparse_bundle_adjuster3
    /*typedef mvg::nview_solution_trajectory_correspondence<Data_Type, 2, 2, 2> solution_type;
    solution_type mstc;
    mstc.resize( recon.number_of_cameras( ) );
    mstc.add_coupled_point_measurements( view_indices.begin( ),
                                        view_indices.end( ),
                                        recon.inlier_2d_trajectory_begin( ),
                                        recon.inlier_2d_trajectory_end( ),
                                        tracking_trajectory_first,
                                        point_unknown_copier_t( recon.inlier_2d_trajectory_begin( ), recon.point2_begin( ) ),
                                        meas_copier_t( tracking_trajectory_first, 
                                                      tracking_feature_first,
                                                      tracking_feature_size ) );
    typedef mvg::sparse_bundle_adjuster3<Data_Type, 
                                         shared_control_type, 
                                         motion_control_type,
                                         Point_Unknown_Updater,
                                         Progress_Monitor,
                                         Message_Reporter> optimizer_type;
    optimizer_type nview_sba_solver( mstc,
                                    shared_control,
                                    motion_controls.begin( ),
                                    Point_Unknown_Updater( ),
                                    Progress_Monitor( ),
                                    Message_Reporter( ),
                                    true, 
                                    nview_sba_max_iterations );
    if ( nview_sba_solver.is_failed( ) ) return false;*/

    // sparse_bundle_adjuster4
    typedef mvg::nview_solution_trajectory<Data_Type, 2, 2, 2> solution4_type;
    solution4_type nst( view_indices.begin( ), view_indices.end( ) );
    nst.add_coupled_tracking_point( recon.inlier_2d_trajectory_begin( ),
                                   recon.inlier_2d_trajectory_end( ),
                                   tracking_trajectory_first,
                                   point_unknown_copier_t( recon.inlier_2d_trajectory_begin( ), recon.point2_begin( ) ),
                                   meas_copier_t( tracking_trajectory_first, 
                                                 tracking_feature_first,
                                                 tracking_feature_size ),
                                   weight_copier_t( ) );
    typedef mvg::sparse_bundle_adjuster4<Data_Type, 
                                         shared_control_type, 
                                         motion_control_type,
                                         Message_Reporter,
                                         Progress_Monitor,
                                         Point_Unknown_Updater> optimizer4_type;
    optimizer4_type nview_sba_solver4( nst,
                                      shared_control,
                                      motion_controls.begin( ),
                                      true, 
                                      nview_sba_max_iterations,
                                      Message_Reporter( ),
                                      Progress_Monitor( ),
                                      Point_Unknown_Updater( ) );
    if ( nview_sba_solver4.is_failed( ) ) return false;

    // Copy intrinsics and motions
    optimizer4_type::motion_control_const_iterator 
        motion_control_first = nview_sba_solver4.motion_control_begin( ),
        motion_control_last  = nview_sba_solver4.motion_control_end( );
    Reconstruction::camera_iterator
        intrinsic_motion_first = recon.camera_begin( );
    while ( motion_control_first!=motion_control_last ) {
        intrinsic_motion_first->second.intrinsic = motion_control_first->get_image_intrinsic( );
        std::copy( motion_control_first->rotation_begin( ),
                  motion_control_first->rotation_begin( )+9,
                  intrinsic_motion_first->second.motion.begin( ) );
        ++motion_control_first;
        ++intrinsic_motion_first;
    }

    // Copy points
    optimizer4_type::const_iterator point_unknown_first = nview_sba_solver4.coupled_point_begin( );
    //solution4_type::size_t_const_iterator point_mapping_first = nst.coupled_mapping_begin( );
    Reconstruction::point2_iterator recon_point_first = recon.point2_begin( );
    for ( std::size_t ii=0;ii<nst.number_of_coupled_points( );++ii ) {
        std::size_t point_mapping = ii; // *point_mapping_first;
        recon_point_first[ point_mapping ].x( ) = point_unknown_first[ 0 ];
        recon_point_first[ point_mapping ].y( ) = point_unknown_first[ 1 ];
        point_unknown_first += 2;
        // ++point_mapping_first;
    }
    return true;
}

template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
std::size_t remove_outlier_trajectory_point( Reconstruction& recon,
                                            Tracking_Trajectory_Iterator tracking_trajectory_first,
                                            Tracking_Feature_Iterator tracking_feature_first,
                                            std::size_t tracking_feature_size,
                                            Data_Type threshold ) {
    assert( recon.number_of_2d_inliers( )==recon.number_of_2d_inliers( ) );

    Data_Type threshold2 = threshold*threshold;
    // Copy intrinsics and motions
    std::vector<std::size_t> view_indices;
    std::vector<Image_Intrinsic> intrinsics;
    std::vector<Matrix33> rotations;
    Reconstruction::camera_const_iterator
        intrinsic_motion_const_first = recon.camera_begin( ),
        intrinsic_motion_const_last  = recon.camera_end( );
    while ( intrinsic_motion_const_first!=intrinsic_motion_const_last ) {
        view_indices.push_back( intrinsic_motion_const_first->first );
        intrinsics.push_back( intrinsic_motion_const_first->second.intrinsic );
        rotations.push_back( intrinsic_motion_const_first->second.motion );
        ++intrinsic_motion_const_first;
    }

    std::vector<int> trajectory_outliers( recon.number_of_2d_inliers( ) );
    //std::vector<int> point_outliers( recon.number_of_2d_inliers( ) );
    std::fill( trajectory_outliers.begin( ), trajectory_outliers.end( ), 0 );
    //std::fill( point_outliers.begin( ), point_outliers.end( ), 1 );
    std::vector<int>::iterator trajectory_outlier_first = trajectory_outliers.begin( );
    //std::vector<int>::iterator point_outlier_first = point_outliers.begin( );
    Reconstruction::point2_const_iterator point_const_first = recon.point2_begin( );
    Reconstruction::size_t_const_iterator
        trajectory_const_first = recon.inlier_2d_trajectory_begin( ),
        trajectory_const_last  = recon.inlier_2d_trajectory_end( );
    std::vector<std::size_t>::const_iterator 
        view_first = view_indices.begin( ),
        view_last  = view_indices.end( );
    std::size_t outlier_point_size = 0;
    int point_id = 0;
    while ( trajectory_const_first!=trajectory_const_last ) {
        int start  = tracking_trajectory_first[ *trajectory_const_first ].start( );
        int finish = tracking_trajectory_first[ *trajectory_const_first ].finish( );
        //int point_id = trajectory_const_first->point_id( );
        Tracking_Feature_Iterator 
            feature_first = tracking_feature_first +
                            tracking_trajectory_first[ *trajectory_const_first ].offset( );
        Data_Type X[ 3 ];
        math::spherical_to_cartesian( point_const_first[ point_id ].x( ), point_const_first[ point_id ].y( ),
                                     X[ 0 ], X[ 1 ], X[ 2 ] );
        std::vector<std::size_t>::const_iterator view = view_first;
        while ( view!=view_last ) {
            int frame = static_cast<int>( *view );
            Data_Type x[ 2 ], y[ 2 ];
            if ( frame>=start && frame<finish ) {
                mvg::project_perspective_rotation3( math::make_iterator_2d_n<3>( rotations[ view-view_first ].begin( ) ),
                                                   X[ 0 ], X[ 1 ], X[ 2 ],
                                                   x[ 0 ], x[ 1 ] );
                mvg::apply_image_intrinsic( intrinsics[ view-view_first ],
                                           x[ 0 ], x[ 1 ],
                                           y[ 0 ], y[ 1 ] );
                Tracking_Feature_Iterator 
                    feature = feature_first+frame*tracking_feature_size;
                Data_Type error = 0, tmp;
                tmp = feature->x( )-y[ 0 ]; error += tmp*tmp;
                tmp = feature->y( )-y[ 1 ]; error += tmp*tmp;
                if ( error>threshold2 ) {
                    *trajectory_outlier_first = 1;
                    //point_outlier_first[ point_id ] = 0;
                    ++outlier_point_size;
                    break;
                }
            }
            ++view;
        }
        ++trajectory_const_first;
        ++trajectory_outlier_first;
        ++point_id;
    }
    std::vector<std::size_t> outlier_indices( outlier_point_size );
    trajectory_outlier_first = trajectory_outliers.begin( );
    Reconstruction::size_t_iterator trajectory_first = recon.inlier_2d_trajectory_begin( );
    std::vector<std::size_t>::iterator outlier_indice_first = outlier_indices.begin( );
    while ( trajectory_outlier_first!=trajectory_outliers.end( ) ) {
        if ( *trajectory_outlier_first ) 
            *outlier_indice_first++ = *trajectory_first;
        ++trajectory_outlier_first;
        ++trajectory_first;
    }
    assert( outlier_indice_first==outlier_indices.end( ) );
    std::vector<std::size_t> outlier_indices_tmp( outlier_point_size+recon.number_of_2d_outliers( ) );
    std::vector<std::size_t>::iterator union_result =
        std::set_union( recon.outlier_2d_trajectory_begin( ), recon.outlier_2d_trajectory_end( ),
                       outlier_indices.begin( ), outlier_indices.end( ),
                       outlier_indices_tmp.begin( ) );
    recon.resize_2d_outliers( union_result-outlier_indices_tmp.begin( ) );
    std::copy( outlier_indices_tmp.begin( ), union_result,
              recon.outlier_2d_trajectory_begin( ) );

    Reconstruction::size_t_iterator trajectory_last =
        utility::remove_if( recon.inlier_2d_trajectory_begin( ), recon.inlier_2d_trajectory_end( ), 
                           trajectory_outliers.begin( ), std::bind2nd( std::equal_to<int>( ), 1 ) );
    std::size_t removed_points = recon.inlier_2d_trajectory_end( )-trajectory_last;
    Reconstruction::point2_iterator point_last =
        utility::remove_if( recon.point2_begin( ), recon.point2_end( ),
                           trajectory_outliers.begin( ), std::bind2nd( std::equal_to<int>( ), 1 ) );
    assert( removed_points==recon.point2_end( )-point_last );
    recon.resize_2d_inliers( point_last-recon.point2_begin( ) );
    // Change point ID
    /*point_outlier_first = point_outliers.begin( );
    std::vector<int>::iterator  point_outlier_last = point_outliers.end( );
    int sum = 0;
    while ( point_outlier_first!=point_outlier_last ) {
        if ( *point_outlier_first ) {
            *point_outlier_first = sum;
            ++sum;
        } else *point_outlier_first = -1;
        ++point_outlier_first;
    }
    trajectory_first = recon.trajectory_begin( );
    trajectory_last  = recon.trajectory_end( );
    while ( trajectory_first!=trajectory_last ) {
        trajectory_first->point_id( ) = point_outliers[ trajectory_first->point_id( ) ];
        ++trajectory_first;
    }*/
    return removed_points;
}

namespace detail {
template <typename Image_Intrinsic,
          typename Rotation_Iterator_2d,
          typename T>
void rotation_point_measurement_to_unknown( const Image_Intrinsic& intrinsic,
                                           Rotation_Iterator_2d rotation_first_2d,
                                           T x, T y,
                                           T& theta, T& phi ) {
    T X[ 3 ], Y[ 3 ];
    image_to_homogeneous_image_intrinsic( intrinsic, 
                                         x, y,
                                         Y[ 0 ], Y[ 1 ] );
    Y[ 2 ] = T( 1 );
    math::vector_multiplies_matrix_plus_constant_k<3,3>( Y,
                                                        rotation_first_2d,
                                                        X,
                                                        T( 0 ) );
    mvg::cartesian_to_spherical( X[ 0 ], X[ 1 ], X[ 2 ], theta, phi );
}

template <typename Point1_Iterator,
          typename Point2_Iterator,
          typename Rotation_Iterator_2d1,
          typename Rotation_Iterator_2d2,
          typename Result_Iterator>
void compute_twoview_unknown( std::size_t point_size,
                             Point1_Iterator point1_first, Point2_Iterator point2_first,
                             const Image_Intrinsic& ii1, const Image_Intrinsic& ii2,
                             Rotation_Iterator_2d1 rot1, Rotation_Iterator_2d2 rot2,
                             Result_Iterator result_first ) {
    for( std::size_t ii=0;ii<point_size;++ii ) {
        Data_Type unknown0[ 2 ], unknown1[ 2 ];
        rotation_point_measurement_to_unknown( ii1,
                                              rot1,
                                              point1_first[ 0 ], point1_first[ 1 ],
                                              unknown0[ 0 ], unknown0[ 1 ] );
        rotation_point_measurement_to_unknown( ii2,
                                              rot2,
                                              point2_first[ 0 ], point2_first[ 1 ],
                                              unknown1[ 0 ], unknown1[ 1 ] );

        result_first[ 0 ] = ( unknown0[ 0 ]+unknown1[ 0 ] )/Data_Type( 2 );   
        result_first[ 1 ] = ( unknown0[ 1 ]+unknown1[ 1 ] )/Data_Type( 2 );
        if ( std::abs( std::abs( unknown0[ 1 ]-unknown1[ 1 ] )-math::pi2 )<0.1 )
            result_first[ 1 ] += math::pi;

        point1_first += 2;
        point2_first += 2;
        result_first += 2;
    }
}
} // namespace detail

template <std::size_t n,
          typename Iterator_2d>
void fill_identity_k( Iterator_2d first_2d ) {
    typedef typename Iterator_2d::iterator_type Iterator;
    typedef typename std::iterator_traits<Iterator>::value_type T;
    for ( std::size_t ii=0;ii<n;++ii ) {
        Iterator first = project( first_2d );
        utility::fill_k<n>( first, T( 0 ) );
        first[ ii ] = T( 1 );
        increment( first_2d );
    }
}

/// Two-view initiailization for video panorama
// Return value
// 0: succeeds; 1: RANSAC fails; 2: SBA fails
template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
int twoview_initialization( Reconstruction& recon,
                           std::size_t frame1, std::size_t frame2,
                           Data_Type fl1, Data_Type fl2, // guess for the two focal lengths
                           Data_Type ic1, Data_Type ic2, Data_Type ar,
                           int intrinsic_mode,
                           Tracking_Trajectory_Iterator tracking_trajectory_first,
                           Tracking_Trajectory_Iterator tracking_trajectory_last,
                           Tracking_Feature_Iterator tracking_feature_first,
                           std::size_t tracking_feature_size,
                           Data_Type ransac_threshold,
                           std::size_t ransac_max_trials,
                           double ransac_confidence,
                           std::size_t sba_max_iterations,
                           Data_Type twoview_outlier_threshold ) {

    std::vector<std::ptrdiff_t> indices( tracking_feature_size );
    std::vector<std::ptrdiff_t>::iterator indice_first = indices.begin( );
    std::vector<std::ptrdiff_t>::iterator indice_last =
        detail::compute_corresponding_points( tracking_trajectory_first,
                                             tracking_trajectory_last,
                                             frame1, frame2, 
                                             indice_first );
    std::size_t correspondences = indice_last-indice_first;
    typedef mvg::pair_solution2<Data_Type, 2, 2> pair_solution_type;
    pair_solution_type ps( correspondences );
    Tracking_Feature_Iterator
        feature1_first = tracking_feature_first+frame1*tracking_feature_size,
        feature2_first = tracking_feature_first+frame2*tracking_feature_size;
	pair_solution_type::iterator
        meas1_first = ps.unknown_measurement_0_begin( ),
        meas2_first = ps.unknown_measurement_1_begin( );
    while ( indice_first!=indice_last ) {
        int offset = tracking_trajectory_first[ *indice_first ].offset( );
        meas1_first[ 0 ] = feature1_first[ offset ].x( );
        meas1_first[ 1 ] = feature1_first[ offset ].y( );
        meas2_first[ 0 ] = feature2_first[ offset ].x( );
        meas2_first[ 1 ] = feature2_first[ offset ].y( );
        ++indice_first;
        meas1_first += 2;
        meas2_first += 2;
    }
    
    typedef utility::step_offset_iterator<pair_solution_type::const_iterator, 2> vector2_iterator;
    
    /// \todo Check intrinsic_mode
    mvg::rotation3_est_2view_ransac_3point<Data_Type> 
        ransac_solver( correspondences,
                      vector2_iterator( ps.unknown_measurement_0_begin( ), 0 ), vector2_iterator( ps.unknown_measurement_0_begin( ), 1 ),
                      vector2_iterator( ps.unknown_measurement_1_begin( ), 0 ), vector2_iterator( ps.unknown_measurement_1_begin( ), 1 ),
                      fl1, fl2,
                      ic1, ic2, ar, 
                      ic1, ic2, ar,
                      ransac_threshold,
                      ransac_max_trials,
                      ransac_confidence );
    if ( ransac_solver.is_failed( ) ) return 1;

    /*mvg::twoview_rotation_bundle_adjuster<Data_Type,
                                          Image_Intrinsic,
                                          Image_Intrinsic_Mode,
                                          Progress_Monitor,
                                          Message_Reporter>
        twoview_sba_solver( correspondences,
                           reinterpret_cast<const Vector2*>( &*ps.unknown_measurement_0_begin( ) ),//points1.begin( ),
                           reinterpret_cast<const Vector2*>( &*ps.unknown_measurement_1_begin( ) ),//points2.begin( ),
                           ransac_solver.best_rotation( ).data_matrix( ),
                           Image_Intrinsic( ransac_solver.best_focal_length_0( ), ic1, ic2, ar ),
                           Image_Intrinsic( ransac_solver.best_focal_length_1( ), ic1, ic2, ar ),
                           mvg::shared_fl1,
                           Progress_Monitor( ),
                           Message_Reporter( ),
                           sba_max_iterations );
    if ( twoview_sba_solver.is_failed( ) ) return 2;*/

    typedef mvg::shared_intrinsic<Image_Intrinsic, Image_Intrinsic_Mode> shared_control_type;
    typedef mvg::motion_control_so3_new<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode> motion_control_type;
    shared_control_type sc;
    motion_control_type mc[ 2 ];
    Matrix33 rot0, rot1;
    fill_identity_k<3>( math::make_iterator_2d_n<3>( rot0.begin( ) ) );
    std::copy( ransac_solver.best_rotation( ).data_matrix( ).begin( ),
              ransac_solver.best_rotation( ).data_matrix( ).end( ),
              rot1.begin( ) );
    if        ( intrinsic_mode==1 ) {
        Data_Type fl = ( ransac_solver.best_focal_length_0( )+
                        ransac_solver.best_focal_length_1( ) )/2;
        Image_Intrinsic ii( fl, ic1, ic2, ar );
        sc    = shared_control_type( ii, mvg::shared_fl1 );
        mc[ 0 ] = motion_control_type( ii, mvg::shared_fl1, math::make_iterator_2d_n<3>( rot0.begin( ) ), 0 );
        mc[ 1 ] = motion_control_type( ii, mvg::shared_fl1, math::make_iterator_2d_n<3>( rot1.begin( ) ), 1 );
    } else if ( intrinsic_mode==2 ) {
        Data_Type fl0 = ransac_solver.best_focal_length_0( );
        Data_Type fl1 = ransac_solver.best_focal_length_1( );
        Data_Type fl = ( fl0+fl1 )/2;
        sc =    shared_control_type( Image_Intrinsic( fl , ic1, ic2, ar ), mvg::private_fl1 );
        mc[ 0 ] = motion_control_type( Image_Intrinsic( fl0, ic1, ic2, ar ), mvg::private_fl1, math::make_iterator_2d_n<3>( rot0.begin( ) ), 0 );
        mc[ 1 ] = motion_control_type( Image_Intrinsic( fl1, ic1, ic2, ar ), mvg::private_fl1, math::make_iterator_2d_n<3>( rot1.begin( ) ), 1 );
    } else throw std::runtime_error( "Unsupported intrinsic mode" );

    detail::compute_twoview_unknown( correspondences,
                                    ps.unknown_measurement_0_begin( ),
                                    ps.unknown_measurement_1_begin( ),
                                    mc[ 0 ].get_image_intrinsic( ),
                                    mc[ 1 ].get_image_intrinsic( ),
                                    math::make_iterator_2d_n<3>( rot0.begin( ) ),
                                    math::make_iterator_2d_n<3>( rot1.begin( ) ),
                                    ps.unknown_begin( ) );

    /*mvg::sparse_bundle_adjuster3<Data_Type,
                            shared_control_type,
                            motion_control_type,
                            Point_Unknown_Updater,
                            Progress_Monitor,
                            Message_Reporter>
        sba_solver( ps,
                   sc,
                   mc,
                   Point_Unknown_Updater( ),
                   Progress_Monitor( ),
                   Message_Reporter( ),
                   true,
                   sba_max_iterations );
    if ( sba_solver.is_failed( ) ) return 2;*/
    mvg::sparse_bundle_adjuster4<Data_Type,
                            shared_control_type,
                            motion_control_type,
                            Message_Reporter,
                            Progress_Monitor,
                            Point_Unknown_Updater>
        sba_solver4( ps,
                    sc,
                    mc,
                    true,
                    sba_max_iterations,
                    Message_Reporter( ),
                    Progress_Monitor( ),
                    Point_Unknown_Updater( ) );
    if ( sba_solver4.is_failed( ) ) return 2;
    // Copy results out
    std::copy( sba_solver4.coupled_point_begin( ), 
              sba_solver4.coupled_point_begin( )+2*correspondences, 
              ps.unknown_begin( ) );
    mc[ 0 ] = sba_solver4.motion_control_begin( )[ 0 ];
    mc[ 1 ] = sba_solver4.motion_control_begin( )[ 1 ];

    std::vector<int> outliers( correspondences );
    std::fill( outliers.begin( ), outliers.end( ), 0 );
    std::vector<int>::iterator outlier_first = outliers.begin( );
	pair_solution_type::const_iterator
        point2_first = ps.unknown_begin( ),
        point2_last  = ps.unknown_end( );
    meas1_first = ps.unknown_measurement_0_begin( );
    meas2_first = ps.unknown_measurement_1_begin( );
    Data_Type twoview_outlier_threshold2 = twoview_outlier_threshold*twoview_outlier_threshold;
    while ( point2_first!=point2_last ) {
        Data_Type X[ 3 ], x[ 2 ], y1[ 2 ], y2[ 2 ], error1, error2, tmp;
        math::spherical_to_cartesian( point2_first[ 0 ], point2_first[ 1 ],
                                     X[ 0 ], X[ 1 ], X[ 2 ] );
        x[ 0 ] = X[ 0 ]/X[ 2 ]; x[ 1 ] = X[ 1 ]/X[ 2 ];
        mvg::apply_image_intrinsic( mc[ 0 ].get_image_intrinsic( ),
                                   x[ 0 ], x[ 1 ],
                                   y1[ 0 ], y1[ 1 ] );
        mvg::project_perspective_rotation3( math::make_iterator_2d_n<3>( rot1.begin( ) ),
                                           X[ 0 ], X[ 1 ], X[ 2 ],
                                           x[ 0 ], x[ 1 ] );
        mvg::apply_image_intrinsic( mc[ 1 ].get_image_intrinsic( ),
                                   x[ 0 ], x[ 1 ],
                                   y2[ 0 ], y2[ 1 ] );
        error1 = 0;
        tmp = meas1_first[ 0 ]-y1[ 0 ]; error1 += tmp*tmp;
        tmp = meas1_first[ 1 ]-y1[ 1 ]; error1 += tmp*tmp;
        error2 = 0;
        tmp = meas2_first[ 0 ]-y2[ 0 ]; error2 += tmp*tmp;
        tmp = meas2_first[ 1 ]-y2[ 1 ]; error2 += tmp*tmp;
        if ( error1>twoview_outlier_threshold2 || 
            error2>twoview_outlier_threshold2 )
            *outlier_first = 1;

        point2_first += 2;
        meas1_first += 2;
        meas2_first += 2;
        ++outlier_first;
    }

    // Copy results into recon
    std::size_t outlier_size = std::count( outliers.begin( ), outliers.end( ), int( 1 ) );
    recon.resize_2d_inliers( correspondences-outlier_size );
    Reconstruction::size_t_iterator
        inlier_trajectory_first = recon.inlier_2d_trajectory_begin( );
    Reconstruction::point2_iterator
        point_first = recon.point2_begin( );
    //const Data_Type* point_est_first =
    //    twoview_sba_solver.pair_solution( ).coupled_point_data_block( );
	pair_solution_type::iterator
        point_est_first = ps.unknown_begin( );
    outlier_first = outliers.begin( );
    int inlier_size = 0;
    for ( std::size_t ii=0;ii<correspondences;++ii ) {
        if ( !*outlier_first ) {
            *inlier_trajectory_first = indices[ ii ];
            //trajectory_first->point_id( ) = static_cast<Reconstruction_Trajectory::data_type>( inlier_size );
            //trajectory_first->start( ) =  static_cast<Reconstruction_Trajectory::data_type>( frame1 );
            //trajectory_first->finish( ) = static_cast<Reconstruction_Trajectory::data_type>( frame2+1 );
            ++inlier_trajectory_first;
            point_first->x( ) = point_est_first[ 0 ];
            point_first->y( ) = point_est_first[ 1 ];
            ++point_first;
            ++inlier_size;
        }
        ++outlier_first;
        point_est_first += 2;
    }
    recon.reference_index( ) = frame1;
    Matrix33 r0, r1;
    std::copy( mc[ 0 ].rotation_begin( ),
              mc[ 0 ].rotation_begin( )+9,
              r0.begin( ) );
    std::copy( mc[ 1 ].rotation_begin( ),
              mc[ 1 ].rotation_begin( )+9,
              r1.begin( ) );
    recon.insert_camera( std::make_pair( frame1, Camera( keyframe_e, mc[ 0 ].get_image_intrinsic( ), r0 ) ) );
    recon.insert_camera( std::make_pair( frame2, Camera( keyframe_e, mc[ 1 ].get_image_intrinsic( ), r1 ) ) );

    return 0;
}

namespace detail {
struct comparator_t1 {
    bool operator( )( const utility::detail::reference_pair<std::size_t, Vector2>& x, 
                    const utility::detail::reference_pair<std::size_t, Vector2>& y ) const {
        return x.first<y.first;
    }
};
} // namespace detail

template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
int keyframe_insertion( Reconstruction& recon,
                       std::size_t frame_n,
                       Tracking_Trajectory_Iterator tracking_trajectory_first,
                       Tracking_Trajectory_Iterator tracking_trajectory_last,
                       Tracking_Feature_Iterator tracking_feature_first,
                       std::size_t tracking_feature_size,
                       Data_Type fl, // guess for the focal length
                       Data_Type ic1, Data_Type ic2, Data_Type ar,
                       int intrinsic_mode,
                       Data_Type bisquare_sigma ) {
    assert( recon.number_of_cameras( )!=0 );
    typedef mvg::robust_cost_function_huber_2<Data_Type> Robust_Cost_Function;

    if ( intrinsic_mode==1 ) {
        // Find two images that are adjacent to the keyframe
        std::size_t frame1, frame2;

        std::size_t frame_f, frame_l;
        assert( recon.camera_begin( )!=recon.camera_end( ) );
        frame_f = recon.camera_begin( )->first;
        Reconstruction::camera_const_iterator intrinsic_motion_last = 
            recon.camera_end( );
        --intrinsic_motion_last;
        frame_l = intrinsic_motion_last->first;
        if        ( frame_n<frame_f ) {assert( 0 );}
        else if ( frame_n>frame_l ) {
            --intrinsic_motion_last;
            frame1 = intrinsic_motion_last->first;
            frame2 = frame_l;
        } else {assert( 0 );}

        // Compute an initial camera motion
        std::vector<std::size_t> candidates( tracking_feature_size );
        std::vector<std::size_t>::iterator candidate_last = 
            detail::find_candidate_points( frame_n, frame1, frame2, 
                                          recon.inlier_2d_trajectory_begin( ), recon.inlier_2d_trajectory_end( ), 
                                          tracking_trajectory_first, 
                                          candidates.begin( ) );
        std::size_t candidate_size = candidate_last-candidates.begin( );
        if ( candidate_size<3 ) return -3;
        std::vector<Data_Type> meas( candidate_size*2 );
        std::vector<Data_Type> knowns( candidate_size*3 );
        std::vector<std::size_t>::const_iterator
            candidate_first = candidates.begin( );
        Reconstruction::point2_const_iterator
            point_first = recon.point2_begin( );
        std::vector<Data_Type>::iterator known_first = knowns.begin( );
        std::vector<Data_Type>::iterator meas_first = meas.begin( );
        Tracking_Feature_Iterator
            tracking_feature_n_first = tracking_feature_first+frame_n*tracking_feature_size;
        for ( std::size_t ii=0;ii<candidate_size;++ii ) {
            std::size_t reference_offset = tracking_trajectory_first[ recon.inlier_2d_trajectory_begin( )[ *candidate_first ] ].offset( );
            //std::size_t point_id = recon.trajectory_begin( )[ *candidate_first ].point_id( );
            math::spherical_to_cartesian( point_first[ *candidate_first ].x( ),
                                         point_first[ *candidate_first ].y( ),
                                         known_first[ 0 ], 
                                         known_first[ 1 ], 
                                         known_first[ 2 ] );
            meas_first[ 0 ] = tracking_feature_n_first[ reference_offset ].x( );
            meas_first[ 1 ] = tracking_feature_n_first[ reference_offset ].y( );
            ++candidate_first;
            known_first += 3;
            meas_first += 2;
        }

        typedef utility::step_iterator_adaptor<const Data_Type*, 2> step2_adaptor_t;
        typedef utility::step_iterator_adaptor<const Data_Type*, 3> step3_adaptor_t;

        /// \todo Replace the linear estimator with a RANSAC solver
        /// \todo Need to take par into account
        mvg::rotation3_resection_linear<Data_Type>
            linear_resection_solver( candidate_size,
                                    step3_adaptor_t( &knowns.front( )   ), // work-around VC debug iterators
                                    step3_adaptor_t( &knowns.front( )+1 ), // work-around VC debug iterators
                                    step3_adaptor_t( &knowns.front( )+2 ), // work-around VC debug iterators
                                    step2_adaptor_t( &meas.front( )   ),   // work-around VC debug iterators
                                    step2_adaptor_t( &meas.front( )+1 ),
                                    fl, ic1, ic2, ar );
        if ( linear_resection_solver.is_failed( ) ) return -1;

        // Refine the camera motion
        /// \todo Need to take par into account
        std::size_t resection_max_iterations = 10;
        mvg::rotation3_fl_resection_robust<Data_Type, Robust_Cost_Function, Progress_Monitor, Message_Reporter> 
            rotation_resection_solver( candidate_size,
                                      step3_adaptor_t( &knowns.front( )   ), // work-around VC debug iterators
                                      step3_adaptor_t( &knowns.front( )+1 ), // work-around VC debug iterators
                                      step3_adaptor_t( &knowns.front( )+2 ), // work-around VC debug iterators
                                      step2_adaptor_t( &meas.front( )   ),   // work-around VC debug iterators
                                      step2_adaptor_t( &meas.front( )+1 ),   // work-around VC debug iterators
                                      math::make_iterator_2d_n<3>( linear_resection_solver.rotation_begin( ) ),
                                      fl, ic1, ic2, ar,
                                      Robust_Cost_Function( bisquare_sigma ),
                                      resection_max_iterations );
        if ( rotation_resection_solver.is_failed( ) ) return -2;

        /// \todo Need to add frame_n into existing trajectories
        Data_Type rot_n[ 9 ];
        math::rodrigues( rotation_resection_solver.rotation_vector_begin( )[ 0 ], 
                        rotation_resection_solver.rotation_vector_begin( )[ 1 ], 
                        rotation_resection_solver.rotation_vector_begin( )[ 2 ], 
                        math::make_iterator_2d_n<3>( rot_n ) );
        Image_Intrinsic intri_n( rotation_resection_solver.focal_length( ), ic1, ic2, ar );

        // Add additional trajectories
        //std::vector<std::size_t> trajectory_n( tracking_feature_size );
        if        ( frame_n<frame_f ) { assert( 0 );
        } else if ( frame_n>frame_l ) {
            std::vector<std::size_t> trajectory_new1( tracking_feature_size ), trajectory_new2( tracking_feature_size );
            std::vector<std::size_t>::iterator trajectory_new1_last = 
                find_overlapping_trajectory( frame_l, frame_n,
                                            tracking_trajectory_first,
                                            tracking_trajectory_last,
                                            trajectory_new1.begin( ) );
            std::vector<std::size_t>::iterator trajectory_new2_last =
                std::set_difference( trajectory_new1.begin( ), trajectory_new1_last,
                                    recon.inlier_2d_trajectory_begin( ), recon.inlier_2d_trajectory_end( ), 
                                    trajectory_new2.begin( ) );
            trajectory_new1_last = 
                std::set_difference( trajectory_new2.begin( ), trajectory_new2_last,
                                    recon.outlier_2d_trajectory_begin( ), recon.outlier_2d_trajectory_end( ), 
                                    trajectory_new1.begin( ) );
            /*std::pair<Tracking_Trajectory_Iterator,
                      std::vector<std::size_t>::iterator> find_result =
                find_additional_trajectories( frame_l, frame_n,
                                             recon.inlier_2d_trajectory_begin( ), recon.inlier_2d_trajectory_end( ), 
                                             tracking_trajectory_first,
                                             tracking_trajectory_first,
                                             tracking_trajectory_last,
                                             trajectory_n.begin( ) );
            if ( !std::equal( trajectory_new2.begin( ), trajectory_new2_last,
                            trajectory_n.begin( ) ) )
                throw std::runtime_error( "Inconsistent result detected" );*/
            std::vector<std::size_t>::iterator trajectory_first_n = trajectory_new1.begin( );
            std::size_t trajectory_size_n = trajectory_new1_last-trajectory_first_n;
            //Reconstruction_Trajectory_Data_Type id = static_cast<Reconstruction_Trajectory_Data_Type>( recon.number_of_2d_inliers( ) );
            std::size_t point_size_old = recon.number_of_2d_inliers( );
            recon.resize_2d_inliers( point_size_old+trajectory_size_n );
            Reconstruction::point2_iterator point_first = recon.point2_begin( )+point_size_old;
            Tracking_Feature_Iterator
                tracking_feature_first_l = tracking_feature_first+frame_l*tracking_feature_size;
            Data_Type rot_l[ 9 ];
            std::copy( recon.find_camera( frame_l )->second.motion.begin( ),
                      recon.find_camera( frame_l )->second.motion.begin( )+9,
                      rot_l );
            while ( trajectory_first_n!=trajectory_new1_last ) {
                //trajectory_first_n->point_id( ) = id;
                Data_Type x[ 3 ], theta_phi[ 3 ];
                x[ 0 ] = static_cast<Data_Type>( tracking_feature_first_l[ tracking_trajectory_first[ *trajectory_first_n ].offset( ) ].x( ) );
                x[ 1 ] = static_cast<Data_Type>( tracking_feature_first_l[ tracking_trajectory_first[ *trajectory_first_n ].offset( ) ].y( ) );
                mvg::image_to_homogeneous_image_intrinsic( intrinsic_motion_last->second.intrinsic, 
                                                          x[ 0 ], x[ 1 ],
                                                          x[ 0 ], x[ 1 ] );
                x[ 2 ] = 1.0;
                // the following is equivalent to $R^t x$
                math::vector_multiplies_matrix_plus_constant_k<3,3>( 
                    x,
                    math::make_iterator_2d_n<3>( rot_l ),
                    theta_phi,
                    Data_Type( 0 ) );
                mvg::cartesian_to_spherical( theta_phi[ 0 ], theta_phi[ 1 ], theta_phi[ 2 ], 
                                            point_first->x( ), point_first->y( ) );
                //++id;
                ++trajectory_first_n;
                ++point_first;
            }
            //std::size_t trajectory_size_old = recon.number_of_2d_inliers( );
            std::copy( trajectory_new1.begin( ), trajectory_new1_last,
                      recon.inlier_2d_trajectory_begin( )+point_size_old );
            /*std::inplace_merge( recon.trajectory_begin( ),
                               recon.trajectory_begin( )+trajectory_size_old,
                               recon.trajectory_end( ),
                               detail::reference_index_comparator2_t<Reconstruction_Trajectory>( ) );*/
            typedef utility::iterator_pair<Reconstruction::size_t_iterator,
                                           Reconstruction::point2_iterator> ip1;
            typedef utility::iterator_pair<std::vector<std::size_t>::iterator,
                                           std::vector<Vector2>::iterator> ip3;
            std::vector<std::size_t> trajectory_n2( point_size_old+trajectory_size_n );
            std::vector<Vector2> points2( point_size_old+trajectory_size_n );
            std::merge( ip1( recon.inlier_2d_trajectory_begin( )               , recon.point2_begin( ) ),
                       ip1( recon.inlier_2d_trajectory_begin( )+point_size_old, recon.point2_begin( )+point_size_old ),
                       ip1( recon.inlier_2d_trajectory_begin( )+point_size_old, recon.point2_begin( )+point_size_old ),
                       ip1( recon.inlier_2d_trajectory_end  ( )               , recon.point2_end  ( ) ),
                       ip3( trajectory_n2.begin( ), points2.begin( ) ),
                       detail::comparator_t1( ) );
            std::copy( trajectory_n2.begin( ), trajectory_n2.end( ), recon.inlier_2d_trajectory_begin( ) );
            std::copy( points2.begin( ), points2.end( ), recon.point2_begin( ) );
            //int id2 = 0;
            //Reconstruction::trajectory_iterator trajectory_first = recon.trajectory_begin( );
            //while ( trajectory_first!=recon.trajectory_end( ) ) trajectory_first++->point_id( )=id2++;

            Matrix33 rotation_n;
            std::copy( rot_n, rot_n+9, rotation_n.begin( ) );
            recon.insert_camera( std::make_pair( frame_n, Camera( keyframe_e, intri_n, rotation_n ) ) );

            return trajectory_size_n;
        } 
        else {assert( 0 );}
    } else if ( intrinsic_mode==2 ) {
    } else throw std::runtime_error( "Unsupported intrinsic mode" );
    return 0;
}

template <typename Rotation_Iterator_2d>
void image_to_polar_unknown( const Image_Intrinsic& ii,
                            Rotation_Iterator_2d rotation_first_2d,
                            Data_Type y1, Data_Type y2,
                            Data_Type& x1, Data_Type& x2 ) {
    Data_Type y3[ 3 ], theta_phi[ 3 ];
    mvg::image_to_homogeneous_image_intrinsic( ii, 
                                              y1, y2,
                                              y3[ 0 ], y3[ 1 ] );
    y3[ 2 ] = 1.0;
    // the following is equivalent to $R^t x$
    math::vector_multiplies_matrix_plus_constant_k<3,3>( 
        y3,
        rotation_first_2d,
        theta_phi,
        Data_Type( 0 ) );
    mvg::cartesian_to_spherical( theta_phi[ 0 ], theta_phi[ 1 ], theta_phi[ 2 ], 
                                x1, x2 );
}

template <typename Tracking_Trajectory_Iterator,
          typename Result_Trajectory_Iterator>
Result_Trajectory_Iterator
find_nonkeyframe_trajectory( std::size_t frame1, std::size_t frame2,
                            Tracking_Trajectory_Iterator tracking_trajectory_first,
                            Tracking_Trajectory_Iterator tracking_trajectory_last,
                            Result_Trajectory_Iterator result_trajectory_first ) {
    assert( frame1<frame2 );
    typedef typename std::iterator_traits<Tracking_Trajectory_Iterator>::value_type::data_type tracking_trajectory_data_type;
    tracking_trajectory_data_type frame1l = static_cast<tracking_trajectory_data_type>( frame1 );
    tracking_trajectory_data_type frame2l = static_cast<tracking_trajectory_data_type>( frame2 );

    // Save the beginning
    Tracking_Trajectory_Iterator tracking_trajectory_first0( tracking_trajectory_first );

    // Move through unrelevant trajectories
    while ( tracking_trajectory_first!=tracking_trajectory_last &&
           tracking_trajectory_first->finish( )<=frame1l ) ++tracking_trajectory_first;

    while ( tracking_trajectory_first!=tracking_trajectory_last ) {
        tracking_trajectory_data_type
            start = tracking_trajectory_first->start( ),
            finish = tracking_trajectory_first->finish( );
        if ( start>frame2l ) break;

        if ( ( start<=frame1l&&finish<=frame2l ) || start>frame1l ) {
            start  = std::max( start , frame1l   );
            finish = std::min( finish, frame2l+1 );
            if ( start+2<=finish ) { // minimum 2 frames
                //result_trajectory_first->start( ) = start;
                //result_trajectory_first->finish( ) = finish;
                *result_trajectory_first = tracking_trajectory_first-tracking_trajectory_first0;
                ++result_trajectory_first;
            }
        }
        ++tracking_trajectory_first;
    }
    return result_trajectory_first;
}

template <typename Reconstruction_Trajectory_Iterator,
          typename Tracking_Trajectory_Iterator,
          typename Result_Iterator>
Result_Iterator
find_keyframe_trajectory( std::size_t frame1,
                         std::size_t frame2,
                         Reconstruction_Trajectory_Iterator trajectory_first,
                         Reconstruction_Trajectory_Iterator trajectory_last,
                         Tracking_Trajectory_Iterator tracking_trajectory_first,
                         Result_Iterator result_first ) {
    assert( frame1<frame2 );
    Reconstruction_Trajectory_Iterator trajectory_first0( trajectory_first );
    typedef typename std::iterator_traits<Tracking_Trajectory_Iterator>::value_type::data_type tracking_trajectory_data_type;
    tracking_trajectory_data_type frame1l = static_cast<tracking_trajectory_data_type>( frame1 );
    tracking_trajectory_data_type frame2l = static_cast<tracking_trajectory_data_type>( frame2 );
    while ( trajectory_first!=trajectory_last ) {
        tracking_trajectory_data_type start  = tracking_trajectory_first[ *trajectory_first ].start( );
        tracking_trajectory_data_type finish = tracking_trajectory_first[ *trajectory_first ].finish( );
        if ( start<=frame1l && finish>frame2l ) {
            *result_first = trajectory_first-trajectory_first0;
            ++result_first;
        }
        ++trajectory_first;
    }
    return result_first;
}

template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
int nonkeyframe_insertion( Reconstruction& recon,
                          std::size_t frame1, std::size_t frame2,
                          Tracking_Trajectory_Iterator tracking_trajectory_first,
                          Tracking_Trajectory_Iterator tracking_trajectory_last,
                          Tracking_Feature_Iterator tracking_feature_first,
                          std::size_t tracking_feature_size,
                          Data_Type fl, // guess for the focal length
                          Data_Type ic1, Data_Type ic2, Data_Type ar,
                          int intrinsic_mode,
                          Data_Type bisquare_sigma ) {
    // Check if ( frame1, frame2 ) are reconstructed already
    assert( frame1<frame2 );
    assert( recon.find_camera( frame1 )!=recon.camera_end( ) );
    assert( recon.find_camera( frame2 )!=recon.camera_end( ) );

    // Find the reconstructed trajectories that span ( frame1, frame2 )
    std::vector<std::size_t> candidates( tracking_feature_size );
    std::vector<std::size_t>::iterator
        candidate_last = find_keyframe_trajectory( frame1, frame2, 
                                                  recon.inlier_2d_trajectory_begin( ), 
                                                  recon.inlier_2d_trajectory_end( ), 
                                                  tracking_trajectory_first,
                                                  candidates.begin( ) );
    std::size_t candidate_size = candidate_last-candidates.begin( );

    // Find the points associated with the trajectories
    std::vector<std::size_t>::const_iterator
        candidate_first = candidates.begin( );
    std::vector<Data_Type> knowns( candidate_size*3 );
    std::vector<Data_Type>::iterator known_first = knowns.begin( );
    Reconstruction::point2_const_iterator point_first = recon.point2_begin( );
    for ( std::size_t ii=0;ii<candidate_size;++ii ) {
        //std::size_t point_id = recon.trajectory_begin( )[ *candidate_first ].point_id( );
        math::spherical_to_cartesian( point_first[ *candidate_first ].x( ),
                                     point_first[ *candidate_first ].y( ),
                                     known_first[ 0 ], 
                                     known_first[ 1 ], 
                                     known_first[ 2 ] );
        ++candidate_first;
        known_first += 3;
    }

    std::vector<Matrix33> rotations( frame2-frame1-1 );
    std::vector<Data_Type> focal_lengths( frame2-frame1-1 );

    if ( intrinsic_mode==1 ) {
        if ( candidate_size<3 ) return 3;
        std::vector<Data_Type> meas( candidate_size*2 );
        // Compute the motion for in-between images
        for ( std::size_t frame_n=frame1+1; frame_n<frame2;++frame_n ) {
            std::vector<std::size_t>::const_iterator
                candidate_first = candidates.begin( );
            std::vector<Data_Type>::iterator meas_first = meas.begin( );
            Tracking_Feature_Iterator
                tracking_feature_n_first = tracking_feature_first+frame_n*tracking_feature_size;
            for ( std::size_t ii=0;ii<candidate_size;++ii ) {
                std::size_t reference_offset = tracking_trajectory_first[ recon.inlier_2d_trajectory_begin( )[ *candidate_first ] ].offset( );
                meas_first[ 0 ] = tracking_feature_n_first[ reference_offset ].x( );
                meas_first[ 1 ] = tracking_feature_n_first[ reference_offset ].y( );
                ++candidate_first;
                meas_first += 2;
            }

            typedef utility::step_iterator_adaptor<const Data_Type*, 2> step2_adaptor_t;
            typedef utility::step_iterator_adaptor<const Data_Type*, 3> step3_adaptor_t;

            /// \todo Replace the linear estimator with a RANSAC solver
            /// \todo Need to take par into account
            mvg::rotation3_resection_linear<Data_Type>
                linear_resection_solver( candidate_size,
                                        step3_adaptor_t( &knowns.front( )   ), // work-around VC debug iterators
                                        step3_adaptor_t( &knowns.front( )+1 ), // work-around VC debug iterators
                                        step3_adaptor_t( &knowns.front( )+2 ), // work-around VC debug iterators
                                        step2_adaptor_t( &meas.front( )   ),   // work-around VC debug iterators
                                        step2_adaptor_t( &meas.front( )+1 ),   // work-around VC debug iterators
                                        fl, ic1, ic2, ar );
            if ( linear_resection_solver.is_failed( ) ) return 1;
            
            // Refine the camera motion
            /// \todo Need to take par into account
            std::size_t resection_max_iterations = 10;
            mvg::rotation3_fl_resection_robust<Data_Type, mvg::robust_cost_function_bisquare_2<Data_Type>, Progress_Monitor, Message_Reporter> 
                rotation_resection_solver( candidate_size,
                                          step3_adaptor_t( &knowns.front( )   ), // work-around VC debug iterators
                                          step3_adaptor_t( &knowns.front( )+1 ), // work-around VC debug iterators
                                          step3_adaptor_t( &knowns.front( )+2 ), // work-around VC debug iterators
                                          step2_adaptor_t( &meas.front( )   ),   // work-around VC debug iterators
                                          step2_adaptor_t( &meas.front( )+1 ),   // work-around VC debug iterators
                                          math::make_iterator_2d_n<3>( linear_resection_solver.rotation_begin( ) ),
                                          fl, ic1, ic2, ar,
                                          mvg::robust_cost_function_bisquare_2<Data_Type>( bisquare_sigma ),
                                          resection_max_iterations );
            if ( rotation_resection_solver.is_failed( ) ) return 2;

            math::rodrigues( rotation_resection_solver.rotation_vector_begin( )[ 0 ], 
                            rotation_resection_solver.rotation_vector_begin( )[ 1 ], 
                            rotation_resection_solver.rotation_vector_begin( )[ 2 ], 
                            math::make_iterator_2d_n<3>( rotations[ frame_n-frame1-1 ].begin( ) ) );
            focal_lengths[ frame_n-frame1-1 ] = rotation_resection_solver.focal_length( );
        }
    } else if ( intrinsic_mode==2 ) {
        assert( 0 );
    } else throw std::runtime_error( "Unsupported intrinsic mode" );

    // Add additional trajectories 
    std::vector<std::size_t> trajectory_n( tracking_feature_size*( frame2-frame1+1 ) );
    std::vector<std::size_t>::iterator
        trajectory_n_first = trajectory_n.begin( );
    std::vector<std::size_t>::iterator trajectory_n_last =
        find_nonkeyframe_trajectory( frame1, frame2,
                                    tracking_trajectory_first,
                                    tracking_trajectory_last,
                                    trajectory_n.begin( ) );

    std::size_t trajectory_size_n = trajectory_n_last-trajectory_n.begin( );

    /*
    // Reconstruct points for the new trajectories
    typedef mvg::shared_intrinsic<Image_Intrinsic, Image_Intrinsic_Mode> shared_control_type;
    //typedef mvg::motion_control_so3_new<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode> motion_control_type;
    typedef mvg::robust_cost_function_bisquare_2<Data_Type> Robust_Cost_Function;
    typedef mvg::motion_control_so3_robust<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode, Robust_Cost_Function > motion_control_type;

    std::vector<Image_Intrinsic> intrinsics;
    std::vector<Matrix33> rotations;
    for ( std::size_t ii=frame1; ii<=frame2;++ii ) {
        intrinsics.push_back( recon.find_camera( ii )->second.intrinsic );
        rotations.push_back( recon.find_camera( ii )->second.motion );
    }
    std::vector<Vector2> points( trajectory_size_n );
    std::vector<Vector2>::iterator point_first = points.begin( );
    while ( trajectory_n_first!=trajectory_n_last ) {
        Reconstruction_Trajectory_Data_Type
            start = trajectory_n_first->start( ),
            finish = trajectory_n_first->finish( ),
            reference_index = trajectory_n_first->reference_index( );
        assert( start>=frame1 && finish<=frame2+1 );
        assert( !( start==frame1&&finish==frame2+1 ) );
        if ( finish==frame2+1 ) start = frame2;
        int offset = tracking_trajectory_first[ reference_index ].offset( );
        Tracking_Feature_Iterator
            feature_first = tracking_feature_first+start*tracking_feature_size;
        Data_Type x[ 2 ];
        x[ 0 ] = static_cast<Data_Type>( feature_first[ offset ].x( ) );
        x[ 1 ] = static_cast<Data_Type>( feature_first[ offset ].y( ) );
        image_to_polar_unknown( intrinsics[ start-frame1 ],
                                math::make_iterator_2d_n<3>( rotations[ start-frame1 ].begin( ) ),
                                x, y,
                                point_first->x( ), point_first->y( ) );
        ++trajectory_n_first;
        ++point_first;
    }

    // N-view bundle adjustment of ( frame1, ..., frame2 )
    typedef mvg::nview_solution_trajectory_correspondence<Data_Type, 2, 2, 2> solution_type;
    solution_type mstc;
    mstc.resize( frame2-frame+1 );

    typedef reconstruction_meas_copier<Tracking_Trajectory_Iterator,
                                        Tracking_Feature_Iterator> meas_copier_t;
    typedef point_unknown2_copier<std::vector<Vector2>::const_iterator> point_unknown_copier_t;
    mstc.add_coupled_point_measurements( utility::value_iterator<std::size_t>( frame1   ),
                                        utility::value_iterator<std::size_t>( frame2+1 ),
                                        trajectory_n.begin( ),
                                        trajectory_n.end( ),
                                        point_unknown_copier_t( points.begin( ) ),
                                        meas_copier_t( tracking_trajectory_first, 
                                                        tracking_feature_first,
                                                        tracking_feature_size ) );
    typedef mvg::sparse_bundle_adjuster3<Data_Type, 
                                            shared_control_type, 
                                            motion_control_type,
                                            mvg::default_point23_unknown_updater,
                                            Progress_Monitor,
                                            Message_Reporter> optimizer_type;
    optimizer_type sba_solver( mstc,
                                shared_control,
                                motion_controls.begin( ),
                                mvg::default_point23_unknown_updater( ),
                                Progress_Monitor( ),
                                Message_Reporter( ),
                                true, 
                                nonkeyframe_sba_max_iterations );
    if ( sba_solver.is_failed( ) ) return 4;

    // Remove outliers

    // Add results into reconstruction
    Image_Intrinsic intri_n( rotation_resection_solver.focal_length( ), ic1, ic2, ar );
    recon.insert_camera( std::make_pair( frame_n, std::make_pair( intri_n, rotation_n ) ) );

    // Merge inlier trajectories with the existing ones
    std::size_t trajectory_size_o = recon.number_of_trajectories( );
    recon.resize_trajectory( trajectory_size_o+trajectory_size_n );
    std::copy( trajectory_n.begin( ), trajectory_n_last,
                recon.trajectory_begin( )+trajectory_size_o );
    std::inplace_merge( recon.trajectory_begin( ),
                        recon.trajectory_begin( )+trajectory_size_o,
                        recon.trajectory_end( ),
                        detail::reference_index_comparator2_t<Reconstruction_Trajectory>( ) );
    */

    return 0;
}

/// \todo Return a more meaningful value
template <typename Tracking_Trajectory_Iterator,
          typename Tracking_Feature_Iterator>
int nview_videopano_reconstruction( Reconstruction& recon,
                                   std::size_t frame_first, std::size_t frame_last,
                                   Tracking_Trajectory_Iterator tracking_trajectory_first,
                                   Tracking_Trajectory_Iterator tracking_trajectory_last,
                                   Tracking_Feature_Iterator tracking_feature_first,
                                   std::size_t tracking_feature_size,
                                   std::size_t video_width, std::size_t video_height,
                                   Data_Type fl, Data_Type ic1, Data_Type ic2, Data_Type ar,
                                   int intrinsic_mode,
                                   const videopano_parameter& param ) {

    assert( frame_first<=frame_last );
    write_log_message_green( "Reconstructing images [ %d, ..., %d )\n", frame_first, frame_last );
    if ( frame_first==frame_last ) return 0;

    // Compute keyframes
    std::vector<std::size_t> keyframes;
    std::size_t kf = frame_first;
    while ( kf<frame_last ) {
        keyframes.push_back( kf );
        kf += param.keyframe_frequency;
    }
    if ( kf!=frame_last-1 ) keyframes.push_back( frame_last-1 );
    assert( keyframes.size( )>=2 );

    std::vector<std::size_t>::const_iterator
        keyframe_first = keyframes.begin( ),
        keyframe_last  = keyframes.end( );

    // Two-view initiailization
    write_log_message_green( "Two-view initialization from images ( %d, %d )\n", keyframe_first[ 0 ], keyframe_first[ 1 ] );
    /// \todo Change the hard-coded parameters
    int twoview_result = 
        twoview_initialization( recon, 
                               keyframe_first[ 0 ], keyframe_first[ 1 ],
                               fl, fl, 
                               ic1, ic2, ar,
                               intrinsic_mode,
                               tracking_trajectory_first, tracking_trajectory_last,
                               tracking_feature_first, tracking_feature_size,
                               param.twoview_ransac_threshold,
                               param.ransac_max_trials,
                               param.ransac_confidence,
                               param.twoview_sba_max_iterations,
                               param.twoview_outlier_threshold ); // 5, 1000, 0.995, 10, 10 );
    if        ( twoview_result==1 ) {
        write_log_message_red( "RANSAC solver failed for images ( %d, %d )\n", keyframe_first[ 0 ], keyframe_first[ 1 ] );
        return 0;
    } else if ( twoview_result==2 ) {
        write_log_message_red( "Two-view SBA solver failed for images ( %d, %d )\n", keyframe_first[ 0 ], keyframe_first[ 1 ] );
        return 0;
    }
    keyframe_first += 2;

    // Keyframes
    while ( keyframe_first!=keyframe_last ) {
        write_log_message_green( "Adding keyframe %d\n", *keyframe_first );
        int keyframe_result = 
            keyframe_insertion( recon,
                               *keyframe_first,
                               tracking_trajectory_first, tracking_trajectory_last,
                               tracking_feature_first, tracking_feature_size,
                               fl, ic1, ic2, ar,
                               intrinsic_mode,
                               param.robust_sigma );
        if        ( keyframe_result==-1 ) {
            write_log_message_red( "Linear solver failed for keyframe %d\n", *keyframe_first );
            return 0;
        } else if ( keyframe_result==-2 ) {
            write_log_message_red( "Nonlinear solver failed for keyframe %d\n", *keyframe_first );
            return 0;
        } else if ( keyframe_result==-3 ) {
            write_log_message_red( "Not enough candidates found for keyframe %d\n", *keyframe_first );
            return 0;
        } else {
            write_log_message( "Added %d points\n", keyframe_result );
        }

        // Global optimization
        write_log_message_green( "N-view bundle adjustment ( %d views )\n", recon.number_of_cameras( ) );
        mvg::image_intrinsic_mode opt_mode;
        if      ( intrinsic_mode==1 ) opt_mode = mvg::shared_fl1;
        else if ( intrinsic_mode==2 ) opt_mode = mvg::private_fl1;
        else throw std::runtime_error( "Unsupport intrinsic mode" );
        bool nview_sba_result = 
            nview_bundle_adjustment( recon, 
                                    tracking_trajectory_first,
                                    tracking_feature_first, tracking_feature_size,
                                    recon.camera_begin( )->second.intrinsic,
                                    opt_mode,
                                    param.robust_sigma,
                                    param.nview_sba_max_iterations );
        if ( !nview_sba_result ) {
            write_log_message_red( "N-view SBA solver failed\n" );
            /// \todo Print out image indices
            return 0;
        }

        std::size_t outliers = 
            remove_outlier_trajectory_point( recon,
                                            tracking_trajectory_first,
                                            tracking_feature_first,
                                            tracking_feature_size,
                                            param.nview_outlier_threshold );
        write_log_message( "Removed %d outliers\n", outliers );
        //if ( *keyframe_first==330 ) return 0;
        ++keyframe_first;
    }

    // Reconstruction non-keyframes
    if ( recon.number_of_cameras( )<2 ) return 0;
    std::vector<std::size_t> recon_keyframes;
	Reconstruction::camera_const_iterator
        camera_first = recon.camera_begin( );
    while ( camera_first!=recon.camera_end( ) ) {
        recon_keyframes.push_back( camera_first->first );
        ++camera_first;
    }
    std::vector<std::size_t>::const_iterator
        recon_keyframe_first = recon_keyframes.begin( ),
        recon_keyframe_last  = recon_keyframes.end( );
    ++recon_keyframe_first;
    while ( recon_keyframe_first!=recon_keyframe_last ) {
        nonkeyframe_insertion( recon, 
                              recon_keyframe_first[ -1 ], recon_keyframe_first[ 0 ],
                              tracking_trajectory_first, tracking_trajectory_last,
                              tracking_feature_first, tracking_feature_size,
                              fl, ic1, ic2, ar,
                              intrinsic_mode,
                              param.robust_sigma );
        ++recon_keyframe_first;
    }

    return 0;
}

} // namespace videopano
} // namespace adobe_agt

#endif
