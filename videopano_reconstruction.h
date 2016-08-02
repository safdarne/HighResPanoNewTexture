#ifndef ADOBE_AGT_MVG_VIDEOPANO_RECONSTRUCTION_H
#define ADOBE_AGT_MVG_VIDEOPANO_RECONSTRUCTION_H

//
/// \author Hailin Jin \n Adobe Systems Incorporated
/// \date 2010-2011 \n Last updated on May 20, 2011
//

#include <cstddef>
#include <cassert>
#include <utility>
#include <functional>
#include <memory>
#include <vector>
#include <map>
#include <math/vector_n.h>

namespace adobe_agt {
namespace videopano {

/// Class that holds variables for a video panorama reconstruction
template <typename T,
          typename Camera_Type,
          typename Alloc = std::allocator<T> >
class videopano_reconstruction {
public:
    typedef T data_type;
    typedef math::vector_n<T,2> point2_type;
    typedef Camera_Type camera_type;
private:
    // Private typedefs
    typedef typename Alloc::template rebind<camera_type>::other _camera_allocator;
    typedef typename Alloc::template rebind<point2_type>::other _point_allocator;
    typedef typename Alloc::template rebind<std::size_t>::other _size_t_allocator;
    // Data members
    std::size_t _ref_index;
    std::map<std::size_t,
             camera_type,
             std::less<std::size_t>,
             _camera_allocator> _cameras;
    // Do NOT change them individually
    std::vector<point2_type, _point_allocator> _inlier_points;
    std::vector<std::size_t, _size_t_allocator> _inlier_trajectories;
    // Do NOT change them individually
    std::vector<std::size_t, _size_t_allocator> _outlier_trajectories;
    std::vector<std::size_t, _size_t_allocator> _outlier_status;
public:
    // Typedefs
    typedef typename std::map<std::size_t,
                              camera_type,
                              std::less<std::size_t>,
                              _camera_allocator>::      iterator
        camera_iterator;
    typedef typename std::map<std::size_t,
                              camera_type,
                              std::less<std::size_t>,
                              _camera_allocator>::const_iterator
        camera_const_iterator;
    typedef typename std::vector<point2_type, _point_allocator>::      iterator 
        point2_iterator;
    typedef typename std::vector<point2_type, _point_allocator>::const_iterator 
        point2_const_iterator;
    typedef typename std::vector<std::size_t, _size_t_allocator>::      iterator 
        size_t_iterator;
    typedef typename std::vector<std::size_t, _size_t_allocator>::const_iterator 
        size_t_const_iterator;

    // Constructor
    explicit videopano_reconstruction( const Alloc& alloc = Alloc( ) ) :
        _cameras( std::less<std::size_t>( ), _camera_allocator( alloc ) ),
        _inlier_points( _point_allocator( alloc ) ),
        _inlier_trajectories( _size_t_allocator( alloc ) ),
        _outlier_trajectories( _size_t_allocator( alloc ) ),
        _outlier_status( _size_t_allocator( alloc ) ) {}

    // Reference index
    std::size_t& reference_index( )       {return _ref_index;}
    std::size_t  reference_index( ) const {return _ref_index;}

    // Intrinsic and motion
    std::size_t number_of_cameras( ) const {return _cameras.size( );}
    camera_iterator       camera_begin( )       {return _cameras.begin( );}
    camera_const_iterator camera_begin( ) const {return _cameras.begin( );}
    camera_iterator       camera_end( )       {return _cameras.end( );}
    camera_const_iterator camera_end( ) const {return _cameras.end( );}
    void insert_camera( const std::pair<std::size_t, camera_type>& x ) {
        _cameras.insert( x );
    }
    camera_iterator       find_camera( std::size_t view )       {
        return _cameras.find( view );
    }
    camera_const_iterator find_camera( std::size_t view ) const {
        return _cameras.find( view );
    }

    // 2D inlier point and trajectory
    point2_iterator       point2_begin( )       {return _inlier_points.begin( );}
    point2_const_iterator point2_begin( ) const {return _inlier_points.begin( );}
    point2_iterator       point2_end( )         {return _inlier_points.end( );}
    point2_const_iterator point2_end( )   const {return _inlier_points.end( );}
    size_t_iterator       inlier_2d_trajectory_begin( )       {return _inlier_trajectories.begin( );}
    size_t_const_iterator inlier_2d_trajectory_begin( ) const {return _inlier_trajectories.begin( );}
    size_t_iterator       inlier_2d_trajectory_end( )         {return _inlier_trajectories.end( );}
    size_t_const_iterator inlier_2d_trajectory_end( )   const {return _inlier_trajectories.end( );}
    void resize_2d_inliers( std::size_t point_size ) {
        assert( _inlier_points.size( )==_inlier_trajectories.size( ) );
        _inlier_points.resize( point_size );
        _inlier_trajectories.resize( point_size );
    }
    std::size_t number_of_2d_inliers( ) const {
        assert( _inlier_points.size( )==_inlier_trajectories.size( ) );
        return _inlier_points.size( );
    }

    // 2D outlier trajectory
    size_t_iterator       outlier_2d_trajectory_begin( )       {return _outlier_trajectories.begin( );}
    size_t_const_iterator outlier_2d_trajectory_begin( ) const {return _outlier_trajectories.begin( );}
    size_t_iterator       outlier_2d_trajectory_end( )         {return _outlier_trajectories.end( );}
    size_t_const_iterator outlier_2d_trajectory_end( )   const {return _outlier_trajectories.end( );}
    size_t_iterator       outlier_2d_status_begin( )       {return _outlier_status.begin( );}
    size_t_const_iterator outlier_2d_status_begin( ) const {return _outlier_status.begin( );}
    size_t_iterator       outlier_2d_status_end( )         {return _outlier_status.end( );}
    size_t_const_iterator outlier_2d_status_end( )   const {return _outlier_status.end( );}
    void resize_2d_outliers( std::size_t size ) {
        assert( _outlier_trajectories.size( )==_outlier_status.size( ) ); 
        _outlier_trajectories.resize( size );
        _outlier_status.resize( size );
    }
    std::size_t number_of_2d_outliers( ) const {
        assert( _outlier_trajectories.size( )==_outlier_status.size( ) ); 
        return _outlier_trajectories.size( );
    }
    void clear_2d_outliers( ) {_outlier_trajectories.clear( );}
};

} // namespace videopano
} // namespace adobe_agt

#endif
