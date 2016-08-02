//
//  CvEstimator.cpp
//  Opencv-directRotationComp
//
//  Created by safdarne on 6/2/16.
//  Copyright © 2016 Adobe. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ctime>

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>


#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

#include "videopano_algorithm.h"
#include "mvg/rotation3_est.h"
#include "math/CubicSolver.h"
#include "mvg/robust_cost_function.h"
#include "mvg/Progress_Monitor.h"
#include "mvg/message_reporter.h"
#include <utility/iterator_adaptor.h>
#include <math/lapack/lapack_macros.h>
//#include <math/lapack/lapack_call_traits.h>


#include <cstddef>
#include <cstdlib>
#include <utility>
#include <vector>
#include <stdexcept>
#include <boost/gil/image.hpp>
#include <boost/gil/typedefs.hpp>
#include <utility/stopwatch.h>
#include <tracking/tracking_algorithm.h>
#include "videopano_typedef.h"
#include "videopano_algorithm.h"
#include "videopano_render.h"
#include "videopano_image_io.h"




#include <stdio.h>
#include <string>

using namespace cv;
using namespace cv::detail;
using namespace adobe_agt;
using namespace adobe_agt::videopano;

class CvEstimator
{
public:
    CvEstimator( );
    ~CvEstimator( );
private:
    cv::Mat _prevImage;
    cv::Mat _currentImage;
    std::vector<cv::KeyPoint> _prevKeypoints, _currentKeypoints;
    cv::UMat _prevDescriptors, _currentDescriptors;
    std::vector<float> _RFromImageRobust;
    Ptr<Feature2D> _featureDetector;
    cv::Mat _K;
    Mat _RFromSensor;
    std::vector<cv::Mat> _storedImages;
    std::vector<std::vector<cv::KeyPoint>> _storedKeypoints;
    std::vector<cv::UMat> _storedDescriptors;
    std::vector<cv::Mat> _refinedRotationArray;
    cv::Mat H;
    cv::Mat initH;
    std::vector<cv::detail::MatchesInfo> _pairwiseMatches;
    std::vector<ImageFeatures> _features;
    std::vector<CameraParams> _cameras;
    
    float _focal;
    int _width;
    int _height;
    
    double xCanvasF;
    double yCanvasF;
    std::vector<float> gH;
    float roll;
    float pitch;
    float yaw;
    
    void putImageInCanvas ( cv::Mat &image, double xCanvasF, double yCanvasF, cv::Mat &initH, bool resizeFlag );
    std::vector< std::vector<cv::Point>>  overlayFrame( cv::Mat &imgA, cv::Mat &WarpImg, cv::Mat &WarpImgAccu, cv::Mat accuH, int mode );
    void overlayFrameWithMask( cv::Mat &imgA, cv::Mat &imgB, cv::Mat &maskA, cv::Mat &maskB, std::vector<cv::Point> &corners, cv::Mat &overlaid );
    void computeHomographyFromRotation( cv::Mat R, float focal, cv::Point2d principalPoint, cv::Mat &H, cv::Mat &K );
    void findRotationFromKeypointMatches( std::vector<cv::Point2f> &cur, std::vector<cv::Point2f> &prev, cv::Mat &K, cv::Mat &R );
    void robustKeypointMatching( int width, int height, cv::Mat &HFromSensor, std::vector<cv::KeyPoint> &prevKeypoints, std::vector<cv::KeyPoint> &currentKeypoints, cv::UMat &prevDescriptors, cv::UMat &currentDescriptors, std::vector< cv::DMatch > &good_matches, std::vector<cv::Point2f> &prev, std::vector<cv::Point2f> &cur, cv::Mat &Mask );
    cv::Vec3d estimateEulerAngles( std::vector<cv::Point2f> &cur, std::vector<cv::Point2f> &prev, float focal, int height, int width );
    void drawFinalMatches( cv::Mat &prevImage, std::vector<cv::KeyPoint> &prevKeypoints, cv::Mat &currentImage, std::vector<cv::KeyPoint> &currentKeypoints, std::vector< cv::DMatch > &good_matches, cv::Mat &Mask, cv::Mat &result );
    void copyBAResultToCamerArray( int num_images, std::vector<CameraParams> &cameras,  std::vector<CameraParams> &initialCameras );
    void copyRotationToCameraArray( std::vector<Mat> &currentRotationUsedToInitBA, std::vector<CameraParams>& cameras );
    
public:
    void saveNextFrame( cv::Mat &nextFrame );
    void procFrame( cv::Mat &nextFrame, bool flag );
    void initialize( int width, int height );
    std::vector<float> getRotation( );
    void setRotationFromSensor( std::vector<float> &rot );
    void restart( );
    std::vector<cv::Mat> refinedStitching( std::vector<Mat> &currentRotationUsedToInitBA, const std::vector<std::vector<int>> &closestFrames );
    float getFocalLength( );
    cv::Mat computeRotationMatrixFromEulerAngles( float roll, float pitch, float yaw );
    void setFocalLength( float focal );
    
};


CvEstimator::CvEstimator( ):_focal( 1116 )
{
    // In Class Pano ( display module ), the index 0 is reserved for camera live feed, so, start from index 1 for compatibility.
    std::vector<cv::KeyPoint> temp;
    _storedImages.push_back( cv::Mat( ) );
    _storedKeypoints.push_back( temp );
    _storedDescriptors.push_back( cv::UMat( ) );
}

CvEstimator::~CvEstimator( )
{
}

float CvEstimator::getFocalLength( )
{
    return _focal;
}

void CvEstimator::putImageInCanvas ( cv::Mat &image, double xCanvasF, double yCanvasF, cv::Mat &initH, bool resizeFlag )
{
    float gH2[ 9 ]={1,0, static_cast<float>( ( xCanvasF/2 - 0.5 ) * image.cols ), 0,1, static_cast<float>( ( yCanvasF/2 - 0.5 ) * image.rows ), 0,0,1};
    cv::Mat temp = cv::Mat( 3, 3, CV_32FC1, gH2 );
    temp.copyTo( initH );
    cv::Mat tempImage;
    warpPerspective( image, tempImage, initH, cv::Size( xCanvasF * image.cols, yCanvasF * image.rows ) );
    if ( resizeFlag )
        resize( tempImage, image, cv::Size( image.cols, image.rows ) );
    else
        tempImage.copyTo( image );
}


std::vector< std::vector<cv::Point>>  CvEstimator::overlayFrame( cv::Mat &imgA, cv::Mat &WarpImg, cv::Mat &WarpImgAccu, cv::Mat accuH, int mode ) {
    // First finds a mask showing the area for which the new warped frame has values, and then,
    // replaces the values on the overlay ( "WarpImgAccu" ) with the values from this new frame ( "WarpImg" )
    cv::Mat mask( WarpImg.rows, WarpImg.cols, CV_8UC1, cv::Scalar( 0 ) );
    
    cv::Point P1( 1,1 );
    cv::Point P2( 1, imgA.rows-1-1 );
    cv::Point P3( imgA.cols-1-1,imgA.rows-1-1 );
    cv::Point P4( imgA.cols-1-1,1 );
    
    std::vector< std::vector<cv::Point>> co_ordinates;
    co_ordinates.push_back( std::vector<cv::Point>( ) );
    std::vector<cv::Point2f> vec;
    vec.push_back( P1 );
    vec.push_back( P2 );
    vec.push_back( P3 );
    vec.push_back( P4 );
    perspectiveTransform( vec, vec, accuH );
    co_ordinates[ 0 ].push_back( vec[ 0 ] );
    co_ordinates[ 0 ].push_back( vec[ 1 ] );
    co_ordinates[ 0 ].push_back( vec[ 2 ] );
    co_ordinates[ 0 ].push_back( vec[ 3 ] );
    drawContours( mask,co_ordinates,0, cv::Scalar( 255 ),CV_FILLED, 8 );
    
    switch ( mode ) {
        case 0:
            WarpImg.copyTo( WarpImgAccu, mask );
            break;
        case 1:
            WarpImg.copyTo( WarpImgAccu );
            break;
    }
    //drawContours( WarpImgAccu,co_ordinates,0, Scalar( 255 ), 1, 8 );
    return co_ordinates;
}

void CvEstimator::overlayFrameWithMask( cv::Mat &imgA, cv::Mat &imgB, cv::Mat &maskA, cv::Mat &maskB, std::vector<cv::Point> &corners, cv::Mat &overlaid ) {
    //cv::detail::Blender temp;
    //cv::Ptr<cv::detail::Blender> blender;
    /*
     int blend_type = cv::detail::FEATHER;
     float blend_strength = 50;
     
     std::vector<cv::Size> sizes( 2 );
     sizes[ 0 ].width = imgA.cols;
     sizes[ 0 ].height = imgA.rows;
     sizes[ 1 ].width = imgB.cols;
     sizes[ 1 ].height = imgB.rows;
     
     cv::Size dst_sz = cv::detail::resultRoi( corners, sizes ).size( );
     float blend_width = sqrt( static_cast<float>( dst_sz.area( ) ) ) * blend_strength / 100.f;
     
     blender = Blender::createDefault( blend_type );
     
     cv::detail::blender::FeatherBlender* fb = dynamic_cast<FeatherBlender*>( blender.get( ) );
     fb->setSharpness( 1.f/blend_width );
     
     blender->prepare( corners, sizes );
     
     
     cv::Mat imgA_s, imgB_s;
     
     cvtColor( imgA,imgA_s, cv::COLOR_RGBA2BGR );
     cvtColor( imgB,imgB_s, cv::COLOR_RGBA2BGR );
     
     
     
     imgA_s.convertTo( imgA_s, CV_16S );
     imgB_s.convertTo( imgB_s, CV_16S );
     
     
     
     //std::cout << imgA_s.type( ) << std::endl;
     //std::cout << imgA_s.channels( ) << std::endl;
     
     blender->feed( imgA_s, maskA, corners[ 0 ] );
     blender->feed( imgB_s, maskB, corners[ 1 ] );
     
     
     cv::Mat result_mask;
     blender->blend( overlaid, result_mask );
     overlaid.convertTo( overlaid, CV_8U );*/
}



cv::Mat CvEstimator::computeRotationMatrixFromEulerAngles( float roll, float pitch, float yaw ) {
    cv::Mat R = cv::Mat::zeros( 3, 3, CV_64FC1 );
    
    cv::Mat XX = cv::Mat::zeros( 3, 3, CV_64FC1 );
    cv::Mat YY = cv::Mat::zeros( 3, 3, CV_64FC1 );
    cv::Mat ZZ = cv::Mat::zeros( 3, 3, CV_64FC1 );
    
    XX.at<double>( 0,0 ) = 1;
    XX.at<double>( 1,1 ) = cos( roll );
    XX.at<double>( 1,2 ) = -sin( roll );
    XX.at<double>( 2,1 ) = sin( roll );
    XX.at<double>( 2,2 ) = cos( roll );
    
    YY.at<double>( 1,1 ) = 1;
    YY.at<double>( 0,0 ) = cos( pitch );
    YY.at<double>( 0,2 ) = sin( pitch );
    YY.at<double>( 2,0 ) = -sin( pitch );
    YY.at<double>( 2,2 ) = cos( pitch );
    
    ZZ.at<double>( 2,2 ) = 1;
    ZZ.at<double>( 0,0 ) = cos( yaw );
    ZZ.at<double>( 0,1 ) = -sin( yaw );
    ZZ.at<double>( 1,0 ) = sin( yaw );
    ZZ.at<double>( 1,1 ) = cos( yaw );
    
    R = ZZ * YY * XX;
    return R;
}


void CvEstimator::computeHomographyFromRotation( cv::Mat R, float focal, cv::Point2d principalPoint, cv::Mat &H, cv::Mat &K ) {
    K = cv::Mat::zeros( 3, 3, CV_64FC1 );
    K.at<double>( 0,0 ) = focal;
    K.at<double>( 1,1 ) = focal;
    K.at<double>( 2,2 ) = 1.0;
    
    K.at<double>( 0,2 ) = principalPoint.x;
    K.at<double>( 1,2 ) = principalPoint.y;
    
    cv::Mat temp2 = K * R * K.inv( );
    temp2.copyTo( H );
}


void CvEstimator::findRotationFromKeypointMatches( std::vector<cv::Point2f> &cur, std::vector<cv::Point2f> &prev, cv::Mat &K, cv::Mat &R ) {
    
    cv::Mat F = findFundamentalMat( cur, prev, cv::FM_RANSAC, 3., 0.97 );
    
    cv::Mat E = K.t( ) * F * K;
    
    
    cv::SVD svd = cv::SVD( E );
    cv::Matx33d W( 0,-1,0,   //HZ 9.13
                  1,0,0,
                  0,0,1 );
    cv::Matx33d Winv( 0,1,0,
                     -1,0,0,
                     0,0,1 );
    
    
    R = svd.u * cv::Mat( W ) * svd.vt; //HZ 9.19
    ////Mat t = svd.u.col( 2 ); //u3
    
    /*
     P1 = Matx34d( R( 0,0 ),    R( 0,1 ), R( 0,2 ), t( 0 ),
     R( 1,0 ),    R( 1,1 ), R( 1,2 ), t( 1 ),
     R( 2,0 ),    R( 2,1 ), R( 2,2 ), t( 2 ) );
     */
}


void CvEstimator::robustKeypointMatching( int width, int height, Mat &HFromSensor, std::vector<KeyPoint> &prevKeypoints, std::vector<KeyPoint> &currentKeypoints, cv::UMat &prevDescriptors, cv::UMat &currentDescriptors, std::vector< DMatch > &good_matches, std::vector<Point2f> &prev, std::vector<Point2f> &cur, Mat &Mask ) {
    
    
    std::vector< DMatch > wellDistributed;
    std::vector<DMatch> matches, sortedMatches;
    std::vector<Point2f> curTransformed;
    
    Ptr<DescriptorMatcher>  descriptorMatcher = DescriptorMatcher::create( String( "BruteForce" ) );
    descriptorMatcher->match( prevDescriptors, currentDescriptors, matches, Mat( ) );
    
    if ( matches.size( ) > 0 )
    {
        
        Mask = Mat::zeros( Mask.rows, Mask.cols, Mask.type( ) );
        
        
        // -- Rejection of keypoints matches based on device motion sensor result and distribution of keypoints. First, keypoints are sorted based on the match strength. Then, take well distributed keypoints, and then, check if the keypoints transformed under estimated transformation from motion sensor match well with matched keypoints.
        Mat index;
        int nbMatch=int( matches.size( ) );
        
        Mat tab( nbMatch, 1, CV_32F );
        for ( int i = 0; i < nbMatch; i++ ) {
            tab.at<float>( i, 0 ) = matches[ i ].distance;
        }
        sortIdx( tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING );
        
        for ( int i = 0; i < nbMatch; i++ ) {
            sortedMatches.push_back( matches[ index.at<int>( i, 0 ) ] );
        }
        
        matches = sortedMatches;
        sortedMatches.clear( );
        
        
        for ( int i = 0; i < matches.size( ); i++ ){
            Point2d currCoord = prevKeypoints[ matches[ i ].queryIdx ].pt;
            int p = 5; // fixme, neighborhoood should be larger than 0, and dependent on resolution
            int xCorner = min( max( 0.0, round( currCoord.x ) - p ), double( height ) - 1 );
            int yCorner = min( max( 0.0, round( currCoord.y ) - p ), double( width ) - 1 );
            int w = min( double( height - xCorner ), double( 2*p ) );
            int h = min( double( width - yCorner ), double( 2*p ) );
            
            Mat roi = Mask( cv::Rect( xCorner, yCorner, w, h ) );
            //!!!!!!!!!!!!!!!!!!!! ROBUST DISABLED
            if ( true ){//Mask.at<char>( round( currCoord.y ), round( currCoord.x ) ) == 0 ) { // Only push in, if there is not a keypoint nearby already
                prev.push_back( currCoord );
                cur.push_back( currentKeypoints[ matches[ i ].trainIdx ].pt );
                wellDistributed.push_back( matches[ i ] );
                roi.setTo( Scalar( 1 ) );
            }
            //else
            //printf( "Thrown away %d\n", Mask.at<char>( round( currCoord.y ), round( currCoord.x ) ) );
        }
        
        // If there is no motion sensor estimation, avoid filtering by copying the source keypoints to the destination keypoints
        //std::cout << HFromSensor << std::endl;
        if ( HFromSensor.rows > 0 ) //// fixme
            perspectiveTransform( cur, curTransformed, HFromSensor );
        else
            curTransformed = prev;
        
        for ( int i = 0; i < prev.size( ); i++ ){
            //if ( norm( prev[ i ] - curTransformed[ i ] ) < 50 ) //fixme //!!!!!!!!!!!!!!!!!!!! ROBUST DISABLED
                good_matches.push_back( wellDistributed[ i ] );
            //else
            //printf( "Thrown away 2\n" );
        }
        prev.clear( );
        cur.clear( );
        for(  int i = 0; i < good_matches.size( ); i++  ){
            //-- Get the keypoints from the good matches
            prev.push_back( prevKeypoints[ good_matches[ i ].queryIdx ].pt  );
            cur.push_back( currentKeypoints[ good_matches[ i ].trainIdx ].pt  );
        }
    }
    else
        std::cout << "Could not match" << std::endl;
}

cv::Vec3d CvEstimator::estimateEulerAngles( std::vector<cv::Point2f> &cur, std::vector<cv::Point2f> &prev, float focal, int height, int width ) {
    //-- Using Essential matrix, find roll, pitch, and yaw from matching keypoitns
    // http://stackoverflow.com/questions/31447128/camera-pose-estimation-how-do-i-interpret-rotation-and-translation-matrices
    Mat t;
    Mat R2;
    
    Point2d principalPoint = Point2d( height/2,width/2 );
    Mat E = findEssentialMat( cur, prev, focal, principalPoint, RANSAC );
    int inliers = recoverPose( E, cur, prev, R2, t, focal, principalPoint );
    Mat mtxR, mtxQ;
    Mat Qx, Qy, Qz;
    Vec3d angles = RQDecomp3x3( R2, mtxR, mtxQ, Qx, Qy, Qz );
    return angles;
}


void CvEstimator::drawFinalMatches( cv::Mat &prevImage, std::vector<cv::KeyPoint> &prevKeypoints, cv::Mat &currentImage, std::vector<cv::KeyPoint> &currentKeypoints, std::vector< cv::DMatch > &good_matches, cv::Mat &Mask, cv::Mat &result ) {
    cv::Mat matrix;
    prevImage.convertTo( matrix, CV_32FC1, 1/255.0 );
    Mask.convertTo( Mask, CV_32FC1 );
    
    multiply( matrix, Mask/2 + 0.5, matrix );
    matrix.convertTo( matrix, CV_8UC1, 255 );
    
    drawMatches( matrix, prevKeypoints, currentImage, currentKeypoints, good_matches, result, cv::Scalar( 0,255,0 ) );
}

void CvEstimator::saveNextFrame( Mat &nextFrame )
{
    nextFrame.copyTo( _prevImage );
    
    
    _featureDetector ->detect( _prevImage, _prevKeypoints, Mat( ) );
    _featureDetector ->compute( _prevImage, _prevKeypoints, _prevDescriptors );
    
    _storedImages.push_back( nextFrame );
    _storedKeypoints.push_back( _prevKeypoints );
    _storedDescriptors.push_back( _prevDescriptors );
    
    
    
    if ( _storedDescriptors.size( ) > 2 && _prevKeypoints.size( ) > 10 )
    {
        Mat RFromImageRobust, HFromSensor;
        std::vector< DMatch > good_matches, wellDistributed;
        std::vector<DMatch> matches, sortedMatches;
        std::vector<Point2f> prev, cur;
        std::vector<float> x1, x2, y1, y2;
        
        // Compute homograpjhy from rotaion matrix via the motion sensor information, to be used to rectify the keypoint matches
        computeHomographyFromRotation( _RFromSensor,  _focal, cv::Point2d( _width / 2, _height / 2 ) , HFromSensor, _K );
        
        int MAXMATCHES = 100;
        Mat R = Mat::zeros( 3, 3, CV_64FC1 );
        Mat K;
        
        _RFromSensor.copyTo( R );
        Ptr<DescriptorMatcher>  descriptorMatcher = DescriptorMatcher::create( String( "BruteForce" ) );
        
        // Match keypoints
        descriptorMatcher -> match(  _storedDescriptors[ _storedDescriptors.size( ) - 1 - 1 ], _storedDescriptors.back( ), matches, Mat( )  );
        Mat index;
        int nbMatch=int( matches.size( ) );
        
        // Only store a maximum of 100 strongest matches ( MAXMATCHES )
        Mat tab( nbMatch, 1, CV_32F );
        for ( int i = 0; i < nbMatch; i++ ) {
            tab.at<float>( i, 0 ) = matches[ i ].distance;
        }
        if ( nbMatch > 0 )
        {
            sortIdx( tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING );
            for ( int i = 0; i < min( nbMatch, MAXMATCHES ); i++ ) {
                sortedMatches.push_back( matches[ index.at<int>( i, 0 ) ] );
            }
            matches = sortedMatches;
            sortedMatches.clear( );
        }
        
        // Extract coordinates of matches
        std::vector<cv::KeyPoint> prevKeypoints, currentKeypoints;
        prevKeypoints = _storedKeypoints[ _storedDescriptors.size( ) - 1 - 1 ];
        currentKeypoints = _storedKeypoints.back();
        
        for ( int j = 0; j < matches.size( ); j++ )
        {
            Point2f prevTemp, curTemp;
            int queryIdx = matches[ j ].queryIdx;
            int trainIdx = matches[ j ].trainIdx;
            prevTemp = prevKeypoints[ queryIdx ].pt;
            curTemp = currentKeypoints[ trainIdx ].pt;
            
            prev.push_back( prevTemp );
            cur.push_back( curTemp );
        }
        /*
        Data_Type fl, fl1, fl2, ic1, ic2, ar;
        fl = _focal;
        fl1 = _focal;
        fl2 = _focal;
        ic1 = _width / 2;
        ic2 = _height / 2;
        ar = 1;
        
        std::size_t video_width = static_cast<std::size_t>( _width );
        std::size_t video_height = static_cast<std::size_t>( _height );
        std::size_t video_fps = static_cast<std::size_t>( 30 );
        adobe_agt::videopano::Reconstruction recon;
        std::vector<adobe_agt::videopano::Tracking_Feature> tracking_features;
        std::vector<adobe_agt::videopano::Tracking_Trajectory> tracking_trajectories;
        adobe_agt::videopano::videopano_parameter param;
        param.compute_derived_parameters(video_width, video_height, video_fps);
        
        //*************************************
        // This is actually what inside the twoview_initialization is being done
        typedef mvg::pair_solution2<Data_Type, 2, 2> pair_solution_type;
        std::size_t correspondences = x1.size();
        typedef mvg::pair_solution2<Data_Type, 2, 2> pair_solution_type;
        pair_solution_type ps(correspondences);
        
        typedef std::vector<Data_Type>::iterator Tracking_Feature_Iterator;

        pair_solution_type::iterator
        meas1_first = ps.unknown_measurement_0_begin(),
        meas2_first = ps.unknown_measurement_1_begin();
        
        int indice_first = 0;
        //while (indice_first!=indice_last) {
        while (indice_first < correspondences)
        {
            meas1_first[0] = x1[indice_first];
            meas1_first[1] = y1[indice_first];
            meas2_first[0] = x2[indice_first];
            meas2_first[1] = y2[indice_first];
            
            ++indice_first;
            meas1_first += 2;
            meas2_first += 2;
        }
        
        typedef utility::step_offset_iterator<pair_solution_type::const_iterator, 2> vector2_iterator;
        
        mvg::rotation3_est_2view_ransac_3point<Data_Type>
        ransac_solver(correspondences,
                      vector2_iterator(ps.unknown_measurement_0_begin(), 0), vector2_iterator(ps.unknown_measurement_0_begin(), 1),
                      vector2_iterator(ps.unknown_measurement_1_begin(), 0), vector2_iterator(ps.unknown_measurement_1_begin(), 1),
                      fl1, fl2,
                      ic1, ic2, ar,
                      ic1, ic2, ar,
                      50,
                      std::size_t(  2000  ),
                      double(  0.995  )
                      );
        
        if (ransac_solver.is_failed())
            std::cout << "ransac_solver_failed" << std::endl;
        
        
        int intrinsic_mode = 1;
        typedef mvg::shared_intrinsic<Image_Intrinsic, Image_Intrinsic_Mode> shared_control_type;
        typedef mvg::motion_control_so3_new<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode> motion_control_type;
        shared_control_type sc;
        motion_control_type mc[2];
        Matrix33 rot0, rot1;
        fill_identity_k<3>(math::make_iterator_2d_n<3>(rot0.begin()));
        std::copy(ransac_solver.best_rotation().data_matrix().begin(),
                  ransac_solver.best_rotation().data_matrix().end(),
                  rot1.begin());
        if        (intrinsic_mode==1) {
            Data_Type fl = (ransac_solver.best_focal_length_0()+
                            ransac_solver.best_focal_length_1())/2;
            Image_Intrinsic ii(fl, ic1, ic2, ar);
            sc    = shared_control_type(ii, mvg::shared_fl1);
            mc[0] = motion_control_type(ii, mvg::shared_fl1, math::make_iterator_2d_n<3>(rot0.begin()), 0);
            mc[1] = motion_control_type(ii, mvg::shared_fl1, math::make_iterator_2d_n<3>(rot1.begin()), 1);
        } else if (intrinsic_mode==2) {
            Data_Type fl0 = ransac_solver.best_focal_length_0();
            Data_Type fl1 = ransac_solver.best_focal_length_1();
            Data_Type fl = (fl0+fl1)/2;
            sc =    shared_control_type(Image_Intrinsic(fl , ic1, ic2, ar), mvg::private_fl1);
            mc[0] = motion_control_type(Image_Intrinsic(fl0, ic1, ic2, ar), mvg::private_fl1, math::make_iterator_2d_n<3>(rot0.begin()), 0);
            mc[1] = motion_control_type(Image_Intrinsic(fl1, ic1, ic2, ar), mvg::private_fl1, math::make_iterator_2d_n<3>(rot1.begin()), 1);
        } else throw std::runtime_error("Unsupported intrinsic mode");
        
        adobe_agt::videopano::detail::compute_twoview_unknown(correspondences,
                                                              ps.unknown_measurement_0_begin(),
                                                              ps.unknown_measurement_1_begin(),
                                                              mc[0].get_image_intrinsic(),
                                                              mc[1].get_image_intrinsic(),
                                                              math::make_iterator_2d_n<3>(rot0.begin()),
                                                              math::make_iterator_2d_n<3>(rot1.begin()),
                                                              ps.unknown_begin());
        
        
        cv::Mat estimatedMat = cv::Mat::zeros( 3, 3, CV_32FC1 );
        
        //// fixme, for test, take it out
        int jj = 0;
        for (  auto it = rot1.begin( ); it < rot1.end( ); it++  )
        {
            estimatedMat.at<float> ( floor ( jj / 3), jj % 3 ) = float(*it);
            jj++;
        }
        ////
        std::cout << estimatedMat << std::endl;
        
        std::size_t sba_max_iterations = param.twoview_sba_max_iterations;
        Data_Type twoview_outlier_threshold = 10;
        
        mvg::sparse_bundle_adjuster4<Data_Type,
        shared_control_type,
        motion_control_type,
        Message_Reporter,
        Progress_Monitor,
        Point_Unknown_Updater>
        sba_solver4(ps,
                    sc,
                    mc,
                    true,
                    sba_max_iterations,
                    Message_Reporter(),
                    Progress_Monitor(),
                    Point_Unknown_Updater());
        
        if (sba_solver4.is_failed())
            std::cout << "sba_solver4 failed" <<std::endl;
        
        // Copy results out
        std::copy(sba_solver4.coupled_point_begin(),
                  sba_solver4.coupled_point_begin()+2*correspondences,
                  ps.unknown_begin());
        mc[0] = sba_solver4.motion_control_begin()[0];
        mc[1] = sba_solver4.motion_control_begin()[1];
        
        std::vector<int> outliers(correspondences);
        std::fill(outliers.begin(), outliers.end(), 0);
        std::vector<int>::iterator outlier_first = outliers.begin();
        pair_solution_type::const_iterator
        point2_first = ps.unknown_begin(),
        point2_last  = ps.unknown_end();
        meas1_first = ps.unknown_measurement_0_begin();
        meas2_first = ps.unknown_measurement_1_begin();
        Data_Type twoview_outlier_threshold2 = twoview_outlier_threshold*twoview_outlier_threshold;
        while (point2_first!=point2_last) {
            Data_Type X[3], x[2], y1[2], y2[2], error1, error2, tmp;
            math::spherical_to_cartesian(point2_first[0], point2_first[1],
                                         X[0], X[1], X[2]);
            x[0] = X[0]/X[2]; x[1] = X[1]/X[2];
            mvg::apply_image_intrinsic(mc[0].get_image_intrinsic(),
                                       x[0], x[1],
                                       y1[0], y1[1]);
            mvg::project_perspective_rotation3(math::make_iterator_2d_n<3>(rot1.begin()),
                                               X[0], X[1], X[2],
                                               x[0], x[1]);
            mvg::apply_image_intrinsic(mc[1].get_image_intrinsic(),
                                       x[0], x[1],
                                       y2[0], y2[1]);
            error1 = 0;
            tmp = meas1_first[0]-y1[0]; error1 += tmp*tmp;
            tmp = meas1_first[1]-y1[1]; error1 += tmp*tmp;
            error2 = 0;
            tmp = meas2_first[0]-y2[0]; error2 += tmp*tmp;
            tmp = meas2_first[1]-y2[1]; error2 += tmp*tmp;
            if (error1>twoview_outlier_threshold2 ||
                error2>twoview_outlier_threshold2)
                *outlier_first = 1;
            
            point2_first += 2;
            meas1_first += 2;
            meas2_first += 2;
            ++outlier_first;
        }
        
        
        std::size_t frame1, frame2;
        
        frame1 = 0;
        frame2 = 1;
        // Copy results into recon
        std::size_t outlier_size = std::count(outliers.begin(), outliers.end(), int(1));
        recon.resize_2d_inliers(correspondences-outlier_size);
        Reconstruction::size_t_iterator
        inlier_trajectory_first = recon.inlier_2d_trajectory_begin();
        Reconstruction::point2_iterator
        point_first = recon.point2_begin();
        //const Data_Type* point_est_first =
        //    twoview_sba_solver.pair_solution().coupled_point_data_block();
        pair_solution_type::iterator
        point_est_first = ps.unknown_begin();
        outlier_first = outliers.begin();
        int inlier_size = 0;
        for (std::size_t ii=0;ii<correspondences;++ii) {
            if (!*outlier_first) {
                ////*inlier_trajectory_first = indices[ii];
                //trajectory_first->point_id() = static_cast<Reconstruction_Trajectory::data_type>(inlier_size);
                //trajectory_first->start() =  static_cast<Reconstruction_Trajectory::data_type>(frame1);
                //trajectory_first->finish() = static_cast<Reconstruction_Trajectory::data_type>(frame2+1);
                ////++inlier_trajectory_first;
                point_first->x() = point_est_first[0];
                point_first->y() = point_est_first[1];
                ++point_first;
                ++inlier_size;
            }
            ++outlier_first;
            point_est_first += 2;
        }
        recon.reference_index() = frame1;
        Matrix33 r0, r1;
        std::copy(mc[0].rotation_begin(),
                  mc[0].rotation_begin()+9,
                  r0.begin());
        std::copy(mc[1].rotation_begin(),
                  mc[1].rotation_begin()+9,
                  r1.begin());
        recon.insert_camera(std::make_pair(frame1, Camera(keyframe_e, mc[0].get_image_intrinsic(), r0)));
        recon.insert_camera(std::make_pair(frame2, Camera(keyframe_e, mc[1].get_image_intrinsic(), r1)));
        
        //// fixme, for test, take it out
        jj = 0;
        //for (  auto it = recon.camera_end()->second.motion.begin(); it < recon.camera_end()->second.motion.end(); it++  )
        for (  auto it = recon.find_camera(1)->second.motion.begin(); it < recon.find_camera(1)->second.motion.end(); it++  )
        {
            
            estimatedMat.at<float> ( floor ( jj / 3), jj % 3 ) = (float)(*it);
            jj++;
        }
        ////
        std::cout << std::endl;
        std::cout << estimatedMat << std::endl;
        //************************************* end of twoview_initialization
        
        
        //_focal = recon.focal
        */
        
        
        
        
        
        
        

        
        

        // Use matched keypoint coordinates to calculate focal length and rotation matrix
        for (  int i = 0; i < prev.size( ); i++  ) {
            x1.push_back(  prev[  i  ].x  );
            y1.push_back(  prev[  i  ].y  );
            x2.push_back(  cur[  i  ].x  );
            y2.push_back(  cur[  i  ].y  );
        }

        
        adobe_agt::mvg::rotation3_fl1_est_2view_2point_ransac<float> rotation_fl_solver(  x1.size( ),
                                                                                        (  &x1.front( )  ),
                                                                                        (  &y1.front( )  ),
                                                                                        (  &x2.front( )  ),
                                                                                        (  &y2.front( )  ),
                                                                                        _focal, _width / 2, _height / 2, 1,
                                                                                        _focal, _width / 2, _height / 2, 1,
                                                                                        10,
                                                                                        std::size_t(  500  ),
                                                                                        double(  0.997  ),
                                                                                        rand
                                                                                        );

       
        if ( ! rotation_fl_solver.is_failed( ) )
        {
             if (abs ( rotation_fl_solver.focal_length2() * _focal - _focal) < _focal / 10 )
                 //_focal = 0.5 * rotation_fl_solver.focal_length2() * _focal + 0.5 * _focal;
                 _focal = rotation_fl_solver.focal_length2() * _focal;
            std::cout << "Estimated frame-to-frame: " << _focal << std::endl;
        }
         else
             std::cout << "!!!!!!!!!!!!!!!!!!!!!!!   Rotation estimation failed" << std::endl;
        
    }    
        
}

template<typename Iter_T>
long double vectorNorm( Iter_T first, Iter_T last ) {
    return sqrt( inner_product( first, last, first, 0.0L ) );
}


void CvEstimator::procFrame( Mat &currentImage, bool flag )
{
    bool success = true;
    float normOfDiff = 0;
    Mat HFromSensor;
    clock_t begin = clock( );
    
    computeHomographyFromRotation( _RFromSensor.inv( ),  _focal, cv::Point2d( _width / 2, _height / 2 ) , HFromSensor, _K );
    //std::cout << HFromSensor << std::endl;
    
    //cv::cvtColor( currentImage, currentImage, CV_BGR2GRAY );
    
    if ( flag )
    {
        if ( _prevImage.rows > 0 )
        {
            _featureDetector ->detect( currentImage, _currentKeypoints, Mat( ) );
            
            if ( _currentKeypoints.size( ) > 20 )
            {
                Mat Mask = Mat::zeros( _height, _width, CV_8UC1 );
                
                Mat RFromImageRobust;
                std::vector< DMatch > good_matches, wellDistributed;
                std::vector<DMatch> matches, sortedMatches;
                std::vector<Point2f> prev, cur;
                std::vector<float> x1, x2, y1, y2;
                
                // Describe keypoints
                _featureDetector ->compute( currentImage, _currentKeypoints, _currentDescriptors );
                
                // Compute homograpjhy from rotaion matrix via the motion sensor information, to be used to rectify the keypoint matches
                computeHomographyFromRotation( _RFromSensor,  _focal, cv::Point2d( _width / 2, _height / 2 ) , HFromSensor, _K );
                
                
                ////////////////////////
                // Compute homography from sensor data
                int MAXMATCHES = 100;
                Mat R = Mat::zeros( 3, 3, CV_64FC1 );
                Mat K;
                
                
                _RFromSensor.copyTo( R );
                /*
                R = R.inv( );
                // Flip x in input images
                for ( int j = 0; j < 3; j++ ) {
                    R.at<float>( j,0 ) = -1 * R.at<float>( j,0 );
                }
                
                // This is equal to 180 degree rotation relative to the y axis to bring back the pixels in 3d space to the front of camera after fliping in z direction
                for ( int j = 0; j < 3; j++ ) {
                    R.at<float>( 0, j ) = -1 * R.at<float>( 0, j );
                    R.at<float>( 2, j ) = -1 * R.at<float>( 2, j );
                }
                R = R.inv( );
                
                // Flip in z direction for consistency
                for ( int j = 0; j < 3; j++ ) {
                    R.at<float>( 2, j ) = -1 * R.at<float>( 2, j );
                }
                R.convertTo( R, CV_64F );
                computeHomographyFromRotation(  R, _focal, cv::Point2d( _width / 2, _height / 2 ), HFromSensor, K  );
                
                
                std::cout << _RFromSensor << std::endl;
                
                std::cout << R << std::endl << std::endl;
                
                */

                Ptr<DescriptorMatcher>  descriptorMatcher = DescriptorMatcher::create( String( "BruteForce" ) );
                
                
                // Match keypoints
                descriptorMatcher -> match(  _prevDescriptors, _currentDescriptors, matches, Mat( )  );
                Mat index;
                int nbMatch=int( matches.size( ) );
                
                // Only store a maximum of 100 strongest matches ( MAXMATCHES )
                Mat tab( nbMatch, 1, CV_32F );
                for ( int i = 0; i < nbMatch; i++ ) {
                    tab.at<float>( i, 0 ) = matches[ i ].distance;
                }
                sortIdx( tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING );
                for ( int i = 0; i < min( nbMatch, MAXMATCHES ); i++ ) {
                    sortedMatches.push_back( matches[ index.at<int>( i, 0 ) ] );
                }
                matches = sortedMatches;
                sortedMatches.clear( );
                
                //robustKeypointMatching( _width, _height, HFromSensor,  _prevKeypoints, _currentKeypoints,  _prevDescriptors, _currentDescriptors, matches, prev, cur, Mask );
                
                // Extract coordinates of matches
                for ( int j = 0; j < matches.size( ); j++ )
                {
                    Point2f prevTemp, curTemp;
                    int queryIdx = matches[ j ].queryIdx;
                    int trainIdx = matches[ j ].trainIdx;
                    prevTemp = _prevKeypoints[ queryIdx ].pt;
                    curTemp = _currentKeypoints[ trainIdx ].pt;
                    
                    prev.push_back( prevTemp );
                    cur.push_back( curTemp );
                }
                
                // Rectify keypoints using sensor information
                std::vector<Point2f> rectifiedPrev, rectifiedCur, prevTransformed, prevTransformed2;
                perspectiveTransform( prev, prevTransformed, HFromSensor.inv( ) );
                
                float err1=0;
                err1 = norm( cur, prevTransformed );
                
                float rejectionThr = 50.0; //fixme, this threshold is dependent on resolution
                int inlierCount = 0;
                for ( int j = 0; j < matches.size( ); j++ )
                {
                    //std::cout << norm (  prev[ j ] - curTransformed[ j ] ) << std::endl;
                    //if ( norm (  prevTransformed[ j ] - cur[ j ]  ) < rejectionThr )
                    {
                        rectifiedPrev.push_back( prev[ j ] );
                        rectifiedCur.push_back( cur[ j ] );
                    }
                }
                
                // Use matched keypoint coordinates to calculate focal length and rotation matrix
                for (  int i = 0; i < rectifiedCur.size( ); i++  ) {
                    x1.push_back(  prev[  i  ].x  );
                    y1.push_back(  prev[  i  ].y  );
                    x2.push_back(  cur[  i  ].x  );
                    y2.push_back(  cur[  i  ].y  );
                }
                
                Data_Type fl, fl1, fl2, ic1, ic2, ar;
                fl = _focal;
                fl1 = _focal;
                fl2 = _focal;
                ic1 = _width / 2;
                ic2 = _height / 2;
                ar = 1;
                
                std::size_t video_width = static_cast<std::size_t>( _width );
                std::size_t video_height = static_cast<std::size_t>( _height );
                std::size_t video_fps = static_cast<std::size_t>( 30 );
                adobe_agt::videopano::Reconstruction recon;
                std::vector<adobe_agt::videopano::Tracking_Feature> tracking_features;
                std::vector<adobe_agt::videopano::Tracking_Trajectory> tracking_trajectories;
                adobe_agt::videopano::videopano_parameter param;
                param.compute_derived_parameters(video_width, video_height, video_fps);
                
                //*************************************
                // This is actually what inside the twoview_initialization is being done
                typedef mvg::pair_solution2<Data_Type, 2, 2> pair_solution_type;
                std::size_t correspondences = x1.size();
                typedef mvg::pair_solution2<Data_Type, 2, 2> pair_solution_type;
                pair_solution_type ps(correspondences);
                
                typedef std::vector<Data_Type>::iterator Tracking_Feature_Iterator;
                
                pair_solution_type::iterator
                meas1_first = ps.unknown_measurement_0_begin(),
                meas2_first = ps.unknown_measurement_1_begin();
                
                int indice_first = 0;
                //while (indice_first!=indice_last) {
                while (indice_first < correspondences)
                {
                    meas1_first[0] = x1[indice_first];
                    meas1_first[1] = y1[indice_first];
                    meas2_first[0] = x2[indice_first];
                    meas2_first[1] = y2[indice_first];
                    
                    ++indice_first;
                    meas1_first += 2;
                    meas2_first += 2;
                }
                
                typedef utility::step_offset_iterator<pair_solution_type::const_iterator, 2> vector2_iterator;
                
                mvg::rotation3_est_2view_ransac_3point<Data_Type>
                ransac_solver(correspondences,
                              vector2_iterator(ps.unknown_measurement_0_begin(), 0), vector2_iterator(ps.unknown_measurement_0_begin(), 1),
                              vector2_iterator(ps.unknown_measurement_1_begin(), 0), vector2_iterator(ps.unknown_measurement_1_begin(), 1),
                              fl1, fl2,
                              ic1, ic2, ar,
                              ic1, ic2, ar,
                              50,
                              std::size_t(  2000  ),
                              double(  0.995  )
                              );
                
                if (ransac_solver.is_failed())
                    std::cout << "ransac_solver_failed" << std::endl;
                
                
                int intrinsic_mode = 1;
                typedef mvg::shared_intrinsic<Image_Intrinsic, Image_Intrinsic_Mode> shared_control_type;
                typedef mvg::motion_control_so3_new<Data_Type, Image_Intrinsic, Image_Intrinsic_Mode> motion_control_type;
                shared_control_type sc;
                motion_control_type mc[2];
                Matrix33 rot0, rot1;
                fill_identity_k<3>(math::make_iterator_2d_n<3>(rot0.begin()));
                std::copy(ransac_solver.best_rotation().data_matrix().begin(),
                          ransac_solver.best_rotation().data_matrix().end(),
                          rot1.begin());
                if        (intrinsic_mode==1) {
                    Data_Type fl = (ransac_solver.best_focal_length_0()+
                                    ransac_solver.best_focal_length_1())/2;
                    Image_Intrinsic ii(fl, ic1, ic2, ar);
                    sc    = shared_control_type(ii, mvg::shared_fl1);
                    mc[0] = motion_control_type(ii, mvg::shared_fl1, math::make_iterator_2d_n<3>(rot0.begin()), 0);
                    mc[1] = motion_control_type(ii, mvg::shared_fl1, math::make_iterator_2d_n<3>(rot1.begin()), 1);
                } else if (intrinsic_mode==2) {
                    Data_Type fl0 = ransac_solver.best_focal_length_0();
                    Data_Type fl1 = ransac_solver.best_focal_length_1();
                    Data_Type fl = (fl0+fl1)/2;
                    sc =    shared_control_type(Image_Intrinsic(fl , ic1, ic2, ar), mvg::private_fl1);
                    mc[0] = motion_control_type(Image_Intrinsic(fl0, ic1, ic2, ar), mvg::private_fl1, math::make_iterator_2d_n<3>(rot0.begin()), 0);
                    mc[1] = motion_control_type(Image_Intrinsic(fl1, ic1, ic2, ar), mvg::private_fl1, math::make_iterator_2d_n<3>(rot1.begin()), 1);
                } else throw std::runtime_error("Unsupported intrinsic mode");
                
                adobe_agt::videopano::detail::compute_twoview_unknown(correspondences,
                                                                      ps.unknown_measurement_0_begin(),
                                                                      ps.unknown_measurement_1_begin(),
                                                                      mc[0].get_image_intrinsic(),
                                                                      mc[1].get_image_intrinsic(),
                                                                      math::make_iterator_2d_n<3>(rot0.begin()),
                                                                      math::make_iterator_2d_n<3>(rot1.begin()),
                                                                      ps.unknown_begin());
                
                
                cv::Mat estimatedMat = cv::Mat::zeros( 3, 3, CV_32FC1 );
                
                //// fixme, for test, take it out
                int jj = 0;
                for (  auto it = rot1.begin( ); it < rot1.end( ); it++  )
                {
                    estimatedMat.at<float> ( floor ( jj / 3), jj % 3 ) = float(*it);
                    jj++;
                }
                ////
                std::cout << estimatedMat << std::endl;
                
                std::size_t sba_max_iterations = param.twoview_sba_max_iterations;
                Data_Type twoview_outlier_threshold = 10;
                
                mvg::sparse_bundle_adjuster4<Data_Type,
                shared_control_type,
                motion_control_type,
                Message_Reporter,
                Progress_Monitor,
                Point_Unknown_Updater>
                sba_solver4(ps,
                            sc,
                            mc,
                            true,
                            sba_max_iterations,
                            Message_Reporter(),
                            Progress_Monitor(),
                            Point_Unknown_Updater());
                
                if (sba_solver4.is_failed())
                    std::cout << "sba_solver4 failed" <<std::endl;
                
                // Copy results out
                std::copy(sba_solver4.coupled_point_begin(),
                          sba_solver4.coupled_point_begin()+2*correspondences,
                          ps.unknown_begin());
                mc[0] = sba_solver4.motion_control_begin()[0];
                mc[1] = sba_solver4.motion_control_begin()[1];
                
                std::vector<int> outliers(correspondences);
                std::fill(outliers.begin(), outliers.end(), 0);
                std::vector<int>::iterator outlier_first = outliers.begin();
                pair_solution_type::const_iterator
                point2_first = ps.unknown_begin(),
                point2_last  = ps.unknown_end();
                meas1_first = ps.unknown_measurement_0_begin();
                meas2_first = ps.unknown_measurement_1_begin();
                Data_Type twoview_outlier_threshold2 = twoview_outlier_threshold*twoview_outlier_threshold;
                while (point2_first!=point2_last) {
                    Data_Type X[3], x[2], y1[2], y2[2], error1, error2, tmp;
                    math::spherical_to_cartesian(point2_first[0], point2_first[1],
                                                 X[0], X[1], X[2]);
                    x[0] = X[0]/X[2]; x[1] = X[1]/X[2];
                    mvg::apply_image_intrinsic(mc[0].get_image_intrinsic(),
                                               x[0], x[1],
                                               y1[0], y1[1]);
                    mvg::project_perspective_rotation3(math::make_iterator_2d_n<3>(rot1.begin()),
                                                       X[0], X[1], X[2],
                                                       x[0], x[1]);
                    mvg::apply_image_intrinsic(mc[1].get_image_intrinsic(),
                                               x[0], x[1],
                                               y2[0], y2[1]);
                    error1 = 0;
                    tmp = meas1_first[0]-y1[0]; error1 += tmp*tmp;
                    tmp = meas1_first[1]-y1[1]; error1 += tmp*tmp;
                    error2 = 0;
                    tmp = meas2_first[0]-y2[0]; error2 += tmp*tmp;
                    tmp = meas2_first[1]-y2[1]; error2 += tmp*tmp;
                    if (error1>twoview_outlier_threshold2 ||
                        error2>twoview_outlier_threshold2)
                        *outlier_first = 1;
                    
                    point2_first += 2;
                    meas1_first += 2;
                    meas2_first += 2;
                    ++outlier_first;
                }
                
                
                std::size_t frame1, frame2;
                
                frame1 = 0;
                frame2 = 1;
                // Copy results into recon
                std::size_t outlier_size = std::count(outliers.begin(), outliers.end(), int(1));
                recon.resize_2d_inliers(correspondences-outlier_size);
                Reconstruction::size_t_iterator
                inlier_trajectory_first = recon.inlier_2d_trajectory_begin();
                Reconstruction::point2_iterator
                point_first = recon.point2_begin();
                //const Data_Type* point_est_first =
                //    twoview_sba_solver.pair_solution().coupled_point_data_block();
                pair_solution_type::iterator
                point_est_first = ps.unknown_begin();
                outlier_first = outliers.begin();
                int inlier_size = 0;
                for (std::size_t ii=0;ii<correspondences;++ii) {
                    if (!*outlier_first) {
                        ////*inlier_trajectory_first = indices[ii];
                        //trajectory_first->point_id() = static_cast<Reconstruction_Trajectory::data_type>(inlier_size);
                        //trajectory_first->start() =  static_cast<Reconstruction_Trajectory::data_type>(frame1);
                        //trajectory_first->finish() = static_cast<Reconstruction_Trajectory::data_type>(frame2+1);
                        ////++inlier_trajectory_first;
                        point_first->x() = point_est_first[0];
                        point_first->y() = point_est_first[1];
                        ++point_first;
                        ++inlier_size;
                    }
                    ++outlier_first;
                    point_est_first += 2;
                }
                recon.reference_index() = frame1;
                Matrix33 r0, r1;
                std::copy(mc[0].rotation_begin(),
                          mc[0].rotation_begin()+9,
                          r0.begin());
                std::copy(mc[1].rotation_begin(),
                          mc[1].rotation_begin()+9,
                          r1.begin());
                recon.insert_camera(std::make_pair(frame1, Camera(keyframe_e, mc[0].get_image_intrinsic(), r0)));
                recon.insert_camera(std::make_pair(frame2, Camera(keyframe_e, mc[1].get_image_intrinsic(), r1)));
                
                //// fixme, for test, take it out
                jj = 0;
                //for (  auto it = recon.camera_end()->second.motion.begin(); it < recon.camera_end()->second.motion.end(); it++  )
                for (  auto it = recon.find_camera(1)->second.motion.begin(); it < recon.find_camera(1)->second.motion.end(); it++  )
                {
                    
                    estimatedMat.at<float> ( floor ( jj / 3), jj % 3 ) = (float)(*it);
                    jj++;
                }
                ////
                std::cout << std::endl;
                std::cout << estimatedMat << std::endl;
                
                
                Mat rotMat;
                
                estimatedMat.copyTo( rotMat );
                
                rotMat = rotMat.inv();
                
                // Flip in z direction for consistency
                for ( int j = 0; j < 3; j++ ) {
                    rotMat.at<float>( 2, j ) = -1 * rotMat.at<float>( 2, j );
                }
                rotMat = rotMat.inv( );
                
                // This is equal to 180 degree rotation relative to the y axis to bring back the pixels in 3d space to the front of camera after fliping in z direction
                for ( int j = 0; j < 3; j++ ) {
                    rotMat.at<float>( 0, j ) = -1 * rotMat.at<float>( 0, j );
                    rotMat.at<float>( 2, j ) = -1 * rotMat.at<float>( 2, j );
                }
                
                // Flip x in input images
                for ( int j = 0; j < 3; j++ ) {
                    rotMat.at<float>( j,0 ) = -1 * rotMat.at<float>( j,0 );
                }
                
                
                for (  int jj = 0; jj < 9; jj++  )
                {
                    _RFromImageRobust[  jj  ] = estimatedMat.at<float> ( floor ( jj / 3), jj % 3 );
                }
                
                
            }
        }
    }
    
    
}

void CvEstimator::initialize( int width, int height )
{
    _featureDetector = ORB::create( 1000, 1.2f, 3, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20 ); //fixme, is 200 good enough?
    _width = width;
    _height = height;
    
    _RFromSensor = Mat::zeros( 3, 3, CV_64FC1 );
    for ( int i = 0; i < 9; i++ ){
        _RFromImageRobust.push_back( 0 );
        _RFromSensor.at<double>( floor( i / 3 ), i % 3 ) = 0;
    }
    _RFromImageRobust[ 0 ] = 1;
    _RFromImageRobust[ 4 ] = 1;
    _RFromImageRobust[ 8 ] = 1;
    
    
    
    _K = cv::Mat::zeros( 3, 3, CV_64FC1 );
    _K.at<double>( 0,0 ) = _focal;
    _K.at<double>( 1,1 ) = _focal;
    _K.at<double>( 2,2 ) = 1.0;
    
    _K.at<double>( 0,2 ) = _width / 2;
    _K.at<double>( 1,2 ) = _height / 2;
    
}

std::vector<float> CvEstimator::getRotation( )
{
    //_RFromImageRobust has rotation estimated from image features between the last reference frame and the current frame,
    // it should be converted first to a global rotation by compositing the rotations calculated up to now together
    return _RFromImageRobust;
}

void CvEstimator::setRotationFromSensor( std::vector<float> &rot )
{
    for ( int i = 0; i < 9; i++  )
        _RFromSensor.at<double>( floor( i / 3 ), i % 3 ) = rot[ i ];
}
void CvEstimator::restart( )
{
    _storedImages.clear( );
    _storedKeypoints.clear( );
    _storedDescriptors.clear( );
    
    // In Class Pano ( display module ), the index 0 is reserved for camera live feed, so, start from index 1 for compatibility.
    std::vector<cv::KeyPoint> temp;
    _storedImages.push_back( cv::Mat( ) );
    _storedKeypoints.push_back( temp );
    _storedDescriptors.push_back( cv::UMat( ) );
    
    _pairwiseMatches.clear( );
    _features.clear( );
    _cameras.clear();
}

void CvEstimator::setFocalLength( float focal )
{
    _focal = focal;
}


///////////////////////////////////////////////////////////////////////////////////////////////
void CvEstimator::copyRotationToCameraArray( std::vector<Mat> &currentRotationUsedToInitBA, std::vector<CameraParams>& cameras )
{
    for ( size_t i = 0; i < currentRotationUsedToInitBA.size( ) - 1; ++i )
    {
        if ( i >= cameras.size( ) )
            cameras.resize( i + 1 );
        
        // Use rotation matrix computed from previous stages ( which is corrected by the device sensor ) as the initial condition for the bundle adjustment
        // The way rotation matrices are used for display is different from what calculated in bundle adjustment, some flipping and rotation is needed to make them consistent
        Mat R;
        currentRotationUsedToInitBA[ i + 1 ].copyTo( R );
        R = R.inv( );
        R.convertTo( R, CV_32FC1 );
        R.copyTo( cameras[ i ].R );
        
        cameras[ i ].ppx = _width / 2;
        cameras[ i ].ppy = _height / 2;
        cameras[ i ].focal = _focal;
    }

}
///////////////////////////////////////////////////////////////////////////////////////////////
void CvEstimator::copyBAResultToCamerArray( int num_images, std::vector<CameraParams> &cameras, std::vector<CameraParams> &initialCameras )
{
    for ( int i = 1; i <= num_images; i++ )
    {
        // Index 0 is reserved for live camera feed, first one is also set to identity
        if ( i >= _refinedRotationArray.size( ) )
            _refinedRotationArray.resize( i + 1 );
        
        cv::Mat rotMat;
        cameras[ ( i - 1 ) ].R.copyTo( rotMat );
        rotMat = rotMat.inv( );
        rotMat.copyTo( _refinedRotationArray[ i ] );
    }
    

    /*
    std::cout << "Input camera:" << std::endl;
    for ( int i = 1; i <= num_images; i++ )
    {
        std::cout <<  initialCameras[ ( i - 1 ) ].R << std::endl;
    }
    std::cout << std::endl;
    */
    
    
    
    
    /*
    std::cout << std::endl;
    // Only copy the BA result if the result is very consistent with rotation data from sensor
    float error = 0;
    for ( int i = 1; i <= num_images; i++ )
    {
        Mat temp =  initialCameras[ ( i - 1 ) ].R;
        temp.at<float>( 0, 2 ) = -1 * temp.at<float>( 0, 2 );
        temp.at<float>( 2, 0 ) = -1 * temp.at<float>( 2, 0 );
        
        std::cout << "Error in rotation for camera " << i << ": " << norm( temp - _refinedRotationArray[ i ] ) << std::endl;
        error += norm( temp - _refinedRotationArray[ i ] );
    }
    std::cout << "Rrror in rotation for cameras: " << error / initialCameras.size( )  << std::endl;
    */
    
    /*
    if ( error / initialCameras.size( ) > 0.03 )
    {
        for ( int i = 1; i <= num_images; i++ )
        {
            
            cv::Mat rotMat;
            initialCameras[ ( i - 1 ) ].R.copyTo( rotMat );
            
            // Flip in z direction for consistency
            for ( int j = 0; j < 3; j++ ) {
                rotMat.at<float>( 2, j ) = -1 * rotMat.at<float>( 2, j );
            }
            rotMat = rotMat.inv( );
            
            // This is equal to 180 degree rotation relative to the y axis to bring back the pixels in 3d space to the front of camera after fliping in z direction
            for ( int j = 0; j < 3; j++ ) {
                rotMat.at<float>( 0, j ) = -1 * rotMat.at<float>( 0, j );
                rotMat.at<float>( 2, j ) = -1 * rotMat.at<float>( 2, j );
            }
            
            // Flip x in input images
            for ( int j = 0; j < 3; j++ ) {
                rotMat.at<float>( j,0 ) = -1 * rotMat.at<float>( j,0 );
            }

            rotMat.copyTo( _refinedRotationArray[ i ] );
            
            
            
            
        }
        std::cout << "Large error in rotation for cameras: " << error / initialCameras.size( )  << std::endl;
    }
    */
    
    for ( int i = 1; i <= num_images; i++ )
    {
        _refinedRotationArray[ i ] = _refinedRotationArray[ 1 ].inv( ) * _refinedRotationArray[ i ];
    }
    
    
    
    //_refinedRotationArray[ 1 ] = Mat::eye( 3, 3, _refinedRotationArray[ 1 ].type( ) );
}
///////////////////////////////////////////////////////////////////////////////////////////////
std::vector<cv::Mat> CvEstimator::refinedStitching( std::vector<Mat> &currentRotationUsedToInitBA, const std::vector<std::vector<int>> &closestFrames )
{
    
    
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Refinement started" << std::endl;
    int num_images = int(_storedImages.size())  - 1;
    
    HomographyBasedEstimator estimator;
    std::vector<CameraParams> cameras;
    
    Ptr<cv::detail::BundleAdjusterBase> adjuster;
    adjuster = new cv::detail::BundleAdjusterRay();
    float conf_thresh = 1.f;
    adjuster->setConfThresh(conf_thresh);
    std::string ba_refine_mask = "xxxxx";
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
    
    
    
    float match_conf = 0.3f;
    BestOf2NearestMatcher matcher(false, match_conf);
    
    
    if ((num_images *  num_images) > _pairwiseMatches.size()) // There is new unprocessed frame, match keypoints and do BA, otherwise, only do BA
    {
        clock_t begin = clock();
        // If pairwise matches not formed yet, start from scratch, otherwise incrementally update it with each new frame captured
        if (_pairwiseMatches.size() < 9) // Do not do incremental for the first 3 frames
        {
            for (int i = 0; i < num_images; i++)
            {
                ImageFeatures tempFeatures;
                
                tempFeatures.img_idx = i;
                tempFeatures.img_size = cv::Size(_width, _height);
                tempFeatures.keypoints = _storedKeypoints[i+1];
                tempFeatures.descriptors = _storedDescriptors[i+1];
                _features.push_back(tempFeatures);
                
                _cameras.push_back(CameraParams()); // Push empty camera param, it is going to be replaced with sensor data estimation
            }
            
            matcher(_features, _pairwiseMatches);
            matcher.collectGarbage();
        }
        else
        {
            // Match the last frame with all the previous frames in its neighborhood
            Ptr<DescriptorMatcher>  descriptorMatcher = DescriptorMatcher::create(String("BruteForce"));
            
            copyRotationToCameraArray(currentRotationUsedToInitBA, cameras);
            
            for (int i = 0; i < num_images - 1; i++)
            {
                std::vector<DMatch> matches;
                std::vector<Point2f> prev, cur;
                cv::detail::MatchesInfo tempMatchesInfo;
                std::vector<uchar> inliers_mask;
                bool frameShouldBeProcessed = false;
                
                tempMatchesInfo.src_img_idx = i;
                tempMatchesInfo.dst_img_idx = num_images - 1;
                tempMatchesInfo.confidence = 0.9;////fixme
                
                if ( i == closestFrames[num_images][0] || i == closestFrames[num_images][1] || i == closestFrames[num_images][2] || i == closestFrames[num_images][3] ) // check against 4 nearest neighbors
                    frameShouldBeProcessed = true;
                
                if ( _storedDescriptors[num_images].rows < 10)
                    frameShouldBeProcessed = false;
                
                if ( frameShouldBeProcessed )
                {
                    // Compute homography from sensor data
                    Mat HFromSensor, K;
                    Mat R = Mat::zeros(3, 3, CV_64FC1);
                    Mat Mask = Mat::zeros(_height, _width, CV_8UC1);
                    //std::vector< DMatch > good_matches, wellDistributed;
                    
                    // Why using sensor data seems to not be working well?!! fixme
                    R = cameras[ i ].R.inv()  * cameras[ num_images - 1 ].R; // A good estimation using index "num_images". Remember index 0 is reserved for camera frame
                    R.convertTo(R, CV_64F);
                    computeHomographyFromRotation( R, _focal, cv::Point2d(_width, _height), HFromSensor, K );
                    
                    
                    
                    
                    
                    // Match keypoints and rectify them using sensor information
                    descriptorMatcher -> match( _storedDescriptors[i+1], _storedDescriptors[num_images], matches, Mat() ); //fixme, error handling for no keypoint in last frame
                    std::vector<DMatch> sortedMatches;
                    Mat index;
                    int nbMatch=int(matches.size());
                    
                    Mat tab(nbMatch, 1, CV_32F);
                    for (int i = 0; i < nbMatch; i++) {
                        tab.at<float>(i, 0) = matches[i].distance;
                    }
                    sortIdx(tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING);
                    for (int i = 0; i < min(nbMatch, 100); i++) {
                        sortedMatches.push_back(matches[index.at<int>(i, 0)]);
                    }
                    matches = sortedMatches;
                    sortedMatches.clear();
                    
                    //robustKeypointMatching(_width, _height, HFromSensor,  _storedKeypoints[i+1], _storedKeypoints[num_images],  _storedDescriptors[i+1], _storedDescriptors[num_images], matches, prev, cur, Mask);
                    
                    for (int j = 0; j < matches.size(); j++)
                    {
                        int queryIdx = matches[j].queryIdx;
                        int trainIdx = matches[j].trainIdx;
                        
                        prev.push_back(_storedKeypoints[i+1][queryIdx].pt);
                        cur.push_back(_storedKeypoints[num_images][trainIdx].pt);
                        inliers_mask.push_back(uint(1));
                    }
                    
                    //tempMatchesInfo.H = (HFromSensor); // + findHomography(prev, cur, CV_RANSAC, 3, noArray(), 1000, 0.995)) / 2;
                    
                    
                    Mat H = findHomography(prev, cur, CV_RANSAC, 3, noArray(), 1000, 0.995) / 2;
                    tempMatchesInfo.H = Mat::eye(3, 3, CV_64FC1); //fixme, H
                    
                    float err1=0;
                    float err2 = 0;
                    std::vector<Point2f> prevTransformed;
                    std::vector<Point2f> prevTransformed2;
                    perspectiveTransform(prev, prevTransformed, HFromSensor);
                    perspectiveTransform(prev, prevTransformed2, H);
                    
                    
                    err1 = norm(cur, prevTransformed);
                    err2 = norm(cur, prevTransformed2);
                    
                    
                    //std::cout << err1 << "," << err2 << std::endl;
                    
                    //std::cout << HFromSensor << std::endl;
                    
                    //std::cout << H << std::endl;
                    
                    //std::cout << R << std::endl;
                    
                    tempMatchesInfo.matches = matches;
                    tempMatchesInfo.num_inliers = matches.size();
                    tempMatchesInfo.inliers_mask = inliers_mask;
                }
                
                
                std::vector<cv::detail::MatchesInfo>::iterator it;
                
                it = _pairwiseMatches.begin();
                _pairwiseMatches.insert(it +  (i+1) * num_images - 1, tempMatchesInfo);
                
                //  After mathing i to j and pushing it, push j to i matches
                tempMatchesInfo.src_img_idx = num_images - 1;
                tempMatchesInfo.dst_img_idx = i;
                
                if (frameShouldBeProcessed)
                {
                    tempMatchesInfo.H = tempMatchesInfo.H.inv();
                    
                    for (int k = 0; k < matches.size(); k++)
                    {
                        // Reverse matches
                        matches[k].queryIdx = tempMatchesInfo.matches[k].trainIdx;
                        matches[k].trainIdx = tempMatchesInfo.matches[k].queryIdx;
                    }
                    tempMatchesInfo.matches = matches;
                }
                _pairwiseMatches.push_back(tempMatchesInfo);
            }
            
            //self-match
            cv::detail::MatchesInfo tempMatchesInfo;
            tempMatchesInfo.src_img_idx = -1;
            tempMatchesInfo.dst_img_idx = -1;
            
            std::vector<cv::detail::MatchesInfo>::iterator it;
            it = _pairwiseMatches.begin();
            _pairwiseMatches.insert(it +  num_images * num_images - 1, tempMatchesInfo);
            
            // push features of the last frame
            int i = num_images - 1;
            ImageFeatures tempFeatures;
            tempFeatures.img_idx = i;
            tempFeatures.img_size = cv::Size(_width, _height);
            tempFeatures.keypoints = _storedKeypoints[i+1];
            tempFeatures.descriptors = _storedDescriptors[i+1];
            _features.push_back(tempFeatures);
            
            _cameras.push_back(CameraParams());
        }
        
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Augmenting matches: " << elapsed_secs << std::endl;
    }
    else
    { // Not a new frame, "refinement" button  pressed again
        clock_t begin = clock();
        
        matcher(_features, _pairwiseMatches); // Most of randomness comes from here, if a good match is found, there won't be any change by repeating the refinement (tested if _cameras overwritten by motion sensor data)
        matcher.collectGarbage();
         
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Re-match: "<< elapsed_secs << std::endl;
    }
    
    
    
     
     
    
     
     
     
    /*
    try
    {
    estimator(_features, _pairwiseMatches, _cameras); // This part introduces some randomness, redo this at each press of "refinement" button to be able to correct mistakes from previous stages
    }
    catch ( const std::exception& e )
    {
        std::cout << " H estimator Error " << std::endl;
    }
    */
    /*
    std::cout << "input" << std::endl;
    for ( int j = 1; j < currentRotationUsedToInitBA.size( ) ; j++ )
    {
        std::cout << currentRotationUsedToInitBA[j] << std::endl;
    }
    
    std::cout << "output" << std::endl;
    for ( int j = 0; j < _cameras.size( ) ; j++ )
    {
        std::cout << _cameras[j].R << std::endl;
    }
    */
    
    
    
    
    
    clock_t begin = clock();
    try
    {
        estimator(_features, _pairwiseMatches, _cameras); // This part introduces some randomness, redo this at each press of "refinement" button to be able to correct mistakes from previous stages
        copyRotationToCameraArray(currentRotationUsedToInitBA, _cameras); //fixme
        adjuster->setRefinementMask(refine_mask);
        (*adjuster)(_features, _pairwiseMatches, _cameras);
    }
    catch ( const std::exception& e )
    {
        std::cout << " BA estimator Error " << std::endl;
    }

    _focal = _cameras[0].focal;
    copyBAResultToCamerArray(num_images, _cameras, _cameras);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "BA main part: "<< elapsed_secs << std::endl;
    
    
    
    return _refinedRotationArray;
    
}

///////////////////////////////////////////////////////////////////////////////////////////////

// ===============================================================================================================
// An interface to objective c.
// ===============================================================================================================
#include "cvEstimatorInterface.h"

void *cvEstimator_create( )
{
    return ( void* ) new CvEstimator;
}


void cvEstimator_initialize( void *_cvEstimator, int width, int height )
{
    ( ( CvEstimator* )_cvEstimator )->initialize( width, height );
}



std::vector<float> cvEstimator_getRotation( void *_cvEstimator )
{
    return ( ( CvEstimator* )_cvEstimator )->getRotation( );
}


float cvEstimator_getFocalLength( void *_cvEstimator )
{
    return ( ( CvEstimator* )_cvEstimator )->getFocalLength( );
}




void cvEstimator_setRotationFromSensor( void *_cvEstimator, std::vector<float> &rot )
{
    ( ( CvEstimator* )_cvEstimator )->setRotationFromSensor( rot );
}


void cvEstimator_restart( void *_cvEstimator )
{
    ( ( CvEstimator* )_cvEstimator )->restart( );
}



void cvEstimator_setFocalLength( void *_cvEstimator, float focal )
{
    ( ( CvEstimator* )_cvEstimator )->setFocalLength( focal );
}


std::vector<std::vector <float>> cvEstimator_refinedStitching( void *_cvEstimator, std::vector<std::vector <float>> currentRotationUsedToInitBA, std::vector<std::vector<int>> closestFrames )
{
    // Convert from vector to Mat
    std::vector<Mat> currentRotation;
    currentRotation.push_back( Mat( 3, 3, CV_32F ) );
    
    for (  int i = 1; i < currentRotationUsedToInitBA.size( ); i++  )
    {
        currentRotation.push_back(  Mat(  3, 3, CV_32F  )  );
        for ( int j = 0; j < 9; j++ ) {
            currentRotation[ i ].at<float>(  floor( j / 3 ), ( j % 3 )  ) = currentRotationUsedToInitBA[ i ][ j ];
        }
    }
    
    // Do bundle adjustment
    std::vector<cv::Mat> refinedRotationArray = ( ( CvEstimator* )_cvEstimator )->refinedStitching( currentRotation, closestFrames );
    
    
    
    // Convert from Mat to vector
    std::vector<std::vector <float>> refinedRotation( refinedRotationArray.size( ) );
    for ( int i = 1; i < refinedRotationArray.size( ); i++ )
    {
        for ( int j = 0; j < 9; j++ ) {
            refinedRotation[ i ].push_back( refinedRotationArray[ i ].at<float>( floor( j / 3 ), ( j % 3 ) ) );
        }
    }
    return refinedRotation;
}


void cvEstimator_saveNextFrame( void *_cvEstimator, Mat& nextFrame )
{
    ( ( CvEstimator* )_cvEstimator )->saveNextFrame( nextFrame );
}


void cvEstimator_procFrame( void *_cvEstimator, Mat& nextFrame, bool flag )
{
    ( ( CvEstimator* )_cvEstimator )->procFrame( nextFrame, flag );
}
