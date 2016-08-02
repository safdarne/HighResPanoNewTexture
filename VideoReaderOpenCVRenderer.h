
/*
 Copyright ( C ) 2016 Apple Inc. All Rights Reserved.
 See LICENSE.txt for this sample’s licensing information
 
 Abstract:
 The VideoReader OpenCV based effect renderer
 */

#import "VideoReaderRenderer.h"
#import <opencv2/opencv.hpp>


// To use the VideoReaderOpenCVRenderer, import this header in VideoReaderCapturePipeline.m
// and intialize _renderer to a VideoReaderOpenCVRenderer.

@interface VideoReaderOpenCVRenderer : NSObject <VideoReaderRenderer>

-( cv::Mat ) getMatImage;

@end

