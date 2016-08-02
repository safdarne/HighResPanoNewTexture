
 /*
 Copyright ( C ) 2016 Apple Inc. All Rights Reserved.
 See LICENSE.txt for this sample’s licensing information
 
 Abstract:
 The class that creates and manages the AVCaptureSession
 */


#import <AVFoundation/AVFoundation.h>

@protocol VideoReaderCapturePipelineDelegate;

@interface VideoReaderCapturePipeline : NSObject 

- ( instancetype )initWithDelegate:( id<VideoReaderCapturePipelineDelegate> )delegate callbackQueue:( dispatch_queue_t )queue; // delegate is weak referenced

// These methods are synchronous
- ( void )startRunning;
- ( void )stopRunning;


@property( atomic ) BOOL renderingEnabled; // When set to NO the GPU will not be used after the setRenderingEnabled: call returns.



- ( CGAffineTransform )transformFromVideoBufferOrientationToOrientation:( AVCaptureVideoOrientation )orientation withAutoMirroring:( BOOL )mirroring; // only valid after startRunning has been called

// Stats
@property( atomic, readonly ) float videoFrameRate;
@property( atomic, readonly ) CMVideoDimensions videoDimensions;

@property( atomic, readonly ) UIImage *image;
@property( atomic, readonly ) CVOpenGLESTextureRef texture;
@property( atomic, readonly ) CVPixelBufferRef renderedPixelBuffer;
@property( atomic, readonly ) cv::Mat imageMat;




@end

@protocol VideoReaderCapturePipelineDelegate <NSObject>
@required

- ( void )capturePipeline:( VideoReaderCapturePipeline * )capturePipeline didStopRunningWithError:( NSError * )error;

// Preview
- ( void )capturePipeline:( VideoReaderCapturePipeline * )capturePipeline previewPixelBufferReadyForDisplay:( CVPixelBufferRef )previewPixelBuffer;
- ( void )capturePipelineDidRunOutOfPreviewBuffers:( VideoReaderCapturePipeline * )capturePipeline;



@end
