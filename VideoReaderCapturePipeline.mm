
/*
 Copyright ( C ) 2016 Apple Inc. All Rights Reserved.
 See LICENSE.txt for this sample’s licensing information
 
 Abstract:
 The class that creates and manages the AVCaptureSession
 */

#import "VideoReaderCapturePipeline.h"

#import "VideoReaderOpenGLRenderer.h"
#import "VideoReaderOpenCVRenderer.h"

#import <CoreMedia/CMBufferQueue.h>
#import <CoreMedia/CMAudioClock.h>
#import <AssetsLibrary/AssetsLibrary.h>
#import <ImageIO/CGImageProperties.h>

#import <opencv2/opencv.hpp>

/*
 RETAINED_BUFFER_COUNT is the number of pixel buffers we expect to hold on to from the renderer. This value informs the renderer how to size its buffer pool and how many pixel buffers to preallocate ( done in the prepareWithOutputDimensions: method ). Preallocation helps to lessen the chance of frame drops in our recording, in particular during recording startup. If we try to hold on to more buffers than RETAINED_BUFFER_COUNT then the renderer will fail to allocate new buffers from its pool and we will drop frames.

 A back of the envelope calculation to arrive at a RETAINED_BUFFER_COUNT of '6':
 - The preview path only has the most recent frame, so this makes the movie recording path the long pole.
 - The movie recorder internally does a dispatch_async to avoid blocking the caller when enqueuing to its internal asset writer.
 - Allow 2 frames of latency to cover the dispatch_async and the -[ AVAssetWriterInput appendSampleBuffer: ] call.
 - Then we allow for the encoder to retain up to 4 frames. Two frames are retained while being encoded/format converted, while the other two are to handle encoder format conversion pipelining and encoder startup latency.

 Really you need to test and measure the latency in your own application pipeline to come up with an appropriate number. 1080p BGRA buffers are quite large, so it's a good idea to keep this number as low as possible.
 */

#define RETAINED_BUFFER_COUNT 6 //fixme, original was 6

#define LOG_STATUS_TRANSITIONS 0



@interface VideoReaderCapturePipeline ( ) <AVCaptureAudioDataOutputSampleBufferDelegate, AVCaptureVideoDataOutputSampleBufferDelegate>
{
	NSMutableArray *_previousSecondTimestamps;

	AVCaptureSession *_captureSession;
	AVCaptureDevice *_videoDevice;
	AVCaptureConnection *_audioConnection;
	AVCaptureConnection *_videoConnection;
	AVCaptureVideoOrientation _videoBufferOrientation;
	BOOL _running;
	BOOL _startCaptureSessionOnEnteringForeground;
	id _applicationWillEnterForegroundNotificationObserver;
	NSDictionary *_videoCompressionSettings;
	
	dispatch_queue_t _sessionQueue;
	dispatch_queue_t _videoDataOutputQueue;
	
    id<VideoReaderRenderer> _renderer;
    VideoReaderOpenCVRenderer *_rendererCV;
	BOOL _renderingEnabled;
		
	UIBackgroundTaskIdentifier _pipelineRunningTask;
	
	__weak id<VideoReaderCapturePipelineDelegate> _delegate;
	dispatch_queue_t _delegateCallbackQueue;
    
    cv::Mat _resizeMat;
    
}

// Redeclared readwrite
@property( atomic, readwrite ) float videoFrameRate;
@property( atomic, readwrite ) CMVideoDimensions videoDimensions;
@property( atomic, readwrite ) UIImage *image;
@property( atomic, readwrite ) CVOpenGLESTextureRef texture;
@property( atomic, readwrite ) CVPixelBufferRef renderedPixelBuffer;
@property( atomic, readwrite ) cv::Mat imageMat;


// Because we specify __attribute__( ( NSObject ) ) ARC will manage the lifetime of the backing ivars even though they are CF types.
@property( nonatomic, strong ) __attribute__( ( NSObject ) ) CVPixelBufferRef currentPreviewPixelBuffer;
@property( nonatomic, strong ) __attribute__( ( NSObject ) ) CMFormatDescriptionRef outputVideoFormatDescription;
@property( nonatomic, strong ) __attribute__( ( NSObject ) ) CMFormatDescriptionRef outputAudioFormatDescription;

@end

@implementation VideoReaderCapturePipeline

- ( instancetype )initWithDelegate:( id<VideoReaderCapturePipelineDelegate> )delegate callbackQueue:( dispatch_queue_t )queue // delegate is weak referenced
{
	NSParameterAssert(  delegate != nil  );
	NSParameterAssert(  queue != nil  );
	
	self = [ super init ];
	if (  self  )
	{
		_previousSecondTimestamps = [ [ NSMutableArray alloc ] init ];
		

		
		_sessionQueue = dispatch_queue_create(  "com.apple.sample.capturepipeline.session", DISPATCH_QUEUE_SERIAL  );
		
		// In a multi-threaded producer consumer system it's generally a good idea to make sure that producers do not get starved of CPU time by their consumers.
		// In this app we start with VideoDataOutput frames on a high priority queue, and downstream consumers use default priority queues.
		// Audio uses a default priority queue because we aren't monitoring it live and just want to get it into the movie.
		// AudioDataOutput can tolerate more latency than VideoDataOutput as its buffers aren't allocated out of a fixed size pool.
		_videoDataOutputQueue = dispatch_queue_create(  "com.apple.sample.capturepipeline.video", DISPATCH_QUEUE_SERIAL  );
		dispatch_set_target_queue(  _videoDataOutputQueue, dispatch_get_global_queue(  DISPATCH_QUEUE_PRIORITY_HIGH, 0  )  );
		
// USE_XXX_RENDERER is set in the project's build settings for each target
//#if USE_OPENGL_RENDERER
		_renderer = [ [ VideoReaderOpenGLRenderer alloc ] init ];
//#elif USE_OPENCV_RENDERER
		_rendererCV = [ [ VideoReaderOpenCVRenderer alloc ] init ];
//#endif
				
		_pipelineRunningTask = UIBackgroundTaskInvalid;
		_delegate = delegate;
		_delegateCallbackQueue = queue;
	}
	return self;
}

- ( void )dealloc
{
	[ self teardownCaptureSession ];
}

#pragma mark Capture Session

- ( void )startRunning
{
	dispatch_sync(  _sessionQueue, ^{
		[ self setupCaptureSession ];
		
		if (  _captureSession  ) {
			[ _captureSession startRunning ];
			_running = YES;
		}
	}  );
}

- ( void )stopRunning
{
	dispatch_sync(  _sessionQueue, ^{
		_running = NO;
		
		// the captureSessionDidStopRunning method will stop recording if necessary as well, but we do it here so that the last video and audio samples are better aligned
		
		[ _captureSession stopRunning ];
		
		[ self captureSessionDidStopRunning ];
		
		[ self teardownCaptureSession ];
	}  );
}

- ( void )setupCaptureSession
{
	if (  _captureSession  ) {
		return;
	}
	
	_captureSession = [ [ AVCaptureSession alloc ] init ];	

	[ [ NSNotificationCenter defaultCenter ] addObserver:self selector:@selector( captureSessionNotification: ) name:nil object:_captureSession ];
	_applicationWillEnterForegroundNotificationObserver = [ [ NSNotificationCenter defaultCenter ] addObserverForName:UIApplicationWillEnterForegroundNotification object:[ UIApplication sharedApplication ] queue:nil usingBlock:^( NSNotification *note ) {
		// Retain self while the capture session is alive by referencing it in this observer block which is tied to the session lifetime
		// Client must stop us running before we can be deallocated
		[ self applicationWillEnterForeground ];
	} ];
	
	
	/* Video */
	AVCaptureDevice *videoDevice = [ AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo ];
    
    /*
    //// Mori
    NSError *error2;
    [ videoDevice lockForConfiguration:&error2 ];
    if ( [ videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeLocked ] ) {
        videoDevice.whiteBalanceMode = AVCaptureWhiteBalanceModeLocked;
        NSLog( @"Whitebalanced locked" );
    }
    if ( [ videoDevice isExposureModeSupported:AVCaptureExposureModeLocked ] ) {
        videoDevice.exposureMode = AVCaptureExposureModeLocked;
        NSLog( @"Exposure locked" );
    }
    [ videoDevice unlockForConfiguration ];
    ////
    */
    
	NSError *videoDeviceError = nil;
	AVCaptureDeviceInput *videoIn = [ [ AVCaptureDeviceInput alloc ] initWithDevice:videoDevice error:&videoDeviceError ];
	if (  [ _captureSession canAddInput:videoIn ]  ) {
		[ _captureSession addInput:videoIn ];
        _videoDevice = videoDevice;
	}
	else {
		[ self handleNonRecoverableCaptureSessionRuntimeError:videoDeviceError ];
		return;
	}
	
	AVCaptureVideoDataOutput *videoOut = [ [ AVCaptureVideoDataOutput alloc ] init ];
	videoOut.videoSettings = @{ ( id )kCVPixelBufferPixelFormatTypeKey : @( _renderer.inputPixelFormat ) };
	[ videoOut setSampleBufferDelegate:self queue:_videoDataOutputQueue ];
	
	// VideoReader records videos and we prefer not to have any dropped frames in the video recording.
	// By setting alwaysDiscardsLateVideoFrames to NO we ensure that minor fluctuations in system load or in our processing time for a given frame won't cause framedrops.
	// We do however need to ensure that on average we can process frames in realtime.
	// If we were doing preview only we would probably want to set alwaysDiscardsLateVideoFrames to YES.
	videoOut.alwaysDiscardsLateVideoFrames = YES; //// Mori
	
	if (  [ _captureSession canAddOutput:videoOut ]  ) {
		[ _captureSession addOutput:videoOut ];
	}
	_videoConnection = [ videoOut connectionWithMediaType:AVMediaTypeVideo ];
		
    
    //// Mori
    if ( [ _videoConnection isVideoOrientationSupported ] )
    {
        [ _videoConnection setVideoOrientation:AVCaptureVideoOrientationPortrait ];
    }
    ////
    
	int frameRate;
	NSString *sessionPreset = AVCaptureSessionPresetHigh;
	CMTime frameDuration = kCMTimeInvalid;
	// For single core systems like iPhone 4 and iPod Touch 4th Generation we use a lower resolution and framerate to maintain real-time performance.
	if (  [ NSProcessInfo processInfo ].processorCount == 1  )
	{
		if (  [ _captureSession canSetSessionPreset:AVCaptureSessionPreset640x480 ]  ) {
			sessionPreset = AVCaptureSessionPreset640x480;
		}
		frameRate = 15;
	}
	else
	{
#if ! USE_OPENGL_RENDERER
		// When using the CPU renderers or the CoreImage renderer we lower the resolution to 720p so that all devices can maintain real-time performance ( this is primarily for A5 based devices like iPhone 4s and iPod Touch 5th Generation ).
		//// if (  [ _captureSession canSetSessionPreset:AVCaptureSessionPreset1280x720 ]  ) {
		////	sessionPreset = AVCaptureSessionPreset1280x720;
        if (  [ _captureSession canSetSessionPreset:AVCaptureSessionPreset3840x2160  ]  ) {
        	sessionPreset = AVCaptureSessionPreset3840x2160;
		}
#else// ! USE_OPENGL_RENDERER
        
        //// Mori
        if (  [ _captureSession canSetSessionPreset:AVCaptureSessionPreset3840x2160 ]  ) {
            sessionPreset = AVCaptureSessionPreset3840x2160;
        }
#endif
        ////

		frameRate = 30;
	}
	
	_captureSession.sessionPreset = sessionPreset;
	
	frameDuration = CMTimeMake(  1, frameRate  );

	NSError *error = nil;
	if (  [ videoDevice lockForConfiguration:&error ]  ) {
		videoDevice.activeVideoMaxFrameDuration = frameDuration;
		videoDevice.activeVideoMinFrameDuration = frameDuration;
		[ videoDevice unlockForConfiguration ];
	}
	else {
		NSLog(  @"videoDevice lockForConfiguration returned error %@", error  );
	}

	// Get the recommended compression settings after configuring the session/device.
	_videoCompressionSettings = [ [ videoOut recommendedVideoSettingsForAssetWriterWithOutputFileType:AVFileTypeQuickTimeMovie ] copy ];
	
	_videoBufferOrientation = _videoConnection.videoOrientation;
	
	return;
}

- ( void )teardownCaptureSession
{
	if (  _captureSession  )
	{
		[ [ NSNotificationCenter defaultCenter ] removeObserver:self name:nil object:_captureSession ];
		
		[ [ NSNotificationCenter defaultCenter ] removeObserver:_applicationWillEnterForegroundNotificationObserver ];
		_applicationWillEnterForegroundNotificationObserver = nil;
		
		_captureSession = nil;
		
		_videoCompressionSettings = nil;
	}
}

- ( void )captureSessionNotification:( NSNotification * )notification
{
	dispatch_async(  _sessionQueue, ^{
		
		if (  [ notification.name isEqualToString:AVCaptureSessionWasInterruptedNotification ]  )
		{
			NSLog(  @"session interrupted"  );
			
			[ self captureSessionDidStopRunning ];
		}
		else if (  [ notification.name isEqualToString:AVCaptureSessionInterruptionEndedNotification ]  )
		{
			NSLog(  @"session interruption ended"  );
		}
		else if (  [ notification.name isEqualToString:AVCaptureSessionRuntimeErrorNotification ]  )
		{
			[ self captureSessionDidStopRunning ];
			
			NSError *error = notification.userInfo[ AVCaptureSessionErrorKey ];
			if (  error.code == AVErrorDeviceIsNotAvailableInBackground  )
			{
				NSLog(  @"device not available in background"  );

				// Since we can't resume running while in the background we need to remember this for next time we come to the foreground
				if (  _running  ) {
					_startCaptureSessionOnEnteringForeground = YES;
				}
			}
			else if (  error.code == AVErrorMediaServicesWereReset  )
			{
				NSLog(  @"media services were reset"  );
				[ self handleRecoverableCaptureSessionRuntimeError:error ];
			}
			else
			{
				[ self handleNonRecoverableCaptureSessionRuntimeError:error ];
			}
		}
		else if (  [ notification.name isEqualToString:AVCaptureSessionDidStartRunningNotification ]  )
		{
			NSLog(  @"session started running"  );
		}
		else if (  [ notification.name isEqualToString:AVCaptureSessionDidStopRunningNotification ]  )
		{
			NSLog(  @"session stopped running"  );
		}
	}  );
}

- ( void )handleRecoverableCaptureSessionRuntimeError:( NSError * )error
{
	if (  _running  ) {
		[ _captureSession startRunning ];
	}
}

- ( void )handleNonRecoverableCaptureSessionRuntimeError:( NSError * )error
{
	NSLog(  @"fatal runtime error %@, code %i", error, ( int )error.code  );
	
	_running = NO;
	[ self teardownCaptureSession ];
	
	[ self invokeDelegateCallbackAsync:^{
		[ _delegate capturePipeline:self didStopRunningWithError:error ];
	} ];
}

- ( void )captureSessionDidStopRunning
{
	[ self teardownVideoPipeline ];
}

- ( void )applicationWillEnterForeground
{
	NSLog(  @"-[ %@ %@ ] called", [ self class ], NSStringFromSelector( _cmd )  );
	
	dispatch_sync(  _sessionQueue, ^{
		
		if (  _startCaptureSessionOnEnteringForeground  )
		{
			NSLog(  @"-[ %@ %@ ] manually restarting session", [ self class ], NSStringFromSelector( _cmd )  );
			
			_startCaptureSessionOnEnteringForeground = NO;
			if (  _running  ) {
				[ _captureSession startRunning ];
			}
		}
	}  );
}

#pragma mark Capture Pipeline

- ( void )setupVideoPipelineWithInputFormatDescription:( CMFormatDescriptionRef )inputFormatDescription
{
	NSLog(  @"-[ %@ %@ ] called", [ self class ], NSStringFromSelector( _cmd )  );
	
	[ self videoPipelineWillStartRunning ];
	
	self.videoDimensions = CMVideoFormatDescriptionGetDimensions(  inputFormatDescription  );
	[ _renderer prepareForInputWithFormatDescription:inputFormatDescription outputRetainedBufferCountHint:RETAINED_BUFFER_COUNT ];
	[ _rendererCV prepareForInputWithFormatDescription:inputFormatDescription outputRetainedBufferCountHint:RETAINED_BUFFER_COUNT ];
    
    
	if (  ! _renderer.operatesInPlace && [ _renderer respondsToSelector:@selector( outputFormatDescription ) ]  ) {
		self.outputVideoFormatDescription = _renderer.outputFormatDescription;
	}
	else {
		self.outputVideoFormatDescription = inputFormatDescription;
	}
}

// synchronous, blocks until the pipeline is drained, don't call from within the pipeline
- ( void )teardownVideoPipeline
{
	// The session is stopped so we are guaranteed that no new buffers are coming through the video data output.
	// There may be inflight buffers on _videoDataOutputQueue however.
	// Synchronize with that queue to guarantee no more buffers are in flight.
	// Once the pipeline is drained we can tear it down safely.

	NSLog(  @"-[ %@ %@ ] called", [ self class ], NSStringFromSelector( _cmd )  );
	
	dispatch_sync(  _videoDataOutputQueue, ^{
		
		if (  ! self.outputVideoFormatDescription  ) {
			return;
		}
		
		self.outputVideoFormatDescription = NULL;
		[ _renderer reset ];
		self.currentPreviewPixelBuffer = NULL;
		
		NSLog(  @"-[ %@ %@ ] finished teardown", [ self class ], NSStringFromSelector( _cmd )  );
		
		[ self videoPipelineDidFinishRunning ];
	}  );
}

- ( void )videoPipelineWillStartRunning
{
	NSLog(  @"-[ %@ %@ ] called", [ self class ], NSStringFromSelector( _cmd )  );
	
	NSAssert(  _pipelineRunningTask == UIBackgroundTaskInvalid, @"should not have a background task active before the video pipeline starts running"  );
	
	_pipelineRunningTask = [ [ UIApplication sharedApplication ] beginBackgroundTaskWithExpirationHandler:^{
		NSLog(  @"video capture pipeline background task expired"  );
	} ];
}

- ( void )videoPipelineDidFinishRunning
{
	NSLog(  @"-[ %@ %@ ] called", [ self class ], NSStringFromSelector( _cmd )  );
	
	NSAssert(  _pipelineRunningTask != UIBackgroundTaskInvalid, @"should have a background task active when the video pipeline finishes running"  );
	
	[ [ UIApplication sharedApplication ] endBackgroundTask:_pipelineRunningTask ];
	_pipelineRunningTask = UIBackgroundTaskInvalid;
}

- ( void )videoPipelineDidRunOutOfBuffers
{
	// We have run out of buffers.
	// Tell the delegate so that it can flush any cached buffers.
	
	[ self invokeDelegateCallbackAsync:^{
		[ _delegate capturePipelineDidRunOutOfPreviewBuffers:self ];
	} ];
}

- ( void )setRenderingEnabled:( BOOL )renderingEnabled
{
	@synchronized(  _renderer  ) {
		_renderingEnabled = renderingEnabled;
	}
}

- ( BOOL )renderingEnabled
{
	@synchronized(  _renderer  ) {
		return _renderingEnabled;
	}
}

- ( void )captureOutput:( AVCaptureOutput * )captureOutput didOutputSampleBuffer:( CMSampleBufferRef )sampleBuffer fromConnection:( AVCaptureConnection * )connection
{
	CMFormatDescriptionRef formatDescription = CMSampleBufferGetFormatDescription(  sampleBuffer  );
	
	if (  connection == _videoConnection  )
	{
		if (  self.outputVideoFormatDescription == NULL  ) {
			// Don't render the first sample buffer.
			// This gives us one frame interval ( 33ms at 30fps ) for setupVideoPipelineWithInputFormatDescription: to complete.
			// Ideally this would be done asynchronously to ensure frames don't back up on slower devices.
			[ self setupVideoPipelineWithInputFormatDescription:formatDescription ];
		}
		else {
			[ self renderVideoSampleBuffer:sampleBuffer ];
		}
	}
	else if (  connection == _audioConnection  )
	{
		self.outputAudioFormatDescription = formatDescription;
		
		@synchronized(  self  ) {

		}
	}
}

- ( void )renderVideoSampleBuffer:( CMSampleBufferRef )sampleBuffer
{
    CVPixelBufferRef renderedPixelBuffer = NULL;
    CVPixelBufferRef renderedPixelBuffer2 = NULL;
    CVPixelBufferRef renderedPixelBuffer3 = NULL;
    
    CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(  sampleBuffer  );
    
    [ self calculateFramerateAtTimestamp:timestamp ];
    
    // We must not use the GPU while running in the background.
    // setRenderingEnabled: takes the same lock so the caller can guarantee no GPU usage once the setter returns.
    @synchronized(  _renderer  )
    {
        if (  _renderingEnabled  ) {
            
            CVPixelBufferRef sourcePixelBuffer = CMSampleBufferGetImageBuffer(  sampleBuffer  );
            
            renderedPixelBuffer = [ _renderer copyRenderedPixelBuffer:sourcePixelBuffer ];
            renderedPixelBuffer2 = [ _renderer copyRenderedPixelBufferWithResize:renderedPixelBuffer ];
            renderedPixelBuffer3 = [ _rendererCV copyRenderedPixelBuffer:renderedPixelBuffer2 ];
            
            _imageMat = [ _rendererCV getMatImage ];
            _image =  [ _rendererCV getUIImage ];
            
            CGSize s = _image.size;
            CFRelease(  renderedPixelBuffer2  );
            CFRelease(  renderedPixelBuffer3  );
            
        }
        else {
            return;
        }
    }
    
    if (  renderedPixelBuffer  )
    {
        @synchronized(  self  )
        {
            [ self outputPreviewPixelBuffer:renderedPixelBuffer ];
            
        }
        
        CFRelease(  renderedPixelBuffer );
        //CFRelease(  renderedPixelBuffer2  );
    }
    else
    {
        [ self videoPipelineDidRunOutOfBuffers ];
    }
}

// call under @synchronized(  self  )
- ( void )outputPreviewPixelBuffer:( CVPixelBufferRef )previewPixelBuffer
{
	// Keep preview latency low by dropping stale frames that have not been picked up by the delegate yet
	// Note that access to currentPreviewPixelBuffer is protected by the @synchronized lock
	self.currentPreviewPixelBuffer = previewPixelBuffer;
	
    
    const CMVideoDimensions srcDimensions = { ( int32_t )CVPixelBufferGetWidth( previewPixelBuffer ), ( int32_t )CVPixelBufferGetHeight( previewPixelBuffer ) };
    
	[ self invokeDelegateCallbackAsync:^{
		
		CVPixelBufferRef currentPreviewPixelBuffer = NULL;
		@synchronized(  self  )
		{
			currentPreviewPixelBuffer = self.currentPreviewPixelBuffer;
			if (  currentPreviewPixelBuffer  ) {
				CFRetain(  currentPreviewPixelBuffer  );
				self.currentPreviewPixelBuffer = NULL;
			}
		}
		
		if (  currentPreviewPixelBuffer  ) {
			[ _delegate capturePipeline:self previewPixelBufferReadyForDisplay:currentPreviewPixelBuffer ];
			CFRelease(  currentPreviewPixelBuffer  );

            //NSLog( @"%d", CVOpenGLESTextureGetName(  _texture  ) );

		}
	} ];
    

}


#pragma mark Utilities

- ( void )invokeDelegateCallbackAsync:( dispatch_block_t )callbackBlock
{
	dispatch_async(  _delegateCallbackQueue, ^{
		@autoreleasepool {
			callbackBlock( );
		}
	}  );
}

// Auto mirroring: Front camera is mirrored; back camera isn't 
- ( CGAffineTransform )transformFromVideoBufferOrientationToOrientation:( AVCaptureVideoOrientation )orientation withAutoMirroring:( BOOL )mirror
{
	CGAffineTransform transform = CGAffineTransformIdentity;
		
	// Calculate offsets from an arbitrary reference orientation ( portrait )
	CGFloat orientationAngleOffset = angleOffsetFromPortraitOrientationToOrientation(  orientation  );
	CGFloat videoOrientationAngleOffset = angleOffsetFromPortraitOrientationToOrientation(  _videoBufferOrientation  );
	
	// Find the difference in angle between the desired orientation and the video orientation
	CGFloat angleOffset = orientationAngleOffset - videoOrientationAngleOffset;
	transform = CGAffineTransformMakeRotation(  angleOffset  );

	if (  _videoDevice.position == AVCaptureDevicePositionFront  )
	{
		if (  mirror  ) {
			transform = CGAffineTransformScale(  transform, -1, 1  );
		}
		else {
			if (  UIInterfaceOrientationIsPortrait(  ( UIInterfaceOrientation )orientation  )  ) {
				transform = CGAffineTransformRotate(  transform, M_PI  );
			}
		}
	}
	
	return transform;
}

static CGFloat angleOffsetFromPortraitOrientationToOrientation( AVCaptureVideoOrientation orientation )
{
	CGFloat angle = 0.0;
	
	switch (  orientation  )
	{
		case AVCaptureVideoOrientationPortrait:
			angle = 0.0;
			break;
		case AVCaptureVideoOrientationPortraitUpsideDown:
			angle = M_PI;
			break;
		case AVCaptureVideoOrientationLandscapeRight:
			angle = -M_PI_2;
			break;
		case AVCaptureVideoOrientationLandscapeLeft:
			angle = M_PI_2;
			break;
		default:
			break;
	}
	
	return angle;
}

- ( void )calculateFramerateAtTimestamp:( CMTime )timestamp
{
	[ _previousSecondTimestamps addObject:[ NSValue valueWithCMTime:timestamp ] ];
	
	CMTime oneSecond = CMTimeMake(  1, 1  );
	CMTime oneSecondAgo = CMTimeSubtract(  timestamp, oneSecond  );
	
	while(  CMTIME_COMPARE_INLINE(  [ _previousSecondTimestamps[ 0 ] CMTimeValue ], <, oneSecondAgo  )  ) {
		[ _previousSecondTimestamps removeObjectAtIndex:0 ];
	}
	
	if (  [ _previousSecondTimestamps count ] > 1  )
	{
		const Float64 duration = CMTimeGetSeconds(  CMTimeSubtract(  [ [ _previousSecondTimestamps lastObject ] CMTimeValue ], [ _previousSecondTimestamps[ 0 ] CMTimeValue ]  )  );
		const float newRate = ( float )(  [ _previousSecondTimestamps count ] - 1  ) / duration;
		self.videoFrameRate = newRate;
	}
}

@end
