#import "ViewController.h"
#import "livePanoInterface.h"  
#import <QuartzCore/QuartzCore.h>
#import "cvEstimatorInterface.h"

#import <OpenGLES/EAGL.h>
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>

#include "opencv2/opencv.hpp"
#include <numeric>      // std::iota

#import "VideoReaderCapturePipeline.h"

@interface ViewController ( ) <VideoReaderCapturePipelineDelegate> {
    void  *_panoContext;
    void *_cvEstimator;
    
    GLuint _imageTexture, _oldTexture;
    int    _panoImageW;
    int    _panoImageH;
    int    _cvImageW;
    int    _cvImageH;
    int _storedImagesCount;
    
    float _roll;
    float _pitch;
    float _yaw;
    float _pitchArray[ 100 ];
    float _rollArray[ 100 ];
    float _yawArray[ 100 ];
    float _xMotion, _xVelocity;
    float _focal;
    float _userPanX;
    float _userPanY;
    float _userZoom;
    
    std::vector<std::vector <float>> _refinedRotations;
    std::vector<float> _rotationFromQuaternion3x3;
    std::vector<float> _rotationMatrixFromImage;
    std::vector<float> _rotationMatrixFromImagePrev;
    std::vector<std::vector<float>> _vcRotationFromSensorArray;
    std::vector<std::vector<int>> _closestFrames;

    UIDeviceOrientation _startDeviceOrientation;
    CVOpenGLESTextureCacheRef _textureCache;
    
    BOOL   _cvSaveNextFlag;
    BOOL   _panoUploadNextFlag;
    BOOL _displayMode;
    BOOL _cvProcessFramesFlag;
    BOOL _captureScreen;
    
    // VideoReader
    BOOL _addedObservers;
    BOOL _recording;
    BOOL _allowedToUseGPU;
    
    NSTimer *_labelTimer;
    VideoReaderCapturePipeline *_capturePipeline;
}

@property ( weak, nonatomic ) IBOutlet UILabel *framerateLabel;
@property ( weak, nonatomic ) IBOutlet UILabel *dimensionsLabel;
@property (weak, nonatomic) IBOutlet UILabel *imageCountLabel;
@property ( strong, nonatomic ) GLKTextureLoader *asyncTextureLoader;
@property ( strong, nonatomic ) GLKTextureInfo *hugeTexture;
@property ( strong, nonatomic ) EAGLContext *context;
@property ( weak, nonatomic ) IBOutlet UIButton *startButton;
@property ( weak, nonatomic ) IBOutlet UIButton *viewButton;
@property ( weak, nonatomic ) IBOutlet UIButton *restartButton;
@property ( weak, nonatomic ) IBOutlet UIButton *refineButton;
@property ( weak, nonatomic ) IBOutlet UISwitch *flagSwitch;
@property (weak, nonatomic) IBOutlet UIButton *captureScreenButton;


- ( void )initialize;
- ( void )setupGL;
- ( void )setupCV;
- ( void ) setupCamera;
- ( void ) setupMotionSensor;
- ( void )tearDownGL;
- ( BOOL )compileShader:( GLuint * )shader type:( GLenum )type file:( NSString * )file;
- ( cv::Mat ) cvMatFromUIImage:( UIImage * )image;
- ( UIImage * )UIImageFromCVMat:( cv::Mat )cvMat;
- (void) checkIfShoudStoreFrame;
- ( std::vector<float> ) getDisplayRotation;


@end





@implementation ViewController



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
- ( void )initialize
{
    // Resolution of the image to be send to vision module for processing
    // fixme, link these numbers to _cvResizeFactor in VideoReaderOpenCVRenderer.mm
    _cvImageW = 360 * 2;//720;
    _cvImageH = 640 * 2;//1280;
    
    // These numbers actually set the desired resolution for rendering using OpenGL and do not need to be the true resolution of the image fed to OpenGL as texture
    _panoImageW       = 216*2;
    _panoImageH       = 384*2;
    
    [ self.imageView.layer setBorderColor: [ [ UIColor redColor ] CGColor ] ];
    [ self.imageView.layer setBorderWidth: 2.0 ];
    
    
    _rotationFromQuaternion3x3.resize(9);
    _rotationMatrixFromImagePrev.resize(9);
    _rotationMatrixFromImage.resize(9);
    
    for ( int i = 0; i <9 ;i++ )
        _rotationFromQuaternion3x3[ i ] = 0;
    
    for ( int i = 0; i <9 ;i++ )
        _rotationMatrixFromImagePrev[ i ] = 0;
    
    for ( int i = 0; i <9 ;i++ )
        _rotationMatrixFromImage[ i ] = 0;
    
    _rotationFromQuaternion3x3[ 0 ] = 1;
    _rotationFromQuaternion3x3[ 4 ] = 1;
    _rotationFromQuaternion3x3[ 8 ] = 1;
    
    _rotationMatrixFromImagePrev[ 0 ] = 1;
    _rotationMatrixFromImagePrev[ 4 ] = 1;
    _rotationMatrixFromImagePrev[ 8 ] = 1;
    
    _rotationMatrixFromImage[ 0 ] = 1;
    _rotationMatrixFromImage[ 4 ] = 1;
    _rotationMatrixFromImage[ 8 ] = 1;
    
    _vcRotationFromSensorArray.push_back( _rotationFromQuaternion3x3 );
    
    _xMotion = 0;
    _xVelocity = 0;
    
    _focal = 1116;
    
    _closestFrames.push_back( std::vector<int>( ) );
    
    _cvProcessFramesFlag = false;
    _cvSaveNextFlag = false;
    _panoUploadNextFlag = false;
    _storedImagesCount = 0;
    _displayMode = false;
    
    
    _vcRotationFromSensorArray.clear( );
    _vcRotationFromSensorArray.push_back( _rotationFromQuaternion3x3 );
    
    _closestFrames.clear( );
    _closestFrames.push_back( std::vector<int>( ) );
    
    _userPanX = 0;
    _userPanY = 0;
    _userZoom = 1;
    
    _captureScreen = false;
    
    _storedImagesCount = 0;
    _displayMode = false;
    
    [ self.imageView setHidden:YES];
    
    if ( _cvProcessFramesFlag )
        [ self.flagLabel setText:[ NSString stringWithFormat:@"On" ] ];
    else
        [ self.flagLabel setText:[ NSString stringWithFormat:@"Off" ] ];
}
////////////////////////////////////////////////////////////////
- ( void )setupGL
{
    [ EAGLContext setCurrentContext:self.context ];

    _imageTexture = info.name;
    
    // create/initialize a Pano context
    _panoContext = pano_create( );
    pano_initializeOpenGLTexture( _panoContext, _imageTexture, _panoImageW, _panoImageH, 0 );
}
////////////////////////////////////////////////////////////////
- ( void )tearDownGL
{
    [ EAGLContext setCurrentContext:self.context ];
    pano_destroy( _panoContext );
}
////////////////////////////////////////////////////////////////
- ( void )setupCV
{
    _cvEstimator = cvEstimator_create( );
    cvEstimator_initialize( _cvEstimator, _cvImageW, _cvImageH );  //fixme
}
////////////////////////////////////////////////////////////////
- (void) setupCamera
{
    //-------- Generate a new texture cash
    CVReturn err2 = CVOpenGLESTextureCacheCreate(
                                                 kCFAllocatorDefault,
                                                 NULL,
                                                 //( __bridge CVEAGLContext )( ( __bridge void * )view.context ),
                                                 ( __bridge CVEAGLContext )( ( __bridge void * )self.context ),
                                                 NULL,
                                                 &_textureCache );
    if ( err2 )
    {
        NSLog( @"Error at CVOpenGLESTextureCacheCreate %d", err2 );
    }

    //-------- VideoReader
    _capturePipeline = [ [ VideoReaderCapturePipeline alloc ] initWithDelegate:self callbackQueue:dispatch_get_main_queue( ) ];
    // the willEnterForeground and didEnterBackground notifications are subsequently used to update _allowedToUseGPU
    _allowedToUseGPU = (  [ UIApplication sharedApplication ].applicationState != UIApplicationStateBackground  );
    _capturePipeline.renderingEnabled = _allowedToUseGPU;
}
////////////////////////////////////////////////////////////////
- ( void ) setupMotionSensor
{
    CMAttitudeReferenceFrame frame = CMAttitudeReferenceFrameXArbitraryCorrectedZVertical;
    self.motionManager = [ [ CMMotionManager alloc ] init ];
    self.motionManager.deviceMotionUpdateInterval = 1.0/60.0;
    [ self.motionManager startDeviceMotionUpdatesUsingReferenceFrame:frame ];

    NSTimer* timer = [ NSTimer timerWithTimeInterval:1.0/60.f target:self selector:@selector( updateLabel ) userInfo:nil repeats:YES ];
    [ [ NSRunLoop mainRunLoop ] addTimer:timer forMode:NSRunLoopCommonModes ];

    _accelerometer = [ UIAccelerometer sharedAccelerometer ];
    _accelerometer.updateInterval = 1./20.;
    _accelerometer.delegate = self;
}
////////////////////////////////////////////////////////////////
- ( void )glkView:( GLKView * )view drawInRect:( CGRect )rect
{
    [ self.rollLabel setText:[ NSString stringWithFormat:@"Roll:   %.01f", (  _roll ) * 180 / M_PI ] ];
    [ self.pitchLabel setText:[ NSString stringWithFormat:@"Pitch: %.01f", (  _pitch ) * 180 / M_PI ] ];
    [ self.yawLabel setText:[ NSString stringWithFormat:@"Yaw:   %.01f", (  _yaw ) * 180 / M_PI ] ];
    
    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    
    CGRect screenBound = [ [ UIScreen mainScreen ] bounds ];
    CGSize screenSize = screenBound.size;
    
    
    //-------- Prepare next frame for OpenCV
    _frameFromCamera = _capturePipeline.image;
    cv::Mat nextMat = [ self cvMatFromUIImage:_frameFromCamera ];
    cv::cvtColor( nextMat, nextMat, CV_BGR2GRAY );
    
    
    //-------- If capturing started, check if the new frame should be stored (overlap with previous frames below a threshold)
    // If yes, set appropriate flags and do required processing
    if ( _storedImagesCount > 0 ) // started capturing
    {
        [ self checkIfShoudStoreFrame ];
    }
    else
        _refAttitude = nil; // Capturing not started yet, set the reference to the current reading of motion sensor so that the current frame appears in the center of the screen
    
    
    //-------- Send the vision module the rotation estimation from the device sensor, and get the updated rotation estimation based on image features and sensor data
    cvEstimator_setRotationFromSensor( _cvEstimator, _rotationFromQuaternion3x3 );
    if ( _cvSaveNextFlag )
    {
        _vcRotationFromSensorArray.push_back( _rotationFromQuaternion3x3 );
        cvEstimator_saveNextFrame( _cvEstimator, nextMat );
        //_focal = cvEstimator_getFocalLength( _cvEstimator );
        _focal = 0.5 * _focal + 0.5 * cvEstimator_getFocalLength( _cvEstimator );
        pano_setRefinedRotations( _panoContext, _refinedRotations, _focal );
        std::cout << "Estimated focal - frame to frame: " << cvEstimator_getFocalLength( _cvEstimator ) << std::endl;
        
        // Do bundle adjustment
        //if ( _storedImagesCount > 2 )
        //    [ self.refinementButton sendActionsForControlEvents:UIControlEventTouchUpInside ];
    }
    else
    {
        cvEstimator_procFrame( _cvEstimator, nextMat, ( _cvProcessFramesFlag && !_displayMode ) );
        //_focal = cvEstimator_getFocalLength( _cvEstimator );
    }
    

    //-------- Convert the raltive rotation estimated between frames to a global rotation value to be consistent with rest of the code
    std::vector<float> rotationMatrixFromImage = cvEstimator_getRotation( _cvEstimator );
    _rotationMatrixFromImage =  multiplyMatrix( rotationMatrixFromImage, _rotationMatrixFromImagePrev, 3, 3 );

    if ( _cvSaveNextFlag )
    {
        _rotationMatrixFromImagePrev = _rotationMatrixFromImage;
        _cvSaveNextFlag = false;
    }
    
    
    //-------- Send the warping and display module, the estimated rotation for the current image
    //pano_setRotationFromImage( _panoContext, rotationMatrixFromImage );
    std::vector<float> fusedRotation = _rotationFromQuaternion3x3; //fixme, this should be the fusion of motion data and image data, not only motion
    
    
    //-------- Get rotation data from device and also user panning the screen to update display rotation
    std::vector<float> displayRotationMixed = [ self getDisplayRotation ];
    if ( _cvProcessFramesFlag )
        pano_updateMotionData( _panoContext, _roll, _pitch, _yaw, _rotationMatrixFromImage, displayRotationMixed ); //fixme, this should be uncommented !!!!!!!!!!!! careful what to send as rotration
    else
        pano_updateMotionData( _panoContext, _roll, _pitch, _yaw, _rotationFromQuaternion3x3, displayRotationMixed ); //fixme, this should be uncommented !!!!!!!!!!!! careful what to send as rotration
    
    
    //-------- Render the frames
    pano_step( _panoContext );
    pano_render( _panoContext, _panoImageW, _panoImageH, _userZoom );
    

    //-------- If user pressed the capture screen button, get the result from OpenGL and save it on device
    if (_captureScreen)
    {
        UIImage* glScreenCapture =[self glToUIImage];
        [ self.imageView setImage:glScreenCapture ];
        
        UIImageWriteToSavedPhotosAlbum(glScreenCapture, self, nil, nil);
        _captureScreen = false;
        [ self.imageView setHidden:NO];
    }
    
    UIDeviceOrientation orientation = [ [ UIDevice currentDevice ] orientation ];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
- ( void )viewDidLoad
{
    NSLog( @"viewDidLoad!" );
    [ super viewDidLoad ];
    [ self initialize ];
    
    [ [ NSNotificationCenter defaultCenter ] addObserver:self
                                             selector:@selector( applicationDidEnterBackground )
                                                 name:UIApplicationDidEnterBackgroundNotification
                                               object:[ UIApplication sharedApplication ] ];
    [ [ NSNotificationCenter defaultCenter ] addObserver:self
                                             selector:@selector( applicationWillEnterForeground )
                                                 name:UIApplicationWillEnterForegroundNotification
                                               object:[ UIApplication sharedApplication ] ];
    [ [ NSNotificationCenter defaultCenter ] addObserver:self
                                             selector:@selector( deviceOrientationDidChange )
                                                 name:UIDeviceOrientationDidChangeNotification
                                               object:[ UIDevice currentDevice ] ];
    
    //-------- Keep track of changes to the device orientation so we can update the capture pipeline
    [ [ UIDevice currentDevice ] beginGeneratingDeviceOrientationNotifications ];
    _addedObservers = YES;

    self.context = [ [ EAGLContext alloc ] initWithAPI:kEAGLRenderingAPIOpenGLES2 ];
    if ( !self.context ) {
        NSLog( @"Failed to create ES context" );
    }
    
    
    GLKView *view = ( GLKView * )self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    
    [ self setupCV ]; //fixme, release at the end
    [ self setupGL ];
    [ self setupCamera ];
    [ self setupMotionSensor ];
}
////////////////////////////////////////////////////////////////
- ( void )viewWillAppear:( BOOL )animated
{
    [ super viewWillAppear:animated ];
    
    [ _capturePipeline startRunning ];
    
    _labelTimer = [ NSTimer scheduledTimerWithTimeInterval:1.0/60.0 target:self selector:@selector( updateLabels ) userInfo:nil repeats:YES ];
}
////////////////////////////////////////////////////////////////
- ( void )viewDidDisappear:( BOOL )animated
{
    [ super viewDidDisappear:animated ];
    
    [ _labelTimer invalidate ];
    _labelTimer = nil;
    
    [ _capturePipeline stopRunning ];
}
////////////////////////////////////////////////////////////////
- ( void )updateLabels
{
    NSString *frameRateString = [ NSString stringWithFormat:@"%d FPS", ( int )roundf(  _capturePipeline.videoFrameRate  ) ];
    self.framerateLabel.text = frameRateString;
    
    NSString *dimensionsString = [ NSString stringWithFormat:@"%d x %d", _capturePipeline.videoDimensions.width, _capturePipeline.videoDimensions.height ];
    self.dimensionsLabel.text = dimensionsString;
    
    NSString *focal = [ NSString stringWithFormat:@"Focal: %4.0f", _focal ];
    self.focalLabel.text = focal;
   
    NSString *count = [ NSString stringWithFormat:@"#: %d", _storedImagesCount ];
    self.imageCountLabel.text = count;

    //[ self.imageView setImage:_frameFromCamera ];
}
////////////////////////////////////////////////////////////////
- ( void )showError:( NSError * )error
{
    UIAlertView *alertView = [ [ UIAlertView alloc ] initWithTitle:error.localizedDescription
                                                        message:error.localizedFailureReason
                                                       delegate:nil
                                              cancelButtonTitle:@"OK"
                                              otherButtonTitles:nil ];
    [ alertView show ];
}
/////////////////////////////////////////////////////////////////
-( UIImage * ) glToUIImage {
    int width = _panoImageW * 1.5; //fixme
    int height = _panoImageH * 1.5;
    
    NSInteger myDataLength = width * height * 4;
    GLubyte *buffer = (GLubyte *) malloc(myDataLength);
    GLubyte *buffer2 = (GLubyte *) malloc(myDataLength);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
    for(int y = 0; y <height; y++) {
        for(int x = 0; x <width * 4; x++) {
            buffer2[int((height - 1 - y) * width * 4 + x)] = buffer[int(y * 4 * width + x)];
        }
    }
    free(buffer);	// YOU CAN FREE THIS NOW
    
    CGDataProviderRef provider = CGDataProviderCreateWithData(NULL, buffer2, myDataLength, releaseData);
    int bitsPerComponent = 8;
    int bitsPerPixel = 32;
    int bytesPerRow = 4 * width;
    CGColorSpaceRef colorSpaceRef = CGColorSpaceCreateDeviceRGB();
    CGBitmapInfo bitmapInfo = kCGBitmapByteOrderDefault;
    CGColorRenderingIntent renderingIntent = kCGRenderingIntentDefault;
    CGImageRef imageRef = CGImageCreate(width, height, bitsPerComponent, bitsPerPixel, bytesPerRow, colorSpaceRef, bitmapInfo, provider, NULL, NO, renderingIntent);
    
    CGColorSpaceRelease(colorSpaceRef);
    CGDataProviderRelease(provider);
    
    UIImage *image = [[UIImage alloc] initWithCGImage:imageRef];
    CGImageRelease(imageRef);
    return image;
}

void releaseData( void *info, const void *data, size_t dataSize ) {
    free( ( void* ) data );
}
////////////////////////////////////////////////////////////////
std::vector<float> multiplyMatrix( std::vector<float> &a, std::vector<float>&b, int rows, int cols )
{
    std::vector<float> result( a.size( ) );
    
    for ( int i = 0; i < rows; i++ )
        for ( int j = 0; j < cols; j++ )
        {
            result[  i * cols + j  ] = 0;
            for ( int k = 0; k < cols; k++ )
                result[  i * cols + j  ] += a[  i * cols + k  ] * b[  k * rows +j  ];
        }
    
    return result;
}
////////////////////////////////////////////////////////////////
- ( UIInterfaceOrientationMask )supportedInterfaceOrientations
{
    return UIInterfaceOrientationMaskPortrait;
}
////////////////////////////////////////////////////////////////
- ( BOOL )prefersStatusBarHidden
{
    return YES;
}
////////////////////////////////////////////////////////////////
- ( void )dealloc
{
    [ self tearDownGL ];
    
    if ( [ EAGLContext currentContext ] == self.context ) {
        [ EAGLContext setCurrentContext:nil ];
    }
    
    _accelerometer.delegate = nil;
    
    if (  _addedObservers  )
    {
        [ [ NSNotificationCenter defaultCenter ] removeObserver:self name:UIApplicationDidEnterBackgroundNotification object:[ UIApplication sharedApplication ] ];
        [ [ NSNotificationCenter defaultCenter ] removeObserver:self name:UIApplicationWillEnterForegroundNotification object:[ UIApplication sharedApplication ] ];
        [ [ NSNotificationCenter defaultCenter ] removeObserver:self name:UIDeviceOrientationDidChangeNotification object:[ UIDevice currentDevice ] ];
        [ [ UIDevice currentDevice ] endGeneratingDeviceOrientationNotifications ];
    }
}
////////////////////////////////////////////////////////////////
- ( void )applicationDidEnterBackground
{
    // Avoid using the GPU in the background
    _allowedToUseGPU = NO;
    _capturePipeline.renderingEnabled = NO; 
    
    // We reset the OpenGLPixelBufferView to ensure all resources have been cleared when going to the background.
    //[ _previewView reset ]; //fixme2
}

- ( void )applicationWillEnterForeground
{
    _allowedToUseGPU = YES;
    _capturePipeline.renderingEnabled = YES;
}
////////////////////////////////////////////////////////////////
- ( void )didReceiveMemoryWarning
{
    NSLog( @"didRecieveMemoryWarning" );
    [ super didReceiveMemoryWarning ]; //fixme!
    
    if ( [ self isViewLoaded ] && ( [ [ self view ] window ] == nil ) ) {
        self.view = nil;
        
        [ self tearDownGL ];
        
        if ( [ EAGLContext currentContext ] == self.context ) {
            [ EAGLContext setCurrentContext:nil ];
        }
        self.context = nil;
    }
    
    // Dispose of any resources that can be recreated.
}

- ( void )updateLabel {
    [ self getMotionData ];
}
////////////////////////////////////////////////////////////////
- ( void )getMotionData {
    CMDeviceMotion *deviceMotion = self.motionManager.deviceMotion;
    
    CMAcceleration acceleration = self.motionManager.deviceMotion.userAcceleration;
    
    float xAccel = acceleration.x;
    if ( std::abs( xAccel ) < 0.01 )
        xAccel = 0;
    
     _xVelocity += xAccel * 1. / 30.;
     _xMotion += 0.5 * xAccel * ( 1. / 30. ) * ( 1. / 30. ) + _xVelocity * ( 1. / 30. );
     //_xMotion +=  _xVelocity * ( 1. / 30. );
     // printf( "%.4f, %.4f, %.3f, %.3f, %.3f \n", std::abs( _xMotion ), std::abs( _xVelocity ), ( xAccel ), std::abs( acceleration.y ), std::abs( acceleration.z ) );
    
    if( deviceMotion == nil )
        return;
    
    _currentAttitude = deviceMotion.attitude;
    _attitudeChange = _currentAttitude;
    
    if ( _refAttitude == nil )
        _refAttitude = [ _currentAttitude copy ];
    
    [ _attitudeChange multiplyByInverseOfAttitude:_refAttitude ];
    
    _roll = _attitudeChange.roll;
    _pitch = _attitudeChange.pitch;
    _yaw = _attitudeChange.yaw;
    
    /*
    _roll = -atan2( -quaternionChange.x*quaternionChange.z - quaternionChange.w*quaternionChange.y, .5 - quaternionChange.y*quaternionChange.y - quaternionChange.z*quaternionChange.z );
    _pitch = -asin( -2*( quaternionChange.y*quaternionChange.z + quaternionChange.w*quaternionChange.x ) );
    _yaw = -atan2( quaternionChange.x*quaternionChange.y - quaternionChange.w*quaternionChange.z, .5 - quaternionChange.x*quaternionChange.x - quaternionChange.z*quaternionChange.z );
    */
        
    CMQuaternion q = _attitudeChange.quaternion;
    GLKQuaternion quaternionChangeGLK = GLKQuaternionMake( q.x, q.y, -q.z, -q.w ); // "-" is added to set the rotation to its inverse, as needed for stitching
    
    //GLKQuaternion quaternionChangeGLK = GLKQuaternionMake( q.x, q.y, q.z, q.w );
    
    _rotationFromQuaternion = GLKMatrix4MakeWithQuaternion( quaternionChangeGLK );
    _rotationFromQuaternion3x3[ 0 ] = _rotationFromQuaternion.m[ 0 ];
    _rotationFromQuaternion3x3[ 1 ] = _rotationFromQuaternion.m[ 1 ];
    _rotationFromQuaternion3x3[ 2 ] = _rotationFromQuaternion.m[ 2 ];
    _rotationFromQuaternion3x3[ 3 ] = _rotationFromQuaternion.m[ 4 ];
    _rotationFromQuaternion3x3[ 4 ] = _rotationFromQuaternion.m[ 5 ];
    _rotationFromQuaternion3x3[ 5 ] = _rotationFromQuaternion.m[ 6 ];
    _rotationFromQuaternion3x3[ 6 ] = _rotationFromQuaternion.m[ 8 ];
    _rotationFromQuaternion3x3[ 7 ] = _rotationFromQuaternion.m[ 9 ];
    _rotationFromQuaternion3x3[ 8 ] = _rotationFromQuaternion.m[ 10 ];
    
    _vcRotationFromSensorArray[ 0 ] = _rotationFromQuaternion3x3;
}
////////////////////////////////////////////////////////////////
- ( BOOL )shouldAutorotateToInterfaceOrientation:( UIInterfaceOrientation )interfaceOrientation
{
    // Native video orientation is landscape with the button on the right.
    // The video processor rotates vide as needed, so don't autorotate also
    return ( interfaceOrientation == UIInterfaceOrientationLandscapeRight );
}
/////////////////////////////////////////////////////////////////
- ( void )deviceOrientationDidChange
{
    UIDeviceOrientation deviceOrientation = [ UIDevice currentDevice ].orientation;
    
    //if ( _storedImagesCount == 0 )
    {
        double rotation = 0;
        UIInterfaceOrientation statusBarOrientation;
        switch ( deviceOrientation ) {
            case UIDeviceOrientationFaceDown:
            case UIDeviceOrientationFaceUp:
            case UIDeviceOrientationUnknown:
                return;
            case UIDeviceOrientationPortrait:
                rotation = 0;
                statusBarOrientation = UIInterfaceOrientationPortrait;
                break;
            case UIDeviceOrientationPortraitUpsideDown:
                rotation = -M_PI;
                statusBarOrientation = UIInterfaceOrientationPortraitUpsideDown;
                break;
            case UIDeviceOrientationLandscapeLeft:
                rotation = M_PI_2;
                statusBarOrientation = UIInterfaceOrientationLandscapeRight;
                break;
            case UIDeviceOrientationLandscapeRight:
                rotation = -M_PI_2;
                statusBarOrientation = UIInterfaceOrientationLandscapeLeft;
                break;
        }
        
        CGAffineTransform transform = CGAffineTransformMakeRotation( rotation );
        [ UIView animateWithDuration:0.4 delay:0.0 options:UIViewAnimationOptionBeginFromCurrentState animations:^{
            [ self.startButton setTransform:transform ];
            [ self.viewButton setTransform:transform ];
            [ self.restartButton setTransform:transform ];
            [ self.refineButton setTransform:transform ];
            [ self.flagSwitch setTransform:transform ];
            [ self.flagLabel setTransform:transform ];
            [ self.captureScreenButton setTransform:transform ];
            //[ self.framerateLabel setTransform:transform ];
            //[ self.dimensionsLabel setTransform:transform ];
            //[ self.focalLabel setTransform:transform ];
            [ self.imageCountLabel setTransform:transform ];

            
            [ [ UIApplication sharedApplication ] setStatusBarOrientation:statusBarOrientation ];
        } completion:nil ];
    }
    if ( _storedImagesCount > 0 )
    {
        if ( deviceOrientation != _startDeviceOrientation )
        {
            if ( !_displayMode )
            {
                UIAlertView * alert = [ [ UIAlertView alloc ] initWithTitle:@"Warning!" message:@"Do not rotate device after starting capturing!" delegate:self cancelButtonTitle:@"Continue" otherButtonTitles:nil ];
                [ alert show ];
            }
        }
    }
}
/////////////////////////////////////////////////////////////////
- ( cv::Mat )cvMatFromUIImage:( UIImage * )image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace( image.CGImage );
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat( rows, cols, CV_8UC4 ); // 8 bits per component, 4 channels ( color channels + alpha )
    
    CGContextRef contextRef = CGBitmapContextCreate( cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[ 0 ],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault ); // Bitmap info flags
    
    CGContextDrawImage( contextRef, CGRectMake( 0, 0, cols, rows ), image.CGImage );
    CGContextRelease( contextRef );
    
    return cvMat;
}
//////////////////////////////////////////
- ( cv::Mat )cvMatGrayFromUIImage:( UIImage * )image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace( image.CGImage );
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat( rows, cols, CV_8UC1 ); // 8 bits per component, 1 channels
    
    CGContextRef contextRef = CGBitmapContextCreate( cvMat.data,                 // Pointer to data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[ 0 ],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault ); // Bitmap info flags
    
    CGContextDrawImage( contextRef, CGRectMake( 0, 0, cols, rows ), image.CGImage );
    CGContextRelease( contextRef );
    
    return cvMat;
}
//////////////////////////////////////////
-( UIImage * )UIImageFromCVMat:( cv::Mat )cvMat
{
    NSData *data = [ NSData dataWithBytes:cvMat.data length:cvMat.elemSize( )*cvMat.total( ) ];
    CGColorSpaceRef colorSpace;
    
    if ( cvMat.elemSize( ) == 1 ) {
        colorSpace = CGColorSpaceCreateDeviceGray( );
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB( );
    }
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData( ( __bridge CFDataRef )data );
    
    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate( cvMat.cols,                                 //width
                                        cvMat.rows,                                 //height
                                        8,                                          //bits per component
                                        8 * cvMat.elemSize( ),                       //bits per pixel
                                        cvMat.step[ 0 ],                            //bytesPerRow
                                        colorSpace,                                 //colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   //CGDataProviderRef
                                        NULL,                                       //decode
                                        false,                                      //should interpolate
                                        kCGRenderingIntentDefault                   //intent
                                         );
    
    
    // Getting UIImage from CGImage
    UIImage *finalImage = [ UIImage imageWithCGImage:imageRef ];
    CGImageRelease( imageRef );
    CGDataProviderRelease( provider );
    CGColorSpaceRelease( colorSpace );
    
    return finalImage;
}
//////////////////////////////////////////
- (void) checkIfShoudStoreFrame
{
    float minRollDiff = 10;
    float minPitchDiff = 10;
    float minYawDiff = 10;
    float minDist = 10;
    float dist;
    std::vector<float> distFromFrames( _storedImagesCount + 1 );

    for ( int i = 1; i <= _storedImagesCount; i++ )
    {
        minRollDiff = fmin( minRollDiff, fabsf( _roll - _rollArray[ i ] ) );
        minPitchDiff = fmin( minPitchDiff, fabsf( _pitch - _pitchArray[ i ] ) );
        minYawDiff = fmin( minYawDiff, fabsf( _yaw - _yawArray[ i ] ) );
        dist = fmax( fabsf( _roll - _rollArray[ i ] ) / fabsf( atan( _cvImageW / _focal ) ), fabsf( _pitch - _pitchArray[ i ] )  / fabsf( atan( _cvImageH / _focal ) ) );
        minDist = fmin( minDist, dist );
        distFromFrames[ i ] = dist;
    }

    if ( !_displayMode )
    {
        // Check if a new frame should be stored in memory
        if ( minDist > 0.5 ) // 50% overlap either is x or y direction
        {
            _cvSaveNextFlag = true;
            _panoUploadNextFlag = true;
            _storedImagesCount++;
            _rollArray[ _storedImagesCount ] = _roll;
            _pitchArray[ _storedImagesCount ] = _pitch;
            _yawArray[ _storedImagesCount ] = _yaw;
            AudioServicesPlaySystemSound( 1108 );
            
            // Sort distances and find closest frame to the frame under process
            std::vector<int> ind( distFromFrames.size( ) );
            std::iota( ind.begin( ), ind.end( ), 0 ); //fixme, check if for consistency the index should start from 0 or 1
            auto comparator = [ &distFromFrames ]( int a, int b ){ return distFromFrames[ a ] < distFromFrames[ b ]; };
            std::sort( ind.begin( ), ind.end( ), comparator );
            //std::sort( distFromFrames.begin( ), distFromFrames.end( ), comparator );
            
            for ( int i = 1; i < ind.size( ); i++ )
            {
                // Mark if there is almost no overlap
                if ( distFromFrames[ ind[ i ] ] > 1.0 )
                    ind[ i ] = -1;
            }
            
            if ( _storedImagesCount >= _closestFrames.size( ) )
                _closestFrames.resize( _storedImagesCount + 1 );
            
            _closestFrames[ _storedImagesCount ] = ind;
        }
    }
}
//////////////////////////////////////////
- ( std::vector<float> ) getDisplayRotation
{
    std::vector<float> displayRotationMixed;
    std::vector<float> displayRotation, displayRotation2;
    displayRotation.resize( 9 );
    displayRotation2.resize( 9 );

    cv::Mat tempMat( 3, 3, CV_32FC1 );

    for ( int j = 0; j < 9; j++ )
    tempMat.at<float>(  floor(  j / 3  ), j % 3  ) = _rotationFromQuaternion3x3[  j  ];
    tempMat = tempMat.inv( );
    for ( int j = 0; j < 9; j++ )
    displayRotation[  j  ] = tempMat.at<float>(  floor(  j / 3  ), j % 3  );


    float displayRoll = _userPanX;
    float displayPitch = _userPanY;
    float displayYaw = 0;

    displayRotation2[ 0 ] = cos( -displayRoll ) * cos( -displayYaw ) - sin( -displayRoll ) * sin( displayPitch ) * sin( -displayYaw );
    displayRotation2[ 1 ] = -cos( -displayRoll ) * sin( -displayYaw ) - sin( -displayRoll ) * sin( displayPitch ) * cos( -displayYaw );
    displayRotation2[ 2 ] = -sin( -displayRoll ) * cos( displayPitch );
    displayRotation2[ 3 ] =  cos( displayPitch ) * sin( -displayYaw );
    displayRotation2[ 4 ] =  cos( displayPitch ) * cos( -displayYaw );
    displayRotation2[ 5 ] =  -sin( displayPitch );
    displayRotation2[ 6 ] =  sin( -displayRoll ) * cos( -displayYaw ) + cos( -displayRoll ) * sin( displayPitch ) * sin( -displayYaw );
    displayRotation2[ 7 ] =  -sin( -displayRoll ) * sin( -displayYaw ) + cos( -displayRoll ) * sin( displayPitch ) * cos( -displayYaw );
    displayRotation2[ 8 ] =  cos( -displayRoll ) * cos( displayPitch );

        displayRotationMixed.resize( 9 );
    int rows = 3;
    int cols = 3;

    // Multiply matrices
    for ( int i = 0; i < rows; i++ )
    for ( int j = 0; j < cols; j++ )
    {
        displayRotationMixed[  i * cols + j  ] = 0;
        for ( int k = 0; k < cols; k++ )
            displayRotationMixed[  i * cols + j  ] += displayRotation[  i * cols + k  ] * displayRotation2[  k * rows +j  ];
    }
    return displayRotationMixed;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma mark - VideoReaderCapturePipelineDelegate
- ( void )capturePipeline:( VideoReaderCapturePipeline * )capturePipeline didStopRunningWithError:( NSError * )error
{
    [ self showError:error ];
}
////////////////////////////////////////////////////////////////
- ( void )capturePipeline:( VideoReaderCapturePipeline * )capturePipeline previewPixelBufferReadyForDisplay:( CVPixelBufferRef )previewPixelBuffer
{
    // Create a CVOpenGLESTexture from a CVPixelBufferRef
    size_t frameWidth = CVPixelBufferGetWidth(  previewPixelBuffer  );
    size_t frameHeight = CVPixelBufferGetHeight(  previewPixelBuffer  );
    CVOpenGLESTextureRef texture = NULL;
    CVReturn err = CVOpenGLESTextureCacheCreateTextureFromImage(  kCFAllocatorDefault,
                                                                _textureCache,
                                                                previewPixelBuffer,
                                                                NULL,
                                                                GL_TEXTURE_2D,
                                                                GL_RGBA,
                                                                ( GLsizei )frameWidth,
                                                                ( GLsizei )frameHeight,
                                                                GL_BGRA,
                                                                GL_UNSIGNED_BYTE,
                                                                0,
                                                                &_textureFromCamera  );
    
    if (  ! _textureFromCamera || err  ) {
        NSLog(  @"CVOpenGLESTextureCacheCreateTextureFromImage failed ( error: %d )", err  );
        return;
    }
    
    CGSize size = [ _frameFromCamera size ]; //fixme!!!! is this the size that should be passed to setTexture?
    
    size.width = _panoImageW;
    size.height = _panoImageH;
    
    if ( _textureFromCamera ) {
        pano_setTexture( _panoContext, CVOpenGLESTextureGetName(  _textureFromCamera  ), size.width, size.height, _panoUploadNextFlag, _displayMode ); //// fixme
        
        CFRelease( _textureFromCamera );
        _textureFromCamera = NULL;
        
        // Delete the texture corresponding to the previous camera frame
        glDeleteTextures( 1, &_oldTexture );
        _oldTexture = 0;
        
        if ( 0 )//_panoUploadNextFlag ) //fixme
        {
            CVPixelBufferLockBaseAddress(  previewPixelBuffer, 0  ); // shared memory, lock to make sure no one else uses the data
            
            unsigned char *base = ( unsigned char * )CVPixelBufferGetBaseAddress(  previewPixelBuffer  );
            size_t width = CVPixelBufferGetWidth(  previewPixelBuffer  );
            size_t height = CVPixelBufferGetHeight(  previewPixelBuffer  );
            size_t stride = CVPixelBufferGetBytesPerRow(  previewPixelBuffer  );
            size_t extendedWidth = stride / sizeof(  uint32_t  ); // each pixel is 4 bytes/32 bits
            
            // Since the OpenCV Mat is wrapping the CVPixelBuffer's pixel data, we must do all of our modifications while its base address is locked.
            // If we want to operate on the buffer later, we'll have to do an expensive deep copy of the pixel data, using memcpy or Mat::clone( ).
            
            // Use extendedWidth instead of width to account for possible row extensions ( sometimes used for memory alignment ).
            // We only need to work on columms from [ 0, width - 1 ] regardless.
            
            cv::Mat mat = cv::Mat(  ( int )height, ( int )extendedWidth, CV_8UC4, base  ); // This is very fast since no data is copied
            cvtColor( mat, mat, CV_BGRA2RGB );
            flip( mat, mat, 0 );
            
            CVPixelBufferUnlockBaseAddress(  previewPixelBuffer, 0  );
            
            // Save captured frame (image) on device
            UIImage* image = [ self UIImageFromCVMat:mat ];
            UIImageWriteToSavedPhotosAlbum( image, nil, nil, nil );
        }
        _panoUploadNextFlag = false;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma mark - UIAccelerometerDelegate Methods

- ( void )accelerometer:( UIAccelerometer * )meter
        didAccelerate:( UIAcceleration * )acceleration
{
    /*
    _xVelocity += acceleration.x * 1. / 20.;
    //_xMotion += 0.5 * acceleration.x * ( 1. / 60. ) * ( 1. / 60. ) + _xVelocity * ( 1. / 60. );
    _xMotion +=  _xVelocity * ( 1. / 20. );
    printf( "%.4f, %.3f, %.3f, %.3f \n", _xMotion, acceleration.x, acceleration.y, acceleration.z );
    */
    
    
    //std::cout << acceleration.x << std::endl;
    /*
    xLabel.text = [ NSString stringWithFormat:@"%f", acceleration.x ];
    xBar.progress = ABS( acceleration.x );
    
    yLabel.text = [ NSString stringWithFormat:@"%f", acceleration.y ];
    yBar.progress = ABS( acceleration.y );
    
    zLabel.text = [ NSString stringWithFormat:@"%f", acceleration.z ];
    zBar.progress = ABS( acceleration.z );
     */
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma mark - GLKView and GLKViewController delegate methods

- ( BOOL )compileShader:( GLuint * )shader type:( GLenum )type file:( NSString * )file
{
    return YES;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

- ( NSError* ) unlockCamera{
    AVCaptureDevice *videoDevice = [ AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo ];
    
    NSError *error2;
    [ videoDevice lockForConfiguration:&error2 ];
    if ( [ videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance ] ) {
        videoDevice.whiteBalanceMode = AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance;
        NSLog( @"Whitebalanced unlocked" );
    }
    if ( [ videoDevice isExposureModeSupported:AVCaptureExposureModeContinuousAutoExposure ] ) {
        videoDevice.exposureMode = AVCaptureExposureModeContinuousAutoExposure;
        NSLog( @"Exposure unlocked" );
    }
    if ( [ videoDevice isFocusModeSupported:AVCaptureFocusModeAutoFocus ] ) {
        videoDevice.focusMode = AVCaptureFocusModeAutoFocus;
        NSLog( @"Focus unlocked" );
    }
    [ videoDevice unlockForConfiguration ];
    
    return error2;
}

- ( NSError* ) lockCamera{
    AVCaptureDevice *videoDevice = [ AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo ];
    
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
    if ( [ videoDevice isFocusModeSupported:AVCaptureFocusModeLocked ] ) {
        videoDevice.focusMode = AVCaptureFocusModeLocked;
        NSLog( @"Focus locked" );
    }
    [ videoDevice unlockForConfiguration ];
    
    return error2;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


- ( IBAction )cvProcessFrames:( id )sender {
    _cvProcessFramesFlag = !_cvProcessFramesFlag;
    if ( _cvProcessFramesFlag )
        [ self.flagLabel setText:[ NSString stringWithFormat:@"On" ] ];
    else
        [ self.flagLabel setText:[ NSString stringWithFormat:@"Off" ] ];
}

- ( IBAction )warpToSphere:( id )sender {
    pano_changeWarpMode( _panoContext );
}

- ( IBAction )startCapturing:( id )sender {
    _cvSaveNextFlag = true;
    _panoUploadNextFlag = true;
    _storedImagesCount++;
    _rollArray[ _storedImagesCount ] = _roll;
    _pitchArray[ _storedImagesCount ] = _pitch;
    _yawArray[ _storedImagesCount ] = _yaw;
    _displayMode = false;
    AudioServicesPlaySystemSound( 1108 );
    
    [ self lockCamera ];
    
    _startDeviceOrientation = [ UIDevice currentDevice ].orientation;
}

- ( IBAction )stopCapturing:( id )sender {
    [ self unlockCamera ];
    [ self initialize ];
    
    pano_restart( _panoContext );
    pano_setFocalLength( _panoContext, _focal );
    
    cvEstimator_restart( _cvEstimator );
    cvEstimator_setFocalLength( _cvEstimator, _focal );

}

- ( IBAction )displayResult:( id )sender {
    _displayMode = !_displayMode;
}

- ( IBAction )startRefinedStitching:( id )sender {
    /*
    UIAlertView * alert = [ [ UIAlertView alloc ] initWithTitle:@"Wait!" message:@"Under construction yet!" delegate:self cancelButtonTitle:@"Continue" otherButtonTitles:nil ];
    [ alert show ];
    */
    
    if ( _storedImagesCount > 2 )
    {
        /*
        UIAlertView * alert = [ [ UIAlertView alloc ] initWithTitle:@"Wait!" message:@"Refining results ..." delegate:self cancelButtonTitle:@"" otherButtonTitles:nil ];
        [ alert show ];
        [ alert performSelector:@selector( dismissWithClickedButtonIndex:animated: ) withObject:[ NSNumber numberWithInt:0 ] afterDelay:2 ];
         */
        //std::vector<std::vector <float>> currentRotationUsedToInitBA = pano_getCurrentRotations( _panoContext ); // roatation strored in viewcontroller is more reliable than this
        
        
        //_refinedRotations = cvEstimator_refinedStitching( _cvEstimator, _vcRotationFromSensorArray, _closestFrames );
        //_focal = 0.5 * _focal + 0.5 * cvEstimator_getFocalLength( _cvEstimator );
        _focal = 1117;//cvEstimator_getFocalLength( _cvEstimator );
        std::cout << " ---- set focal: " << cvEstimator_getFocalLength( _cvEstimator ) << std::endl;
        pano_setRefinedRotations( _panoContext, _refinedRotations, _focal );
    }
    else
    {
        UIAlertView * alert = [ [ UIAlertView alloc ] initWithTitle:@"Error!" message:@"Capture at least 3 frames before refinement" delegate:self cancelButtonTitle:@"Continue" otherButtonTitles:nil ];
        [ alert show ];
    }
    
}

- (IBAction)captureScreen:(id)sender {
    _captureScreen = true;
    AudioServicesPlaySystemSound( 1109 );
}

- ( IBAction )panRecognizer:( UIPanGestureRecognizer * )sender {
    int x = 1;
    x = x + 2;
    CGPoint ss = [ sender translationInView:self.view ];
    
    _userPanX -= ss.x / 1000 * _userZoom;
    _userPanY -= ss.y / 2000 * _userZoom;
    
    //NSLog( @"%f, %f", _userPanX, _userPanY );
}

- (IBAction)pinchRecognizer:( UIPinchGestureRecognizer * )sender {
    CGFloat scale = [ sender scale ];
    
    // _userZoom = fmin( fmax ( 0.5 * _userZoom + 0.5 * 1 / scale, 0.2 ), 1.5 );
    
    _userZoom /= pow( scale, 0.2 );
    
    _userZoom = fmin( fmax (  _userZoom, 0.2 ), 1.0 );
    
    //NSLog( @"%f", scale );
}

@end
