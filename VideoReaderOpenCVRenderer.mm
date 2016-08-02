
/*
 Copyright ( C ) 2016 Apple Inc. All Rights Reserved.
 See LICENSE.txt for this sample’s licensing information
 
 Abstract:
 The VideoReader OpenCV based effect renderer
 */

#import "VideoReaderOpenCVRenderer.h"

// To build OpenCV into the project:
//	- Download opencv2.framework for iOS
//	- Insert framework into project's Frameworks group
//	- Make sure framework is included under the target's Build Phases -> Link Binary With Libraries.
#import <opencv2/opencv.hpp>

@interface VideoReaderOpenCVRenderer ( ) {
   
    cv::Mat _resizedMat;
    cv::Mat _resizedMatGray;
    UIImage *_uiImage;
    
    
}

-( UIImage * )UIImageFromCVMat:( cv::Mat )cvMat;

 @end
    

@implementation VideoReaderOpenCVRenderer

-( cv::Mat ) getMatImage
{
    return _resizedMatGray;
}

#pragma mark VideoReaderRenderer

-( UIImage * ) getUIImage
{
    return _uiImage;
}

- ( BOOL )operatesInPlace
{
	return YES;
}

- ( FourCharCode )inputPixelFormat
{
	return kCVPixelFormatType_32BGRA;
}

- ( void )prepareForInputWithFormatDescription:( CMFormatDescriptionRef )inputFormatDescription outputRetainedBufferCountHint:( size_t )outputRetainedBufferCountHint
{
	// nothing to do, we are stateless
}

- ( void )reset
{
	// nothing to do, we are stateless
}

- ( CVPixelBufferRef )copyRenderedPixelBuffer:( CVPixelBufferRef )pixelBuffer
{
	CVPixelBufferLockBaseAddress(  pixelBuffer, 0  ); // shared memory, lock to make sure no one else uses the data
	
	unsigned char *base = ( unsigned char * )CVPixelBufferGetBaseAddress(  pixelBuffer  );
	size_t width = CVPixelBufferGetWidth(  pixelBuffer  );
	size_t height = CVPixelBufferGetHeight(  pixelBuffer  );
	size_t stride = CVPixelBufferGetBytesPerRow(  pixelBuffer  );
	size_t extendedWidth = stride / sizeof(  uint32_t  ); // each pixel is 4 bytes/32 bits
	
	// Since the OpenCV Mat is wrapping the CVPixelBuffer's pixel data, we must do all of our modifications while its base address is locked.
	// If we want to operate on the buffer later, we'll have to do an expensive deep copy of the pixel data, using memcpy or Mat::clone( ).
	
	// Use extendedWidth instead of width to account for possible row extensions ( sometimes used for memory alignment ).
	// We only need to work on columms from [ 0, width - 1 ] regardless.
	
	_resizedMat = cv::Mat(  ( int )height, ( int )extendedWidth, CV_8UC4, base  ); // This is very fast since no data is copied
    
    /*
    _resizedMatGray = cv::Mat( ( int )height, ( int )width, CV_8UC4 ); //fix this, there is sized problem if I directly use the one with extendedwidth
    
	//fixme, directly only create grayscale image
	for (  uint32_t y = 0; y < height / 2; y++  )
	{
		for (  uint32_t x = 0; x < width / 2; x++  )
		{
            _resizedMatGray.at<cv::Vec<uint8_t,4> >( y,x )[ 1 ] = _resizedMat.at<cv::Vec<uint8_t,4> >( y,x )[ 1 ];
            _resizedMatGray.at<cv::Vec<uint8_t,4> >( y,x )[ 2 ] = _resizedMat.at<cv::Vec<uint8_t,4> >( y,x )[ 2 ];
            _resizedMatGray.at<cv::Vec<uint8_t,4> >( y,x )[ 3 ] = _resizedMat.at<cv::Vec<uint8_t,4> >( y,x )[ 3 ];
            _resizedMatGray.at<cv::Vec<uint8_t,4> >( y,x )[ 4 ] = _resizedMat.at<cv::Vec<uint8_t,4> >( y,x )[ 4 ];
		}
	}
	*/
    
    _resizedMatGray = _resizedMat(  cvRect(  0, 0, width, height  )  );
    
    //std::cout << _resizedMat.type( ) <<std::endl;
    
    cvtColor( _resizedMatGray, _resizedMatGray, CV_BGRA2RGB );
    //_resizedMat.copyTo( _resizedMatGray );
    
    _uiImage = [ self UIImageFromCVMat:_resizedMatGray ];
    //std::cout << _resizedMatGray.type( ) <<std::endl;
    
	CVPixelBufferUnlockBaseAddress(  pixelBuffer, 0  );
	
	return ( CVPixelBufferRef )CFRetain(  pixelBuffer  );
    ////return (  pixelBuffer  );
}


- ( CVPixelBufferRef )copyRenderedPixelBufferToMat:( CVPixelBufferRef )pixelBuffer
{
    
    return ( CVPixelBufferRef )CFRetain(  pixelBuffer  );
}

- ( CVPixelBufferRef )copyRenderedPixelBufferWithResize:( CVPixelBufferRef )pixelBuffer
{

    return (  pixelBuffer  );
}

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


@end
