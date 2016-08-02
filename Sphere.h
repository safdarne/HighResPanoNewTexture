//
//  Sphere.h
//  sharpenOGL
//
//  Created by safdarne on 6/10/16.
//  Copyright © 2016 gaoyuan. All rights reserved.
//

#ifndef Sphere_h
#define Sphere_h

#import <GLKit/GLKit.h>
#import <CoreMotion/CoreMotion.h>
#import <OpenGLES/ES1/gl.h>


#define FPS 60
#define FOV_MIN 1
#define FOV_MAX 155
#define Z_NEAR 0.1f
#define Z_FAR 100.0f


@interface Sphere : NSObject

    -(bool) execute;
    -(id) init:(GLint)stacks slices:(GLint)slices radius:(GLfloat)radius textureFile:(NSString *)textureFile;
    -(void) swapTexture:(NSString*)textureFile;
    -(void) swapTextureWithImage:(UIImage*)image;
    -(CGPoint) getTextureSize;


    GLKTextureInfo *m_TextureInfo;
    GLfloat *m_TexCoordsData;
    GLfloat *m_VertexData;
    GLfloat *m_NormalData;
    GLint m_Stacks, m_Slices;
    GLfloat m_Scale;
}
-(GLKTextureInfo *) loadTextureFromBundle:(NSString *) filename;
-(GLKTextureInfo *) loadTextureFromPath:(NSString *) path;
-(GLKTextureInfo *) loadTextureFromImage:(UIImage *) image;
@end





@implementation Sphere
-(id) init:(GLint)stacks slices:(GLint)slices radius:(GLfloat)radius textureFile:(NSString *)textureFile{
    // modifications:
    //   flipped(inverted) texture coords across the Z
    //   vertices rotated 90deg
    if(textureFile != nil) m_TextureInfo = [self loadTextureFromBundle:textureFile];
    m_Scale = radius;
    if((self = [super init])){
        m_Stacks = stacks;
        m_Slices = slices;
        m_VertexData = nil;
        m_TexCoordsData = nil;
        // Vertices
        GLfloat *vPtr = m_VertexData = (GLfloat*)malloc(sizeof(GLfloat) * 3 * ((m_Slices*2+2) * (m_Stacks)));
        // Normals
        GLfloat *nPtr = m_NormalData = (GLfloat*)malloc(sizeof(GLfloat) * 3 * ((m_Slices*2+2) * (m_Stacks)));
        GLfloat *tPtr = nil;
        tPtr = m_TexCoordsData = (GLfloat*)malloc(sizeof(GLfloat) * 2 * ((m_Slices*2+2) * (m_Stacks)));
        unsigned int phiIdx, thetaIdx;
        // Latitude
        for(phiIdx = 0; phiIdx < m_Stacks; phiIdx++){
            //starts at -pi/2 goes to pi/2
            //the first circle
            float phi0 = M_PI * ((float)(phiIdx+0) * (1.0/(float)(m_Stacks)) - 0.5);
            //second one
            float phi1 = M_PI * ((float)(phiIdx+1) * (1.0/(float)(m_Stacks)) - 0.5);
            float cosPhi0 = cos(phi0);
            float sinPhi0 = sin(phi0);
            float cosPhi1 = cos(phi1);
            float sinPhi1 = sin(phi1);
            float cosTheta, sinTheta;
            //longitude
            for(thetaIdx = 0; thetaIdx < m_Slices; thetaIdx++){
                float theta = -2.0*M_PI * ((float)thetaIdx) * (1.0/(float)(m_Slices - 1));
                cosTheta = cos(theta+M_PI*.5);
                sinTheta = sin(theta+M_PI*.5);
                //get x-y-x of the first vertex of stack
                vPtr[0] = m_Scale*cosPhi0 * cosTheta;
                vPtr[1] = m_Scale*sinPhi0;
                vPtr[2] = m_Scale*(cosPhi0 * sinTheta);
                //the same but for the vertex immediately above the previous one.
                vPtr[3] = m_Scale*cosPhi1 * cosTheta;
                vPtr[4] = m_Scale*sinPhi1;
                vPtr[5] = m_Scale*(cosPhi1 * sinTheta);
                nPtr[0] = cosPhi0 * cosTheta;
                nPtr[1] = sinPhi0;
                nPtr[2] = cosPhi0 * sinTheta;
                nPtr[3] = cosPhi1 * cosTheta;
                nPtr[4] = sinPhi1;
                nPtr[5] = cosPhi1 * sinTheta;
                if(tPtr!=nil){
                    GLfloat texX = (float)thetaIdx * (1.0f/(float)(m_Slices-1));
                    tPtr[0] = 1.0-texX;
                    tPtr[1] = (float)(phiIdx + 0) * (1.0f/(float)(m_Stacks));
                    tPtr[2] = 1.0-texX;
                    tPtr[3] = (float)(phiIdx + 1) * (1.0f/(float)(m_Stacks));
                }
                vPtr += 2*3;
                nPtr += 2*3;
                if(tPtr != nil) tPtr += 2*2;
            }
            //Degenerate triangle to connect stacks and maintain winding order
            vPtr[0] = vPtr[3] = vPtr[-3];
            vPtr[1] = vPtr[4] = vPtr[-2];
            vPtr[2] = vPtr[5] = vPtr[-1];
            nPtr[0] = nPtr[3] = nPtr[-3];
            nPtr[1] = nPtr[4] = nPtr[-2];
            nPtr[2] = nPtr[5] = nPtr[-1];
            if(tPtr != nil){
                tPtr[0] = tPtr[2] = tPtr[-2];
                tPtr[1] = tPtr[3] = tPtr[-1];
            }
        }
    }
    return self;
}
-(void) dealloc{
    GLuint name = m_TextureInfo.name;
    glDeleteTextures(1, &name);
    
    if(m_TexCoordsData != nil){
        free(m_TexCoordsData);
    }
    if(m_NormalData != nil){
        free(m_NormalData);
    }
    if(m_VertexData != nil){
        free(m_VertexData);
    }
}
-(bool) execute{
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    if(m_TexCoordsData != nil){
        glEnable(GL_TEXTURE_2D);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        if(m_TextureInfo != 0)
            glBindTexture(GL_TEXTURE_2D, m_TextureInfo.name);
        glTexCoordPointer(2, GL_FLOAT, 0, m_TexCoordsData);
    }
    glVertexPointer(3, GL_FLOAT, 0, m_VertexData);
    glNormalPointer(GL_FLOAT, 0, m_NormalData);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, (m_Slices +1) * 2 * (m_Stacks-1)+2);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    return true;
}
-(GLKTextureInfo *) loadTextureFromBundle:(NSString *) filename{
    if(!filename) return nil;
    NSString *path = [[NSBundle mainBundle] pathForResource:filename ofType:NULL];
    return [self loadTextureFromPath:path];
}
-(GLKTextureInfo *) loadTextureFromPath:(NSString *) path{
    if(!path) return nil;
    NSError *error;
    GLKTextureInfo *info;
    NSDictionary *options = [NSDictionary dictionaryWithObjectsAndKeys:[NSNumber numberWithBool:YES], GLKTextureLoaderOriginBottomLeft, nil];
    info=[GLKTextureLoader textureWithContentsOfFile:path options:options error:&error];
    glBindTexture(GL_TEXTURE_2D, info.name);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, IMAGE_SCALING);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, IMAGE_SCALING);
    return info;
}
-(GLKTextureInfo *) loadTextureFromImage:(UIImage *) image {
    if(!image) return nil;
    NSError *error;
    GLKTextureInfo *info;
    NSDictionary *options = [NSDictionary dictionaryWithObjectsAndKeys:[NSNumber numberWithBool:YES],GLKTextureLoaderOriginBottomLeft, nil];
    info = [GLKTextureLoader textureWithCGImage:image.CGImage options:options error:&error];
    glBindTexture(GL_TEXTURE_2D, info.name);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, IMAGE_SCALING);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, IMAGE_SCALING);
    return info;
}
-(void)swapTexture:(NSString*)textureFile{
    GLuint name = m_TextureInfo.name;
    glDeleteTextures(1, &name);
    if ([[NSFileManager defaultManager] fileExistsAtPath:textureFile]) {
        m_TextureInfo = [self loadTextureFromPath:textureFile];
    }
    else {
        m_TextureInfo = [self loadTextureFromBundle:textureFile];
    }
}
-(void)swapTextureWithImage:(UIImage*)image {
    GLuint name = m_TextureInfo.name;
    glDeleteTextures(1, &name);
    m_TextureInfo = [self loadTextureFromImage:image];
}
-(CGPoint)getTextureSize{
    if(m_TextureInfo){
        return CGPointMake(m_TextureInfo.width, m_TextureInfo.height);
    }
    else{
        return CGPointZero;
    }
}

@end

#endif /* Sphere_h */
