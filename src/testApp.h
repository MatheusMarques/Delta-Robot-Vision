#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxXmlSettings.h"

class testApp : public ofBaseApp {
public:
    
    void setup();
    void update();
    void draw();
    void exit();
	
    void drawPointCloud();
    
    void keyPressed  (int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofxKinect kinect;
    
    ofxCvColorImage		colorImg;
    
    ofxCvGrayscaleImage 	grayImage;
    ofxCvGrayscaleImage 	grayThresh;
    ofxCvGrayscaleImage 	grayThreshFar;
    
    ofxCvContourFinder 	contourFinder;
    
    bool				bThreshWithOpenCV;
    bool				drawPC;
    
    int 				nearThreshold;
    int					farThreshold;
    
    int					angle;
    
    int 				pointCloudRotationY;
    
    ofPoint viewportOrigin;
    ofPoint viewportEnd;
    
    void drawGrid(int spacing, int lines, int zDepth);
    ofPoint normalizeOfPoint(float x, float w, float y, float h, float z, float d);
    ofPoint mapPointTo(ofPoint p, float _mx, float _my);
    ofColor HSVToRGB(float H, float S, float V, ofColor &in);
    
    ofxOscSender sender;

    bool bDumpXML;
    ofxXmlSettings xml;
    
};
