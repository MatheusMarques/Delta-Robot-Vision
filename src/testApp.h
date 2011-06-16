#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxMultiTouchPad.h"
#include "depthMath.h"
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
        ofxCvContourFinder 	contourFinder;

		ofxCvColorImage         colorImg;
		ofxCvGrayscaleImage 	grayImage;
		ofxCvGrayscaleImage 	grayThresh;
		ofxCvGrayscaleImage 	grayThreshFar;
    

        ofxMultiTouchPad pad;
        void padUpdates(int & t);
        void newTouch(int & n);
        void removedTouch(int & r);
        
    ofPoint getDepthAsOfPoint(float _x, float _y, float zDepth);
        
    
        vector<MTouch> touches;
        vector<ofPoint> pois;
        vector<ofPoint> points;

		bool	bThreshWithOpenCV;
		bool	drawPC;

		int 	nearThreshold;
		int		farThreshold;

		int		angle;
        int     pMouseX, pMouseY;
    
		int 	pointCloudRotationY, pointCloudRotationX;
        bool    calibrate;
        bool    showReport;
        bool    povLock;
        ofColor HSVToRGB(float H, float S, float V, ofColor &in);
    
    depthMath depth;
        
};
