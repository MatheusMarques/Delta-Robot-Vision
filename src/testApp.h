#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxMultiTouchPad.h"

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
        
        
        
        vector<MTouch> touches;
        vector<ofPoint> pois;

		bool	bThreshWithOpenCV;
		bool	drawPC;

		int 	nearThreshold;
		int		farThreshold;

		int		angle;
        int     pMouseX, pMouseY;
    
		int 	pointCloudRotationY;
        bool    calibrate;
        bool    showReport;
        bool    povLock;
};
