#include "testApp.h"

void testApp::setup() {
	//kinect.init(true);  //shows infrared image
	kinect.init();
	kinect.setVerbose(true);
	kinect.open();

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThresh.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold  = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);

	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	pointCloudRotationY = 180;
	
	drawPC = true;
    calibrate = false;
    povLock = false;

    
    ofAddListener(pad.update, this, &testApp::padUpdates);
    ofAddListener(pad.touchAdded, this, &testApp::newTouch);
    ofAddListener(pad.touchRemoved, this, &testApp::removedTouch);
}

void testApp::update() {
	ofBackground(100, 100, 100);
	
	kinect.update();
	if(kinect.isFrameNew())	// there is a new frame and we are connected
	{

		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
			
		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThresh.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}else{
		
			unsigned char * pix = grayImage.getPixels();
			int numPixels = grayImage.getWidth() * grayImage.getHeight();

			for(int i = 0; i < numPixels; i++){
				if( pix[i] < nearThreshold && pix[i] > farThreshold ){
					pix[i] = 255;
				}else{
					pix[i] = 0;
				}
			}
		}

		grayImage.flagImageChanged();
    	contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
}

void testApp::draw() {
	ofSetColor(255, 255, 255);
    
    kinect.draw(0, 0, ofGetWidth(), ofGetHeight());

    
    
	if(drawPC){
		ofPushMatrix();
		ofTranslate(420, 320);
            drawPointCloud();
		ofPopMatrix();
	}
    
    if(calibrate){
        kinect.drawDepth(10, 10, 400, 300);
        kinect.draw(420, 10, 400, 300);

		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
	}
	

    
    if(showReport){
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
								 << ofToString(kinect.getMksAccel().y, 2) << " / " 
								 << ofToString(kinect.getMksAccel().z, 2) << endl
				 << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
				 << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
				 << "set near threshold " << nearThreshold << " (press: + -)" << endl
				 << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
				 	<< ", fps: " << ofGetFrameRate() << endl
				 << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
				 << "press UP and DOWN to change the tilt angle: " << angle << " degrees";
	ofDrawBitmapString(reportStream.str(),20,666);
    }
}

void testApp::drawPointCloud() {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect.getDistancePixels();
    
    glPushMatrix();
	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect.getWorldCoordinateFor(x,y);
			ofColor color = kinect.getCalibratedColorAt(x,y);
			glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
			glVertex3f(cur.x, cur.y, cur.z);
		}
	}
	glEnd();
    glPopMatrix();
}

void testApp::exit() {
	kinect.setCameraTiltAngle(0);
	kinect.close();
}

void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
//			bThreshWithOpenCV = !bThreshWithOpenCV;
            povLock = !povLock;
            break;
		case'p':
			drawPC = !drawPC;
            calibrate = !calibrate;
            break;            
            
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
        case 'r':
            showReport = !showReport;
            break;
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
		case 'o':
			kinect.setCameraTiltAngle(angle);	// go back to prev tilt
			kinect.open();
			break;
		case 'c':
			kinect.setCameraTiltAngle(0);		// zero the tilt
			kinect.close();
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

void testApp::mouseMoved(int x, int y) {
	if (!povLock){
        pointCloudRotationY = x;
    }
}

void testApp::mouseDragged(int x, int y, int button)
{}

void testApp::mousePressed(int x, int y, int button)
{

    
    // clean mouse values - get pixel point on kinect.draw() frame
    // normalize and convert to ofPoint
    // send to Zak
    
    float normX = ofNormalize(x, 0, ofGetWidth());
    float normY = ofNormalize(y, 0, ofGetHeight());
    float zDepth = kinect.getDistanceAt(ofPoint(ofMap(normX, 0, 1, 0, 640), ofMap(normY, 0, 1, 0, 480)));
    float normZ = ofNormalize(zDepth, 0, 400);
    
    ofPoint tp;
    tp.set(normX, normY,normZ);
    pois.push_back(tp);
    cout << "Distance at " << x << " : " << y << " is " << kinect.getDistanceAt(ofPoint(ofMap(normX, 0, 1, 0, 640), ofMap(normY, 0, 1, 0, 480))) << "\n";    
    cout << tp.x << ", " << tp.y << ", " << tp.z << "\n";
    
}

void testApp::mouseReleased(int x, int y, int button)
{}

void testApp::windowResized(int w, int h)
{}

void testApp::padUpdates(int & touchCount) {
    
    // will be used to speed up calibration details.
    // 
    if (touchCount == 2){

        if (pMouseX > mouseX) {
            cout << "left";
        } else if (pMouseX < mouseX) {
            cout << "right";
        }
        
        if (pMouseY > mouseY){
            cout << "up";
        } else if (pMouseY < mouseY){
            cout << "down";
        }
        cout << "\n";
        pMouseY = mouseY; 
        pMouseX = mouseX;
    }
}

void testApp::newTouch(int & n) {
//        cout << "++++++ a new touch"<<n<<"\n";
}

void testApp::removedTouch(int & r) {
//        cout << "------ a removed touch"<<r<<"\n";
}
