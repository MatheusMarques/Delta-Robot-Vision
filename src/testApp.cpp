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
	
	drawPC = FALSE;
    calibrate = false;
    povLock = true;

    
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
    if(!drawPC){
        kinect.draw(0, 0, ofGetWidth(), ofGetHeight());
        stringstream reportStream;
        ofSetColor(0,0,0);

//        ofSetColor(255, 255, 255);
        reportStream << "Depth Select Mode" << endl;
        if(pois.size() > 0){
            int last = pois.size()-1;
            reportStream << "Last point added had values:  X:" << ofToString(pois[last].x)
            << " Y: " << ofToString(pois[last].y) 
            << " Z: " << ofToString(pois[last].z);
        } else { reportStream << "Click mouse to get point data." ; }
        
        ofSetColor(255, 255 ,255);
        ofRect(0, 0, ofGetWidth(), 40);
        ofSetColor(0,0,0);
        ofDrawBitmapString(reportStream.str(),20,20);
        ofSetColor(255, 255, 255);

    }
    
	if(drawPC){
        ofSetColor(0, 0, 0);
        ofRect(0,0, ofGetWidth(), ofGetHeight());
		ofPushMatrix();
		ofTranslate(420, 320);
            drawPointCloud();
		ofPopMatrix();
        ofSetColor(255, 255 ,255);
        ofRect(0, 0, ofGetWidth(), 30);
        ofSetColor(0,0,0);
        stringstream reportStream;
        
        if(povLock) {
            reportStream << "Now viewing:  3D Point Cloud";
        } else {
            reportStream << "Now viewing:  Free View Mode (Move mouse to alter point cloud)";
        }
        
        ofDrawBitmapString(reportStream.str(),20,20);
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
	float* distancePixels = kinect.getDistancePixels();
    
    glPushMatrix();

    float mx = ofMap(mouseX, 0, ofGetWidth(), -5, 1);
    float my = ofMap(mouseY, 0, ofGetHeight(), -5, 1);
    
    glRotatef( 500 , -.1, 0, 0);
    if(!povLock){
        glTranslatef(mx, 0, my);
    }

    
    if (angle > 360) {
        angle =0;
    } 
    float maxValue = 0;
    ofPoint maxPoint;
	glBegin(GL_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint cur = kinect.getWorldCoordinateFor(x,y);
//            ofColor color = kinect.getCalibratedColorAt(x,y);
            ofColor color;  color.r = 125; color.g = 255; color.b = 255;
            ofColor newColor = HSVToRGB(cur.z*0.75, 0.5, 1, color);
            
            ofSetColor(newColor.r, newColor.g, newColor.b);
            
            if(maxValue < cur.z) { 
                maxValue = cur.z;
                maxPoint.set(cur.x, cur.y, cur.z);
            }
//            ofSetColor(0,0,0);
//			glColor3ub((unsigned char)color.r,(unsigned char)color.g,(unsigned char)color.b);
			glVertex3f(cur.x*2, cur.y*2, cur.z*2);
		}
	}
	glEnd();
//    cout << maxValue <<"\n";
    glBegin(GL_POINTS);
    glPointSize(20);
    ofSetColor(255, 0, 0);
    glVertex3f(maxPoint.x, maxPoint.y, maxPoint.z);
    glPointSize(1);
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
         //   calibrate = !calibrate;
            break;
		case'p':
			drawPC = !drawPC;
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
//            showReport = !showReport;
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
        pointCloudRotationX = y;
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

    
    // push into array
    pois.push_back(getDepthAsOfPoint(x, y, zDepth));
    cout << "Distance at " << x << " : " << y << " is " << kinect.getDistanceAt(ofPoint(ofMap(normX, 0, 1, 0, 640), ofMap(normY, 0, 1, 0, 480))) << "\n";   
    int loc = pois.size()-1;
    cout << pois[loc].x << ", " << pois[loc].y << ", " << pois[loc].z << "\n";
    
}


// automatically maps x / y to ofGetWidth() ofGetHeight().
// needs a method that accepts a bounding box to map
ofPoint testApp::getDepthAsOfPoint(float _x, float _y, float _zDepth){

    float normX = ofNormalize(_x, 0, ofGetWidth());
    float normY = ofNormalize(_y, 0, ofGetHeight());
    float zDepth = _zDepth;
    float normZ = ofNormalize(zDepth, 0, 400);
    
    return ofPoint(normX, normY, normZ);
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


ofColor testApp::HSVToRGB(float H, float S, float V, ofColor &in){ // (0-1, 0-1, 0-1) //applies hsv transform to the ofColor &in
    
    H = H*360;
    
    float VSU = V*S*cos(H*M_PI/180);
    float VSW = V*S*sin(H*M_PI/180);
    
    ofColor ret;
    ret.r = (.299*V+.701*VSU+.168*VSW)*in.r + (.587*V-.587*VSU+.330*VSW)*in.g + (.114*V-.114*VSU-.497*VSW)*in.b;
    ret.g = (.299*V-.299*VSU-.328*VSW)*in.r + (.587*V+.413*VSU+.035*VSW)*in.g + (.114*V-.114*VSU+.292*VSW)*in.b;
    ret.b = (.299*V-.3*VSU+1.25*VSW)*in.r + (.587*V-.588*VSU-1.05*VSW)*in.g + (.114*V+.886*VSU-.203*VSW)*in.b;
    return ret;
}

