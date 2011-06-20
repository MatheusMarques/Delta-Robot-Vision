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
    povLock = true;

    tolerance = 1.0f;
    
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
    
    // get depth details here
    // therefore, clear point vector
    points.clear();
    points.empty();
    
    // move to points vector
    
}



void testApp::drawHighPoints(){
    ofPoint cur = kinect.getWorldCoordinateFor( ofGetWidth()/2, ofGetHeight()/2);
    // MAP THE FUCKEN Z AXIS!
    cur.z = ofNormalize(cur.z, 0, 4.0f);
    cout << cur.z << "\n";
         
}




void testApp::draw() {
    drawHighPoints();
    
    if(!drawPC){
        kinect.draw(0, 0, ofGetWidth(), ofGetHeight());
        stringstream reportStream;
        ofSetColor(0,0,0);

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
            drawPointCloud(tolerance);
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

bool topdown = true;

void testApp::drawPointCloud(float tolerance) {
	ofScale(400, 400, 400);
	int w = 640;
	int h = 480;
    
    glPushMatrix();

    float mx = ofMap(mouseX, 0, ofGetWidth(), -5, 1);
    float my = ofMap(mouseY, 0, ofGetHeight(), -5, 1);
    
    if(topdown){
     //   glRotatef( 500 , -.1, 0, 0);
    }
    
	glBegin(GL_POINTS);
	int step = 2    ;
    // tk clear points array to repopulate with fresh data
    points.clear();
    points.empty();
    
    ofPoint chosenPoint;
    
    if(pois.size() > 0){
        chosenPoint = pois[pois.size()];
    } else {chosenPoint = ofPoint(0.5, 0.5, 0.5);}

    
    
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {

			ofPoint cur = kinect.getWorldCoordinateFor(x,y);
            cur.z = ofMap(cur.z, 0, 4.0, 0, 1); 
            if (cur.z > chosenPoint.z - tolerance/2 && cur.z < chosenPoint.z + tolerance / 2) {
                ofColor color;  color.r = 125; color.g = 255; color.b = 255;
                ofColor newColor = HSVToRGB(cur.z*0.75, 0.5, 1, color);
                ofSetColor(newColor.r, newColor.g, newColor.b);
                

                glVertex3f(cur.x, cur.y, cur.z);
                points.push_back(cur);
            }

		}
	}
    cout << tolerance;
    depth.update(points);
	glEnd();
    
    glPopMatrix();

//    cout << "\n highpoint: " << depth.findHighPoint().z << "\n";
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
{
    tolerance = ofNormalize(x, 0, ofGetWidth());
}

void testApp::mousePressed(int x, int y, int button)
{

    
    // clean mouse values - get pixel point on kinect.draw() frame
    // normalize and convert to ofPoint
    // send to Zak
    
    float normX = ofNormalize(x, 0, ofGetWidth());
    float normY = ofNormalize(y, 0, ofGetHeight());
    float zDepth = kinect.getDistanceAt(ofPoint(ofMap(normX, 0, 1, 0, 640), ofMap(normY, 0, 1, 0, 480)));
    cout << zDepth << "\n";
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
    
    if (touchCount == 2){
        MTouch t1, t2;
        if(pad.getTouchAt(0, &t1) && pad.getTouchAt(1, &t2)){
            
        }
        
        
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

