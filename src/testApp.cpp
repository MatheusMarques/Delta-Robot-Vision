#include "testApp.h"

bool findHighest;
bool tumble;
bool debug;
bool viewport;
bool drawviewport;
bool bOscSend;
float tolerance;
float interestThresh;

#pragma mark - TODO
#pragma mark : add colors to match Zak's colorverse
#pragma mark : add grids
#pragma mark : send via OSC to port 6969
#pragma mark : tolerance via pinch
#pragma mark -

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
    
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	pointCloudRotationY = 180;
	
	drawPC = true;
    
    //some model / light stuffx
    
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glClear (GL_COLOR_BUFFER_BIT);
    glEnable (GL_BLEND);
    glEnable (GL_POLYGON_SMOOTH);
    //    glDisable (GL_DEPTH_TEST);
    glLineWidth(4.0);
    
    
    //fog
    GLfloat fogColor[4] = {1,1,1, 1.0};
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_DENSITY, 0.2);
    glHint(GL_FOG_HINT, GL_DONT_CARE);
    glFogf(GL_FOG_START, 500);
    glFogf(GL_FOG_END, 4000);
    glEnable(GL_FOG);
    
    ofEnableAlphaBlending();
    
    findHighest = true;
    tolerance = 1.0f; // shows everything
    tumble = true;
    debug = false;
    viewport = false;
    
    interestThresh = 0.0f;
    
    mouseY = 220;
    ofBackground(0, 0, 0);
    bOscSend = true;
    sender.setup( "localhost", 6969 );

}

//--------------------------------------------------------------
void testApp::update() {

	
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
		}
        
		grayImage.flagImageChanged();
    	contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
}

ofPoint poi;

void testApp::draw() {
	if(drawPC && !viewport){
		ofPushMatrix();
        glTranslatef(ofGetWidth()/2,ofGetHeight()/2,0);
        ofRotateZ(90);
            drawPointCloud();
 		ofPopMatrix();
        
        if(debug){
            grayImage.draw(ofGetWidth()-210, ofGetHeight()-160, 200, 150);
            contourFinder.draw(ofGetWidth()-210, ofGetHeight()-160, 200, 150);
        }
        
//        glRotatef(-mouseY,1,0,0);
//        glRotatef(-mouseX,0,1,0);
        
//        ofSetColor(0xdddddd);
//        glLineWidth(1);
//        drawGrid(50, 150, -500);
//        glLineWidth(4);
	
        if (!tumble){
//            glRotatef(-mouseY,1,0,0);
//            glRotatef(-mouseX,0,1,0);
        }else{
            
//            glRotatef(ofGetElapsedTimef()*10,0,0,0);
//            glRotatef(ofGetElapsedTimef()*11,0,1,0);
//            glRotatef(ofGetElapsedTimef()*7,0,0,1);
        }
        
        stringstream reportStream;
        reportStream << "VIEWING:  POINTCLOUD" << endl;
        reportStream << "press \"v\" to toggle viewport selection";
        ofSetColor(40, 40, 40, 125);
        ofRect(0, 0, ofGetWidth(), 40);
        ofSetColor(200, 200, 200);
        ofDrawBitmapString(reportStream.str(),20,20);	   
        
        
        
    } else if(drawPC && viewport){
        ofPushMatrix();
    
        kinect.draw(0, 0, ofGetWidth(), ofGetHeight());
        
        ofPopMatrix();
        // dont draw antthing if no origin mark witnessed
        if(drawviewport){
            ofSetColor(255, 0, 0);
            ofNoFill();
            ofTranslate(0, 0);
            ofSetRectMode(OF_RECTMODE_CORNER);
            ofRect(viewportOrigin.x, viewportOrigin.y, mouseX-viewportOrigin.x, mouseY-viewportOrigin.y);
            viewportEnd = ofPoint(mouseX, mouseY);
            poi = kinect.getWorldCoordinateFor(mouseX, mouseY);
            poi = normalizeOfPoint(poi.x, 640, poi.y, 480, poi.z, 6.0);  
            ofFill();

        }
        
        stringstream reportStream;
        reportStream << "VIEWING:  VIEWPORT SELECTION" << endl;
        reportStream << "Click and drag to trim kinect's visible viewport & PRESS \"v\" to return to pointcloud.";
        ofSetColor(40, 40, 40, 125);
        ofRect(0, 0, ofGetWidth(), 40);
        ofSetColor(200, 200, 200);
        ofDrawBitmapString(reportStream.str(),20,20);	 
        
//        ofSetColor(255, 255, 255);
//        stringstream reportStream;
//        reportStream << "Depth at cursor -   x: " << poi.x << " y: " << poi.y << " DEPTH: " << poi.z;
//        ofSetColor(0, 0, 0);
//        ofRect(20, 790, ofGetWidth(), 30);
//        ofDrawBitmapString(reportStream.str(),20,800);	
//        ofSetColor(255, 255, 255);
    
    }
    if (debug){
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
        
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
	}
	

//	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
//    << ofToString(kinect.getMksAccel().y, 2) << " / " 
//    << ofToString(kinect.getMksAccel().z, 2) << endl
//    << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
//    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
//    << "set near threshold " << nearThreshold << " (press: + -)" << endl
//    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
//    << ", fps: " << ofGetFrameRate() << endl
//    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
//    << "press UP and DOWN to change the tilt angle: " << angle << " degrees";
    

}

void rotatePixels(ofPixels &pix, float angle){
    
}


vector<ofPoint> normPoints;

void testApp::drawPointCloud() {
	ofScale(500, 500, 500);
	int w = 640;
	int h = 480;
    int width = 0; int height = 0;
    
    ofPoint closePoint(1.0, 1.0, 1.0);

    ofPoint vs(0, 0, 0.0f); 
    ofPoint ve(w, h, 0.0f);
    
    // if viewportOrigin.x != 0.0, then it has been set by user
    if (viewportOrigin.x != 0.0f){
        
        // map values to kinect ratio
        vs = mapPointTo(viewportOrigin, 640, 480);
        ve = mapPointTo(viewportEnd, 640, 480);
        
    } 
    
	ofRotateY(pointCloudRotationY);
//    ofRotateZ(180);
    
    float closeZ = 1.0; // a LOW value ie: 0.1 is closer to the camera.  Higher is further
    
    // clear each frame
    normPoints.clear(); normPoints.empty();
    
    
    // get info from depth loop
    glScalef(1, 1, 0.4);
	glBegin(GL_POINTS);
	int step = 3;    
		for(int x = vs.x; x < ve.x; x += step) {
            for(int y = vs.y; y < ve.y; y += step) {

			ofPoint raw = kinect.getWorldCoordinateFor(x, y);
            
//            cout << "raw.x: " << raw.x << " raw.y " << raw.y << " raw.z " << raw.z << "\n";
            // leave points normal for drawing onscreen
            ofPoint drawPnt = raw;
            // normalise points for whatever purpose
            raw = normalizeOfPoint(raw.x, w, raw.y, h, raw.z, 6.0);  
            // find closest point that isn't 0.0, clipped near threshold
                
            if(findHighest){
                if (raw.z < closePoint.z && raw.z != 0.0) {
                    // assign values to send
                    closePoint = raw;
                    // cout << "\n raw.z: " << raw.z << "\n";
                }
            }
            
            float lowerLimit = closePoint.z - tolerance / 2;
            float upperLimit = closePoint.z + tolerance / 2 ;
            
            ofColor colorRgb;
                colorRgb.r = 0;
                colorRgb.g = 0;
                colorRgb.b = 0;
            ofSetColor(colorRgb.r, colorRgb.g, colorRgb.b);
            
           // lowerLimit = 0.0f; // temp
           // upperLimit = 1.0f; // temp
            
            // filter drawn points
            if(raw.z > lowerLimit && raw.z < upperLimit){

                HSVToRGB(raw.z, 0.0, 1.0, colorRgb);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);
                
                
                // move drawn points to global array 
                normPoints.push_back(raw);
            }
            
            // depthMath
            // crude dist measurements vs. highpoints
            if(ofDist(1.0, raw.z, 1.0, interestThresh) < 0.1f){
                ofSetColor(0, 255, 255);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);   
                ofSetColor(255, 255,255);
            }
                
            // depthMath
            // crude high point
            // this needs to be averaged like the dist measurement above - not returning correct information
            if(ofDist(1.0, raw.z, 1.0, closePoint.z) < 0.001f){
                ofSetColor(255, 0, 0);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);
            }
                

            
		}
	}
	glEnd();
 
    if  (bOscSend == true) {
        // send high point as ofPoint
        ofxOscMessage m;
        m.setAddress( "/delta/highpoint" );
        m.addFloatArg( closePoint.x );
        m.addFloatArg( closePoint.y );
        m.addFloatArg( closePoint.z );
        sender.sendMessage( m );
        cout << "osc sent: x  " << closePoint.x << " y: " << closePoint.y << " z: " << closePoint.z << "\n";
        //cout << closePoint.z << "\n";
    }
    
    stringstream reportStream;
    reportStream << "Depth at cursor -   x: " << poi.x << " y: " << poi.y << " DEPTH: " << poi.z;
    ofSetColor(0, 0, 0);
    ofRect(20, 790, ofGetWidth(), 30);
    ofDrawBitmapString(reportStream.str(),20,800);	   
    
}

ofPoint testApp::mapPointTo(ofPoint _p, float _mx, float _my){
    ofPoint p;
    p.x = ofMap(_p.x, 0, ofGetWidth(), 0, _mx);
    p.y = ofMap(_p.y, 0, ofGetHeight(), 0, _my);    
    return p;
}

ofPoint testApp::normalizeOfPoint(float x, float w, float y, float h, float z, float d){
    float _x = ofNormalize(x, -2.0f, 2.0f);
    float _y = ofNormalize(y, -2.0f, 2.0f);
    float _z = ofNormalize(z, 0.0f, d);
    
    // reminder for close values = lower numbers
    return ofPoint(_x, _y, _z);
}

void testApp::drawGrid(int spacing, int lines, int zDepth){
    for (int i=-(lines/2); i<(lines/2); i++){
        glBegin(GL_LINES);
        glVertex3f(i*spacing, zDepth, -(spacing*lines)/2);
        glVertex3f(i*spacing, zDepth, spacing*lines/2);
        glEnd();
    }
    
    for (int i=-(lines/2); i<(lines/2); i++){
        glBegin(GL_LINES);
        glVertex3f(-(spacing*lines)/2, zDepth, i*spacing);
        glVertex3f(spacing*lines/2, zDepth, i*spacing);
        glEnd();
    }
}

void selectViewport(){
    
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
		case'd':
            debug = !debug;
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
            
        case 's':
            bOscSend = !bOscSend;
            break;
            
        case 'v':
            if( viewport == false ) 
            {viewport = true;} else {viewport = false;}
            break;
            
		case OF_KEY_UP:
//			angle++;
//			if(angle>30) angle=30;
//			kinect.setCameraTiltAngle(angle);
            tolerance += 0.1f;
            if (tolerance > 1.0f) tolerance = 1.0f;
            cout << tolerance << "\n";
			break;
            
		case OF_KEY_DOWN:
//			angle--;
//			if(angle<-30) angle=-30;
//			kinect.setCameraTiltAngle(angle);
            tolerance -=0.1f;
            if (tolerance < 0.0f) tolerance = 0.0f;
            cout << tolerance << "\n";
			break;
        
        case OF_KEY_LEFT:
            interestThresh -=0.1f;
            if(interestThresh < 0.0f) interestThresh = 0.0f;
            cout << "interest point: " << interestThresh << "\n";            
            break;
            
        case OF_KEY_RIGHT:
            interestThresh += 0.1f;
            if(interestThresh > 1.0f) interestThresh = 1.0f;
            cout << "interest point: " << interestThresh << "\n";
            break;
            
	}
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
    int _x = ofMap(x, 0, ofGetWidth(), 490, 540);
//    pointCloudRotationY = _x;
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
//    cout << tolerance;
    if(viewport){
        if(viewportOrigin.x != 0.0){
//            ofSetColor(255, 0, 0);
//            ofRect(viewportOrigin.x, viewportOrigin.y, x, y);
        }
    }
    
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
    if(viewport){
        drawviewport = true;
        viewportOrigin = ofPoint(x, y, 0.0);
//        cout << viewportOrigin.x << " : " << viewportOrigin.y << "viewport origin \n";
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{
    if(viewport){
        drawviewport = false;
//        viewportEnd = ofPoint(x, y, 0.0);
//        cout << viewportEnd.x << " : " << viewportEnd.y << "viewport end \n";

    }
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

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

