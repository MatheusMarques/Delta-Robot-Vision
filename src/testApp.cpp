#include "testApp.h"

bool findHighest;
bool tumble;
float tolerance;

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
    
    findHighest = true;
    tolerance = 1.0f;
    tumble = true;
    
    mouseY = 220;
    ofBackground(0, 0, 0);
}

//--------------------------------------------------------------
void testApp::update() {

	
	kinect.update();
	if(kinect.isFrameNew())	// there is a new frame and we are connected
	{
        
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
		//we do two thresholds - one for the far plane and one for the near plane
		//we then do a cvAnd to get the pixels which are a union of the two thresholds.	
		if( bThreshWithOpenCV ){
			grayThreshFar = grayImage;
			grayThresh = grayImage;
			grayThresh.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThresh.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		}else{
            
			//or we do it ourselves - show people how they can work with the pixels
            
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

//--------------------------------------------------------------
void testApp::draw() {
	if(drawPC){
		ofPushMatrix();
        glTranslatef(ofGetWidth()/2,ofGetHeight()/2,0);
            drawPointCloud();
 		ofPopMatrix();
        
        grayImage.draw(ofGetWidth()-210, ofGetHeight()-160, 200, 150);
        contourFinder.draw(ofGetWidth()-210, ofGetHeight()-160, 200, 150);
        
        glRotatef(-mouseY,1,0,0);
        glRotatef(-mouseX,0,1,0);
        
//        ofSetColor(0xdddddd);
//        glLineWidth(1);
//        drawGrid(50, 150, -500);
//        glLineWidth(4);
	
        if (!tumble){
            glRotatef(-mouseY,1,0,0);
            glRotatef(-mouseX,0,1,0);
        }else{
            
//            glRotatef(ofGetElapsedTimef()*10,0,0,0);
//            glRotatef(ofGetElapsedTimef()*11,0,1,0);
//            glRotatef(ofGetElapsedTimef()*7,0,0,1);
            

        }
        
    }else{
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);
        
		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);
	}
	

//	ofSetColor(0, 0, 0);
	stringstream reportStream;
}

vector<ofPoint> normPoints;

void testApp::drawPointCloud() {
	ofScale(500, 500, 500);
	int w = 640;
	int h = 480;
    
	ofRotateY(pointCloudRotationY);
	float* distancePixels = kinect.getDistancePixels();
    
    float closeZ = 1.0; // a LOW value ie: 0.1 is closer to the camera.  Higher is further
    ofPoint closePoint = ofPoint (1.0, 1.0, 1.0);
    
    
    // clear each frame
    normPoints.clear(); normPoints.empty();
    

    glScalef(1, 1, 0.5);
    
    if(tumble){
//        glRotatef(ofGetElapsedTimef()*30,0,0,1);
//        glRotatef(ofGetElapsedTimef()*11,0,1,0);
//        glRotatef(ofGetElapsedTimef()*7,0,0,1);
    }
    
	glBegin(GL_POINTS);
	int step = 3;    
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			ofPoint raw = kinect.getWorldCoordinateFor(x, y);
            
            // leave points normal for drawing onscreen
            ofPoint drawPnt = raw;
            
            // normalise points for whatever purpose
            raw = normalizeOfPoint(raw.x, w, raw.y, h, raw.z, 6.0);  
            
            // find closest point that isn't 0.0, clipped near threshold
            if(findHighest){
                if (raw.z < closePoint.z && raw.z != 0.0) {
                    closePoint = raw;
                    // cout << "\n raw.z: " << raw.z << "\n";
                }
            }
            
            float lowerLimit = closePoint.z - tolerance / 2;
            float upperLimit = closePoint.z + tolerance / 2 ;
                
            ofColor colorRgb;
            colorRgb.r = 125;
            colorRgb.g = 125;
            colorRgb.b = 125;
            ofSetColor(colorRgb.r, colorRgb.g, colorRgb.b);
            
            lowerLimit = 0.0f; // temp
            upperLimit = 1.0f; // temp
            
            // filter drawn points
            if(raw.z > lowerLimit && raw.z < upperLimit){

                HSVToRGB(raw.z, 0.0, 1.0, colorRgb);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);
                
                
                // move drawn points to global array 
                normPoints.push_back(raw);
            }
            
            
            // crude dist measurements vs. highpoints
            if(ofDist(1.0, raw.z, 1.0, closePoint.z) < 0.2f){
                ofSetColor(0, 255, 255);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);   
            }
            
            // crude high point
            if(raw.z == closePoint.z){
                ofSetColor(255, 0, 0);
                glVertex3f(drawPnt.x, drawPnt.y, drawPnt.z);
            }
            
		}
	}
	glEnd();
    
}

ofPoint testApp::normalizeOfPoint(float x, float w, float y, float h, float z, float d){
    float _x = ofNormalize(x, 0.0f, w);
    float _y = ofNormalize(y, 0.0f, h);
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

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
    int _x = ofMap(x, 0, ofGetWidth(), 490, 540);
    pointCloudRotationY = _x;
//    pointCloudRotationY = x;
    cout << "x: " << x << " \n";
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
    tolerance = ofNormalize(y, 0, ofGetHeight());
    cout << tolerance;
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

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

