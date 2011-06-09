Getting z-depth values based on user mouse click.  Currently prepares a vector<ofPoint> named pois as normalized x, y, z values.
	
Dependancies
	ofxKinect, ofxOpenCv, ofxMultiTouchPad, ofxThread, ofxVectorMath
	
TODO
	GUI setup
		Add OSC send for both single points and sending of POI vector
		Calibrate gui interactions including depthmap throttle setup, OSC setup
		Add reporting of values added - whether poi returned NaN, 0 or 1 (extreme results)

	Pointcloud
		Add zoom
		Mirror output to match webcam capture
		Scale to select pois directly from PC
		
	
	Calibrate gui
		4 screen hud with sliders to calibrate depth throttle and tweaking
		
	Mathy
		Add rectangle selection tool which adds gathers all points in area and checks they are 		within a tolerance level.  Once confirmed, the results become the pois vector
		
	OpenCV
		Shadowing of depthmap can cause depth inaccuracies
		
	