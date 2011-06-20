#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
//	ofSetupOpenGL(&window, 1024, 768, OF_FULLSCREEN);
//    window.setGlutDisplayString("rgba double samples>=8 depth"); //antialiasing

  	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);
	ofRunApp(new testApp());
}
