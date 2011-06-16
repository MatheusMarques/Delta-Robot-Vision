//
//  tkPointCloud.h
//  Delta Sim
//
//  Created by Tarei on 7/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _DEPTH_MATH // if this class hasn't been defined, the program can define it
#define _DEPTH_MATH // by using this if statement you prevent the class to be called more 

#include "ofMain.h"


class depthMath {
	
    
public:
    depthMath();
    
    bool update(vector<ofPoint> _pts);
    bool findHighPoint();
    
    
    vector<ofPoint> pts;
};
#endif 