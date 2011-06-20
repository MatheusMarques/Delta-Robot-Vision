//
//  tkPointCloud.cpp
//  Delta Sim
//
//  Created by Tarei on 7/06/11.
//  Copyright 2011. All rights reserved.
//

#include <depthMath.h>

depthMath::depthMath(){
    cout << "\n depthMath constructor \n";
}

bool depthMath::update(vector<ofPoint> _pts){
    pts.empty();
    pts.clear();
    
    pts = _pts;
    return true;
}

ofPoint depthMath::findHighPoint(){
    ofPoint maxValue = pts[0];
    
    for (int i=1; i < pts.size()-1; i++){
        if(pts[i].z > maxValue.z) {
            maxValue = pts[i];
        }
    }
    
    
    return maxValue;
}

vector<ofPoint> depthMath::nearestNeighbours(ofPoint origin, float tolerance){
    float dist = ofNormalize(tolerance, 0, 100);    
}

