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
    pts = _pts;
    return true;
}

bool depthMath::findHighPoint(){
    ofPoint tempHighest = ofPoint(0.0, 0.0, 0.0);
    
    if(pts.size()>0){
        for (int i = 0; i < pts.size(); i++) {
            if(pts[i].z > tempHighest.z){

            }
        }
    }
}