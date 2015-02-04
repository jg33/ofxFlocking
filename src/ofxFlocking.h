/*
 *  Flock.h
 *
 *  Created by Jeffrey Crouse on 3/30/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Modified by Takara Hokao & Jesse Garrison
 */

#pragma once

#include "ofMain.h"
#include "Boid.h"

class ofxFlocking {
public:
	void update();
	void draw();
	void addBoid();
	void addBoid(int x, int y);
    void removeBoid(int x, int y, int radius);
    int flockSize();
    
    void setSeparate(float amt);
    void setCohesion(float amt);
    void setAlign(float amt);
    void setDrift(float freq, float amt);

    
	vector<Boid> boids;
};
