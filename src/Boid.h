/*
 *  Boid.h
 *  boid
 *
 *  Created by Jeffrey Crouse on 3/29/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Updated by Takara Hokao & Jesse Garrison
 */

#ifndef BOID_H
#define BOID_H

#include "ofMain.h"

class Boid {
public:
	Boid();
	Boid(int x, int y);
	
	void update(vector<Boid> &boids);
	void draw();
	
    void seek(ofVec3f target);
    void avoid(ofVec3f target);
    void arrive(ofVec3f target);
    void drift(float freq, float amp);
	
    void flock(vector<Boid> &boids);
    bool isHit(int x,int y, int radius);
    
	ofVec3f steer(ofVec3f target, bool slowdown);
	
	ofVec3f separate(vector<Boid> &boids);
	ofVec3f align(vector<Boid> &boids);
	ofVec3f cohesion(vector<Boid> &boids);
	
	ofVec3f loc,vel,acc;
    
	float size;
	float maxforce;
	float maxspeed;
    
    int seed;
    
    float separateAmt, alignAmt, cohesionAmt;
    float driftFreq, driftAmt;
    
    //TODO: life
};

#endif