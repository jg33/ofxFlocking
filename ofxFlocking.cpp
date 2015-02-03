/*
 *  ofxFlocking.cpp
 *  flock
 *
 *  Created by Jeffrey Crouse on 3/30/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Modified by Takara Hokao & Jesse Garrison
 */

#include "ofxFlocking.h"

void ofxFlocking::update() {
    int i;
	for(i = 0; i < boids.size(); i++) {
		boids[i].update(boids);
	}
}

void ofxFlocking::draw() {
    int i;
	for(i = 0; i < boids.size(); i++) {
		boids[i].draw();
	}
}

void ofxFlocking::addBoid() {
	boids.push_back(Boid());
}

void ofxFlocking::addBoid(int x, int y) {
	boids.push_back(Boid(x, y));
}

void ofxFlocking::removeBoid(int x, int y, int radius) {
    int i;
    for (i=0; i<boids.size(); i++) {
        if(boids[i].isHit(x, y, radius)) {
            boids.erase(boids.begin()+i);
        }
    }
}

void ofxFlocking::setSeparate(float amt){
    for (int i=0; i<boids.size(); i++) {
        boids[i].separateAmt = amt;
    }
}

void ofxFlocking::setAlign(float amt){
    for (int i=0; i<boids.size(); i++) {
        boids[i].separateAmt = amt;
    }
}

void ofxFlocking::setCohesion(float amt){
    for (int i=0; i<boids.size(); i++) {
        boids[i].cohesionAmt = amt;
    }
}
void ofxFlocking::setDrift(float freq, float amt){
    for (int i=0; i<boids.size(); i++) {
        boids[i].driftFreq = freq;
        boids[i].driftAmt = amt;
    }
}

int ofxFlocking::flockSize() {
    return boids.size();
}