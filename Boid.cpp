/*
 *  Boid.cpp
 *  boid
 *
 *  Created by Jeffrey Crouse on 3/29/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Updated by Takara Hokao
 *
 */

#include "Boid.h"

Boid::Boid() {
    loc.set(ofRandomWidth(),ofRandomHeight());
	vel.set(0,0,0);
	acc.set(0,0,0);
    r = 3.0;
    maxspeed = 0.5;
    maxforce = 0.01;
}

Boid::Boid(int x, int y) {
    loc.set(x,y,-100);
	vel.set(0,0,0);
	acc.set(0,0,0);
    r = 1.0;
    maxspeed = 0.5;
    maxforce = 0.01;
}

// Method to update location
void Boid::update(vector<Boid> &boids) {

    maxspeed=10;
    r=3.0;
	flock(boids);

    vel += acc;   // Update velocity
    vel.x = ofClamp(vel.x, -maxspeed, maxspeed);  // Limit speed
	vel.y = ofClamp(vel.y, -maxspeed, maxspeed);  // Limit speed
    vel.z = ofClamp(vel.z, -maxspeed, maxspeed);
    loc += vel;
    acc.set(0);  // Reset accelertion to 0 each cycle
    
	if (loc.x < -r) loc.x = ofGetWidth()+r;
    if (loc.y < -r) loc.y = ofGetHeight()+r;

    if (loc.x > ofGetWidth()+r) loc.x = -r;
    if (loc.y > ofGetHeight()+r) loc.y = -r;
    
}

void Boid::seek(ofVec3f target) {
    acc += steer(target, false);
}

void Boid::avoid(ofVec3f target) {
    acc -= steer(target, false);
}

void Boid::arrive(ofVec3f target) {
    acc += steer(target, true);
}

// A method that calculates a steering vector towards a target
// Takes a second argument, if true, it slows down as it approaches the target
ofVec3f Boid::steer(ofVec3f target, bool slowdown) {
    ofVec3f steer;  // The steering vector
    ofVec3f desired = target - loc;  // A vector pointing from the location to the target

	float d = loc.distance(target); // Distance from the target is the magnitude of the vector


	// If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (d > 0) {

		desired /= d; // Normalize desired
		// Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
		if ((slowdown) && (d < 100.0f)) {
			desired *= maxspeed * (d/100.0f); // This damping is somewhat arbitrary
		} else {
			desired *= maxspeed;
		}
		// Steering = Desired minus Velocity
		steer = desired - vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce); // Limit to maximum steering force
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
        steer.z = ofClamp(steer.z, -maxforce, maxforce);

    }

    return steer;
}

void Boid::draw() {
    // Draw a triangle rotated in the direction of velocity
	float angle = (float)atan2(-vel.y, vel.x);
    float theta =  -1.0*angle;
	float heading2D = ofRadToDeg(theta)+90;

	ofPushStyle();
    ofFill();
    ofPushMatrix();
    ofTranslate(loc.x, loc.y,loc.z);
    ofRotateZ(heading2D);
    
    
	ofBeginShape();
    ofVertex(0, -2,0);
    ofVertex(-1, 2,0);
    ofVertex(1, 2,0);
    ofEndShape(true);
    
    
    //ofDrawCircle(0,0,10);
    ofPopMatrix();
	ofPopStyle();
}

void Boid::flock(vector<Boid> &boids) {
	ofVec3f sep = separate(boids);
	ofVec3f ali = align(boids);
	ofVec3f coh = cohesion(boids);

	// Arbitrarily weight these forces
	sep *= 0.5;
	ali *= 0.2;
	coh *= 0.2;

    acc += sep+ ali + coh;
    //acc += coh;
}

/*
 postscript
 */
bool Boid::isHit(int x, int y, int radius) {
    int r = 1;
    int dist =r + radius;
    if(pow((x-loc.x),2)+pow((y-loc.y),2) < dist * dist) {
        return true;
    }
    return false;
}

// Separation
// Method checks for nearby boids and steers away
ofVec3f Boid::separate(vector<Boid> &boids) {
    float desiredseparation = 25.0f;
    ofVec3f steer;
    int count = 0;

    // For every boid in the system, check if it's too close
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];

		float d = loc.distance(other.loc);

		// If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
		if ((d > 0) && (d < desiredseparation)) {
			// Calculate vector pointing away from neighbor
			ofVec3f diff = loc - other.loc;
			diff /= d;			// normalize
			diff /= d;        // Weight by distance
			steer += diff;
			count++;            // Keep track of how many
		}
    }
    // Average -- divide by how many
    if (count > 0) {
		steer /= (float)count;
    }


    // As long as the vector is greater than 0
	//float mag = sqrt(steer.x*steer.x + steer.y*steer.y);

	float mag = steer.length();
    if (mag > 0) {

		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;

		steer *= maxspeed;

		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
        steer.z = ofClamp(steer.z, -maxforce, maxforce);
        

    }
    

    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
ofVec3f Boid::align(vector<Boid> &boids) {
    float neighbordist = 50.0;
    ofVec3f steer;
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];

		float d = loc.distance(other.loc);
		if ((d > 0) && (d < neighbordist)) {
			steer += (other.vel);
			count++;
		}
    }
    if (count > 0) {
		steer /= (float)count;
    }

    // As long as the vector is greater than 0
	float mag = steer.length();
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= 0.5;
		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
        steer.z = ofClamp(steer.z, -maxforce, maxforce);
    }
    return steer;
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
ofVec3f Boid::cohesion(vector<Boid> &boids) {
    float neighbordist = 50.0;
    ofVec3f sum;   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		float d = loc.distance(other.loc);
		if ((d > 0) && (d < neighbordist)) {
			sum += other.loc; // Add location
			count++;
		}
    }
    if (count > 0) {
		sum /= (float)count;
		return steer(sum, false);  // Steer towards the location
    }
    return sum;
}

