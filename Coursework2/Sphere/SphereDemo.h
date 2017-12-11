#pragma once

#include "App.h"
#include "Particle.h"

#define NUM_PARTICLES 2

class SphereDemo : public Application
{
private:
	int xStep;
	int yStep;

	Particle* particle[NUM_PARTICLES];

	float g = 10;
public:
	SphereDemo();
	~SphereDemo();
	
	virtual void display();
	virtual void update();

	void box_collision_resolve(Particle* particle);
	bool out_of_box_test(Particle* particle);
	void out_of_box_resolve(Particle* particle);
};
