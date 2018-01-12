#pragma once

#include "pcontacts.h"
#include "particle.h"
#include <vector>

using namespace std;

class ParticleCollision : public ParticleContactGenerator
{
private:
	//Total number of particles in array
	const int NUM_PARTICLES;

	//Pointer to the first element of the particle array (saves passing in an entire array to ParticleCollision object)
	Particle* particles;

	//Default restitution value
	float restitution = 0.9;

	//Far borders of the convex polygon created by the Minkowski difference.
	//Recalculated for every collision where at least one particle is not a circle.
	vector<Vector2> MDVertices;

public:
	//When instantiating particle collision object, tell it how many other particles there are to collide with
	//and give it a pointer to first particle in the array.
	ParticleCollision(int numParticles, Particle* arrayPtr);

	void setRestitution(float restitution) { this->restitution = restitution; }

	//Add all of the particle's current contact data to the relevant ParticleContact objects
	unsigned addContact(ParticleContact *contact, unsigned limit);

	//Determine if one particle and another are touching
	bool checkCollision(Particle& particle1, Particle& particle2, float distance);

	//Calculate vertices for Minkowski difference of two particles and return it as a vector of Vector2 objects
	vector<Vector2> calcMinkowskiDifferenceVertices(Particle& particle1, Particle& particle2);

	//Calculates 8 of a sphere particle's vertices (this gives an imperfect representation of the shape, but
	//is close enough for our purposes)
	vector<Vector2> estimateSphereVertices(Particle& particle) const;

	//Determines whether the origin lies within the bounds of the calculated Minkowski difference
	bool polygonContainsOrigin(vector<Vector2>& vertices);
};