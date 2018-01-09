#pragma once

#include "pcontacts.h"
#include "particle.h"
#include <vector>

using namespace std;

//Used for storing boundary of Minkowski difference
struct Boundary
{
	float left;
	float right;
	float bottom;
	float top;

	Boundary()
	{
		left = 0;
		right = 0;
		bottom = 0;
		top = 0;
	}
	Boundary(float left, float right, float bottom, float top)
	{
		Boundary::left = left;
		Boundary::right = right;
		Boundary::bottom = bottom;
		Boundary::top = top;
	}
};

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
	Boundary MDBounds;

public:
	//When instantiating particle collision object, tell it how many other particles there are to collide with
	//and give it a pointer to first particle in the array.
	ParticleCollision(int numParticles, Particle* arrayPtr);

	void setRestitution(float restitution) { this->restitution = restitution; }

	//Add all of the particle's current contact data to the relevant ParticleContact objects
	unsigned addContact(ParticleContact *contact, unsigned limit);

	//Determine if one particle and another are touching
	bool checkCollision(Particle& particle1, Particle& particle2, float distance);

	//Calculate outer boundary of Minkowski difference of two particles and return it in a Boundary struct
	Boundary calcMinkowskiDifferenceBounds(Particle& particle1, Particle& particle2);


	//Calculates 8 of a sphere-shaped particle's vertices (this gives an imperfect representation of the shape, but
	//is close enough for our purposes)
	vector<Vector2> estimateSphereVertices(Particle& particle) const;
};

//struct Boundary
//{
//	Vector2 bottomLeft;
//	Vector2 topRight;
//
//	Boundary()
//	{
//		bottomLeft = Vector2(0, 0);
//		topRight = Vector2(0, 0);
//	}
//	Boundary(Vector2 bottomLeft, Vector2 topRight)
//	{
//		Boundary::bottomLeft = bottomLeft;
//		Boundary::topRight = topRight;
//	}
//};