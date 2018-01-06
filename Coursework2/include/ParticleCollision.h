#pragma once

#include "pcontacts.h"
#include "particle.h"
#include <vector>

using namespace std;

struct Boundary;

class ParticleCollision : public ParticleContactGenerator
{
public:
	//Total number of particles in array
	const int NUM_PARTICLES;

	//Pointer to the first element of the particle array (saves passing in an entire array to ParticleCollision object)
	Particle* particles;

	//Default restitution value
	float restitution = 0.9;

	//When instantiating particle collision object, tell it how many other particles there are to collide with
	//and give it a pointer to first particle in the array.
	ParticleCollision(int numParticles, Particle* arrayPtr) : NUM_PARTICLES(numParticles)
	{
		particles = arrayPtr;
	}

	void setRestitution(float restitution) { this->restitution = restitution; }

	//Add all of the particle's current contact data to the relevant ParticleContact objects
	unsigned addContact(ParticleContact *contact, unsigned limit) const;

	//Determine if one particle and another are touching
	bool checkCollision(Particle& particle1, Particle& particle2, float distance) const;

	//Calculate outer boundary of Minkowski difference of two particles and return it in a Boundary struct
	Boundary calcMinkowskiDifferenceBounds(Particle& particle1, Particle& particle2) const;

	//Calculates 8 of a sphere-shaped particle's vertices (this gives an imperfect representation of the shape, but
	//is close enough for our purposes)
	vector<Vector2> estimateSphereVertices(Particle& particle) const;
};

//Used for storing boundary of Minkowski difference
//struct Boundary
//{
//	Vector2 left;
//	Vector2 right;
//	Vector2 bottom;
//	Vector2 top;
//
//	Boundary()
//	{
//		left = Vector2(0, 0);
//		right = Vector2(0, 0);
//		bottom = Vector2(0, 0);
//		top = Vector2(0, 0);
//	}
//	Boundary(Vector2 left, Vector2 right, Vector2 bottom, Vector2 top)
//	{
//		Boundary::left = left;
//		Boundary::right = right;
//		Boundary::bottom = bottom;
//		Boundary::top = top;
//	}
//};

struct Boundary
{
	Vector2 bottomLeft;
	Vector2 topRight;

	Boundary()
	{
		bottomLeft = Vector2(0, 0);
		topRight = Vector2(0, 0);
	}
	Boundary(Vector2 bottomLeft, Vector2 topRight)
	{
		Boundary::bottomLeft = bottomLeft;
		Boundary::topRight = topRight;
	}
};