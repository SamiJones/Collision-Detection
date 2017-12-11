#include "Particle.h"
#include <assert.h>
#include <float.h>

//sets default state for the particle upon creation (in case the values are not set in the SphereDemo class)
Particle::Particle()
{
	radius = 10;

	position.x = 0;
	position.y = 0;

	velocity.x = 10;
	velocity.y = 10;

	acceleration.x = 0;
	acceleration.y = -10;
}

// calculates the next position of the particle.
// Duration denotes the time interval
//since the last update
void Particle::integrate(float duration)
{
	//do not attempt to integrate infinite or negative masses
	if (inverseMass <= 0.0f)
		return;

	assert(duration > 0.0);

	Vector2 resultingAcc = acceleration;
	resultingAcc.addScaledVector(forceAccum, inverseMass);

	velocity.addScaledVector(resultingAcc, duration);

	position.addScaledVector(velocity, duration);

	clearAccumulator();
}

void Particle::setPosition(const float x, const float y)
{
	position.x = x;
	position.y = y;
}

// Gets the position of the particle. 
Vector2 Particle::getPosition()
{
	return position;
}

// Sets the velocity of the particle by component.
void Particle::setVelocity(const float x, const float y)
{
	velocity.x = x;
	velocity.y = y;
}

// Gets the velocity of the particle.
Vector2 Particle::getVelocity()
{
	return velocity;
}

//Sets the radius of the particle.
void Particle::setRadius(const float r)
{
	radius = r;
}

// Gets the radius of the particle
float Particle::getRadius()
{
	return radius;
}

void Particle::setAcceleration(const Vector2 &acceleration)
{
	Particle::acceleration = acceleration;
}

void Particle::setAcceleration(const float x, const float y)
{
	acceleration = Vector2(x, y);
}

Vector2 Particle::getAcceleration(void)
{
	return acceleration;
}

void Particle::setMass(const float mass)
{
	assert(mass != 0);
	Particle::inverseMass = ((float)1.0) / mass;
}

float Particle::getMass() const
{
	if (inverseMass == 0)
		return DBL_MAX;
	else
		return ((float)1.0) / inverseMass;
}

void Particle::setInverseMass(const float inverseMass)
{
	Particle::inverseMass = inverseMass;
}

float Particle::getInverseMass() const
{
	return inverseMass;
}

bool Particle::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

void Particle::clearAccumulator()
{
	forceAccum.clear();
}

void Particle::addForce(const Vector2 &force)
{
	forceAccum += force;
}