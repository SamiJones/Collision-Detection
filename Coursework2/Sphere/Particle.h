#pragma once

#include "CoreMath.h"

class Particle
{
private:
	Vector2 position;
	Vector2 velocity;
	Vector2 acceleration;
	Vector2 forceAccum;

	float inverseMass;

	float radius;
public:
	Particle();

	// calculates the next position of the particle.
	// Duration denotes the time interval
	//since the last update
	void integrate(float duration);

	// Sets the position of the particle by component
	void setPosition(const float x, const float y);
	
	// Gets the position of the particle. 
	Vector2 getPosition();
	
	// Sets the velocity of the particle by component.
	void setVelocity(const float x, const float y);
	void setVelocity(const Vector2& velocity);
	
	// Gets the velocity of the particle.
	Vector2 getVelocity();

	//Sets the radius of the particle.
	void setRadius(const float r);

	// Gets the radius of the particle
	float getRadius();

	// Sets the acceleration of the particle.
	void setAcceleration(const Vector2 &acceleration);
	void setAcceleration(const float x, const float y);

	//Gets the acceleration of the particle.
	Vector2 getAcceleration(void);

	void setDamping(const float damping);
	
	float getDamping();

	void setMass(const float mass);
	float getMass() const;

	void setInverseMass(const float inverseMass);
	float getInverseMass() const;

	bool hasFiniteMass() const;

	void clearAccumulator();
	void addForce(const Vector2 &force);
};
