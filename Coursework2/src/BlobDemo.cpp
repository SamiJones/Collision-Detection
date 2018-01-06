/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include "ParticleCollision.h"
#include <iostream>
#include <vector>

using namespace std;

const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);
const int NUM_SPHERES = 20;
const int NUM_QUADS = 1;
const int NUM_PARTICLES = NUM_SPHERES + NUM_QUADS;
const int NUM_PLATFORMS = 3;
const int BASE_SPHERE_RADIUS = 5;
const int BASE_MASS = 5;

/**
 * Platforms are two dimensional: lines on which the
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
	Vector2 start;
	Vector2 end;
	/**
	 * Holds a pointer to the particles we're checking for collisions with.
	 */
	Particle* particle[NUM_PARTICLES];

	//default restitution value
	float restitution = 0.8;

	void setRestitution(float restitution) { this->restitution = restitution; }

	virtual unsigned addContact(ParticleContact *contact, unsigned limit) const;
};

unsigned Platform::addContact(ParticleContact *contact, unsigned limit) const
{
	//const static float restitution = 1.0f;
	int used = 0;

	for (int i = 0; i < NUM_PARTICLES; i++)
	{
		// Check for penetration
		Vector2 toParticle = particle[i]->getPosition() - start;
		Vector2 lineDirection = end - start;

		float projected = toParticle * lineDirection;
		float platformSqLength = lineDirection.squareMagnitude();
		float squareRadius = particle[i]->getRadius()*particle[i]->getRadius();;

		if (projected <= 0)
		{

			// The blob is nearest to the start point
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particle[i];
				contact->particle[1] = 0;
				contact->penetration = particle[i]->getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}

		}
		else if (projected >= platformSqLength)
		{
			// The blob is nearest to the end point
			toParticle = particle[0]->getPosition() - end;
			if (toParticle.squareMagnitude() < squareRadius)
			{
				// We have a collision
				contact->contactNormal = toParticle.unit();
				contact->restitution = restitution;
				contact->particle[0] = particle[i];
				contact->particle[1] = 0;
				contact->penetration = particle[i]->getRadius() - toParticle.magnitude();
				used++;
				contact++;
			}
		}
		else
		{
			// the blob is nearest to the middle.
			float distanceToPlatform = toParticle.squareMagnitude() - projected*projected / platformSqLength;
			if (distanceToPlatform < squareRadius)
			{
				// We have a collision
				Vector2 closestPoint = start + lineDirection*(projected / platformSqLength);

				contact->contactNormal = (particle[i]->getPosition() - closestPoint).unit();
				contact->restitution = restitution;
				contact->particle[0] = particle[i];
				contact->particle[1] = 0;
				contact->penetration = particle[i]->getRadius() - sqrt(distanceToPlatform);
				used++;
				contact++;
			}
		}
	}

	return used;
}

class BlobDemo : public Application
{
	Particle* blob;
	ParticleCollision* particleCollision;

	Platform* platform[NUM_PLATFORMS];

	ParticleWorld world;

public:
	/** Creates a new demo object. */
	BlobDemo();
	virtual ~BlobDemo();

	/** Returns the window title for the demo. */
	virtual const char* getTitle();

	/** Display the particles. */
	virtual void display();

	/** Update the particle positions. */
	virtual void update();

	void BlobDemo::boxCollisionResolve(Particle* particle);
	bool BlobDemo::outOfBoxTest(Particle* particle);
	void BlobDemo::outOfBoxResolve(Particle* particle);
};

// Method definitions
BlobDemo::BlobDemo() : world((NUM_PARTICLES + NUM_PLATFORMS) * (NUM_PARTICLES + NUM_PLATFORMS - 1), NUM_PLATFORMS * 3)
{
	width = 400; height = 400;
	nRange = 100.0;

	// Create the blob storage
	blob = new Particle[NUM_PARTICLES];

	//Create a new particle collision object, and tell it how many other particles there are to watch for collisions with.
	//Also, give it a pointer to the array of particles, and a pointer to the specific particle it is associated with
	particleCollision = new ParticleCollision(NUM_PARTICLES, blob);

	// Create the platform
	platform[0] = new Platform;
	platform[0]->setRestitution(0.6);
	platform[0]->start = Vector2(-50.0, 20.0);
	platform[0]->end = Vector2(35.0, 15.0);

	// Set up platform 2
	platform[1] = new Platform;
	platform[1]->setRestitution(0.9);
	platform[1]->start = Vector2(-20.0, -25.0);
	platform[1]->end = Vector2(80.0, -20.0);

	// Set up platform 3
	platform[2] = new Platform;
	platform[2]->setRestitution(0.8);
	platform[2]->start = Vector2(-80.0, -40.0);
	platform[2]->end = Vector2(0.0, -60.0);

	// Make sure the platform knows which particle it should collide with.
	for (int i = 0; i < NUM_PLATFORMS; i++)
		for (int j = 0; j < NUM_PARTICLES; j++)
			platform[i]->particle[j] = blob + j;

	//Add platforms to world object's vector of platform contact generators
	for (int i = 0; i < NUM_PLATFORMS; i++)
		world.getPlatformContactGenerators().push_back(platform[i]);

	//Add particle collision object to world object's vector of particle contact generators
	world.getParticleContactGenerator().push_back(particleCollision);

	// Initialise sphere particles
	for (int i = 0; i < NUM_SPHERES; i++)
	{
		(blob + i)->setPosition((i % 2) ? -20 : 20, 90);
		(blob + i)->setRadius(BASE_SPHERE_RADIUS + (i % 10));
		(blob + i)->setMass(BASE_MASS + (i % 10));
		(blob + i)->setVelocity(0, -1);
		(blob + i)->setAcceleration(Vector2::GRAVITY * 20.0f);
		(blob + i)->clearAccumulator();

		world.getParticles().push_back(blob + i);
	}

	//Initialise non-sphere particles
	for (int i = NUM_SPHERES; i < NUM_PARTICLES; i++)
	{
		(blob + i)->setPosition((i % 2) ? -20 : 20, 90);
		(blob + i)->setRadius(15);
		(blob + i)->setMass(15);
		(blob + i)->setVelocity(0, -1);
		(blob + i)->setAcceleration(Vector2::GRAVITY * 20.0f);
		(blob + i)->clearAccumulator();

		float radius = (blob + i)->getRadius();

		//Set up vertices for non-sphere particles
		vector<Vector2> vertices = {
			Vector2(-radius, 0),
			Vector2(0, -radius),
			Vector2(radius, 0),
			Vector2(0, radius)
		};

		(blob + NUM_SPHERES)->setVertices(vertices);

		world.getParticles().push_back(blob + i);
	}
}

BlobDemo::~BlobDemo()
{
	// Release the blob storage
	delete[] blob;
}

void BlobDemo::display()
{
	Application::display();

	//Render platforms
	for (int i = 0; i < 3; i++)
	{
		const Vector2 &p0 = platform[i]->start;
		const Vector2 &p1 = platform[i]->end;

		glBegin(GL_LINES);
		glColor3f(0, 1, 1);
		glVertex2f(p0.x, p0.y);
		glVertex2f(p1.x, p1.y);
		glEnd();
	}
	//Render platforms

	//Render sphere particles
	float r = 0.0, g = 0.0, b = 1;

	for (int i = 0; i < NUM_SPHERES; i++, r += 0.1, g += 0.1)
	{
		if (r > 0.9 && g > 0.9)
			r = g = 0;

		glColor3f(r, g, b);

		Vector2 &p = (blob + i)->getPosition();
		glPushMatrix();
		glTranslatef(p.x, p.y, 0);
		glutSolidSphere((blob + i)->getRadius(), 12, 12);
		glPopMatrix();
	}
	//Render sphere particles

	//Render non-sphere particles
	glColor3f(1, 0, 0);

	vector<Vector2> vertices = (blob + NUM_SPHERES)->getVertices();
	int numVertices = vertices.size();
	Vector2 position = (blob + NUM_SPHERES)->getPosition();

	glPushMatrix();
	glTranslatef(position.x, position.y, 0);

	glBegin(GL_QUADS);
	for (int i = 0; i < numVertices; i++)
		glVertex2f(vertices[i].x, vertices[i].y);
	glEnd();

	glPopMatrix();
	//Render non-sphere shapes
	
	glutSwapBuffers();

}

void BlobDemo::update()
{
	// Recenter the axes
	float duration = timeinterval / 1000;
	// Run the simulation

	world.runPhysics(duration);

	//Boundary collision detection and resolution
	for (int i = 0; i < NUM_PARTICLES; i++)
	{
		boxCollisionResolve(blob + i);

		if (outOfBoxTest(blob + i))
			outOfBoxResolve(blob + i);
	}

	Application::update();
}

// detect if the particle colided with the box and produce a response
void BlobDemo::boxCollisionResolve(Particle* particle)
{
	Vector2 position = particle->getPosition();
	Vector2 velocity = particle->getVelocity();
	float radius = particle->getRadius();

	float w = Application::width;
	float h = Application::height;

	// Reverse direction when you reach left or right edge
	if (position.x > w - radius || position.x < -w + radius)
		particle->setVelocity(-velocity.x, velocity.y);

	// Reverse direction when you reach top or bottom edge
	if (position.y > h - radius || position.y < -h + radius)
		particle->setVelocity(velocity.x, -velocity.y);
}

//  Check bounds. This is in case the window is made
//  smaller while the sphere is bouncing and the 
//  sphere suddenly finds itself outside the new
//  clipping volume
bool BlobDemo::outOfBoxTest(Particle* particle)
{
	Vector2 position = particle->getPosition();
	Vector2 velocity = particle->getVelocity();
	float radius = particle->getRadius();
	if ((position.x > Application::width - radius) || (position.x < -Application::width + radius)) return true;
	if ((position.y > Application::height - radius) || (position.y < -Application::height + radius)) return true;

	return false;
}

//  Check bounds. This is in case the window is made
//  smaller while the sphere is bouncing and the 
//  sphere suddenly finds itself outside the new
//  clipping volume
void BlobDemo::outOfBoxResolve(Particle* particle)
{
	Vector2 position = particle->getPosition();
	Vector2 velocity = particle->getVelocity();
	float radius = particle->getRadius();


	if (position.x > Application::width - radius)        position.x = Application::width - radius;
	else if (position.x < -Application::width + radius)  position.x = -Application::width + radius;

	if (position.y > Application::height - radius)        position.y = Application::height - radius;
	else if (position.y < -Application::height + radius)  position.y = -Application::height + radius;

	particle->setPosition(position.x, position.y);
}

const char* BlobDemo::getTitle()
{
	return "Blob Demo";
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
	return new BlobDemo();
}