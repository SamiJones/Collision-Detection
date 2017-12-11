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


const Vector2 Vector2::GRAVITY = Vector2(0, -9.81);
const int NUM_PARTICLES = 60;
const int NUM_PLATFORMS = 3;
const int BASE_SPHERE_RADIUS = 5;

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
	Particle *particle[NUM_PARTICLES];

	//default restitution value
	float restitution = 0.8;

	void setRestitution(float restitution) { this->restitution = restitution; }

	virtual unsigned addContact(
		ParticleContact *contact,
		unsigned limit
		) const;
};

unsigned Platform::addContact(ParticleContact *contact, unsigned limit) const
{
	//const static float restitution = 1.0f;
	unsigned used = 0;

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
	Particle *blob[NUM_PARTICLES];

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
BlobDemo::BlobDemo() : world(NUM_PARTICLES + NUM_PLATFORMS, NUM_PLATFORMS)
{
	width = 400; height = 400;
	nRange = 100.0;

	// Create the blob storage
	for (int i = 0; i < NUM_PARTICLES; i++)
		blob[i] = new Particle;

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
			platform[i]->particle[j] = blob[j];

	for (int i = 0; i < NUM_PLATFORMS; i++)
		world.getContactGenerators().push_back(platform[i]);

	// Create the blob
	for (int i = 0; i < NUM_PARTICLES; i++)
	{
		blob[i]->setPosition((i%2)?-20:20, 90);
		blob[i]->setRadius(BASE_SPHERE_RADIUS + (i%10));
		blob[i]->setVelocity((i%2)?i*3:-i*3, 0);
		//blob->setDamping(0.9);
		blob[i]->setDamping(1.0);
		blob[i]->setAcceleration(Vector2::GRAVITY * 20.0f);
		blob[i]->setMass(i%10 * 10 + 1);
		blob[i]->clearAccumulator();
		world.getParticles().push_back(blob[i]);
	}
}

BlobDemo::~BlobDemo()
{
	// Create the blob storage
	for (int i = 0; i < NUM_PARTICLES; i++)
		delete blob[i];
}

void BlobDemo::display()
{
	Application::display();

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

	float r = 0.0, g = 0.0, b = 1;

	for (int i = 0; i < NUM_PARTICLES; i++, r += 0.1, g += 0.1)
	{
		if (r > 0.9 && g > 0.9)
			r = g = 0;

		glColor3f(r, g, b);

		Vector2 &p = blob[i]->getPosition();
		glPushMatrix();
		glTranslatef(p.x, p.y, 0);
		glutSolidSphere(blob[i]->getRadius(), 12, 12);
		glPopMatrix();
	}
	glutSwapBuffers();

}

void BlobDemo::update()
{
	// Recenter the axes
	float duration = timeinterval / 1000;
	// Run the simulation
	world.runPhysics(duration);

	//collision detection and resolution
	for (int i = 0; i < NUM_PARTICLES; i++)
	{
		boxCollisionResolve(blob[i]);

		if (outOfBoxTest(blob[i]))
			outOfBoxResolve(blob[i]);

		Vector2 pos1 = blob[i]->getPosition();
		float radius1 = blob[i]->getRadius();
		Vector2 velocity1 = blob[i]->getVelocity();
		float mass1 = blob[i]->getMass();

		for (int j = 0 + i; j < NUM_PARTICLES; j++)
		{
			Vector2 pos2 = blob[j]->getPosition();
			Vector2 velocity2 = blob[j]->getVelocity();
			float radius2 = blob[j]->getRadius();
			float mass2 = blob[j]->getMass();

			//distance from sphere 2 to sphere 1
			Vector2 sphereDistanceVec = pos1 - pos2;
			float distance = sphereDistanceVec.magnitude();

			//if length of sphereDistanceVec <= (radius1 + radius2), then spheres have collided or inter-penetrated
			if (distance <= radius1 + radius2)
			{
				//resolve inter-penetration by moving two spheres apart from each other by the distance they have inter-penetrated
				float interpenetrationDist = (radius1 + radius2) - distance;
				Vector2 interpenetrationVec = sphereDistanceVec;
				
				interpenetrationVec.normalise();
				interpenetrationVec *= interpenetrationDist;

				blob[i]->setPosition(blob[i]->getPosition() + interpenetrationVec * 0.5);
				blob[j]->setPosition(blob[j]->getPosition() - interpenetrationVec * 0.5);
				//resolve inter-penetration

				//normalise sphereDistanceVec to get the normal of sphere 2 at the point of contact
				Vector2 normal = sphereDistanceVec;
				normal.normalise();

				//multiply normal by velocity of sphere 1
				float x1 = normal * velocity1;

				//find perpendicular components of velocity
				Vector2 velocity1X = normal * x1;
				Vector2 velocity1Y = velocity1 - velocity1X;

				//reverse normal
				normal *= -1;

				//multiply normal by velocity of sphere 2
				float x2 = normal * velocity2;

				//find perpendicular components of velocity
				Vector2 velocity2X = normal * x2;
				Vector2 velocity2Y = velocity2 - velocity2X;

				//based on formula for calculating final velocities of two particles in an elastic collision
				blob[i]->setVelocity(((velocity1X * (mass1 - mass2)) / (mass1 + mass2) + (velocity2X * (2 * mass2)) / (mass1 + mass2) + velocity1Y) * 0.995);
				blob[j]->setVelocity(((velocity1X * (2 * mass1)) / (mass1 + mass2) + (velocity2X * (mass2 - mass1)) / (mass1 + mass2) + velocity2Y) * 0.995);
			}
		}
	}
	//collision detection and resolution

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