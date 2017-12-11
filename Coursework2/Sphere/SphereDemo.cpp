// SphereDemo.cpp
#include <gl/glut.h>// OpenGL toolkit
#include "App.h"
#include "SphereDemo.h"
#include "CoreMath.h"
#include <cmath>

SphereDemo::SphereDemo()
{
	particle[0] = new Particle();
	particle[0]->setMass(1);
	particle[0]->setRadius(10);
	particle[0]->setPosition(0, 0);
	particle[0]->setVelocity(50, 50);
	particle[0]->setAcceleration(0, 0);

	particle[1] = new Particle();
	particle[1]->setMass(30);
	particle[1]->setRadius(20);
	particle[1]->setPosition(40, 40);
	particle[1]->setVelocity(50, 50);
	particle[1]->setAcceleration(0, 0);

	xStep = 2;
	yStep = 2;

	width = 600;
	height = 600;
}

SphereDemo::~SphereDemo()
{
	for (int i = 0; i < NUM_PARTICLES; i++)
		delete particle[i];
}

void SphereDemo::update(void)
{
	for (int i = 0; i < 2; i++)
	{
		float radius = particle[i]->getRadius();
		float duration = timeInterval / 1000;

		float k1 = 0;
		float k2 = 0.01 * radius * radius;

		Vector2 position = particle[i]->getPosition();
		Vector2 velocity = particle[i]->getVelocity();

		float dragCoeff = k2 * velocity.magnitude() * velocity.magnitude();

		Vector2 dragForce = velocity;
		dragForce *= -dragCoeff;
		dragForce.normalise();

		/*float drag = -k1 * velocity.magnitude() - k2 * velocity.magnitude() * velocity.magnitude();
		Vector2 dragForce = Vector2(drag, drag);
		dragForce.normalise();*/

		particle[i]->addForce(Vector2(0, -g * particle[i]->getMass()));
		particle[i]->addForce(dragForce);

		particle[i]->integrate(duration);

		box_collision_resolve(particle[i]);

		if (out_of_box_test(particle[i]))
			out_of_box_resolve(particle[i]);
	}

	//simple collision detection between the two particles
	Vector2 pos1 = particle[0]->getPosition();
	Vector2 pos2 = particle[1]->getPosition();

	float radius1 = particle[0]->getRadius();
	float radius2 = particle[1]->getRadius();

	float dist = sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) + (pos1.y - pos2.y) * (pos1.y - pos2.y));

	if (dist <= particle[0]->getRadius() + particle[1]->getRadius())
	{
		particle[0]->setVelocity(-particle[0]->getVelocity().x, -particle[0]->getVelocity().y);
		particle[1]->setVelocity(-particle[1]->getVelocity().x, -particle[1]->getVelocity().y);
	}
	//collision detection

	Application::update();
}

void SphereDemo::display(void)
{
	Application::display();

	glLoadIdentity();

	Vector2 position = particle[0]->getPosition();

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glColor3ub(255, 0, 0);
	glutSolidSphere(particle[0]->getRadius(), 30, 30);
	glPopMatrix();

	position = particle[1]->getPosition();

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glColor3ub(0, 255, 0);
	glutSolidSphere(particle[1]->getRadius(), 30, 30);
	glPopMatrix();

	glutSwapBuffers();
}

// detect if the particle colided with the box and produce a response
void SphereDemo::box_collision_resolve(Particle* particle)
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

	////reverse direction when you reach left or right edge
	//if (position.x >= Application::width - radius)
	//{
	//	particle.setVelocity(-abs(velocity.x), velocity.y);
	//	particle.setPosition(Application::width - radius - 1, particle.getPosition().y);
	//}
	//else if (position.x <= -Application::width + radius)
	//{
	//	particle.setVelocity(abs(velocity.x), velocity.y);
	//	particle.setPosition(-Application::width + radius + 1, particle.getPosition().y);
	//}
	//
	////reverse direction when you reach top or bottom edge
	//if (position.y >= Application::height - radius)
	//{
	//	particle.setVelocity(velocity.x, -abs(velocity.y));
	//	particle.setPosition(particle.getPosition().x, Application::height - radius - 1);
	//}
	//else if (position.y <= -Application::height + radius)
	//{
	//	particle.setVelocity(velocity.x, abs(velocity.y));
	//	particle.setPosition(particle.getPosition().x, -Application::height + radius + 1);
	//}
}

//  Check bounds. This is in case the window is made
//  smaller while the sphere is bouncing and the 
//  sphere suddenly finds itself outside the new
//  clipping volume
bool SphereDemo::out_of_box_test(Particle* particle)
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
void SphereDemo::out_of_box_resolve(Particle* particle)
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

Application* getApplication()
{
	return new SphereDemo();
}
