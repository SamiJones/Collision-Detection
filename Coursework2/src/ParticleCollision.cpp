#include "pcontacts.h"
#include "ParticleCollision.h"
#include <gl/glut.h>

using namespace std;

unsigned ParticleCollision::addContact(ParticleContact *contact, unsigned limit) const
{
	//const static float restitution = 1.0f;
	unsigned used = 0;

	for (int i = 0; i < NUM_PARTICLES; i++)
	{
		Vector2 pos1 = particles[i].getPosition();
		float radius1 = particles[i].getRadius();
		Vector2 velocity1 = particles[i].getVelocity();
		float mass1 = particles[i].getMass();

		for (int j = 0; j < NUM_PARTICLES; j++)
		{
			//Particle cannot collide with itself
			if (i == j)
				continue;

			Vector2 pos2 = (particles[j]).getPosition();
			Vector2 velocity2 = (particles[j]).getVelocity();
			float radius2 = (particles[j]).getRadius();
			float mass2 = (particles[j]).getMass();

			//Distance from sphere 2 to sphere 1
			Vector2 sphereDistanceVec = pos1 - pos2;
			float distance = sphereDistanceVec.magnitude();

			if (checkCollision(particles[i], particles[j], distance))
			{
				// We have a collision
				contact->contactNormal = sphereDistanceVec.unit();
				contact->restitution = restitution;
				contact->particle[0] = &particles[i];
				contact->particle[1] = &particles[j];

				//if (particles[i].isSphere() && particles[j].isSphere())
				contact->penetration = (radius1 + radius2) - distance;

				used++;
				contact++;
			}
		}
	}

	return used;
}

bool ParticleCollision::checkCollision(Particle& particle1, Particle& particle2, float distance) const
{
	//Simple check for collision between two spheres
	if (particle1.isSphere() && particle2.isSphere())
	{
		//If length of sphereDistanceVec <= (radius1 + radius2), then spheres have collided or inter-penetrated
		if (distance <= particle1.getRadius() + particle2.getRadius())
			return true;
		else
			return false;
	}
	//One of the shapes is a convex polygon, so check for collision using Minkowski difference
	else
	{
		Boundary MDBounds = calcMinkowskiDifferenceBounds(particle1, particle2);

		if (MDBounds.bottomLeft.x <= 0 && MDBounds.bottomLeft.y <= 0 && MDBounds.topRight.x >= 0 && MDBounds.topRight.y >= 0)
			return true;
		else
			return false;
	}
}

//Boundary ParticleCollision::calcMinkowskiDifferenceBounds(Particle& particle1, Particle& particle2) const
//{
//	//Far borders of the convex polygon created by the Minkowski difference
//	Boundary boundaryPoints;
//
//	vector<Vector2> particle1Vertices;
//	vector<Vector2> particle2Vertices;
//	vector<Vector2> minkowskiDiffVertices;
//
//	if (particle1.isSphere())
//		particle1Vertices = estimateSphereVertices(particle1);
//	else
//		particle1Vertices = particle1.getVertices();
//
//	if (particle2.isSphere())
//		particle2Vertices = estimateSphereVertices(particle2);
//	else
//		particle2Vertices = particle2.getVertices();
//
//	for (int i = 0; i < particle1Vertices.size(); i++)
//		for (int j = 0; j < particle2Vertices.size(); j++)
//			minkowskiDiffVertices.push_back((particle1Vertices[i] + particle1.getPosition()) - (particle2Vertices[j] + particle2.getPosition()));
//
//	//These values will be overwritten if vertices with further extremes are found
//	boundaryPoints.left = minkowskiDiffVertices[0];
//	boundaryPoints.right = minkowskiDiffVertices[0];
//	boundaryPoints.top = minkowskiDiffVertices[0];
//	boundaryPoints.bottom = minkowskiDiffVertices[0];
//
//	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
//	{
//		if (minkowskiDiffVertices[i].x < boundaryPoints.left.x)
//			boundaryPoints.left = minkowskiDiffVertices[i];
//		else if (minkowskiDiffVertices[i].x > boundaryPoints.right.x)
//			boundaryPoints.right = minkowskiDiffVertices[i];
//
//		if (minkowskiDiffVertices[i].y < boundaryPoints.bottom.y)
//			boundaryPoints.bottom = minkowskiDiffVertices[i];
//		else if (minkowskiDiffVertices[i].y > boundaryPoints.top.y)
//			boundaryPoints.top = minkowskiDiffVertices[i];
//	}
//
//	/*glBegin(GL_POLYGON);
//	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
//		glVertex2f(minkowskiDiffVertices[i].x, minkowskiDiffVertices[i].y);
//	glEnd();*/
//
//	/*glBegin(GL_POLYGON);
//	glVertex2f(boundaryPoints.left.x, boundaryPoints.left.y);
//	glVertex2f(boundaryPoints.top.x, boundaryPoints.top.y);
//	glVertex2f(boundaryPoints.right.x, boundaryPoints.right.y);
//	glVertex2f(boundaryPoints.bottom.x, boundaryPoints.bottom.y);
//	glEnd();*/
//
//	return boundaryPoints;
//}

Boundary ParticleCollision::calcMinkowskiDifferenceBounds(Particle& particle1, Particle& particle2) const
{
	//Far borders of the convex polygon created by the Minkowski difference
	Boundary boundaryPoints;

	vector<Vector2> particle1Vertices;
	vector<Vector2> particle2Vertices;
	vector<Vector2> minkowskiDiffVertices;

	if (particle1.isSphere())
		particle1Vertices = estimateSphereVertices(particle1);
	else
		particle1Vertices = particle1.getVertices();

	if (particle2.isSphere())
		particle2Vertices = estimateSphereVertices(particle2);
	else
		particle2Vertices = particle2.getVertices();

	for (int i = 0; i < particle1Vertices.size(); i++)
		for (int j = 0; j < particle2Vertices.size(); j++)
			minkowskiDiffVertices.push_back((particle1Vertices[i] + particle1.getPosition()) - (particle2Vertices[j] + particle2.getPosition()));

	//These values will be overwritten if vertices with further extremes are found
	boundaryPoints.bottomLeft = minkowskiDiffVertices[0];
	boundaryPoints.topRight = minkowskiDiffVertices[0];

	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
	{
		if ((minkowskiDiffVertices[i].x + minkowskiDiffVertices[i].y) < (boundaryPoints.bottomLeft.x + boundaryPoints.bottomLeft.y))
			boundaryPoints.bottomLeft = minkowskiDiffVertices[i];
		if ((minkowskiDiffVertices[i].x + minkowskiDiffVertices[i].y) > (boundaryPoints.topRight.x + boundaryPoints.topRight.y))
			boundaryPoints.topRight = minkowskiDiffVertices[i];
	}

	/*glBegin(GL_POLYGON);
	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
		glVertex2f(minkowskiDiffVertices[i].x, minkowskiDiffVertices[i].y);
	glEnd();
	glutSwapBuffers();*/

	/*glBegin(GL_POLYGON);
	glVertex2f(boundaryPoints.bottomLeft.x, boundaryPoints.bottomLeft.y);
	glVertex2f(boundaryPoints.bottomLeft.x, boundaryPoints.topRight.y);
	glVertex2f(boundaryPoints.topRight.x, boundaryPoints.topRight.y);
	glVertex2f(boundaryPoints.topRight.x, boundaryPoints.bottomLeft.y);
	glEnd();
	glutSwapBuffers();*/

	return boundaryPoints;
}

vector<Vector2> ParticleCollision::estimateSphereVertices(Particle& particle) const
{
	vector<Vector2> vertices;
	float radius = particle.getRadius();

	vertices.push_back(Vector2(-radius, 0));
	vertices.push_back(Vector2(radius, 0));
	vertices.push_back(Vector2(0, -radius));
	vertices.push_back(Vector2(0, -radius));

	vertices.push_back(Vector2(1, 1).unit() * radius);
	vertices.push_back(Vector2(-1, 1).unit() * radius);
	vertices.push_back(Vector2(-1, -1).unit() * radius);
	vertices.push_back(Vector2(1, -1).unit() * radius);

	return vertices;
}