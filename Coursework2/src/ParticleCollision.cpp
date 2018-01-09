#include "pcontacts.h"
#include "ParticleCollision.h"
#include <gl/glut.h>

using namespace std;

ParticleCollision::ParticleCollision(int numParticles, Particle* arrayPtr) : NUM_PARTICLES(numParticles)
{
	particles = arrayPtr;
}

unsigned ParticleCollision::addContact(ParticleContact *contact, unsigned limit)
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

				if (particles[i].isSphere() && particles[j].isSphere())
					contact->penetration = (radius1 + radius2) - distance;
				else if (!particles[i].isSphere() || !particles[j].isSphere())
				{
					float interPenetrationDist = abs(MDBounds.left);

					if (abs(MDBounds.right) < interPenetrationDist)
						interPenetrationDist = abs(MDBounds.right);

					if (abs(MDBounds.bottom) < interPenetrationDist)
						interPenetrationDist = abs(MDBounds.bottom);

					if (abs(MDBounds.top) < interPenetrationDist)
						interPenetrationDist = abs(MDBounds.top);
						
					contact->penetration = interPenetrationDist;
				}

				used++;
				contact++;
			}
		}
	}

	return used;
}

bool ParticleCollision::checkCollision(Particle& particle1, Particle& particle2, float distance)
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
		MDBounds = calcMinkowskiDifferenceBounds(particle1, particle2);

		if (MDBounds.left <= 0 && MDBounds.right >= 0 && MDBounds.bottom <= 0 && MDBounds.top >= 0)
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

Boundary ParticleCollision::calcMinkowskiDifferenceBounds(Particle& particle1, Particle& particle2)
{
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

	//These boundary values will be overwritten if vertices with further extremes are found
	MDBounds.left = minkowskiDiffVertices[0].x;
	MDBounds.right = minkowskiDiffVertices[0].x;
	MDBounds.bottom = minkowskiDiffVertices[0].y;
	MDBounds.top = minkowskiDiffVertices[0].y;

	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
	{
		if (minkowskiDiffVertices[i].x < MDBounds.left)
			MDBounds.left = minkowskiDiffVertices[i].x;
		else if (minkowskiDiffVertices[i].x > MDBounds.right)
			MDBounds.right = minkowskiDiffVertices[i].x;
		
		if (minkowskiDiffVertices[i].y < MDBounds.bottom)
			MDBounds.bottom = minkowskiDiffVertices[i].y;
		else if (minkowskiDiffVertices[i].y > MDBounds.top)
			MDBounds.top = minkowskiDiffVertices[i].y;
	}


	/*glBegin(GL_POLYGON);
	for (int i = 0; i < minkowskiDiffVertices.size(); i++)
	glVertex2f(minkowskiDiffVertices[i].x, minkowskiDiffVertices[i].y);
	glEnd();
	glutSwapBuffers();*/

	/*glBegin(GL_POLYGON);
	glVertex2f(MDBounds.left, MDBounds.bottom);
	glVertex2f(MDBounds.right, MDBounds.bottom);
	glVertex2f(MDBounds.right, MDBounds.top);
	glVertex2f(MDBounds.left, MDBounds.top);
	glEnd();
	glutSwapBuffers();*/

	return MDBounds;
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