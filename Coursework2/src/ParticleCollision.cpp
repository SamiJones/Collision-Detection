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
					Vector2 closestPoint = MDVertices[0];
					float interPenetrationDist = 100;

					//Find closest point of Minkowski difference to origin and calculate inter-penetration from it
					for (int i = 1; i < MDVertices.size(); i++)
					{
						float temp = (Vector2(0, 0) - MDVertices[i]).magnitude();

						if (temp < interPenetrationDist)
							interPenetrationDist = temp;
					}

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
		//Calculate vertices of Minkowski difference between two particles
		MDVertices = calcMinkowskiDifferenceVertices(particle1, particle2);

		//Check if Minkowski difference contains the origin or not
		bool collision = polygonContainsOrigin(MDVertices);
		
		return collision;
	}
}

/*
Code used in this function to detect whether the Minkowski difference contains origin is based on
code written by W Randolph Franklin. See https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html.
Method is based on projecting a ray through the polygon from the origin and working out
whether it is inside or outside the polygon by the number of edges it intersects.
*/
bool ParticleCollision::polygonContainsOrigin(vector<Vector2>& vertices)
{
	bool collision = false;
	
	for (int i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++)
	{
		if (((vertices[i].y>0) != (vertices[j].y>0)) &&
			(0 < (vertices[j].x - vertices[i].x) * (0 - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x))
			collision = !collision;
	}

	return collision;
}

vector<Vector2> ParticleCollision::calcMinkowskiDifferenceVertices(Particle& particle1, Particle& particle2)
{
	vector<Vector2> particle1Vertices;
	vector<Vector2> particle2Vertices;
	vector<Vector2> minkowskiDiffVertices;

	//Populate particle1Vertices vector. If particle1 == sphere, estimate vertices. Else, retrieve particle1's stored vertices
	if (particle1.isSphere())
		particle1Vertices = estimateSphereVertices(particle1);
	else
		particle1Vertices = particle1.getVertices();

	//Populate particle2Vertices vector. If particle2 == sphere, estimate vertices. Else, retrieve particle2's stored vertices
	if (particle2.isSphere())
		particle2Vertices = estimateSphereVertices(particle2);
	else
		particle2Vertices = particle2.getVertices();

	//Populate vector of Minkowski difference vertices with particle 1 vertices - particle 2 vertices
	for (int i = 0; i < particle1Vertices.size(); i++)
		for (int j = 0; j < particle2Vertices.size(); j++)
			minkowskiDiffVertices.push_back((particle1Vertices[i] + particle1.getPosition()) - (particle2Vertices[j] + particle2.getPosition()));

	return minkowskiDiffVertices;
}

//calculate 8 of the vertices of a sphere particle (cannot calculate all, since a circle has infinite vertices)
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