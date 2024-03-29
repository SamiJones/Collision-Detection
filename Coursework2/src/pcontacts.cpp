#include <float.h>
#include <pcontacts.h>

// Contact implementation
void ParticleContact::resolve(float duration)
{
	resolveVelocity(duration);
}

float ParticleContact::calculateSeparatingVelocity() const
{
	Vector2 relativeVelocity = particle[0]->getVelocity();
	if (particle[1]) relativeVelocity -= particle[1]->getVelocity();
	return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(float duration)
{
	// Find the velocity in the direction of the contact
	float separatingVelocity = calculateSeparatingVelocity();

	// Check if it needs to be resolved
	if (separatingVelocity > 0.05)
	{
		// The contact is either separating, or stationary - there's
		// no impulse required.
		return;
	}

	// Calculate the new separating velocity
	float newSepVelocity = -separatingVelocity * restitution;
	float deltaVelocity = newSepVelocity - separatingVelocity;

	// We apply the change in velocity to each object in proportion to
	// their inverse mass (i.e. those with lower inverse mass [higher
	// actual mass] get less change in velocity)..
	float totalInverseMass = particle[0]->getInverseMass();
	if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

	// If all particles have infinite mass, then impulses have no effect
	if (totalInverseMass <= 0) return;

	// Calculate the impulse to apply
	float impulse = deltaVelocity / totalInverseMass;

	// Find the amount of impulse per unit of inverse mass
	Vector2 impulsePerIMass = contactNormal * impulse;

	// Apply impulses: they are applied in the direction of the contact,
	// and are proportional to the inverse mass.
	particle[0]->setVelocity(particle[0]->getVelocity() +
		impulsePerIMass * particle[0]->getInverseMass()
		);
	if (particle[1])
	{
		// Particle 1 goes in the opposite direction
		particle[1]->setVelocity(particle[1]->getVelocity() +
			impulsePerIMass * -particle[1]->getInverseMass()
			);
	}

	//resolve inter-penetration by moving two spheres apart from each other by the distance they have inter-penetrated
	if (particle[0] && particle[1])
	{
		if (penetration > 0)
		{
			Vector2 interpenetrationVec = particle[0]->getPosition() - particle[1]->getPosition();

			interpenetrationVec.normalise();
			interpenetrationVec *= penetration;

			particle[0]->setPosition(particle[0]->getPosition() + interpenetrationVec * 0.5);
			particle[1]->setPosition(particle[1]->getPosition() - interpenetrationVec * 0.5);
		}
	}
	//resolve inter-penetration
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
	:
	iterations(iterations)
{
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
	ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact *contactArray,
	unsigned numContacts,
	float duration)
{
	unsigned i;

	iterationsUsed = 0;
	while (iterationsUsed < iterations)
	{
		// Find the contact with the largest closing velocity;
		float max = DBL_MAX;
		unsigned maxIndex = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			float sepVel = contactArray[i].calculateSeparatingVelocity();
			if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}
		//Do we have anything worth resolving?
		if (maxIndex == numContacts) break;

		// Resolve this contact
		contactArray[maxIndex].resolve(duration);

		iterationsUsed++;
	}

}