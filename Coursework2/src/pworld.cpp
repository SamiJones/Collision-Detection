#include <cstdlib>
#include <pworld.h>

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
	:
	resolver(iterations),
	maxContacts(maxContacts)
{
	contacts = new ParticleContact[maxContacts];
	calculateIterations = (iterations == 0);

}

ParticleWorld::~ParticleWorld()
{
	delete[] contacts;
}

unsigned ParticleWorld::generateContacts()
{
	int limit = maxContacts;
	ParticleContact *nextContact = contacts;

	//generate contacts for particles
	unsigned used = particleContactGenerator[0]->addContact(nextContact, limit);
	limit -= used;
	nextContact += used;

	//If the max number of contacts has been exceeded, do not attempt to generate contacts for platforms,
	//and return the number of contacts used (all of them)
	if (limit <= 0)
		return maxContacts;

	//generate contacts for platforms
	for (ContactGenerators::iterator g = platformContactGenerators.begin(); g != platformContactGenerators.end(); g++)
	{
		used = (*g)->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		// We've run out of contacts to fill. This means we're missing
		// contacts.
		if (limit <= 0)
			break;
	}

	// Return the number of contacts used.
	return maxContacts - limit;
}

void ParticleWorld::integrate(float duration)
{
	for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
	{
		// Remove all forces from the accumulator
		(*p)->integrate(duration);
	}
}

void ParticleWorld::runPhysics(float duration)
{
	//Apply drag force to every particle
	for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
	{
		float radius = (*p)->getRadius();
		Vector2 velocity = (*p)->getVelocity();
		float magnitude = velocity.magnitude();

		float k2 = 0.1 * radius * radius;
		float dragCoeff = k2 * magnitude * magnitude;

		Vector2 dragForce = velocity;
		dragForce *= -dragCoeff;
		dragForce.normalise();
		dragForce *= 50; //normalised drag force is too small to produce much effect, so scale it up
		(*p)->addForce(dragForce);
	}

	// Then integrate the objects
	integrate(duration);

	// Generate contacts
	unsigned usedContacts = generateContacts();

	// And process them
	if (usedContacts)
	{
		if (calculateIterations) resolver.setIterations(usedContacts * 2);
		resolver.resolveContacts(contacts, usedContacts, duration);
	}
}

ParticleWorld::Particles& ParticleWorld::getParticles()
{
	return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getPlatformContactGenerators()
{
	return platformContactGenerators;
}

ParticleWorld::ContactGenerators& ParticleWorld::getParticleContactGenerator()
{
	return particleContactGenerator;
}