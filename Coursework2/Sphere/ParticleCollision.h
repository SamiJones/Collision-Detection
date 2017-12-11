#include "pcontacts.h"

#pragma once

class ParticleCollision : public ParticleContactGenerator
{
public:
	unsigned addContact(ParticleContact *contact, unsigned limit);
	bool checkCollision();
};