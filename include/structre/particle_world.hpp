#pragma once

#include "structre/particle.hpp"
#include "structre/particle_force.hpp"
#include "structre/pcontacts.hpp"
#include <GL/gl.h>
#include <memory>
#include <my.h>
#include <vector>

namespace my {
class ParticleWorld{
    protected:
    bool calculateIterations;
    unsigned maxContacts;
    std::vector<std::shared_ptr<ParticleContact>> contacts;
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<std::shared_ptr<ParticleContactGenerator>> contactGenerators;
    ParticleForceRegistry registry;
    ParticleContactResolver resolver;

    public:
    ParticleWorld(unsigned maxContacts, unsigned iterations=0) : resolver(iterations), maxContacts(maxContacts){
        contacts.reserve(maxContacts);
        calculateIterations = (iterations == 0);
    }

    void startFrame(){
        for(auto particle : particles){
            particle->clearAccumulator();
        }
    }

    unsigned generateContacts(){
        auto cur_size = contacts.size();
        for (auto contact_generator : contactGenerators){
            contact_generator->addContact(contacts);
            if (contacts.size() == contacts.capacity()) break;
        }
        return contacts.size() - cur_size;
    }

    void integrate(real duration){
        for (auto particle : particles){
            particle->integrate(duration);
        }
    }

    void runPhysics(real duration){
        registry.updateForces(duration);
        integrate(duration);
        unsigned used_contacts = generateContacts();
        if (used_contacts){
            if (calculateIterations) resolver.setIterations(used_contacts * 2);
            resolver.resolveContacts(contacts, duration);
        }
        contacts.clear();
    }

    auto getParticles(){
        return &particles;
    }

    auto getContactGenerators(){
        return &contactGenerators;
    }

    auto getForceRegistry(){
        return &registry;
    }

};

}