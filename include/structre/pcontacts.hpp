#pragma once

#include "math/base.hpp"
#include "math/precision.hpp"
#include "structre/particle.hpp"
#include <memory>
#include <my.h>
#include <vector>

namespace my {
class ParticleContactResolver;
class ParticleContact{
    friend class ParticleContactResolver;

    public:
    real restitution;
    real penetration;
    Vector3 contactNormal;
    std::shared_ptr<Particle> particle[2];

    protected:
    void resolve(real duration){
        resolveVelocity(duration);
        resolveInterpenetration(duration);
    }

    real calculateSeparatingVelocity() const{
        Vector3 relativeVelocity = particle[0]->getVelocity();
        if(particle[1]) relativeVelocity -= particle[1]->getVelocity();
        return relativeVelocity * contactNormal;
    }    

    private:
    void resolveVelocity(real duration){
        real separatingVelocity = calculateSeparatingVelocity();
        if (separatingVelocity > 0) return;
        
        real newSepVelocity = - separatingVelocity * restitution;

        Vector3 accCausedVelocity = particle[0]->getAcceleration();
        real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;
        if (accCausedSepVelocity < 0){
            newSepVelocity += restitution * accCausedSepVelocity;
            if (newSepVelocity < 0) newSepVelocity = 0;
        }

        real deltaVelocity = newSepVelocity - separatingVelocity;

        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

        if (totalInverseMass <= 0) return;

        real impulse = deltaVelocity / totalInverseMass;

        Vector3 impulsePerIMass = contactNormal * impulse;

        particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());
        if (particle[1]) {
            particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass());
        }
    }

    void resolveInterpenetration(real duration){
        if (penetration <= 0) return;

        real totalInverseMass = particle[0]->getInverseMass();
        if (particle[1]) totalInverseMass += particle[1]->getInverseMass();

        if (totalInverseMass <= 0) return;

        Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);
        particle[0]->setPosition(particle[0]->getPosition() + movePerIMass * particle[0]->getInverseMass());
        if (particle[1]){
            particle[1]->setPosition(particle[1]->getPosition() - movePerIMass * particle[1]->getInverseMass());
        }
    }
};

class ParticleContactResolver{
    protected:
    unsigned iterations;
    unsigned iterationsUsed;

    public:
    ParticleContactResolver(unsigned iterations): iterations(iterations){}
    void setIterations(unsigned iterations) {
        ParticleContactResolver::iterations = iterations;
    }

    void resolveContacts(std::vector<std::shared_ptr<ParticleContact>> &contactArray, real duration){
        iterationsUsed = 0;
        while(iterationsUsed < iterations){
            auto max = REAL_MAX;
            auto maxIndex = contactArray.size();
            auto length = contactArray.size();
            for (auto i = 0; i< length; i++){
                real sepVel = contactArray[i]->calculateSeparatingVelocity();
                if (sepVel < max){
                    max = sepVel;
                    maxIndex = i;
                }
            }
            contactArray[maxIndex]->resolve(duration);
            iterationsUsed ++;
        }
    }

};

class ParticleContactGenerator{
    public:
    virtual void addContact(std::vector<std::shared_ptr<ParticleContact>> &contacts) = 0;
};

class GroundContacts : public ParticleContactGenerator{
    std::vector<std::shared_ptr<Particle>> particles;

    public:
    void init(std::vector<std::shared_ptr<Particle>> &particles) {
        particles = particles;
    }

    virtual void addContact(std::vector<std::shared_ptr<ParticleContact>> &contacts){
        for(auto particle : particles){
            auto y = particle->getPosition().y;
            if (y < 0.0f){
                auto contact = std::make_shared<ParticleContact> ();
                contact->contactNormal = UP;
                contact->particle[0] = particle;
                contact->particle[1] = nullptr;
                contact->penetration = -y;
                contact->restitution = 0.2f;
                contacts.push_back(contact);
            }
            if (contacts.size() == contacts.capacity()) break;
        }
    }
};
}