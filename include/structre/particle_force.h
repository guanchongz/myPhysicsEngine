#pragma once

#include "math/base.h"
#include "structre/particle.h"
#include <memory>
#include <my.h>
#include <vector>

namespace my{
class ParticleForceGenerator{
    public:
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration) = 0;
};

class ParticleForceRegistry{
    protected:
    struct ParticleForceRegistration{
        std::shared_ptr<Particle> particle;
        std::shared_ptr<ParticleForceGenerator> fg;
    };
    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

    public:
    void addRegistration(std::shared_ptr<Particle> particle, std::shared_ptr<ParticleForceGenerator> fg){
        ParticleForceRegistration new_registration;
        new_registration.particle = particle;
        new_registration.fg = fg;
        registrations.push_back(new_registration);
    }

    void updateForces(real duration){
        for (auto registration : registrations){
            registration.fg->updateForce(registration.particle, duration);
        }
    }

    void clear(){
        registrations.clear();
    }
};

class ParticleGravity : public ParticleForceGenerator{
    Vector3 gravity;
    
    public:
    ParticleGravity(const Vector3 &grav) : gravity(grav){}
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration) override{
        if(particle->hasFiniteMass()){
            particle->addForce(gravity * particle->getMass());
        }
    }
};

class ParticleDrag : public ParticleForceGenerator{
    real k1;
    real k2;
    
    public:
    ParticleDrag(real k1, real k2) : k1(k1), k2(k2){}
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration){
        Vector3 force;
        particle->getVelocity(&force);

        real dragCoeff = force.magnitude();
        dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

        force.normalize();
        force *= -dragCoeff;
        particle->addForce(force);
    }
};
}