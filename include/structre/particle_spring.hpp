#pragma once

#include "math/base.hpp"
#include "math/precision.hpp"
#include "structre/particle.hpp"
#include <memory>
#include <my.h>
#include <structre/particle_force.hpp>

namespace my{
class ParticleSpring : public ParticleForceGenerator{
    std::shared_ptr<Particle> other;   
    real springConstant;
    real restLength;
    real minLength;
    real MaxLength;
    
    public:
    ParticleSpring(std::shared_ptr<Particle> other, real springConstant, real restLength, real minLength = 0.0f, real MaxLength = REAL_MAX) : other(other), springConstant(springConstant), restLength(restLength), minLength(minLength), MaxLength(MaxLength) {}
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration) override{
        Vector3 force;
        particle->getPosition(&force);
        force -= other->getPosition();

        real magnitude = force.magnitude();
        if (magnitude <= minLength or magnitude >= MaxLength) return;
        magnitude = restLength - magnitude;
        magnitude *= springConstant;

        force.normalize();
        force *= magnitude;
        particle->addForce(force);
    }
};

class ParticleBuoyancy : public ParticleForceGenerator{
    real waterHeight;
    real volume;
    real maxDepth;
    real liquidDensity;
    
    public:
    ParticleBuoyancy(real volume, real waterHeight, real maxDepth = REAL_MAX, real liquidDensity = 1000.0f) : maxDepth(maxDepth), volume(volume), waterHeight(waterHeight), liquidDensity(liquidDensity) {}
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration){
        real depth = particle->getPosition().y;
        if (depth >= waterHeight ) return;

        Vector3 force(0,0,0);

        if (depth <= waterHeight - maxDepth){
            force.y = liquidDensity * volume;
            particle->addForce(force);
            return;
        }

        force.y = liquidDensity * volume * (waterHeight - depth - maxDepth ) / maxDepth;
        particle->addForce(force);
    } 
};

class ParticleRealSpring : public ParticleForceGenerator{
    std::shared_ptr<Vector3> anchor;
    real springConstant;
    real damping;

    public:
    ParticleRealSpring(std::shared_ptr<Particle> anchor, real springConstant, real damping) : anchor(anchor), springConstant(springConstant), damping(damping) {}
    virtual void updateForce(std::shared_ptr<Particle> particle, real duration){
        if (!particle->hasFiniteMass()) return;

        Vector3 position;
        particle->getPosition(&position);
        position -= *anchor;

        real gamma = 0.5f * real_sqrt(4 * springConstant - damping*damping);
        if (gamma == 0.0f) return;
        Vector3 c = position * (damping / (2.0f * gamma)) +
            particle->getVelocity() * (1.0f / gamma);

        Vector3 target = position * real_cos(gamma * duration) +
            c * real_sin(gamma * duration);
        target *= real_exp(-0.5f * duration * damping);

        Vector3 accel = (target - position) * ((real)1.0 / (duration*duration)) -
            particle->getVelocity() * ((real)1.0/duration);
        particle->addForce(accel * particle->getMass());
    }
};
}
