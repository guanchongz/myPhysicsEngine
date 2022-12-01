#pragma once

#include "math/base.hpp"
#include "structre/particle.hpp"
#include <memory>
#include <my.h>

namespace my {
class ParticleContact{
    public:
    std::shared_ptr<Particle> particle[2];
    real restitution;
    Vector3 contactNormal;

    protected:
    void resolve(real duration){

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
};

}