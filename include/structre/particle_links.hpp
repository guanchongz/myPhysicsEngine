#pragma once

#include "math/base.hpp"
#include "structre/particle.hpp"
#include "structre/pcontacts.hpp"
#include <memory>
#include <my.h>

namespace my{
class ParticleLink{
    public:
    std::shared_ptr<Particle> particle[2];

    protected:
    real currentLength() const{
        Vector3 relativePos = particle[0]->getPosition() - particle[1]->getPosition();
        return relativePos.magnitude();
    }

    private:
    virtual unsigned fillContact(std::shared_ptr<ParticleContact> contact, unsigned limit) const = 0;
};

class ParticleCable : public ParticleLink{
    public:
    real maxLength;
    real restitution;
    
    public:
    virtual unsigned fillContact(std::shared_ptr<ParticleContact> contact, unsigned limit) const{
        auto length = currentLength();
        if (length < maxLength) return 0;

        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();
        contact->contactNormal = normal;

        contact->penetration = length - maxLength;
        contact->restitution = restitution;

        return 1;
    }
};

class ParticleRod : public ParticleLink{
    public:
    real length;
    
    public:
    virtual unsigned fillContact(std::shared_ptr<ParticleContact> contact, unsigned limit) const{
        auto curlength = currentLength();
        if (curlength == length) return 0;

        contact->particle[0] = particle[0];
        contact->particle[1] = particle[1];

        Vector3 normal = particle[1]->getPosition() - particle[0]->getPosition();
        normal.normalize();

        if (curlength > length){
            contact->contactNormal = normal;
            contact->penetration = curlength - length;
        }else{
            contact->contactNormal = normal * -1;
            contact->penetration = curlength - length;
        }

        contact->restitution = 0;
        return 1;
    }
};
}