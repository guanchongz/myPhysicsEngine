#include "math/base.hpp"
#include "math/precision.hpp"
#include "structre/particle.hpp"
#include "structre/particle_force.hpp"
#include "structre/pcontacts.hpp"
#include <cstddef>
#include <memory>
#include <my.h>
#include <vector>

#define BLOB_COUNT 5
#define PLATFORM_COUNT 10
#define BLOB_RADIUS 0.4f

class Platform : public my::ParticleContactGenerator{
    public:
    my::Vector3 start;
    my::Vector3 end;
    std::vector<std::shared_ptr<my::Particle>> particles;

    virtual void addContact(std::vector<std::shared_ptr<my::ParticleContact>> contacts) {
        my::real restitution = 0.0f;
        for (auto particle : particles){
            auto toParticle = particle->getPosition() - start;
            auto lineDirection = end - start;
            auto projected = toParticle * lineDirection;
            auto platformSqLength = lineDirection.squareMagnitude();
            if (projected <= 0.0){
                if (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS){
                    auto contact = std::make_shared<my::ParticleContact>();
                    contact->contactNormal = toParticle.unit();
                    contact->contactNormal.z = 0;
                    contact->restitution = restitution;
                    contact->particle[0] = particle;
                    contact->particle[1] = nullptr;
                    contact->penetration = BLOB_RADIUS - toParticle.magnitude();
                    contacts.push_back(contact);
                }
            }
            else if (projected >= platformSqLength) {
                toParticle = particle->getPosition() - end;
                if (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS){
                    auto contact = std::make_shared<my::ParticleContact>();
                    contact->contactNormal = toParticle.unit();
                    contact->contactNormal.z = 0;
                    contact->restitution = restitution;
                    contact->particle[0] = particle;
                    contact->particle[1] = nullptr;
                    contact->penetration = BLOB_RADIUS - toParticle.magnitude();
                    contacts.push_back(contact);
                }
            }
            else{
                auto distanceToPlatform = toParticle.squareMagnitude() - projected * projected / lineDirection.squareMagnitude();
                if (distanceToPlatform < BLOB_RADIUS * BLOB_RADIUS){
                    auto closestPoint = start + lineDirection * (projected / platformSqLength);
                    auto contact = std::make_shared<my::ParticleContact>();
                    contact->contactNormal = (particle->getPosition() - closestPoint).unit();
                    contact->contactNormal.z = 0;
                    contact->restitution = restitution;
                    contact->particle[0] = particle;
                    contact->particle[1] = nullptr;
                    contact->penetration = BLOB_RADIUS - real_sqrt(distanceToPlatform);
                    contacts.push_back(contact);
                }
            }
            if (contacts.size() == contacts.capacity()) break;
        }
    }
};

class BlobForceGenerator : public my::ParticleForceGenerator{
    public:
    unsigned maxFloat;
    my::real maxReplusion;
    my::real maxAttraction;
    my::real minNaturalDistance;
    my::real maxNaturalDistance;
    my::real floatHead;
    my::real maxDistance; 
    std::vector<std::shared_ptr<my::Particle>> particles;

    virtual void updateForce(std::shared_ptr<my::Particle> aimParticle, my::real duration){
        unsigned joincount = 0;
        for (auto particle : particles){
            if (particle == aimParticle) continue;

            auto separation = particle->getPosition() - aimParticle->getPosition();
            separation.z = 0.0f;
            auto distance = separation.magnitude();

            if (distance < minNaturalDistance){
                distance = 1.0f - distance / minNaturalDistance;
                aimParticle->addForce(separation.unit() * (1.0f - distance) * maxReplusion * -1.0f);
                joincount ++;
            } else if (distance > maxNaturalDistance && distance < maxDistance){
                distance = (distance - maxNaturalDistance) / (maxDistance - maxNaturalDistance);
                aimParticle->addForce(separation.unit() * distance * maxAttraction);
                joincount ++;
            }
        }

        if (aimParticle == particles.front() && joincount > 0 && maxFloat > 0){
            my::real force = my::real(float(joincount) / maxFloat) * floatHead;
            if (force > floatHead) force = floatHead;
            aimParticle->addForce(my::Vector3(0, force, 0));
        }
    }
};