#include "gl/glut.h"
#include "structre/particle.hpp"
#include "structre/particle_force.hpp"
#include "structre/particle_world.hpp"
#include "structre/pcontacts.hpp"
#include <GL/glu.h>
#include <cstddef>
#include <memory>
#include <my.h>
#include <type_traits>
#include <iostream>

#define BLOB_COUNT 5
#define PLATFORM_COUNT 10
#define BLOB_RADIUS 0.4f

class Platform : public my::ParticleContactGenerator{
    public:
    my::Vector3 start;
    my::Vector3 end;
    std::vector<std::shared_ptr<my::Particle>> particles;

    virtual void addContact(std::vector<std::shared_ptr<my::ParticleContact>> &contacts){
        my::real restitution = 1.0f;
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

class BlobDemo : public Application{
    float xAxis;
    float yAxis;

    std::shared_ptr<BlobForceGenerator> blobForceGenerator;
    std::vector<std::shared_ptr<my::Particle>> blobs;
    std::vector<std::shared_ptr<Platform>> platforms;
    my::ParticleWorld world;

    void reset(){
        my::Random r;
        std::shared_ptr<Platform> p = platforms[PLATFORM_COUNT - 2];
        my::real fraction = (my::real) 1.0 / BLOB_COUNT;
        my::Vector3 delta = p->end - p->start;

        auto count = blobs.size();
        for (auto i = 0; i<count; i++){
            unsigned me = (i + BLOB_COUNT / 2) % BLOB_COUNT;
            blobs[i]->setPosition(p->start + delta * (my::real(me) * 0.8f * fraction + 0.1f ) + my::Vector3(0, 1.0f + r.randomReal(), 0));
            blobs[i]->setVelocity(0, 0, 0);
            blobs[i]->clearAccumulator();
        }
    }

    public:
    BlobDemo() : xAxis(0.0f), yAxis(0.0f), world(PLATFORM_COUNT + BLOB_COUNT){
        // Create the blob storage
        for (auto i = 0; i < BLOB_COUNT; i++){
            blobs.push_back(std::make_shared<my::Particle> ());
        }

        my::Random r;
    
        // Create the platforms
        for (unsigned i = 0; i < PLATFORM_COUNT; i++)
        {
            auto platform = std::make_shared<Platform>();
            platform->start = my::Vector3(
                my::real(i%2)*10.0f - 5.0f,
                my::real(i)*4.0f + ((i%2)?0.0f:2.0f),
                0);
            platform->start.x += r.randomBinomial(2.0f);
            platform->start.y += r.randomBinomial(2.0f);
    
            platform->end = my::Vector3(
                my::real(i%2)*10.0f + 5.0f,
                my::real(i)*4.0f + ((i%2)?2.0f:0.0f),
                0);
            platform->end.x += r.randomBinomial(2.0f);
            platform->end.y += r.randomBinomial(2.0f);
    
            // Make sure the platform knows which particles it 
            // should collide with.
            platform->particles = blobs;
            world.getContactGenerators()->push_back(platform);
            platforms.push_back(platform);
        }
    
        // Create the force generator
        blobForceGenerator = std::make_shared<BlobForceGenerator> ();
        blobForceGenerator->particles = blobs;
        blobForceGenerator->maxAttraction = 20.0f;
        blobForceGenerator->maxReplusion = 10.0f;
        blobForceGenerator->minNaturalDistance = BLOB_RADIUS*0.75f;
        blobForceGenerator->maxNaturalDistance = BLOB_RADIUS*1.5f;
        blobForceGenerator->maxDistance = BLOB_RADIUS * 2.5f;
        blobForceGenerator->maxFloat = 2;
        blobForceGenerator->floatHead = 8.0f;
 
        // Create the blobs.
        std::shared_ptr<Platform> p = platforms[PLATFORM_COUNT - 2];
        my::real fraction = (my::real)1.0 / BLOB_COUNT;
        my::Vector3 delta = p->end - p->start;
        for (unsigned i = 0; i < BLOB_COUNT; i++)
        {
            unsigned me = (i+BLOB_COUNT/2) % BLOB_COUNT;
            blobs[i]->setPosition(
                p->start + delta * (my::real(me)*0.8f*fraction+0.1f) +
                my::Vector3(0, 1.0f+r.randomReal(), 0));

            auto g = my::GRAVITY;
            blobs[i]->setVelocity(0,0,0);
            blobs[i]->setDamping(0.2f);
            blobs[i]->setAcceleration(g * my::real(0.4f));
            blobs[i]->setMass(1.0f);
            blobs[i]->clearAccumulator();

            std::shared_ptr<my::Particle> tmp = blobs[i];
            world.getParticles()->push_back(tmp);
            world.getForceRegistry()->addRegistration(tmp, blobForceGenerator);
        }
    }

    void display() override{
        my::Vector3 pos = blobs[0]->getPosition();
    
        // Clear the view port and set the camera direction
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        gluLookAt(pos.x, pos.y, 6.0,  pos.x, pos.y, 0.0,  0.0, 1.0, 0.0);
    
        glColor3f(0,0,0);
    
    
        glBegin(GL_LINES);
        glColor3f(0,0,1);
        for (auto platform : platforms)
        {
            const my::Vector3 &p0 = platform->start;
            const my::Vector3 &p1 = platform->end;
            glVertex3f(p0.x, p0.y, p0.z);
            glVertex3f(p1.x, p1.y, p1.z);
        }
        glEnd();
    
        glColor3f(1,0,0);
        for (auto blob : blobs)
        {
            const my::Vector3 &p = blob->getPosition();
            glPushMatrix();
            glTranslatef(p.x, p.y, p.z);
            glutSolidSphere(BLOB_RADIUS, 12, 12);
            glPopMatrix();
        }
        
        my::Vector3 p = blobs[0]->getPosition();
        my::Vector3 v = blobs[0]->getVelocity() * 0.05f;
        v.trim(BLOB_RADIUS*0.5f);
        p = p + v;
        glPushMatrix();
        glTranslatef(p.x-BLOB_RADIUS*0.2f, p.y, BLOB_RADIUS);
        glColor3f(1,1,1);
        glutSolidSphere(BLOB_RADIUS*0.2f, 8, 8);
        glTranslatef(0,0,BLOB_RADIUS*0.2f);
        glColor3f(0,0,0);
        glutSolidSphere(BLOB_RADIUS*0.1f, 8, 8);
        glTranslatef(BLOB_RADIUS*0.4f, 0, -BLOB_RADIUS*0.2f);
        glColor3f(1,1,1);
        glutSolidSphere(BLOB_RADIUS*0.2f, 8, 8);
        glTranslatef(0,0,BLOB_RADIUS*0.2f);
        glColor3f(0,0,0);
        glutSolidSphere(BLOB_RADIUS*0.1f, 8, 8);
        glPopMatrix();
    }

    void update() override{
        // Clear accumulators
        world.startFrame();
    
        // Find the duration of the last frame in seconds
        float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
        //auto duration = 0.4f * 0.001f;
        if (duration <= 0.0f) return;
    
        // Recenter the axes
        xAxis *= pow(0.1f, duration);
        yAxis *= pow(0.1f, duration);
    
        // Move the controlled blob
        blobs[0]->addForce(my::Vector3(xAxis, yAxis, 0)*10.0f);
    
        // Run the simulation
        world.runPhysics(duration);
    
        // Bring all the particles back to 2d
        my::Vector3 position;
        for (auto blob : blobs)
        {
            blob->getPosition(&position);
            position.z = 0.0f;
            blob->setPosition(position);
        }
    
        Application::update();
    }

    const char* getTitle() override{
        return "Blob Demo";
    }

    void key(unsigned char key) override{
        switch(key)
        {
        case 'w': case 'W':
            yAxis = 1.0;
            break;
        case 's': case 'S':
            yAxis = -1.0;
            break;
        case 'a': case 'A':
            xAxis = -1.0f;
            break;
        case 'd': case 'D':
            xAxis = 1.0f;
            break;
        case 'r': case 'R':
            reset();
            break;
        } 
    }
};

auto getApplication()
{
    return std::make_shared<BlobDemo>();
}