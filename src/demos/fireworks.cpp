#include "module/app.h"
#include "module/timing.h"
#include <GL/gl.h>
#include <memory>

#include <gl/glut.h>
#include <my.h>
#include <stdexcept>

static my::Random crandom;

class Firework : public my::Particle{
    public:
        unsigned type;
        my::real age;
        bool update(my::real duration){
            integrate(duration);
            age -= duration;
            return (age < 0) || (position.y < 0);
        }

};

struct FireworkRule{
    unsigned type;
    my::real minAge;
    my::real maxAge;
    my::Vector3 minVelocity;
    my::Vector3 maxVelocity;
    my::real damping;

    struct Payload{
        unsigned type;
        unsigned count;
        void set(unsigned type, unsigned count){
            Payload::type = type;
            Payload::count = count;
        }
    };
    unsigned payloadCount;
    std::unique_ptr<Payload[]> payloads;
    
    FireworkRule() : payloadCount(0), payloads(nullptr){}

    void init(unsigned payload_count){
        payloadCount = payload_count;
        payloads = std::unique_ptr<Payload[]> (new Payload[payloadCount]);
    }

    void setParameters(unsigned type, my::real minAge, my::real maxAge, const my::Vector3& minVelocity, const my::Vector3 maxVelocity, my::real damping){
        FireworkRule::type = type;
        FireworkRule::minAge = minAge;
        FireworkRule::maxAge = maxAge;
        FireworkRule::minVelocity = minVelocity;
        FireworkRule::maxVelocity = maxVelocity;
        FireworkRule::damping = damping;
    }

    void create(Firework* firework, const Firework* parent = nullptr) const{
        firework->type = type;
        firework->age = crandom.randomReal(minAge, maxAge);

        if(parent){
            firework->setPosition(parent->getPosition());
            firework->setVelocity(parent->getVelocity());
        }
        else{
            my::Vector3 start;
            int x = (int)crandom.randomInt(3) - 1;
            start.x = 5.0f * my::real(x);
            firework->setPosition(start);
        }

        firework->addVelocity(crandom.randomVector(minVelocity, maxVelocity));
        firework->setMass(1);
        firework->setDamping(damping);
        firework->setAcceleration(my::GRAVITY);
    }
};

class FireworkDemo : public Application{
    const static unsigned maxFireworks = 1024;
    unsigned nextUseFirework;
    const static unsigned rulecount = 9;
    std::unique_ptr<Firework[]> fireworks; 
    std::unique_ptr<FireworkRule[]> rules;
    
    void create(unsigned type, const Firework* parent = nullptr){
        rules[type-1].create(&(fireworks[nextUseFirework]), parent);
        nextUseFirework = (nextUseFirework + 1) % maxFireworks;
    }

    void create(unsigned type, const Firework* parent, unsigned number){
        for (auto i = 0; i<number; i++){
            create(type, parent);
        }
    }

    void initFireWorkRules(){
        // Go through the firework types and create their rules.
        rules[0].init(2);
        rules[0].setParameters(
            1, // type
            0.5f, 1.4f, // age range
            my::Vector3(-5, 25, -5), // min velocity
            my::Vector3(5, 28, 5), // max velocity
            0.1 // damping
            );
        rules[0].payloads[0].set(3, 5);
        rules[0].payloads[1].set(5, 5);
    
        rules[1].init(1);
        rules[1].setParameters(
            2, // type
            0.5f, 1.0f, // age range
            my::Vector3(-5, 10, -5), // min velocity
            my::Vector3(5, 20, 5), // max velocity
            0.8 // damping
            );
        rules[1].payloads[0].set(4, 2);
    
        rules[2].init(0);
        rules[2].setParameters(
            3, // type
            0.5f, 1.5f, // age range
            my::Vector3(-5, -5, -5), // min velocity
            my::Vector3(5, 5, 5), // max velocity
            0.1 // damping
            );
    
        rules[3].init(0);
        rules[3].setParameters(
            4, // type
            0.25f, 0.5f, // age range
            my::Vector3(-20, 5, -5), // min velocity
            my::Vector3(20, 5, 5), // max velocity
            0.2 // damping
            );
    
        rules[4].init(1);
        rules[4].setParameters(
            5, // type
            0.5f, 1.0f, // age range
            my::Vector3(-20, 2, -5), // min velocity
            my::Vector3(20, 18, 5), // max velocity
            0.01 // damping
            );
        rules[4].payloads[0].set(3, 5);
    
        rules[5].init(0);
        rules[5].setParameters(
            6, // type
            3, 5, // age range
            my::Vector3(-5, 5, -5), // min velocity
            my::Vector3(5, 10, 5), // max velocity
            0.95 // damping
            );
    
        rules[6].init(1);
        rules[6].setParameters(
            7, // type
            4, 5, // age range
            my::Vector3(-5, 50, -5), // min velocity
            my::Vector3(5, 60, 5), // max velocity
            0.01 // damping
            );
        rules[6].payloads[0].set(8, 10);
    
        rules[7].init(0);
        rules[7].setParameters(
            8, // type
            0.25f, 0.5f, // age range
            my::Vector3(-1, -1, -1), // min velocity
            my::Vector3(1, 1, 1), // max velocity
            0.01 // damping
            );
    
        rules[8].init(0);
        rules[8].setParameters(
            9, // type
            3, 5, // age range
            my::Vector3(-15, 10, -5), // min velocity
            my::Vector3(15, 15, 5), // max velocity
            0.95 // damping
            );
        // ... and so on for other firework types ...
    }

    public:
    FireworkDemo(): nextUseFirework(0){
        fireworks = std::unique_ptr<Firework[]> (new Firework[maxFireworks]);
        for(auto i = 0; i < maxFireworks; i++){
            fireworks[i].type = 0;
        }
        rules = std::unique_ptr<FireworkRule[]> (new FireworkRule[rulecount]);
        initFireWorkRules();
    }
    
    void initGraphics() override{
        Application::initGraphics();
        glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
    }

    constexpr const char* getTitle() override{
        return "Firework Demo";
    }

    void update() override{
        auto duration = (float)TimingData::get().lastFrameDuration*0.001f;
        if(duration <= 0.0f) return;
        for(auto fi = 0; fi < maxFireworks; fi++){
            auto &curfire = fireworks[fi];
            if(curfire.type > 0){
                if (curfire.update(duration)){
                    for (auto i = 0; i < rules[curfire.type - 1].payloadCount; i++){
                        auto &currule = rules[curfire.type - 1];
                        create(currule.payloads[i].type, &(fireworks[fi]), currule.payloads[i].count);
                    }
                    fireworks[fi].type = 0;
                }
            }
        }
        Application::update();
    }

    void display() override{
        const static my::real size = 0.1f;
    
        // Clear the viewport and set the camera direction
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        gluLookAt(0.0, 4.0, 10.0,  0.0, 4.0, 0.0,  0.0, 1.0, 0.0);
    
        // Render each firework in turn
        glBegin(GL_QUADS);
        for (auto fi = 0;
            fi < maxFireworks;
            fi++)
        {
            // Check if we need to process this firework.
            if (fireworks[fi].type > 0)
            {
                switch (fireworks[fi].type)
                {
                case 1: glColor3f(1,0,0); break;
                case 2: glColor3f(1,0.5f,0); break;
                case 3: glColor3f(1,1,0); break;
                case 4: glColor3f(0,1,0); break;
                case 5: glColor3f(0,1,1); break;
                case 6: glColor3f(0.4f,0.4f,1); break;
                case 7: glColor3f(1,0,1); break;
                case 8: glColor3f(1,1,1); break;
                case 9: glColor3f(1,0.5f,0.5f); break;
                };
    
                const auto &pos = fireworks[fi].getPosition();
                glVertex3f(pos.x-size, pos.y-size, pos.z);
                glVertex3f(pos.x+size, pos.y-size, pos.z);
                glVertex3f(pos.x+size, pos.y+size, pos.z);
                glVertex3f(pos.x-size, pos.y+size, pos.z);
    
                // Render the firework's reflection
                glVertex3f(pos.x-size, -pos.y-size, pos.z);
                glVertex3f(pos.x+size, -pos.y-size, pos.z);
                glVertex3f(pos.x+size, -pos.y+size, pos.z);
                glVertex3f(pos.x-size, -pos.y+size, pos.z);
            }
        }
        glEnd();
    }

    void key(unsigned char key) override{
        switch (key){
        case '1': create(1, nullptr, 1); break;
        case '2': create(2, nullptr, 1); break;
        case '3': create(3, nullptr, 1); break;
        case '4': create(4, nullptr, 1); break;
        case '5': create(5, nullptr, 1); break;
        case '6': create(6, nullptr, 1); break;
        case '7': create(7, nullptr, 1); break;
        case '8': create(8, nullptr, 1); break;
        case '9': create(9, nullptr, 1); break;
        }
    }
};

auto getApplication()
{
    return std::make_shared<FireworkDemo>();
}