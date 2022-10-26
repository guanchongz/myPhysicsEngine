#include <memory>

//#include <gl/glut.h>
//#include <my.h>

static my::Random crandom;

class Firwork : public my::Particle{
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
    std::shared_ptr<Payload> payloads;
    
    FireworkRule : payloadCount(0), payloads(nullptr){}
    void init(unsigned payload_count){
        payloadCount = payload_count;
        payloads = new Payload[payloadCount];
    }
};
