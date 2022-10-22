#include <assert.h>

#include <math/base.h>
#include <math/precision.h>

namespace my{
    class Particle{
        public:

        protected:
            Vector3 position;
            Vector3 velocity;
            Vector3 acceleration;
            Vector3 forceAccum;
            real dampling;
            real inverseMass;
        
        public:

            void setMass(const real mass){
                assert(mass > 0.0);
                inverseMass = ((real)1)/mass;                
            }

            void setDampling(const real damp){
                dampling = damp;
            }

            void setPosition(const Vector3& posi){
                position = posi;
            }

            void setPosition(const real& x, const real& y, const real& z){
                position.x = x;
                position.y = y;
                position.z = z;
            }

            void setVelocity(const Vector3& velo){
                velocity = velo;
            }

            void setVelocity(const real& x, const real& y, const real& z){
                velocity.x = x;
                velocity.y = y;
                velocity.z = z;
            }

            void setAcceleration(const Vector3& acce){
                acceleration = acce;
            }

            void setAcceleration(const real& x, const real& y, const real& z){
                acceleration.x = x;
                acceleration.y = y;
                acceleration.z = z;
            }

            void setForceAccum(const Vector3& force){
                forceAccum = force;
            }

            void setForceAccum(const real& x, const real& y, const real& z){
                forceAccum.x = x;
                forceAccum.y = y;
                forceAccum.z = z;
            }

            void clearAccumulator(){
                forceAccum.clear();
            }


            void integrate(real duration){
                assert(duration > 0.0);

                position.addScaledVector(velocity, duration);
                Vector3 resultAcc = acceleration;
                resultAcc.addScaledVector(forceAccum, inverseMass);
                velocity.addScaledVector(resultAcc, duration);
                velocity *= real_pow(dampling, duration);
            }
    };
}