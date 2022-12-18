#pragma once

#include "math/base.hpp"
#include <my.h>

namespace my{
    class RigidBody{
        private:
        void _transformInertiaTensor(Matrix3 &iitWorld,
                                                   const Quaternion &q,
                                                   const Matrix3 &iitBody,
                                                   const Matrix4 &rotmat){
            real t4 = rotmat.data[0]*iitBody.data[0]+
                rotmat.data[1]*iitBody.data[3]+
                rotmat.data[2]*iitBody.data[6];
            real t9 = rotmat.data[0]*iitBody.data[1]+
                rotmat.data[1]*iitBody.data[4]+
                rotmat.data[2]*iitBody.data[7];
            real t14 = rotmat.data[0]*iitBody.data[2]+
                rotmat.data[1]*iitBody.data[5]+
                rotmat.data[2]*iitBody.data[8];
            real t28 = rotmat.data[4]*iitBody.data[0]+
                rotmat.data[5]*iitBody.data[3]+
                rotmat.data[6]*iitBody.data[6];
            real t33 = rotmat.data[4]*iitBody.data[1]+
                rotmat.data[5]*iitBody.data[4]+
                rotmat.data[6]*iitBody.data[7];
            real t38 = rotmat.data[4]*iitBody.data[2]+
                rotmat.data[5]*iitBody.data[5]+
                rotmat.data[6]*iitBody.data[8];
            real t52 = rotmat.data[8]*iitBody.data[0]+
                rotmat.data[9]*iitBody.data[3]+
                rotmat.data[10]*iitBody.data[6];
            real t57 = rotmat.data[8]*iitBody.data[1]+
                rotmat.data[9]*iitBody.data[4]+
                rotmat.data[10]*iitBody.data[7];
            real t62 = rotmat.data[8]*iitBody.data[2]+
                rotmat.data[9]*iitBody.data[5]+
                rotmat.data[10]*iitBody.data[8];
        
            iitWorld.data[0] = t4*rotmat.data[0]+
                t9*rotmat.data[1]+
                t14*rotmat.data[2];
            iitWorld.data[1] = t4*rotmat.data[4]+
                t9*rotmat.data[5]+
                t14*rotmat.data[6];
            iitWorld.data[2] = t4*rotmat.data[8]+
                t9*rotmat.data[9]+
                t14*rotmat.data[10];
            iitWorld.data[3] = t28*rotmat.data[0]+
                t33*rotmat.data[1]+
                t38*rotmat.data[2];
            iitWorld.data[4] = t28*rotmat.data[4]+
                t33*rotmat.data[5]+
                t38*rotmat.data[6];
            iitWorld.data[5] = t28*rotmat.data[8]+
                t33*rotmat.data[9]+
                t38*rotmat.data[10];
            iitWorld.data[6] = t52*rotmat.data[0]+
                t57*rotmat.data[1]+
                t62*rotmat.data[2];
            iitWorld.data[7] = t52*rotmat.data[4]+
                t57*rotmat.data[5]+
                t62*rotmat.data[6];
            iitWorld.data[8] = t52*rotmat.data[8]+
                t57*rotmat.data[9]+
                t62*rotmat.data[10];
        }

        static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                                     const Vector3 &position,
                                                     const Quaternion &orientation){
            transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
                2*orientation.k*orientation.k;
            transformMatrix.data[1] = 2*orientation.i*orientation.j -
                2*orientation.r*orientation.k;
            transformMatrix.data[2] = 2*orientation.i*orientation.k +
                2*orientation.r*orientation.j;
            transformMatrix.data[3] = position.x;
        
            transformMatrix.data[4] = 2*orientation.i*orientation.j +
                2*orientation.r*orientation.k;
            transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
                2*orientation.k*orientation.k;
            transformMatrix.data[6] = 2*orientation.j*orientation.k -
                2*orientation.r*orientation.i;
            transformMatrix.data[7] = position.y;
        
            transformMatrix.data[8] = 2*orientation.i*orientation.k -
                2*orientation.r*orientation.j;
            transformMatrix.data[9] = 2*orientation.j*orientation.k +
                2*orientation.r*orientation.i;
            transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
                2*orientation.j*orientation.j;
            transformMatrix.data[11] = position.z;
        }

        protected:
            bool isAwake;
            bool canSleep;
            real linearDamping;
            real angularDamping;
            real inverseMass;
            real motion;
            Vector3 position;
            Vector3 velocity;
            Vector3 acceleration;
            Vector3 forceAccum;
            Vector3 torqueAccum;
            Vector3 rotation;
            Vector3 lastFrameAcceleration;
            Quaternion orientation;
            Matrix3 inverseInertiaTensor;
            Matrix3 inverseInertiaTensorWorld;
            Matrix4 transformMatrix;
        
        public:

            void setMass(const real mass){
                assert(mass > 0.0);
                inverseMass = ((real)1)/mass;                
            }

            void setDamping(const real ldamp, const real adamp){
                linearDamping = ldamp;
                angularDamping = adamp;
            }

            void setPosition(const Vector3& posi){
                position.x = posi.x;
                position.y = posi.y;
                position.z = posi.z;
            }

            void setPosition(const real& x, const real& y, const real& z){
                position.x = x;
                position.y = y;
                position.z = z;
            }

            void setVelocity(const Vector3& velo){
                velocity.x = velo.x;
                velocity.y = velo.y;
                velocity.z = velo.z;
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


            void integrate(real &duration){
                assert(duration > 0.0);

                position.addScaledVector(velocity, duration);
                Vector3 resultAcc = acceleration;
                resultAcc.addScaledVector(forceAccum, inverseMass);
                velocity.addScaledVector(resultAcc, duration);
                velocity *= real_pow(damping, duration);

                clearAccumulator();
            }

            void getPosition(Vector3* pposi) const{
                *pposi = position;
            }

            Vector3 getPosition() const{
                return position;
            }

            void getVelocity(Vector3* pvelo) const{
                *pvelo = velocity;
            }

            Vector3 getVelocity() const{
                return velocity;
            }

            void getAcceleration(Vector3* pacc) const{
                *pacc = acceleration;
            }

            Vector3 getAcceleration() const{
                return acceleration;
            }

            void addVelocity(Vector3 &velo){
                velocity += velo;
            }

            void addForce(const Vector3 &force){
                forceAccum += force;
            }

            bool hasFiniteMass() const{
                return inverseMass > 0.0f;
            }

            real getInverseMass() const{
                return inverseMass;
            }

            real getMass() const{
                if (inverseMass > 0.0f){
                    return (real)1.0 / inverseMass;
                }else{
                    return REAL_MAX;
                }
            }
    };
}

