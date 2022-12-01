#ifndef MY_MATH_BASE_H
#define MY_MATH_BASE_H

#include <cmath>
#include <cstddef>

#include <math/precision.hpp>

namespace my{
    class Vector3{
        public:
            real x;
            real y;
            real z;
        public:
            Vector3(): x(0), y(0), z(0) {}
            Vector3(const Vector3 &vector){
                x = vector.x;
                y = vector.y;
                z = vector.z;
            }
            Vector3(const real x , const real y, const real z) : x(x), y(y), z(z) {}

            void operator+=(const Vector3& v){
                x += v.x;
                y += v.y;
                z += v.z;
            }

            Vector3 operator+(const Vector3& v) const{
                return Vector3(x+v.x, y+v.y, z+v.z);
            }

            void operator-=(const Vector3& v){
                x -= v.x;
                y -= v.y;
                z -= v.z;
            }

            Vector3 operator-(const Vector3& v) const{
                return Vector3(x-v.x, y-v.y, z-v.z);
            }

            void operator*=(const real value){
                x *= value;
                y *= value;
                z *= value;
            }

            Vector3 operator*(const real value){
                return Vector3(x*value, y*value, z*value);
            }

            void addScaledVector(const Vector3& v, real scale){
                x += v.x * scale;
                y += v.y * scale;
                z += v.z * scale;
            }

            Vector3 componentProduct(const Vector3& v) const{
                return Vector3(x*v.x, y*v.y, z*v.z);
            }

            void componentProductUpdate(const Vector3& v){
                x *= v.x;
                y *= v.y;
                z *= v.z;
            }

            real scalarProduct(const Vector3& v) const{
                return x*v.x + y*v.y + z*v.z;
            }

            real operator*(const Vector3& v) const{
                return x*v.x + y*v.y + z*v.z;
            }

            Vector3 vectorProduct(const Vector3& v) const{
                return Vector3(y*v.z - z*v.y,
                               z*v.x - x*v.z,
                               x*v.y - y*v.x);
            }

            void operator%=(const Vector3& v){
                *this = vectorProduct(v);
            }

            Vector3 operator%(const Vector3& v) const{
                return Vector3(y*v.z - z*v.y,
                               z*v.x - x*v.z,
                               x*v.y - y*v.x);
            }

            real magnitude() const{
                return real_sqrt(x*x + y*y + z*z);
            }

            real squareMagnitude() const {
                return x*x + y*y + z*z;
            }

            void normalize() {
                real norm = magnitude();
                if (norm > 0){
                    (*this)*=((real)1)/norm;
                }
            }

            Vector3 unit(){
                Vector3 result = *this;
                result.normalize();
                return result;
            }

            void invert(){
                x = -x;
                y = -y;
                z = -z;
            }
            
            void clear(){
                x = y = z = 0;
            }
    };


    const Vector3 GRAVITY = Vector3(0, -9.81, 0);
    const Vector3 HIGH_GRAVITY = Vector3(0, -19.62, 0);
    const Vector3 UP = Vector3(0, 1, 0);
    const Vector3 RIGHT = Vector3(1, 0, 0);
    const Vector3 OUT_OF_SCREEN = Vector3(0, 0, 1);
    const Vector3 X = Vector3(0, 1, 0);
    const Vector3 Y = Vector3(1, 0, 0);
    const Vector3 Z = Vector3(0, 0, 1);

}

#endif