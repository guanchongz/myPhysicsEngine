#ifndef MY_RANDOM_H
#define MY_RANDOM_H

#include <ctime>

#include <my.h>

namespace my{
    class Random{
    public:
    	/**
    	 * left bitwise rotation
    	 */

    	unsigned rotl(unsigned n, unsigned r);
    	/**
    	 * right bitwise rotation
    	 */
    	unsigned rotr(unsigned n, unsigned r);

        /**
         * Creates a new random number stream with a seed based on
         * timing data.
         */
        Random();

        /**
         * Creates a new random stream with the given seed.
         */
        Random(unsigned seed);

        /**
         * Sets the seed value for the random stream.
         */
        void seed(unsigned seed);

        /**
         * Returns the next random bitstring from the stream. This is
         * the fastest method.
         */
        unsigned randomBits();

        /**
         * Returns a random floating point number between 0 and 1.
         */
        real randomReal();

        /**
         * Returns a random floating point number between 0 and scale.
         */
        real randomReal(real scale);

        /**
         * Returns a random floating point number between min and max.
         */
        real randomReal(real min, real max);

        /**
         * Returns a random integer less than the given value.
         */
        unsigned randomInt(unsigned max);

        /**
         * Returns a random binomially distributed number between -scale
         * and +scale.
         */
        real randomBinomial(real scale);

        /**
         * Returns a random vector where each component is binomially
         * distributed in the range (-scale to scale) [mean = 0.0f].
         */
        Vector3 randomVector(real scale);

        /**
         * Returns a random vector where each component is binomially
         * distributed in the range (-scale to scale) [mean = 0.0f],
         * where scale is the corresponding component of the given
         * vector.
         */
        Vector3 randomVector(const Vector3 &scale);

        /**
         * Returns a random vector in the cube defined by the given
         * minimum and maximum vectors. The probability is uniformly
         * distributed in this region.
         */
        Vector3 randomVector(const Vector3 &min, const Vector3 &max);

        /**
         * Returns a random vector where each component is binomially
         * distributed in the range (-scale to scale) [mean = 0.0f],
         * except the y coordinate which is zero.
         */
        Vector3 randomXZVector(real scale);

        /**
         * Returns a random orientation (i.e. normalized) quaternion.
         */
        Quaternion randomQuaternion();

    private:
        // Internal mechanics
        int p1, p2;
        unsigned buffer[17];
    };

}

using namespace my;

Random::Random()
{
    seed(0);
}

Random::Random(unsigned seed)
{
    Random::seed(seed);
}

void Random::seed(unsigned s)
{
    if (s == 0) {
        s = (unsigned)clock();
    }

    // Fill the buffer with some basic random numbers
    for (unsigned i = 0; i < 17; i++)
    {
        // Simple linear congruential generator
        s = s * 2891336453 + 1;
        buffer[i] = s;
    }

    // Initialize pointers into the buffer
    p1 = 0;  p2 = 10;
}

unsigned Random::rotl(unsigned n, unsigned r)
{
	  return	(n << r) |
			  (n >> (32 - r));
}

unsigned Random::rotr(unsigned n, unsigned r)
{
	  return	(n >> r) |
				(n << (32 - r));
}

unsigned Random::randomBits()
{
    unsigned result;

    // Rotate the buffer and store it back to itself
    result = buffer[p1] = rotl(buffer[p2], 13) + rotl(buffer[p1], 9);

    // Rotate pointers
    if (--p1 < 0) p1 = 16;
    if (--p2 < 0) p2 = 16;

    // Return result
    return result;
}

real Random::randomReal()
{
    // Get the random number
    unsigned bits = randomBits();

    // Set up a reinterpret structure for manipulation
    union {
        real value;
        unsigned words[2];
    } convert;

    // Now assign the bits to the words. This works by fixing the ieee
    // sign and exponent bits (so that the size of the result is 1-2)
    // and using the bits to create the fraction part of the float. Note
    // that bits are used more than once in this process.
    convert.words[0] =  bits << 20; // Fill in the top 16 bits
    convert.words[1] = (bits >> 12) | 0x3FF00000; // And the bottom 20

    // And return the value
    return convert.value - 1.0;
}
#endif

real Random::randomReal(real min, real max)
{
    return randomReal() * (max-min) + min;
}

real Random::randomReal(real scale)
{
    return randomReal() * scale;
}

unsigned Random::randomInt(unsigned max)
{
    return randomBits() % max;
}

real Random::randomBinomial(real scale)
{
    return (randomReal()-randomReal())*scale;
}

Quaternion Random::randomQuaternion()
{
    Quaternion q(
        randomReal(),
        randomReal(),
        randomReal(),
        randomReal()
        );
    q.normalise();
    return q;
}

Vector3 Random::randomVector(real scale)
{
    return Vector3(
        randomBinomial(scale),
        randomBinomial(scale),
        randomBinomial(scale)
        );
}

Vector3 Random::randomXZVector(real scale)
{
    return Vector3(
        randomBinomial(scale),
        0,
        randomBinomial(scale)
        );
}

Vector3 Random::randomVector(const Vector3 &scale)
{
    return Vector3(
        randomBinomial(scale.x),
        randomBinomial(scale.y),
        randomBinomial(scale.z)
        );
}

Vector3 Random::randomVector(const Vector3 &min, const Vector3 &max)
{
    return Vector3(
        randomReal(min.x, max.x),
        randomReal(min.y, max.y),
        randomReal(min.z, max.z)
        );
}

#endif
