#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include "math.hpp"

#define EPSILON_COSINE 1e-6f
#define EPSILON_RAY 1e-3f

// sRGB luminance
float Luminance(const Vec3f &aRGB)
{
    return 0.212671f * aRGB.x +
           0.715160f * aRGB.y +
           0.072169f * aRGB.z;
}

// reflect vector through (0,0,1)
Vec3f ReflectLocal(const Vec3f &aVector)
{
    return Vec3f(-aVector.x, -aVector.y, aVector.z);
}

//////////////////////////////////////////////////////////////////////////
// Utilities for converting PDF between Area (A) and Solid angle (W)
// WtoA = PdfW * cosine / distance_squared
// AtoW = PdfA * distance_squared / cosine

float PdfWtoA(
    const float aPdfW,
    const float aDist,
    const float aCosThere)
{
    return aPdfW * std::abs(aCosThere) / Sqr(aDist);
}

float PdfAtoW(
    const float aPdfA,
    const float aDist,
    const float aCosThere)
{
    return aPdfA * Sqr(aDist) / std::abs(aCosThere);
}

/**
 * @brief Returns a 2D point sampled uniformly on a righ-angled triangle's surface with side lengths of 1 next
 * to the right angle.
 *
 * @return Vec2f (u,v) sampled surface coordinates
 */
Vec2f sampleTriangleUniform(Vec2f samples)
{
    float r1 = sqrt(samples.Get(0));
    return Vec2f(1 - r1, samples.Get(1) * r1);
}

Vec3f sampleUnitSphereUniform(Vec2f samples)
{
    float z = 1.0f - 2.0f * samples.Get(0);
    float r = sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2 * PI_F * samples.Get(1);
    return Vec3f(r * cos(phi), r * sin(phi), z);
}

Vec3f sampleUnitHemisphere(Vec2f samples)
{
    float z = samples.Get(0);
    float r = sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2 * PI_F * samples.Get(1);
    return Vec3f(r * cos(phi), r * sin(phi), z);
}

Vec3f sampleCosUnitHemisphere(Vec2f samples)
{
    float r1=samples.Get(0);
    float r2=samples.Get(1);

    float z =sqrt(r2);
    float x= cos(2*PI_F*r1)*sqrt(1-r2);
    float y=sin(2*PI_F*r1)*sqrt(1-r2);

    return Vec3f(x,y,z);
}

Vec3f sampleSpecular(Vec2f samples,float exponent)
{
    float r1=samples.Get(0);
    float r2=samples.Get(1);

    float sqrtTerm=sqrt(1.0f-pow(r2,2.0f/(exponent+1)));

    float x=cos(2.0f*PI_F*r1)*sqrtTerm;
    float y=sin(2.0f*PI_F*r1)*sqrtTerm;
    float z=pow(r2,1.0f/(exponent+1));

    return Vec3f(x,y,z);
}
