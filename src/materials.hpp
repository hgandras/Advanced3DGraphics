#pragma once

#include <utility>
#include <tuple>
#include <stdexcept>
#include "math.hpp"
#include "rng.hpp"

class Material
{
public:
    Material()
    {
        Reset();
    }

    void Reset()
    {
        mDiffuseReflectance = Vec3f(0);
        mPhongReflectance = Vec3f(0);
        mPhongExponent = 1.f;
    }

    /**
     * Randomly chooses an outgoing direction that is reflected from the material surface
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled reflected outgoing direction
     *  - the intensity corresponding to the reflected light
     *  - the probability density (PDF) of choosing this direction
     */
    std::tuple<Vec3f, Vec3f, float> SampleReflectedDirection(const Vec3f &incomingDirection, Rng &rng) const
    {
        /*float pDiff=mDiffuseReflectance.Max();
        float pSpec=mPhongReflectance.Max();
        float norm=1.0f/(pDiff+pSpec);
        pDiff*=norm;
        pSpec*=norm;

        Vec2f sample=rng.GetVec2f();
        Vec3f outGoingDirection=Vec3f(0.0);

        if(rng.GetFloat()<=pDiff)
        {
            outGoingDirection=sampleCosUnitHemisphere(sample);
        }
        else
        {
            CoordinateFrame frame;
            frame.SetFromZ(ReflectLocal(incomingDirection)); //Frame is built from the local frame, so world in this case is the shading local surface system.
            Vec3f sampleLobe=sampleSpecular(sample,mPhongExponent);
            outGoingDirection=frame.ToWorld(sampleLobe);
        }

        float pdf=pDiff*diffPDF(outGoingDirection)+pSpec*specPDF(incomingDirection,outGoingDirection);*/

        Vec2f sample=rng.GetVec2f();
        Vec3f outGoingDirection=sampleUnitHemisphere(sample);

        return {outGoingDirection,EvaluateBRDF(incomingDirection,outGoingDirection),1/(2*PI_F)};
    }

    /**
     * Returns the probability density corresponding to sampleReflectedDirection,
     * i.e., what is the probability that calling sampleReflectedDirection would randomly choose the given outgoingDirection
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - outgoingDirection = the randomly sampled (normalized) outgoing direction
     */
    float PDF(const Vec3f &incomingDirection, const Vec3f &outgoingDirection) const
    {
        return outgoingDirection.Get(2)/(PI_F);
    }

    float specPDF(const Vec3f &incomingDirection,const Vec3f &sampledDirection) const
    {
        Vec3f reflected_direction = ReflectLocal(incomingDirection);
        float angle_cos = Dot(sampledDirection, reflected_direction);
        return (mPhongExponent+1)/(2*PI_F)*pow(angle_cos,mPhongExponent);
    }

    float diffPDF(const Vec3f &outgoingDirection) const
    {
        return outgoingDirection.Get(2)/(PI_F);
    }

    /**
     * Returns the intensity corresponding to the reflected light according to this material's BRDF
     * Arguments:
     *  - incomingDirection = a normalized direction towards the previous (origin) point in the scene
     *  - outgoingDirection = a normalized outgoing reflected direction
     */
    Vec3f EvaluateBRDF(const Vec3f &incomingDirection, const Vec3f &outgoingDirection) const
    {
        if (incomingDirection.z <= 0 && outgoingDirection.z <= 0)
        {
            return Vec3f(0);
        }

        Vec3f diffuseComponent = mDiffuseReflectance / PI_F;

        Vec3f reflected_direction = ReflectLocal(outgoingDirection);
        float angle_cos = Dot(incomingDirection, reflected_direction);
        Vec3f glossyComponent = mPhongReflectance * (mPhongExponent + 2.0f) * pow(angle_cos, mPhongExponent)/ (2.0f * PI_F);

        return diffuseComponent + glossyComponent;
    }

    Vec3f mDiffuseReflectance;
    Vec3f mPhongReflectance;
    float mPhongExponent;
};
