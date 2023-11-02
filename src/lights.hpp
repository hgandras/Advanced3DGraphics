#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include <tuple>
#include <stdexcept>
#include "math.hpp"
#include "rng.hpp"

class AbstractLight
{
public:
    /**
     * Randomly chooses a point on the light source
     * Arguments:
     *  - origin = our current position in the scene
     *  - rng = random generator
     * Returns:
     *  - a randomly sampled point on the light source
     *  - the illumination intensity corresponding to the sampled direction
     *  - the probability density (PDF) of choosing this point
     */
    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f &origin, Rng &rng) const
    {
        throw std::logic_error("Not implemented");
    }

    /**
     * Returns the probability density corresponding to samplePointOnLight,
     * i.e., what is the probability that calling samplePointOnLight would randomly choose the given lightPoint
     * Arguments:
     *  - origin = our current position in the scene
     *  - lightPoint = the randomly sampled point on the light source
     */
    virtual float PDF(const Vec3f &origin, const Vec3f &lightPoint) const
    {
        throw std::logic_error("Not implemented");
    }

    /**
     * Returns the illumination intensity in the given direction
     * Arguments:
     *  - direction = direction towards the light source
     */
    virtual Vec3f Evaluate(const Vec3f &direction) const
    {
        throw std::logic_error("Not implemented");
    }

    virtual ~AbstractLight() = default;
};

//////////////////////////////////////////////////////////////////////////
class AreaLight : public AbstractLight
{
public:
    AreaLight(
        const Vec3f &aP0,
        const Vec3f &aP1,
        const Vec3f &aP2)
    {
        p0 = aP0;
        e1 = aP1 - aP0;
        e2 = aP2 - aP0;

        Vec3f normal = Cross(e1, e2);
        float len = normal.Length();
        mInvArea = 2.f / len;
        mFrame.SetFromZ(normal);
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f &origin, Rng &rng) const override
    {
        Vec2f uv = sampleTriangleUniform(rng.GetVec2f());
        Vec3f u3d = e1 * uv.Get(0);
        Vec3f v3d = e2 * uv.Get(1);
        Vec3f sampledPoint = p0 + u3d + v3d;

        Vec3f outgoingDirection = sampledPoint - origin;
        Vec3f outDirNormal = Normalize(outgoingDirection);
        float cosTheta = abs(Dot(-outDirNormal, mFrame.Normal()));

        float distanceSquared = outgoingDirection.LenSqr();

        float coefficient = cosTheta / distanceSquared;
        Vec3f emission_value = mRadiance * coefficient;
        Vec3f final_value = cosTheta > 0.0f ? emission_value : Vec3f(0.0f);

        return {sampledPoint, emission_value, mInvArea};
    }

    virtual Vec3f Evaluate(const Vec3f &direction) const override
    {
        return mRadiance;
    }

public:
    Vec3f p0, e1, e2;
    CoordinateFrame mFrame;
    Vec3f mRadiance;
    float mInvArea;
};

//////////////////////////////////////////////////////////////////////////
class PointLight : public AbstractLight
{
public:
    PointLight(const Vec3f &aPosition)
    {
        mPosition = aPosition;
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f &origin, Rng &rng) const override
    {
        Vec3f outgoingDirection = mPosition - origin;
        float distanceSquared = outgoingDirection.LenSqr();

        return {mPosition, mIntensity / distanceSquared, 1.0f};
    }

public:
    Vec3f mPosition;
    Vec3f mIntensity;
};

//////////////////////////////////////////////////////////////////////////
class BackgroundLight : public AbstractLight
{
public:
    BackgroundLight()
    {
        mBackgroundColor = Vec3f(135, 206, 250) / Vec3f(255.f);
        mRadius = 100.f; // a radius big enough to cover the whole scene
    }

    virtual std::tuple<Vec3f, Vec3f, float> SamplePointOnLight(const Vec3f &origin, Rng &rng) const override
    {
        throw std::logic_error("Not implemented");
    }

public:
    Vec3f mBackgroundColor;
    float mRadius; // we model the background light as a huge sphere around the whole scene, with a given radius
};
