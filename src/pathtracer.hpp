#pragma once

#include <vector>
#include <cmath>
#include <omp.h>
#include <cassert>
#include "renderer.hpp"
#include "rng.hpp"
#include "utils.hpp"

#define COS_SAMPLING true

class PathTracer : public AbstractRenderer
{
public:
    PathTracer(
        const Scene &aScene,
        int aSeed = 1234) : AbstractRenderer(aScene), mRandomGenerator(aSeed)
    {
    }

    virtual void RunIteration(int iteration)
    {
        const int resolutionX = int(mScene.mCamera.mResolution.x);
        const int resolutionY = int(mScene.mCamera.mResolution.y);

        for (int pixelID = 0; pixelID < resolutionX * resolutionY; pixelID++)
        {
            // Current pixel coordinates (as integers):
            const int x = pixelID % resolutionX;
            const int y = pixelID / resolutionX;

            // Current pixel coordinates (as floating point numbers, randomly positioned inside a pixel square):
            // E.g., for x = 5, y = 12, we can have sample coordinates from x = 5.00 to 5.99.., and y = 12.00 to 12.99..
            const Vec2f sample = Vec2f(float(x), float(y)) + mRandomGenerator.GetVec2f();

            // Generating a ray with an origin in the camera with a direction corresponding to the pixel coordinates:
            Ray ray = mScene.mCamera.GenerateRay(sample);

            auto intersection = mScene.FindClosestIntersection(ray);
            if (intersection)
            {
                if (intersection->lightID >= 0)
                {
                    const AbstractLight *intersectedLightPtr = mScene.GetLightPtr(intersection->lightID);
                    Vec3f intensity = intersectedLightPtr->Evaluate(ray.direction);
                    mFramebuffer.AddColor(sample, intensity);
                    continue;
                }

                const Vec3f surfacePoint = ray.origin + ray.direction * intersection->distance;
                CoordinateFrame frame;
                frame.SetFromZ(intersection->normal);
                const Vec3f incomingDirection = frame.ToLocal(-ray.direction);

                Vec3f LoDirect = Vec3f(0);
                const Material &mat = mScene.GetMaterial(intersection->materialID);
#if COS_SAMPLING
                //Sampling the material
                auto [direction,brdfIntensity,pdf] = mat.SampleReflectedDirection(incomingDirection,mRandomGenerator);
                Ray sampleRay=Ray(surfacePoint,frame.ToWorld(direction),EPSILON_RAY);
                
                //Checking for light inersection
                auto sampleIntersection= mScene.FindClosestIntersection(sampleRay);
                if(sampleIntersection && sampleIntersection->lightID>=0)
                { 
                    //Evaluating light source
                    const AbstractLight *light = mScene.GetLightPtr(sampleIntersection->lightID); 
                    assert(light!=0);
                    Vec3f intensity = light->Evaluate(sampleRay.direction);   
                    float cosTheta = Dot(frame.mZ, sampleRay.direction);
                    LoDirect += intensity * brdfIntensity * cosTheta / pdf;     
                }
                else if(!sampleIntersection && mScene.mBackground)
                {
                    const AbstractLight *light = mScene.mBackground;
                    if(light!=0)
                    {
                        Vec3f intensity = light->Evaluate(sampleRay.direction);   
                        float cosTheta = Dot(frame.mZ, sampleRay.direction);
                        LoDirect += intensity * brdfIntensity * cosTheta / pdf;
                    }
                }            
#else
                // Connect from the current surface point to every light source in the scene:
                for (int i = 0; i < mScene.GetLightCount(); i++)
                {
                    const AbstractLight *light = mScene.GetLightPtr(i);
                    assert(light != 0);

                    auto [lightPoint, intensity, pdf] = light->SamplePointOnLight(surfacePoint, mRandomGenerator);
                    Vec3f outgoingDirection = Normalize(lightPoint - surfacePoint);
                    float lightDistance = sqrt((lightPoint - surfacePoint).LenSqr());
                    float cosTheta = Dot(frame.mZ, outgoingDirection);

                    if (cosTheta > 0 && intensity.Max() > 0)
                    {
                        Ray rayToLight(surfacePoint, outgoingDirection, EPSILON_RAY); // Note! To prevent intersecting the same object we are already on, we need to offset the ray by EPSILON_RAY
                        if (!mScene.FindAnyIntersection(rayToLight, lightDistance))
                        { // Testing if the direction towards the light source is not occluded
                            LoDirect += intensity * mat.EvaluateBRDF(incomingDirection,frame.ToLocal(outgoingDirection)) * cosTheta / pdf;
                        }
                    }
                }
#endif
                mFramebuffer.AddColor(sample, LoDirect);
            }
        }

        mIterations++;
    }

    Rng mRandomGenerator;
};
