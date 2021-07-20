#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Scene.Scene;
import Scene.Shading;
import Scene.HitInfo;

struct PackedBoundingBox
{
    float3 minPoint;
    float3 maxPoint;
    float Pad0;
    float Pad1;
};

struct VirtualLightContainer
{
    RWStructuredBuffer<float3> positionBuffer;
    RWStructuredBuffer<PackedHitInfo> hitInfoBuffer;
    RWStructuredBuffer<PackedBoundingBox> boundingBoxBuffer;
    RaytracingAccelerationStructure as;
    float boundingBoxRadius;
    uint count;

    uint pushLight(float3 posW, HitInfo hitInfo)
    {
        uint ind = positionBuffer.IncrementCounter();

        positionBuffer[ind] = posW;
        hitInfoBuffer[ind] = hitInfo.encode();

        float3 radius = float3(boundingBoxRadius);
        boundingBoxBuffer[ind].minPoint = posW - radius;
        boundingBoxBuffer[ind].maxPoint = posW + radius;

        return ind;
    }

    float3 getPosition(uint i)
    {
        return positionBuffer[i];
    }

    float getRadius(uint i)
    {
        return (boundingBoxBuffer[i].maxPoint.x - boundingBoxBuffer[i].minPoint.x) * 0.5f;
    }

    PackedHitInfo getHitInfo(uint i)
    {
        return hitInfoBuffer[i];
    }

    ShadingData getShadingData(uint i)
    {
        HitInfo hit;
        ShadingData sd;
        bool success = hit.decode(hitInfoBuffer[i]);
        if (success)
        {
            VertexData v = gScene.getVertexData(hit);
            const uint materialID = gScene.getMaterialID(hit.instanceID);
            sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], 0.f, 0.f);
            adjustShadingNormal(sd, v);
        }
        return sd;
    }
}
