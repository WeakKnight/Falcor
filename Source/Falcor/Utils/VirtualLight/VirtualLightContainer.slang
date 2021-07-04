#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
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

    void pushLight(float3 posW, HitInfo hitInfo)
    {
        uint ind = _positions.IncrementCounter();

        positionBuffer[ind] = posW;
        hitInfoBuffer[ind] = hitInfo.encode();

        float3 radius = float3(boundingBoxRadius);
        boundingBoxBuffer[ind].minPoint = posW - radius;
        boundingBoxBuffer[ind].maxPoint = posW + radius;
    }
}
