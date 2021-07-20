import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Experimental.Scene.Lights.LightCollection;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Scene.RaytracingInlineHelper;
import Utils.VirtualLight.VirtualLightContainer;
import Utils.VirtualLight.MegaTextureContainer;
import Utils.Sampling.AliasTable;

/*
Macros:
_PER_FRAME_PATH_COUNT
_INV_PER_FRAME_PATH_COUNT
*/

cbuffer CB
{
    uint gFrameIndex;
    VirtualLightContainer gVirtualLightContainer;
    MegaTextureContainer gSpecRadianceContainer;
    AliasTable gEmissiveTriTable;
};

RWByteAddressBuffer gDiffuseRadianceBuffer;
RWByteAddressBuffer gFluxBuffer;

float3 cosineWeightedSampling(float2 XY, float3 N)
{
    float3 w = N;
    float3 u = 0.0f;
    if (abs(w.x) > 0.1f)
    {
        u = normalize(cross(float3(0.0f, 1.0f, 0.0f), w));
    }
    else
    {
        u = normalize(cross(float3(1.0f, 0.0f, 0.0f), w));
    }
    float3 v = cross(w, u);
    float r1 = 2.0f * M_PI * XY.x;
    float r2 = XY.y;
    float r2s = sqrt(r2);                                                                                                       
    float3 dir = normalize((u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1.0f - r2)));

    return dir;
}

void interlockAddDiffuseRadiance(uint index, float3 radiance)
{
    uint address = index * 12u;
    NvInterlockedAddFp32(gDiffuseRadianceBuffer, address + 0, radiance.r);
    NvInterlockedAddFp32(gDiffuseRadianceBuffer, address + 4, radiance.g);
    NvInterlockedAddFp32(gDiffuseRadianceBuffer, address + 8, radiance.b);
}

void interlockAddFlux(uint index, float flux)
{
    uint address = index * 4u;
    NvInterlockedAddFp32(gFluxBuffer, address, flux);
}

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (launchIndex.x >= _PER_FRAME_PATH_COUNT) 
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(uint2(launchIndex.x, 1), gFrameIndex);
    uint triIndex = gEmissiveTriTable.sample(sampleNext2D(sg));
    float triPdf = gEmissiveTriTable.getWeight(triIndex) / gEmissiveTriTable.weightSum;

    EmissiveTriangle emissiveTri = gScene.lightCollection.getTriangle(triIndex);
    float samplePdf = triPdf / emissiveTri.area;
    float3 barycentric = sample_triangle(sampleNext2D(sg));

    float3 samplePos = emissiveTri.getPosition(barycentric);
    float2 sampleUV = emissiveTri.getTexCoord(barycentric);
    float3 sampleEmission = gScene.getEmissive(emissiveTri.materialID, sampleUV, gScene.materialResources[emissiveTri.materialID]);

    float3 rayDir = cosineWeightedSampling(sampleNext2D(sg), emissiveTri.normal);
    float3 rayOrigin = computeRayOrigin(samplePos, emissiveTri.normal);
    sampleEmission = sampleEmission / samplePdf * _INV_PER_FRAME_PATH_COUNT * M_PI;

    [loop]
    for (int bounce = 0; bounce < 4; bounce++)
    {
        ShadingData sd;
        if (!traceScene(rayOrigin, rayDir, sd))
        {
            break;
        }

        RayDesc searchRay;
        searchRay.Origin = sd.posW;
        searchRay.Direction = float3(0.0f, 1.0f, 0.0f);
        searchRay.TMin = 0.0f;
        searchRay.TMax = 0.00001f;
        RayQuery<RAY_FLAG_NONE> rayQuery;

        /*
        if intersect more than 8 times, clamp
        */
        int intersectIndicis[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
        int intersectPtr = 0;
        rayQuery.TraceRayInline(gVirtualLightContainer.as, RAY_FLAG_NONE, 0xff, searchRay);
        float maxDisSquare = 0.0f;
        while (rayQuery.Proceed())
        {
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint index = rayQuery.CandidatePrimitiveIndex();
                float3 posW = gVirtualLightContainer.getPosition(index);
                float radius = gVirtualLightContainer.getRadius(index);
                float disSquare = dot(sd.posW - posW, sd.posW - posW);
                if(disSquare <= radius * radius)
                {
                    ShadingData virtualLightSd = gVirtualLightContainer.getShadingData(index);
                    if(dot(sd.N, virtualLightSd.N) > 0.65f)
                    {
                        intersectIndicis[intersectPtr] = index;
                        intersectPtr++;
                        maxDisSquare = max(maxDisSquare, disSquare);
                    }
                }
            }

            if (intersectPtr >= 8)
            {
                rayQuery.Abort();
            }
        }

        /*
        Compute Total Weight
        */
        float weightSum = 0.0f;
        for (uint i = 0; i < 8; i++)
        {
            uint index = intersectIndicis[i];
            if (index == -1)
            {
                break;
            }
            float3 posW = gVirtualLightContainer.getPosition(index);
            float disSquare = dot(sd.posW - posW, sd.posW - posW);
            weightSum = weightSum + 1.0f - disSquare / (maxDisSquare + 0.001f);
        }

        /*
        Do Radiance Transfer
        */
        for (uint i = 0; i < 8; i++)
        {
            uint index = intersectIndicis[i];
            if (index == -1)
            {
                break;
            }
            float3 posW = gVirtualLightContainer.getPosition(index);
            float disSquare = dot(sd.posW - posW, sd.posW - posW);
            float weight = 1.0f - disSquare / (maxDisSquare + 0.001f);
        }
    }
}