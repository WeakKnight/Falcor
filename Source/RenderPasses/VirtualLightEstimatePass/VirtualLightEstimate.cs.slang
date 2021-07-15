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
    MegaTextureContainer gRadianceContainer;
    AliasTable gEmissiveTriTable;
};

float3 cosineWeightedSampling(float2 XY, float3 N)
{
    float3 w = N;
    float3 u = 0.0;
    if (abs(w.x) > 0.1)
    {
        u = normalize(cross(float3(0.0, 1.0, 0.0), w));
    }
    else
    {
        u = normalize(cross(float3(1.0, 0.0, 0.0), w));
    }
    float3 v = cross(w, u);
    float r1 = 2.0 * M_PI * XY.x;
    float r2 = XY.y;
    float r2s = sqrt(r2);
    float3 dir = normalize((u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1.0 - r2)));

    return dir;
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
        bool hit = traceScene(rayOrigin, rayDir, sd);
        
        RayDesc searchRay;
        searchRay.Origin = sd.posW;
        searchRay.Direction = float3(0.0, 1.0, 0.0);
        searchRay.TMin = 0.0f;
        searchRay.TMax = 0.00001f;
        RayQuery<RAY_FLAG_NONE> searchRayQuery;

        /*
        if intersect more than 5 times, clamp
        */
        int intersectIndicis[5] = {-1, -1, -1, -1, -1};
        searchRayQuery.TraceRayInline(gVirtualLightContainer.as, RAY_FLAG_NONE, 0xff, searchRay);

        
    }
}