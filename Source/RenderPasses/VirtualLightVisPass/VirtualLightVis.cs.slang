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

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    float gRadius;
    VirtualLightContainer gVirtualLightContainer;
};

Texture2D<float4> gPos;
RWTexture2D<float4> gOutput;

[numthreads(16, 16, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (any(launchIndex.xy >= gViewportDims)) 
    {
        return;
    }
    
    float4 gPosVal  = gPos[launchIndex.xy];
    float3 posW     = gPosVal.xyz;
    float  opacity  = gPosVal.w;
    gOutput[launchIndex.xy] = float4(0.0f, 0.0f, 0.0f, 1.0f);
    if(opacity > 0.0f)
    {
        RayDesc dummyRay;
        dummyRay.Origin = posW;
        dummyRay.Direction = float3(0.0, 1.0, 0.0);
        dummyRay.TMin = 0.0f;
        dummyRay.TMax = 0.0000001f;

        RayQuery<RAY_FLAG_NONE> rayQuery;

        rayQuery.TraceRayInline(
            gVirtualLightContainer.as,
            RAY_FLAG_NONE, // OR'd with template flags above
            0xff, // InstanceInclusionMask
            dummyRay);

        while (rayQuery.Proceed())
        {
            if(rayQuery.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
            {
                uint AABBIndex = rayQuery.CandidatePrimitiveIndex();
                PackedBoundingBox boundingBox = gVirtualLightContainer.boundingBoxBuffer[AABBIndex];
                float3 vplOrigin = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
                
                float3 origin = posW;
                float dis = sqrt(dot(origin - vplOrigin, origin - vplOrigin));
                if(dis <= gRadius)
                {
                    gOutput[launchIndex.xy] = float4(1.0f, 1.0f, 1.0f, 1.0f);
                    rayQuery.Abort();
                    return;
                }
            }
        }
    }
}
