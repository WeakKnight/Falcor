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
import Utils.VirtualLight.VirtualLightContainer;
import Experimental.Scene.Material.MaterialShading;
import Scene.RaytracingInlineHelper;

cbuffer CB
{
    uint gFrameIndex;
    uint gRaySampleNum;
    VirtualLightContainer gVirtualLightContainer;
}

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if(launchIndex.x >= gRaySampleNum)
    {
        return;
    }

    SampleGenerator sg = SampleGenerator.create(launchIndex.xy, gFrameIndex);
    CameraRay ray = gScene.camera.sampleRayPinhole(sampleNext2D(sg));

    ShadingData sd;
    bool hit = traceScene(ray.origin, ray.dir, sd);
    if (hit)
    {
        BSDFSample result;
        bool valid;
        valid = sampleBSDF(sd, sg, result);
        if (valid)
        {
            float3 offsetPos = computeRayOrigin(sd.posW, sd.faceN);
            ShadingData sdNext;
            VertexData v;
            uint instanceId;
            hit = traceScene(offsetPos, result.wi, v, instanceId);
            if (hit)
            {
                gVirtualLightContainer.pushLight(v, instanceId);
            }
        }
    }
}
