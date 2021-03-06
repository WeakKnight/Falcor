import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;
import Utils.VirtualLight.VirtualLightContainer;

cbuffer CB
{
    bool gUseDMaxForASBuilding;
    float gRadiusScalerForASBuilding;
    VirtualLightContainer gInitialVirtualLights;
    VirtualLightContainer gSampleEliminatedVirtualLights;
};

StructuredBuffer<uint> gIndices;
StructuredBuffer<float> gDMaxs;

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if(launchIndex.x >= gSampleEliminatedVirtualLights.count)
    {
        return;
    }

    uint eliminatedVPLIndex = launchIndex.x;
    float3 posW = gSampleEliminatedVirtualLights.getPosition(eliminatedVPLIndex);

    float3 radius = float3(gUseDMaxForASBuilding? gRadiusScalerForASBuilding * gDMaxs[eliminatedVPLIndex]: gSampleEliminatedVirtualLights.boundingBoxRadius);
    gSampleEliminatedVirtualLights.boundingBoxBuffer[eliminatedVPLIndex].minPoint = posW - radius;
    gSampleEliminatedVirtualLights.boundingBoxBuffer[eliminatedVPLIndex].maxPoint = posW + radius;

    uint originIndex = gIndices[eliminatedVPLIndex];
    gSampleEliminatedVirtualLights.setNormal(eliminatedVPLIndex, gInitialVirtualLights.getNormal(originIndex));
    gSampleEliminatedVirtualLights.setFaceNormal(eliminatedVPLIndex, gInitialVirtualLights.getFaceNormal(originIndex));
    gSampleEliminatedVirtualLights.setTexC(eliminatedVPLIndex, gInitialVirtualLights.getTexC(originIndex));
    gSampleEliminatedVirtualLights.setInstanceId(eliminatedVPLIndex, gInitialVirtualLights.getInstanceId(originIndex));
}
