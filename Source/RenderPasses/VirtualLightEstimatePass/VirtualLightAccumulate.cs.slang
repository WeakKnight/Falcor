import Utils.VirtualLight.MegaTextureContainer;

/*
Macros:
_PER_FRAME_PATH_COUNT
_INV_PER_FRAME_PATH_COUNT
*/

cbuffer CB
{
    uint gAccumulatedCount; 
    uint gVirtualLightCount;
    MegaTextureContainer gCurSpecRadianceContainer;
    MegaTextureContainer gSpecRadianceContainer;
};

RWStructuredBuffer<float3> gCurDiffuseRadianceBuffer;
RWStructuredBuffer<float3> gDiffuseRadianceBuffer;

RWStructuredBuffer<float> gCurFluxBuffer;
RWStructuredBuffer<float> gFluxBuffer;

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
    if (launchIndex.x >= gVirtualLightCount)
    {
        return;
    }

    uint lightIndex = launchIndex.x;

    /*
    Accumulate Specular Radiance
    */
    for(uint u = 0; u < 8; u++)
    {
        for(uint v = 0; v < 8; v++)
        {
            uint address = lightIndex * gCurSpecRadianceContainer.perItemStride +  (v * gCurSpecRadianceContainer.perItemSize + u) * 12u;
            
            uint3 oldData = gSpecRadianceContainer.dataBuffer.Load3(address);
            float3 oldCol = float3(asfloat(oldData.r), asfloat(oldData.g), asfloat(oldData.b));
            uint3 curData = gCurSpecRadianceContainer.dataBuffer.Load3(address);
            float3 curCol = float3(asfloat(curData.r), asfloat(curData.g), asfloat(curData.b));
            
            float3 newCol = (gAccumulatedCount * oldCol + curCol) / (gAccumulatedCount + 1);

            gSpecRadianceContainer.dataBuffer.Store3(address, uint3(asuint(newCol.r), asuint(newCol.g), asuint(newCol.b)));
            gCurSpecRadianceContainer.dataBuffer.Store3(address, uint3(asuint(0.0f), asuint(0.0f), asuint(0.0f)));
        }
    }

    /*
    Accumulate Diffuse Radiance
    */
    {
        float3 oldCol = gDiffuseRadianceBuffer[lightIndex];
        float3 curCol = gCurDiffuseRadianceBuffer[lightIndex];

        float3 newCol = (gAccumulatedCount * oldCol + curCol) / (gAccumulatedCount + 1);
        
        gDiffuseRadianceBuffer[lightIndex] = newCol;
        gCurDiffuseRadianceBuffer[lightIndex] = 0.0f;
    }

    /*
    Accumulate Flux
    */
    {
        float oldFlux = gFluxBuffer[lightIndex];
        float curFlux = gCurFluxBuffer[lightIndex];

        float newFlux = (gAccumulatedCount * oldFlux + curFlux) / (gAccumulatedCount + 1);
        
        gFluxBuffer[lightIndex] = newFlux;
        gCurFluxBuffer[lightIndex] = 0.0f;
    }
}
