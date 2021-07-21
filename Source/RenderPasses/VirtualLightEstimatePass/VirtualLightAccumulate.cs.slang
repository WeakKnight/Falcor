import Utils.VirtualLight.MegaTextureContainer;

/*
Macros:
_PER_FRAME_PATH_COUNT
_INV_PER_FRAME_PATH_COUNT
*/

cbuffer CB
{
    uint gAccumulatedCount; 
    MegaTextureContainer gCurSpecRadianceContainer;
    MegaTextureContainer gSpecRadianceContainer;
};

RWByteAddressBuffer gCurDiffuseRadianceBuffer;
RWByteAddressBuffer gCurFluxBuffer;
RWByteAddressBuffer gDiffuseRadianceBuffer;
RWByteAddressBuffer gFluxBuffer;

[numthreads(256, 1, 1)]
void main(uint3 launchIndex : SV_DispatchThreadID)
{
      
}
