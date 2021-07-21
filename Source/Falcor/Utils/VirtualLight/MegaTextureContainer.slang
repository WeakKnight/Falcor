import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;

#define NV_SHADER_EXTN_SLOT u63
#define NV_SHADER_EXTN_REGISTER_SPACE space0
#include "NVAPI/nvHLSLExtns.h"

float2 sphericalCoordinateToUV(float3 dir)
{
    float3 p = normalize(dir);
    float2 uv;
    p /= dot(float3(1.0f), abs(p));
    return 0.5 * float2(p.x - p.y + 1.0, -p.x - p.y + 1.0);
}

float3 UVToHemisphere(uint2 uvPos, float stepSize)
{
    float2 uv = float2(uvPos.x * stepSize + 0.5 * stepSize, uvPos.y * stepSize + 0.5 * stepSize);
    float2 xy = float2((uv.x - uv.y), (1.0f - uv.y - uv.x));
    
    float2 absXY = abs(xy);
    float3 pPrime = float3(xy.x, xy.y, 1.0 - absXY.x - absXY.y);
    
    return normalize(pPrime * dot(float3(1.0f, 1.0f, 1.0f), abs(pPrime)));
}

float3 toLocal(float3 v, float3 N)
{
    float3 T = 0.0;
    if (abs(N.x) > 0.1)
    {
        T = normalize(cross(float3(0.0, 1.0, 0.0), N));
    }
    else
    {
        T = normalize(cross(float3(1.0, 0.0, 0.0), N));
    }
    float3 B = cross(N, T);

    return float3(dot(v, T), dot(v, B), dot(v, N));
}

float3 fromLocal(float3 v, float3 N)
{
    float3 T = 0.0;
    if (abs(N.x) > 0.1)
    {
        T = normalize(cross(float3(0.0, 1.0, 0.0), N));
    }
    else
    {
        T = normalize(cross(float3(1.0, 0.0, 0.0), N));
    }
    float3 B = cross(N, T);

    return v.x * T + v.y * B + v.z * N;
}

struct MegaTextureContainer
{
    uint capacity;
    uint perItemSize;
    uint perItemStride;
    float stepSize;
    RWByteAddressBuffer dataBuffer;

    void interlockAddColor(uint itemIndex, uint2 uv, float3 color)
    {
        uint address = itemIndex * perItemStride +  (uv.y * perItemSize + uv.x) * 12u;
        NvInterlockedAddFp32(dataBuffer, address + 0, color.r);
        NvInterlockedAddFp32(dataBuffer, address + 4, color.g);
        NvInterlockedAddFp32(dataBuffer, address + 8, color.b);
    }

    void interlockAddColorBiliear(uint itemIndex, float2 uv, float3 color)
    {
        float2 floatUV = uv * (perItemSize) - 0.5;
        floatUV = float2(clamp(floatUV.x, 0.0, perItemSize - 1.0), clamp(floatUV.y, 0.0, perItemSize - 1.0));
        
        uint2 topLeft = floor(floatUV);
        uint2 bottomRight = topLeft + uint2(1, 1);
        bottomRight = uint2(min(bottomRight.x, perItemSize - 1), min(bottomRight.y, perItemSize - 1));

        float2 fracPos = floatUV - floor(floatUV);
        
        float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
        float areaTopRight = fracPos.x * (1.0 - fracPos.y);
        float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
        float areaBottomRight = fracPos.x * fracPos.y;

        float3 colTopLeft = color * areaTopLeft;
        float3 colTopRight = color * areaTopRight;
        float3 colBottomLeft = color * areaBottomLeft;
        float3 colBottomRight = color * areaBottomRight;

        interlockAddColor(itemIndex, uint2(topLeft.x, topLeft.y), colTopLeft);
        interlockAddColor(itemIndex, uint2(bottomRight.x, topLeft.y), colTopRight);
        interlockAddColor(itemIndex, uint2(topLeft.x, bottomRight.y), colBottomLeft);
        interlockAddColor(itemIndex, uint2(bottomRight.x, bottomRight.y), colBottomRight);
    }

    float3 readColor(uint itemIndex, uint2 uv)
    {
        uint address = itemIndex * perItemStride +  (uv.y * perItemSize + uv.x) * 12u;
        float3 result = asfloat(dataBuffer.Load3(address));
        return result;
    }

    float3 readColorBiliear(uint itemIndex, float2 uv)
    {
        float2 floatUV = uv * perItemSize - 0.5;
        floatUV = float2(clamp(floatUV.x, 0.0, perItemSize - 1.0), clamp(floatUV.y, 0.0, perItemSize - 1.0));
        
        uint2 topLeft = floor(floatUV);
        uint2 bottomRight = topLeft + uint2(1, 1);
        bottomRight = uint2(min(bottomRight.x, perItemSize - 1), min(bottomRight.y, perItemSize - 1));

        float3 colTopLeft = readColor(block, uint2(topLeft.x, topLeft.y));
        float3 colTopRight = readColor(block, uint2(bottomRight.x, topLeft.y));
        float3 colBottomLeft = readColor(block, uint2(topLeft.x, bottomRight.y));
        float3 colBottomRight = readColor(block, uint2(bottomRight.x, bottomRight.y));

        float2 fracPos = floatUV - floor(floatUV);
        
        float areaTopLeft = (1.0 - fracPos.x) * (1.0 - fracPos.y);
        float areaTopRight = fracPos.x * (1.0 - fracPos.y);
        float areaBottomLeft = (1.0 - fracPos.x) * fracPos.y;
        float areaBottomRight = fracPos.x * fracPos.y;

        return areaTopLeft * colTopLeft + areaTopRight * colTopRight + areaBottomLeft * colBottomLeft + areaBottomRight * colBottomRight;
    }
}