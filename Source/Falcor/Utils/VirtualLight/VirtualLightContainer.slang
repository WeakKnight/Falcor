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
    RWStructuredBuffer<uint> normalBuffer;
    RWStructuredBuffer<uint> faceNormalBuffer;
    RWStructuredBuffer<uint> texC;
    RWStructuredBuffer<uint> instanceIdBuffer;

    RWStructuredBuffer<PackedBoundingBox> boundingBoxBuffer;
    RaytracingAccelerationStructure as;
    float boundingBoxRadius;
    uint count;

    uint pushLight(VertexData v, uint instanceId)
    {
        uint ind = positionBuffer.IncrementCounter();
        setVertexData(ind, v);
        setInstanceId(ind, instanceId);
 
        float3 radius = float3(boundingBoxRadius);
        boundingBoxBuffer[ind].minPoint = v.posW - radius;
        boundingBoxBuffer[ind].maxPoint = v.posW + radius;

        return ind;
    }

    /*
    Pos Getter/Setter
    */
    inline float3 getPosition(uint i)
    {
        return positionBuffer[i];
    }
    inline void setPosition(uint i, float3 pos)
    {
        positionBuffer[i] = pos;
    }

    /*
    Shading Normal Getter/Setter
    */
    inline float3 getNormal(uint i)
    {
        return decodeNormal2x16(normalBuffer[i]);
    }
    inline void setNormal(uint i, float3 normal)
    {
        normalBuffer[i] = encodeNormal2x16(normal);
    }

    /*
    Face Normal Getter/Setter
    */
    inline float3 getFaceNormal(uint i)
    {
        return decodeNormal2x16(faceNormalBuffer[i]);
    }
    inline void setFaceNormal(uint i, float3 normal)
    {
        faceNormalBuffer[i] = encodeNormal2x16(normal);
    }

    /* 
    Texture Coord Getter/Setter
    */
    inline float2 getTexC(uint i)
    {
        return float2(faceNormalBuffer[i] & 0xffff, faceNormalBuffer[i] >> 16) / float(0xffff);
    }
    inline void setTexC(uint i, float2 uv)
    {
        texC[i] = uint(saturate(uv.x) * 0xffff) | (uint(saturate(uv.y) * 0xffff) << 16);
    }

    /*
    Vertex Data Getter/Setter
    */
    VertexData getVertexData(uint i)
    {
        VertexData v;
        v.posW = getPosition(i);
        v.normalW = getNormal(i);
        v.texC = getTexC(i);
        v.faceNormalW = getFaceNormal(i);
        return v;
    }
    void setVertexData(uint i, VertexData v)
    {
        positionBuffer[i] = v.posW;
        setNormal(i, v.normalW);
        setTexC(i, v.texC);
        setFaceNormal(i, v.faceNormalW);
    }

    /*
    id Getter/Setter
    */
    uint getInstanceId(uint i)
    {
        return instanceIdBuffer[i];
    }
    void setInstanceId(uint i, uint instanceId)
    {
        instanceIdBuffer[i] = instanceId;
    }

    inline float getRadius(uint i)
    {
        return (boundingBoxBuffer[i].maxPoint.x - boundingBoxBuffer[i].minPoint.x) * 0.5f;
    }

    ShadingData getShadingData(uint i)
    {
        ShadingData sd;
        VertexData v = getVertexData(i);
        const uint materialID = gScene.getMaterialID(getInstanceId(i));
        sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], 0.f, 0.f);
        adjustShadingNormal(sd, v);
        return sd;
    }
}
