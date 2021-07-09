#pragma once
#include "BoundingBoxAccelerationStructureBuilder.h"

namespace Falcor
{
    class dlldecl VirtualLightContainer
    {
    public:
        using SharedPtr = std::shared_ptr<VirtualLightContainer>;
        static SharedPtr create(uint32_t capacity, float boundingBoxRadius);
        void setShaderData(const ShaderVar& var) const;
        void setRayTracingData(const ShaderVar& var, const std::string name) const;
        void updateCounterToCPU(RenderContext* renderContext);
        void buildAS(RenderContext* renderContext);

        void setCount(RenderContext* renderContext, uint count);
        uint32_t getCount() const { return mCount; }
        float getBoundingBoxRadius() const { return mBoundingBoxRadius; }
        Buffer::SharedPtr getPositionBuffer() const { return  mpPositionBuffer; }

    private:
        VirtualLightContainer(uint32_t capacity, float boundingBoxRadius);
        uint32_t mCapacity;
        uint32_t mCount;
        float mBoundingBoxRadius;
        bool mHaveAS = false;
        Buffer::SharedPtr mpPositionBuffer;
        Buffer::SharedPtr mpHitInfoBuffer;
        Buffer::SharedPtr mpBoundBoxBuffer;
        BoundingBoxAccelerationStructureBuilder::SharedPtr mpAccelerationStructureBuilder;
    };
}

