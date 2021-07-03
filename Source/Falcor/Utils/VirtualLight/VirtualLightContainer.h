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

        void updateCounterToCPU(RenderContext* renderContext);
        void buildBVH(RenderContext* renderContext);

        void setCount(RenderContext* renderContext, uint count);
        uint32_t getCount() const { return mCount; }

    private:
        VirtualLightContainer(uint32_t capacity, float boundingBoxRadius);
        uint32_t mCapacity;
        uint32_t mCount;
        float mBoundingBoxRadius;
        Buffer::SharedPtr mpPositionBuffer;
        Buffer::SharedPtr mpHitInfoBuffer;
        Buffer::SharedPtr mpBoundBoxBuffer;
        BoundingBoxAccelerationStructureBuilder::SharedPtr mpAccelerationStructureBuilder;
    };
}

