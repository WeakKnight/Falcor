#include "stdafx.h"
#include "VirtualLightContainer.h"

namespace Falcor
{
    VirtualLightContainer::SharedPtr VirtualLightContainer::create(uint32_t capacity, float boundingBoxRadius)
    {
        return SharedPtr(new VirtualLightContainer(capacity, boundingBoxRadius));
    }

    void VirtualLightContainer::setShaderData(const ShaderVar& var) const
    {
        var["positionBuffer"] = mpPositionBuffer;
        var["hitInfoBuffer"] = mpPositionBuffer;
        var["boundingBoxBuffer"] = mpPositionBuffer;
        if (mHaveAS)
        {
            mpAccelerationStructureBuilder->SetRaytracingShaderData(var, "as", 1);
        }
        var["count"] = mCount;
        var["boundingBoxRadius"] = mBoundingBoxRadius;
    }

    void VirtualLightContainer::updateCounterToCPU(RenderContext* renderContext)
    {
        Buffer::SharedPtr counterReadBuffer = Buffer::create(sizeof(uint), ResourceBindFlags::None, Buffer::CpuAccess::Read);
        Buffer::SharedPtr counterBuffer = mpPositionBuffer->getUAVCounter();

        renderContext->copyBufferRegion(counterReadBuffer.get(), 0, counterBuffer.get(), 0, counterBuffer->getSize());
        renderContext->flush(true);

        uint* data = (uint*)counterReadBuffer->map(Buffer::MapType::Read);
        mCount = data[0];
        counterReadBuffer->unmap();
    }

    void VirtualLightContainer::buildAS(RenderContext* renderContext)
    {
        mpAccelerationStructureBuilder->BuildAS(renderContext, mCount, 1);
        mHaveAS = true;
    }

    void VirtualLightContainer::setCount(RenderContext* renderContext, uint count)
    {
        renderContext->clearUAVCounter(mpPositionBuffer, count);
        mCount = count;
    }

    VirtualLightContainer::VirtualLightContainer(uint32_t capacity, float boundingBoxRadius)
        :mCapacity(capacity),
        mCount(0),
        mBoundingBoxRadius(boundingBoxRadius)
    {
        mpPositionBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpHitInfoBuffer = Buffer::createStructured(sizeof(uint2), mCapacity);
        mpBoundBoxBuffer = Buffer::createStructured(sizeof(float) * 8, mCapacity);

        mpAccelerationStructureBuilder = BoundingBoxAccelerationStructureBuilder::Create(mpBoundBoxBuffer);
    }
}
