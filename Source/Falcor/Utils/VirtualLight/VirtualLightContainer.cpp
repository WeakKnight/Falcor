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
        var["normalBuffer"] = mpNormalBuffer;
        var["faceNormalBuffer"] = mpFaceNormalBuffer;
        var["texC"] = mpTexCoordBuffer;
        var["instanceIdBuffer"] = mpInstanceIdBuffer;
        var["boundingBoxBuffer"] = mpBoundBoxBuffer;
        if (mHaveAS)
        {
            mpAccelerationStructureBuilder->SetRaytracingShaderData(var, "as", 1);
        }
        var["count"] = mCount;
        var["boundingBoxRadius"] = mBoundingBoxRadius;
    }

    void VirtualLightContainer::setRayTracingData(const ShaderVar& var, const std::string name) const
    {
        mpAccelerationStructureBuilder->SetRaytracingShaderData(var, name, 1);
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

    VirtualLightContainer::VirtualLightContainer(uint32_t capacity, float boundingBoxRadius):
        mCapacity(capacity),
        mCount(0),
        mBoundingBoxRadius(boundingBoxRadius)
    {
        mpPositionBuffer = Buffer::createStructured(sizeof(float3), mCapacity);
        mpNormalBuffer = Buffer::createStructured(sizeof(uint), mCapacity);;
        mpFaceNormalBuffer = Buffer::createStructured(sizeof(uint), mCapacity);;
        mpTexCoordBuffer = Buffer::createStructured(sizeof(uint), mCapacity);;
        mpInstanceIdBuffer = Buffer::createStructured(sizeof(uint), mCapacity);;
        mpBoundBoxBuffer = Buffer::createStructured(sizeof(float) * 8, mCapacity);

        mpAccelerationStructureBuilder = BoundingBoxAccelerationStructureBuilder::Create(mpBoundBoxBuffer);
    }
}
