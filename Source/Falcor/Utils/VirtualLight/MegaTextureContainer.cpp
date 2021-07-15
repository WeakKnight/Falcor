#include "stdafx.h"
#include "MegaTextureContainer.h"

namespace Falcor
{
    MegaTextureContainer::SharedPtr MegaTextureContainer::create(uint32_t capacity, uint perItemSize)
    {
        return SharedPtr(new MegaTextureContainer(capacity, perItemSize));
    }

    void MegaTextureContainer::setShaderData(const ShaderVar& var) const
    {
        var["capacity"] = mCapacity;
        var["perItemSize"] = mPerItemSize;
        var["perItemStride"] = mPerItemSize * mPerItemSize * 12;
        var["stepSize"] = 1.0f / (float)mPerItemSize;
        var["dataBuffer"] = mpDataBuffer;
    }
    
    MegaTextureContainer::MegaTextureContainer(uint32_t capacity, uint perItemSize):
        mCapacity(capacity),
        mPerItemSize(perItemSize)
    {
        /* RGB Float Stride = 12 */
        mpDataBuffer = Buffer::create(static_cast<size_t>(12) * static_cast<size_t>(mPerItemSize) * static_cast<size_t>(mPerItemSize) * static_cast<size_t>(capacity));
    }
}
