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
        var["dataBuffer"] = mpDataBuffer;
    }

    Shader::DefineList MegaTextureContainer::getDefineList() const
    {
        Shader::DefineList defineList = Shader::DefineList();
        defineList.add("MEGA_TEXTURE_CONTAINER_CAPACITY", std::to_string(mCapacity));
        defineList.add("MEGA_TEXTURE_CONTAINER_PER_ITEM_SIZE", std::to_string(mPerItemSize));
        return defineList;
    }

    MegaTextureContainer::MegaTextureContainer(uint32_t capacity, uint perItemSize):
        mCapacity(capacity),
        mPerItemSize(perItemSize)
    {
        /* RGB Float Stride = 12 */
        mpDataBuffer = Buffer::create(12lu * mPerItemSize * mPerItemSize * capacity);
    }
}
