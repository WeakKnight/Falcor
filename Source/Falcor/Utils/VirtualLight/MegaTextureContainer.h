#pragma once

namespace Falcor
{
    class dlldecl MegaTextureContainer
    {
    public:
        using SharedPtr = std::shared_ptr<MegaTextureContainer>;
        static SharedPtr create(uint32_t capacity, uint perItemSize);
        void setShaderData(const ShaderVar& var) const;

    private:
        MegaTextureContainer(uint32_t capacity, uint perItemSize);
        uint32_t mCapacity;
        uint32_t mPerItemSize;
        Buffer::SharedPtr mpDataBuffer;
    };
}
