/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"

using namespace Falcor;

class SampleEliminatePass : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<SampleEliminatePass>;

    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    void eliminatie(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr initialVirtualLights, uint targetCount, std::vector<uint>& outputIndices, std::vector<float3>& outputPositions, std::vector<float>& dmaxs);

private:
    SampleEliminatePass() = default;
    float   mRatio = 0.2f;
    float   mRadiusSearchRange = 0.37f;
    uint    mRadiusSearchCount = 350;
    float   mRadius = 0.05f;
    bool    mUniformSE = false;
    float   mRadiusScalerForASBuilding = 1.5f;
    bool    mUseDMaxForASBuilding = false;
    Scene::SharedPtr                    mpScene;
    SampleGenerator::SharedPtr          mpSampleGenerator;
    ComputePass::SharedPtr              mpComputePass;
    VirtualLightContainer::SharedPtr    mpSampleEliminatedVirtualLights;
};


struct comparatorFunc
{
    bool operator ()(const uint2& c1, const uint2& c2) const
    {
        if (c1.x != c2.x)
            return c1.x < c2.x;
        if (c1.y != c2.y)
            return c1.y < c2.y;
        return false;
    }
};

class multiplePartition
{
public:
    class multiplePartition(float3& min, float3& max)
    {
        boundingBoxMin = min;
        boundingBoxMax = max;
        diagonalVector = max - min;
    }
    uint3 putIntoPartitionOne(float3 &point)
    {
        uint3 gridIndex;
        float3 relativePos = point - boundingBoxMin;
        if (relativePos.x <= diagonalVector.x * 0.5f)
            gridIndex.x = 0;
        else
            gridIndex.x = 1;
        if (relativePos.y <= diagonalVector.y * 0.5f)
            gridIndex.y = 0;
        else
            gridIndex.y = 1;
        if (relativePos.z <= diagonalVector.z * 0.5f)
            gridIndex.z = 0;
        else
            gridIndex.z = 1;

        return gridIndex;

    }

    uint3 putIntoPartitionTwo(float3& point)
    {
        uint3 gridIndex;
        float3 relativePos = point - boundingBoxMin;
        if (relativePos.x <= diagonalVector.x / 3.0f)
            gridIndex.x = 0;
        else if(relativePos.x <= diagonalVector.x * 2.0f/ 3.0f)
            gridIndex.x = 1;
        else
            gridIndex.x = 2;

        if (relativePos.y <= diagonalVector.y / 3.0f)
            gridIndex.y = 0;
        else if (relativePos.y <= diagonalVector.y * 2.0f / 3.0f)
            gridIndex.y = 1;
        else
            gridIndex.y = 2;

        if (relativePos.z <= diagonalVector.z / 3.0f)
            gridIndex.z = 0;
        else if (relativePos.z <= diagonalVector.z * 2.0f / 3.0f)
            gridIndex.z = 1;
        else
            gridIndex.z = 2;

        return gridIndex;
    }

    int linearIndexForPartitionOne(uint3 gridIndex)
    {
        return gridIndex.x + gridIndex.y * 2 + gridIndex.z * 4;

    }

    int linearIndexForPartitionTwo(uint3 gridIndex)
    {
        return gridIndex.x + gridIndex.y * 3 + gridIndex.z * 9;
    }

private:
    float3 boundingBoxMin;
    float3 boundingBoxMax;
    float3 diagonalVector;


};


