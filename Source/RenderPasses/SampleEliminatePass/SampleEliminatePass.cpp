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
#include "SampleEliminatePass.h"
#include "cyCore.h"
#include "cyHeap.h"
#include "cyPointCloud.h"
#include <tbb/parallel_for.h>

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "get a blue-noise-distributed set of samples";
    const char kRatio[] = "ratio";
    const char kRadiusSearchRange[] = "radiusSearchRange";
    const char kRadiusSearchCount[] = "radiusSearchCount";
    const char kRadius[] = "radius";
    
    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicSampleEliminatedVirtualLights = "sampleEliminatedVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

static void regPythonApi(pybind11::module& m)
{
    pybind11::class_<SampleEliminatePass, RenderPass, SampleEliminatePass::SharedPtr> pass(m, "SampleEliminatePass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("SampleEliminatePass", kDesc, SampleEliminatePass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

SampleEliminatePass::SharedPtr SampleEliminatePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new SampleEliminatePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kRatio)
        {
            pPass->mRatio = value;
        }
        else if (key == kRadiusSearchRange)
        {
            pPass->mRadiusSearchRange = value;
        }
        else if (key == kRadiusSearchCount)
        {
            pPass->mRadiusSearchCount = value;
        }
        else if (key == kRadius)
        {
            pPass->mRadius = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/SampleEliminatePass/SampleEliminate.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);
    return pPass;
}

std::string SampleEliminatePass::getDesc()
{
    return kDesc;
}

Dictionary SampleEliminatePass::getScriptingDictionary()
{
    Dictionary d;
    d[kRatio] = mRatio;
    d[kRadiusSearchRange] = mRadiusSearchRange;
    d[kRadiusSearchCount] = mRadiusSearchCount;
    d[kRadius] = mRadius;
    return d;
}

RenderPassReflection SampleEliminatePass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy input");
    reflector.addOutput(kDummyOutput, "useless dummy output");
    return reflector;
}

void SampleEliminatePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    VirtualLightContainer::SharedPtr initialVirtualLights = renderData.getDictionary()[kDicInitialVirtualLights];
    if (initialVirtualLights == nullptr)
    {
        debugBreak(); // should not be nullptr here
        return;
    }

    if (mpSampleEliminatedVirtualLights == nullptr)
    {
        uint targetCount = (float)initialVirtualLights->getCount() * mRatio;
        mpSampleEliminatedVirtualLights = VirtualLightContainer::create(targetCount, initialVirtualLights->getBoundingBoxRadius());

        std::vector<uint32_t> outputIndices;
        outputIndices.resize(targetCount);
        std::vector<float3> outputPositions;
        outputPositions.resize(targetCount);
        eliminatie(pRenderContext, initialVirtualLights, targetCount, outputIndices, outputPositions);

        mpSampleEliminatedVirtualLights->getPositionBuffer()->setBlob(outputPositions.data(), 0, outputPositions.size() * sizeof(float3));
        mpSampleEliminatedVirtualLights->setCount(pRenderContext, targetCount);
        pRenderContext->flush(true);
        ShaderVar cb = mpComputePass["CB"];
        initialVirtualLights->setShaderData(cb["gInitialVirtualLights"]);
        mpSampleEliminatedVirtualLights->setShaderData(cb["gSampleEliminatedVirtualLights"]);
        Buffer::SharedPtr indicesBuffer = Buffer::createStructured(sizeof(uint), targetCount, ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, outputIndices.data());
        mpComputePass["gIndices"] = indicesBuffer;
        mpComputePass->execute(pRenderContext, uint3(targetCount, 1, 1));
        pRenderContext->flush(true);
        mpSampleEliminatedVirtualLights->buildAS(pRenderContext);
    }

    renderData.getDictionary()[kDicCurVirtualLights] = mpSampleEliminatedVirtualLights;
    renderData.getDictionary()[kDicSampleEliminatedVirtualLights] = mpSampleEliminatedVirtualLights;
}

void SampleEliminatePass::renderUI(Gui::Widgets& widget)
{
    widget.var("Ratio", mRatio, 0.02f, 1.0f);
    widget.var("RadiusSearchRange", mRadiusSearchRange, 0.0f, 1.0f);
    widget.var("RadiusSearchCount", mRadiusSearchCount, 0u, 1000u);
    widget.var("Radius", mRadius, 0.0f, 1.0f);
}

void SampleEliminatePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");

        mpComputePass->getProgram()->addDefines(defines);
        mpComputePass->setVars(nullptr); // Trigger recompile
    }
}

void SampleEliminatePass::eliminatie(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr initialVirtualLights, uint targetCount, std::vector<uint>& outputIndices, std::vector<float3>& outputPositions)
{
    uint inputCount = initialVirtualLights->getCount();
    Buffer::SharedPtr positionReadBuffer = Buffer::create(initialVirtualLights->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, initialVirtualLights->getPositionBuffer().get(), 0, initialVirtualLights->getPositionBuffer()->getSize());
    pRenderContext->flush(true);
    float3* inputPositions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);
    {
        cy::PointCloud<float3, float, 3, uint> kdtree;
        kdtree.Build(inputCount, inputPositions);

        float ratio = float(inputCount) / float(targetCount);

        auto getPoissonDiskRadius = [&](float3 pos)
        {
            uint actualCount;
            float actualSquaredRadius;
            kdtree.PointsSearchExtRadiusFirst(pos, mRadiusSearchCount, mRadiusSearchRange, actualCount, actualSquaredRadius);
            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
            float result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
            result = std::min(mRadius, result);
            return result;
        };

        std::vector<float> dMaxList(inputCount);
        std::vector<float> reverseDMaxList(inputCount);
        tbb::parallel_for(0u, inputCount, 1u, [&](uint i)
        {
            dMaxList[i] = 2.0f * getPoissonDiskRadius(inputPositions[i]);
            reverseDMaxList[i] = dMaxList[i];
        });

        auto weightFunction = [&](float d2, float dMax)
        {
            float d = sqrt(d2);
            const float alpha = 8.0f;
            return std::pow(1.0 - d / dMax, alpha);
        };

        std::vector<float> weights(inputCount, 0.0f);
        for (uint index = 0; index < inputCount; index++)
        {
            float3 point = inputPositions[index];
            float dMax = dMaxList[index];
            kdtree.GetPoints(point, dMaxList[index], [&](uint i, float3 const& p, float d2, float&)
            {
                if (i >= inputCount)
                {
                    return;
                }
                if (i != index)
                {
                    float weight = weightFunction(d2, dMax);
                    weights[index] += weight;
                    if (reverseDMaxList[i] < dMax)
                    {
                        reverseDMaxList[i] = dMax;
                    }
                }
            });
        }

        cy::Heap<float, uint> maxHeap;
        maxHeap.SetDataPointer(weights.data(), inputCount);
        maxHeap.Build();

        uint remainCount = inputCount;
        while (remainCount > targetCount)
        {
            uint index = maxHeap.GetTopItemID();
            maxHeap.Pop();
            float3 point = inputPositions[index];
            kdtree.GetPoints(point, reverseDMaxList[index], [&](uint i, float3 const& p, float d2, float&)
            {
                if (i > inputCount)
                {
                    return;
                }
                if (i != index)
                {
                    float dMax = dMaxList[i];
                    if (dMax * dMax > d2)
                    {
                        float weight = weightFunction(d2, dMax);
                        weights[i] -= weight;
                        maxHeap.MoveItemDown(i);
                    }
                }
            });
            remainCount--;
        }
        for (uint i = 0; i < targetCount; i++)
        {
            outputIndices[i] = maxHeap.GetIDFromHeap(i);
            outputPositions[i] = inputPositions[outputIndices[i]];
        }
    }
    positionReadBuffer->unmap();
}


