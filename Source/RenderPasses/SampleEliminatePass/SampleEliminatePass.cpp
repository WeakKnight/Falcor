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
#include <tbb/task_scheduler_init.h>
#include <limits>

#define parallel false

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
    const char kUniformSE[] = "uniformSE";
    const char kRadiusScalerForASBuilding[] = "radiusScalerForASBuilding";
    const char kUseDMaxForASBuilding[] = "useDMaxForASBuilding";
    
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
        else if (key == kUniformSE)
        {
            pPass->mUniformSE = value;
        }
        else if (key == kRadiusScalerForASBuilding)
        {
            pPass->mRadiusScalerForASBuilding = value;
        }
        else if (key == kUseDMaxForASBuilding)
        {
            pPass->mUseDMaxForASBuilding = value;
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
    d[kUniformSE] = mUniformSE;
    d[kRadiusScalerForASBuilding] = mRadiusScalerForASBuilding;
    d[kUseDMaxForASBuilding] = mUseDMaxForASBuilding;
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
        outputIndices.reserve(targetCount);
        std::vector<float3> outputPositions;
        outputPositions.reserve(targetCount);
        std::vector<float> dmaxs;
        dmaxs.reserve(targetCount);
        eliminatie(pRenderContext, initialVirtualLights, targetCount, outputIndices, outputPositions, dmaxs);

        mpSampleEliminatedVirtualLights->getPositionBuffer()->setBlob(outputPositions.data(), 0, outputPositions.size() * sizeof(float3));
        mpSampleEliminatedVirtualLights->setCount(pRenderContext, targetCount);
        pRenderContext->flush(true);
        ShaderVar cb = mpComputePass["CB"];
        cb["gUseDMaxForASBuilding"] = mUseDMaxForASBuilding;
        cb["gRadiusScalerForASBuilding"] = mRadiusScalerForASBuilding;
        initialVirtualLights->setShaderData(cb["gInitialVirtualLights"]);
        mpSampleEliminatedVirtualLights->setShaderData(cb["gSampleEliminatedVirtualLights"]);
        Buffer::SharedPtr indicesBuffer = Buffer::createStructured(sizeof(uint), outputIndices.size(), ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, outputIndices.data());
        Buffer::SharedPtr dMaxBuffer = Buffer::createStructured(sizeof(float), dmaxs.size(), ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, dmaxs.data());
        mpComputePass["gIndices"] = indicesBuffer;
        mpComputePass["gDMaxs"] = dMaxBuffer;
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
    widget.checkbox("UniformSE", mUniformSE);
    widget.var("RadiusScalerForASBuilding", mRadiusScalerForASBuilding, 0.0f, 10.0f);
    widget.checkbox("UseDMaxForASBuilding", mUseDMaxForASBuilding);
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

void SampleEliminatePass::eliminatie(RenderContext* pRenderContext, VirtualLightContainer::SharedPtr initialVirtualLights, uint targetCount, std::vector<uint>& outputIndices, std::vector<float3>& outputPositions, std::vector<float>& dmaxs)
{
    uint inputCount = initialVirtualLights->getCount();
    Buffer::SharedPtr positionReadBuffer = Buffer::create(initialVirtualLights->getPositionBuffer()->getSize(), Resource::BindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(positionReadBuffer.get(), 0, initialVirtualLights->getPositionBuffer().get(), 0, initialVirtualLights->getPositionBuffer()->getSize());
    pRenderContext->flush(true);
    float3* inputPositions = (float3*)positionReadBuffer->map(Buffer::MapType::Read);

    if (parallel)
    {
        float ratio = float(inputCount) / float(targetCount);

        float3 boundingBoxMin(std::numeric_limits<float>::max());
        float3 boundingBoxMax(-std::numeric_limits<float>::max());
        // build the large boundingbox
        {
            //to do: parallel reduction, now just in a bruce-force manner
            //int num_threads = tbb::task_scheduler_init::default_num_threads();
            for (int i = 0; i < inputCount; i++)
            {
                float3 thisPoint = inputPositions[i];

                if (thisPoint.x < boundingBoxMin.x) boundingBoxMin.x = thisPoint.x;
                if (thisPoint.y < boundingBoxMin.y) boundingBoxMin.y = thisPoint.y;
                if (thisPoint.z < boundingBoxMin.z) boundingBoxMin.z = thisPoint.z;

                if (thisPoint.x > boundingBoxMax.x) boundingBoxMax.x = thisPoint.x;
                if (thisPoint.y > boundingBoxMax.y) boundingBoxMax.y = thisPoint.y;
                if (thisPoint.z > boundingBoxMax.z) boundingBoxMax.z = thisPoint.z;
            }
        }

        multiplePartition mp(boundingBoxMin, boundingBoxMax);

        //seperate the box with 2*2*2 grid cells or 3*3*3 grid cells
        // 
        //global indices
        std::vector<uint> partitionOneIndices[8];
        std::vector<uint> partitionTwoIndices[27];
        //positions
        std::vector<float3> partitionOne[8];
        std::vector<float3> partitionTwo[27];
        //weights
        std::vector<float> partitionOneWeights[8];
        std::vector<float> partitionTwoWeights[27];
        //kdtrees
        cy::PointCloud<float3, float, 3, uint> kdtrees1[8];
        cy::PointCloud<float3, float, 3, uint> kdtrees2[27];

        for (int i = 0; i < 8; i++)
        {
            partitionOneIndices[i].reserve(int(inputCount / 8.0f));
            partitionOne[i].reserve(int(inputCount / 8.0f));
        }

        for (int i = 0; i < 27; i++)
        {
            partitionTwoIndices[i].reserve(int(inputCount / 27.0f));
            partitionTwo[i].reserve(int(inputCount / 27.0f));
        }

        std::map<uint2, uint2, comparatorFunc> partitionMapping;
        std::map<uint2, uint2, comparatorFunc> partitionMapping2;


        //initialization phase 1
        for (uint index = 0; index < inputCount; index++)
        {
            float3 point = inputPositions[index];

            uint3 gridindexForP1 = mp.putIntoPartitionOne(point);
            uint linearIndex1 = mp.linearIndexForPartitionOne(gridindexForP1);

            uint3 gridindexForP2 = mp.putIntoPartitionTwo(point);
            uint linearIndex2 = mp.linearIndexForPartitionTwo(gridindexForP2);

            partitionOneIndices[linearIndex1].push_back(index);
            partitionOne[linearIndex1].push_back(point);


            partitionTwoIndices[linearIndex2].push_back(index);
            partitionTwo[linearIndex2].push_back(point);

            uint2 mappingKey = uint2(linearIndex1, partitionOneIndices[linearIndex1].size()-1);
            uint2 mappingValue = uint2(linearIndex2, partitionTwoIndices[linearIndex2].size()-1);

            partitionMapping.insert(std::pair<uint2, uint2>(mappingKey, mappingValue));
            partitionMapping2.insert(std::pair<uint2, uint2>(mappingValue, mappingKey));
        }

        //initialization phase 2
        std::vector<float> dMaxList1[8];
        std::vector<float> reverseDMaxList1[8];

        std::vector<float> dMaxList2[27];
        std::vector<float> reverseDMaxList2[27];

        for (int i = 0; i < 8; i++)
        {
            kdtrees1[i].Build(partitionOne[i].size(), partitionOne[i].data());
            dMaxList1[i].resize(partitionOne[i].size());
            reverseDMaxList1[i].resize(partitionOne[i].size());
            partitionOneWeights[i].resize(partitionOne[i].size());
        }

        for (int i = 0; i < 27; i++)
        {
            kdtrees2[i].Build(partitionTwo[i].size(), partitionTwo[i].data());
            dMaxList2[i].resize(partitionTwo[i].size());
            reverseDMaxList2[i].resize(partitionTwo[i].size());
            partitionTwoWeights[i].resize(partitionTwo[i].size());
        }


        auto getPoissonDiskRadiusInP1 = [&](float3 pos,uint i)
        {
            uint actualCount;
            float actualSquaredRadius;
            kdtrees1[i].PointsSearchExtRadiusFirst(pos, mRadiusSearchCount, mRadiusSearchRange, actualCount, actualSquaredRadius);
            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
            float result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
            result = std::max(std::min(mRadius, result), 1e-6f);
            return result;
        };

        auto getPoissonDiskRadiusInP2 = [&](float3 pos, uint i)
        {
            uint actualCount;
            float actualSquaredRadius;
            kdtrees2[i].PointsSearchExtRadiusFirst(pos, mRadiusSearchCount, mRadiusSearchRange, actualCount, actualSquaredRadius);
            float sampleDomainArea = ratio * cy::Pi<float>() * actualSquaredRadius / (float)actualCount;
            float result = sqrt(sampleDomainArea / (2.0 * sqrt(3.0)));
            result = std::max(std::min(mRadius, result), 1e-6f);
            return result;
        };

        for (int j = 0; j < 8; j++)
        {
            uint count = partitionOne[j].size();
            tbb::parallel_for(0u, count, 1u, [&](uint i)
            {
                if (mUniformSE)
                {
                    dMaxList1[j][i] = (2.0f * mRadius);
                }
                else
                {
                    dMaxList1[j][i] = (2.0f * getPoissonDiskRadiusInP1(partitionOne[j].at(i), j));
                }
                reverseDMaxList1[j][i] = (dMaxList1[j].at(i));
            });
        }

        for (int j = 0; j < 27; j++)
        {
            uint count = partitionTwo[j].size();
            tbb::parallel_for(0u, count, 1u, [&](uint i)
            {
                if (mUniformSE)
                {
                    dMaxList2[j][i] = (2.0f * mRadius);
                }
                else
                {
                    dMaxList2[j][i] = (2.0f * getPoissonDiskRadiusInP2(partitionTwo[j].at(i), j));
                }
                reverseDMaxList2[j][i] = (dMaxList2[j].at(i));
            });
        }
        //initialization phase 3
        auto weightFunction = [&](float d2, float dMax)
        {
            float d = sqrt(d2);
            const float alpha = 8.0f;
            return std::pow(1.0 - d / dMax, alpha);
        };

        for (int j = 0; j < 8; j++)
        {
            uint count = partitionOne[j].size();
            tbb::parallel_for(0u, count, 1u, [&](uint index)
            {
                float3 point = partitionOne[j][index];
                float dMax = dMaxList1[j][index];
                kdtrees1[j].GetPoints(point, dMax, [&](uint i, float3 const& p, float d2, float&)
                {
                    if (i >= count)
                    {
                        return;
                    }
                    if (i != index)
                    {
                        float weight = weightFunction(d2, dMax);
                        partitionOneWeights[j][index] += weight;
                        if (reverseDMaxList1[j][i] < dMax)
                        {
                            reverseDMaxList1[j][i] = dMax;
                        }
                    }
                });
                
            });
        }

        for (int j = 0; j < 27; j++)
        {
            uint count = partitionTwo[j].size();
            tbb::parallel_for(0u, count, 1u, [&](uint index)
            {
                float3 point = partitionTwo[j][index];
                float dMax = dMaxList2[j][index];
                kdtrees2[j].GetPoints(point, dMax, [&](uint i, float3 const& p, float d2, float&)
                {
                    if (i >= count)
                    {
                        return;
                    }
                    if (i != index)
                    {
                        float weight = weightFunction(d2, dMax);
                        partitionTwoWeights[j][index] += weight;
                        if (reverseDMaxList2[j][i] < dMax)
                        {
                            reverseDMaxList2[j][i] = dMax;
                        }
                    }
                });

            });
        }

        
        cy::Heap<float, uint> maxHeaps1[8];
        cy::Heap<float, uint> maxHeaps2[27];

        for (int i = 0; i < 8; i++)
        {
            maxHeaps1[i].SetDataPointer(partitionOneWeights[i].data(), partitionOne[i].size());
            maxHeaps1[i].Build();
        }

        for (int i = 0; i < 27; i++)
        {
            maxHeaps2[i].SetDataPointer(partitionTwoWeights[i].data(), partitionTwo[i].size());
            maxHeaps2[i].Build();
        }
        uint remainCount = inputCount;

        bool stopAtPartitionTwo = true;
        while (remainCount > targetCount)
        {
            int2 deletedPoints1[8];
            float3 deletedPointsPos1[8];

            tbb::parallel_for(0u, 8u, 1u, [&](uint grid)
            {
                uint num = maxHeaps1[grid].NumItemsInHeap();
                if (num > 0)
                {
                    uint index = maxHeaps1[grid].GetTopItemID();
                    maxHeaps1[grid].Pop();

                    float3 point = partitionOne[grid].at(index);

                    uint maxCount = partitionOne[grid].size();
                    kdtrees1[grid].GetPoints(point, reverseDMaxList1[grid][index], [&](uint i, float3 const& p, float d2, float&)
                    {
                        if (i > maxCount)
                        {
                            return;
                        }
                        if (i != index)
                        {
                            float dMax = dMaxList1[grid][i];
                            if (dMax * dMax > d2)
                            {
                                float weight = weightFunction(d2, dMax);
                                partitionOneWeights[grid].at(i) -= weight;
                                maxHeaps1[grid].MoveItemDown(i);
                            }
                        }
                    });
                    auto iter = partitionMapping.find(uint2(grid, index));
                    deletedPoints1[grid] = iter->second;
                    deletedPointsPos1[grid] = point;
                }
                else
                {
                    deletedPoints1[grid] = int2(-1, -1);
                }
              
            });

            tbb::parallel_for(0u, 27u, 1u, [&](uint grid)
            {
                std::vector<uint> indices;
                for (int i = 0; i < 8; i++)
                {
                    if (deletedPoints1[i].x == grid)
                        indices.push_back(i);
                }

                for (int i = 0; i < indices.size(); i++)
                {
                    uint2 mappedIndex = deletedPoints1[indices.at(i)];
                    uint index = mappedIndex.y;

                    maxHeaps2[grid].SetItem(index, std::numeric_limits<float>::max());
                    maxHeaps2[grid].MoveItemUp(index);
                    maxHeaps2[grid].Pop();


                    uint maxCount = partitionTwo[grid].size();
                    kdtrees2[grid].GetPoints(deletedPointsPos1[indices.at(i)], reverseDMaxList2[grid][index], [&](uint i, float3 const& p, float d2, float&)
                    {
                        if (i > maxCount)
                        {
                            return;
                        }
                        if (i != index)
                        {
                            float dMax = dMaxList2[grid][i];
                            if (dMax * dMax > d2)
                            {
                                float weight = weightFunction(d2, dMax);
                                partitionTwoWeights[grid].at(i) -= weight;
                                maxHeaps2[grid].MoveItemDown(i);
                            }
                        }
                    });

                }

            });

            remainCount -= 8;
            if (remainCount < targetCount)
            {
                stopAtPartitionTwo = false;
                break;
            }   

            int2 deletedPoints2[27];
            float3 deletedPointsPos2[27];

            tbb::parallel_for(0u, 27u, 1u, [&](uint grid)
            {
                uint num = maxHeaps2[grid].NumItemsInHeap();
                if (num > 0)
                {
                    uint index = maxHeaps2[grid].GetTopItemID();
                    maxHeaps2[grid].Pop();

                    float3 point = partitionTwo[grid].at(index);
                    uint maxCount = partitionTwo[grid].size();
                    kdtrees2[grid].GetPoints(point, reverseDMaxList2[grid][index], [&](uint i, float3 const& p, float d2, float&)
                    {
                        if (i > maxCount)
                        {
                            return;
                        }
                        if (i != index)
                        {
                            float dMax = dMaxList2[grid][i];
                            if (dMax * dMax > d2)
                            {
                                float weight = weightFunction(d2, dMax);
                                partitionTwo[grid].at(i) -= weight;
                                maxHeaps2[grid].MoveItemDown(i);
                            }
                        }
                    });

                    auto iter = partitionMapping2.find(uint2(grid, index));
                    deletedPoints2[grid] = iter->second;
                    deletedPointsPos2[grid] = point;

                }
                else
                {
                    deletedPoints2[grid] = int2(-1, -1);
                }
                
            });
            

            tbb::parallel_for(0u, 8u, 1u, [&](uint grid)
            {
                std::vector<uint> indices;
                for (int i = 0; i < 27; i++)
                {
                    if (deletedPoints2[i].x == grid)
                        indices.push_back(i);
                }

                for (int i = 0; i < indices.size(); i++)
                {
                    uint2 mappedIndex = deletedPoints2[indices.at(i)];
                    uint index = mappedIndex.y;

                    maxHeaps1[grid].SetItem(index, std::numeric_limits<float>::max());
                    maxHeaps1[grid].MoveItemUp(index);
                    maxHeaps1[grid].Pop();
                    uint maxCount = partitionOne[grid].size();
                    kdtrees1[grid].GetPoints(deletedPointsPos2[indices.at(i)], reverseDMaxList1[grid][index], [&](uint i, float3 const& p, float d2, float&)
                    {
                        if (i > maxCount)
                        {
                            return;
                        }
                        if (i != index)
                        {
                            float dMax = dMaxList1[grid][i];
                            if (dMax * dMax > d2)
                            {
                                float weight = weightFunction(d2, dMax);
                                partitionOneWeights[grid].at(i) -= weight;
                                maxHeaps1[grid].MoveItemDown(i);
                            }
                        }
                    });

                }

            });

            remainCount -= 27;
        }

        if (stopAtPartitionTwo)
        {
            for (uint i = 0; i < 27; i++)
            {
                uint num = maxHeaps2[i].NumItemsInHeap();
                for (int j = 0; j < num; j++)
                {
                    uint localIndex = maxHeaps2[i].GetIDFromHeap(i);
                    outputIndices.push_back(partitionTwoIndices[i][localIndex]);
                    outputPositions.push_back(partitionTwo[i][localIndex]);
                    dmaxs.push_back(dMaxList2[i][localIndex]);

                }

            }

        }
        else
        {
            for (uint i = 0; i < 8; i++)
            {
                uint num = maxHeaps1[i].NumItemsInHeap();
                for (int j = 0; j < num; j++)
                {
                    uint localIndex = maxHeaps1[i].GetIDFromHeap(i);
                    outputIndices.push_back(partitionOneIndices[i][localIndex]);
                    outputPositions.push_back(partitionOne[i][localIndex]);
                    dmaxs.push_back(dMaxList1[i][localIndex]);

                }

            }
            
        }

    }
    else
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
            result = std::max(std::min(mRadius, result), 1e-6f);
            return result;
        };

        std::vector<float> dMaxList(inputCount);
        std::vector<float> reverseDMaxList(inputCount);
        tbb::parallel_for(0u, inputCount, 1u, [&](uint i)
        {
            if (mUniformSE)
            {
                dMaxList[i] = 2.0f * mRadius;
            }
            else
            {
                dMaxList[i] = 2.0f * getPoissonDiskRadius(inputPositions[i]);
            }
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
            outputIndices.push_back(maxHeap.GetIDFromHeap(i));
            outputPositions.push_back(inputPositions[outputIndices[i]]);
            dmaxs.push_back(dMaxList[outputIndices[i]]);
        }
    }
    positionReadBuffer->unmap();
}


