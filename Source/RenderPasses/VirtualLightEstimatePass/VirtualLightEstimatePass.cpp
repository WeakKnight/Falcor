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
#include "VirtualLightEstimatePass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDummyInput[] = "input";
    const char kDummyOutput[] = "output";
    const char kDesc[] = "Insert pass description here";

    const char kPhotonPathCount[] = "Photon Path Count";
    const char kTextureItemSize[] = "Texture Item Size";

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
    pybind11::class_<VirtualLightEstimatePass, RenderPass, VirtualLightEstimatePass::SharedPtr> pass(m, "VirtualLightEstimatePass");
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VirtualLightEstimatePass", kDesc, VirtualLightEstimatePass::create);
    ScriptBindings::registerBinding(regPythonApi);
}

VirtualLightEstimatePass::SharedPtr VirtualLightEstimatePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightEstimatePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kPhotonPathCount)
        {
            pPass->mPhotonPathCount = value;
        }
        else if (key == kTextureItemSize)
        {
            pPass->mTextureItemSize = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/VirtualLightEstimatePass/VirtualLightEstimate.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);
    return pPass;
}

std::string VirtualLightEstimatePass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightEstimatePass::getScriptingDictionary()
{
    Dictionary d;
    d[kPhotonPathCount] = mPhotonPathCount;
    d[kTextureItemSize] = mTextureItemSize;
    return d;
}

RenderPassReflection VirtualLightEstimatePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummyInput, "useless dummy Input");
    reflector.addOutput(kDummyOutput, "useless dummy Output");
    return reflector;
}

void VirtualLightEstimatePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    VirtualLightContainer::SharedPtr sampleEliminatedVirtualLights = renderData.getDictionary()[kDicSampleEliminatedVirtualLights];
    if (sampleEliminatedVirtualLights == nullptr)
    {
        debugBreak(); // should not be nullptr here
        return;
    }

    if (mpRadianceContainer == nullptr)
    {
        const uint capacity = 200000;
        if (sampleEliminatedVirtualLights->getCount() > capacity)
        {
            debugBreak(); // should not be nullptr here
            logError("exceed radiance container's maximum size");
            return;
        }
        mpRadianceContainer = MegaTextureContainer::create(capacity, 8);
    }

    /*
    Construct Alias Table For Emissive Triangles
    */
    if (mpEmissiveTriTable == nullptr)
    {
        auto lightCollection = mpScene->getLightCollection(pRenderContext);
        auto emissiveTris = lightCollection->getMeshLightTriangles();
        std::vector<float> weights(emissiveTris.size());
        for (size_t i = 0; i < emissiveTris.size(); i++)
        {
            weights[i] = emissiveTris[i].flux;
        }
        std::mt19937 rng;
        mpEmissiveTriTable = AliasTable::create(weights, rng);
    }

    ShaderVar cb = mpComputePass["CB"];
    cb["gFrameIndex"] = gpFramework->getGlobalClock().getFrame();
    sampleEliminatedVirtualLights->setShaderData(cb["gVirtualLightContainer"]);
    mpRadianceContainer->setShaderData(cb["gRadianceContainer"]);
    mpEmissiveTriTable->setShaderData(cb["gEmissiveTriTable"]);
    mpScene->setRaytracingShaderData(pRenderContext, mpComputePass->getRootVar());

    mpComputePass->execute(pRenderContext, uint3(mPhotonPathCount, 1, 1));
}

void VirtualLightEstimatePass::renderUI(Gui::Widgets& widget)
{
}

void VirtualLightEstimatePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_PER_FRAME_PATH_COUNT", std::to_string(mPhotonPathCount));
        defines.add("_INV_PER_FRAME_PATH_COUNT", std::to_string(1.0f / (float)mPhotonPathCount));

        mpComputePass->getProgram()->addDefines(defines);
        mpComputePass->setVars(nullptr); // Trigger recompile
    }
}


