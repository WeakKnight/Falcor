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
#include "VirtualLightGeneratePass.h"


namespace
{
    const char kDesc[] = "Place initial virtual light samples";
    const char kRaySampleNum[] = "raySampleNum";
    const char kBoundBoxRadius[] = "boundBoxRadius";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VirtualLightGeneratePass", kDesc, VirtualLightGeneratePass::create);
}

VirtualLightGeneratePass::SharedPtr VirtualLightGeneratePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightGeneratePass);
    for (const auto& [key, value] : dict)
    {
        if (key == kRaySampleNum)
        {
            pPass->mRaySampleNum = value;
        }
        else if (key == kBoundBoxRadius)
        {
            pPass->mBoundBoxRadius = value;
        }
    }
    pPass->mpVirtualLightContainer = VirtualLightContainer::create(pPass->mRaySampleNum, pPass->mBoundBoxRadius);
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/VirtualLightGeneratePass/VirtualLightGenerate.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);
    return pPass;
}

std::string VirtualLightGeneratePass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightGeneratePass::getScriptingDictionary()
{
    Dictionary d;
    d[kRaySampleNum] = mRaySampleNum;
    d[kBoundBoxRadius] = mBoundBoxRadius;
    return d;
}

RenderPassReflection VirtualLightGeneratePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput("aldebo", "aldedo texture from GBuffer");
    reflector.addOutput("accelerationStructureBuffer", "accelerationStructureBuffer for ray tracing");
    return reflector;
}

void VirtualLightGeneratePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    renderData.getDictionary()["initialVirtualLights"] = mpVirtualLightContainer;

    ShaderVar cb = mpComputePass["CB"];
    mpVirtualLightContainer->setShaderData(cb["VirtualLightContainer"]);
    mpComputePass->execute(pRenderContext, uint3(mRaySampleNum, 1, 1));
    mpVirtualLightContainer->updateCounterToCPU(pRenderContext);
}

void VirtualLightGeneratePass::renderUI(Gui::Widgets& widget)
{
    mRaySampleNum = widget.var("Ray Sample Num", mRaySampleNum, 100u, 10000000u);
    mBoundBoxRadius = widget.var("Bound Box Radius", mBoundBoxRadius, 0.0f, 1.0f);
}

void VirtualLightGeneratePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    if (mpScene)
    {
        // Prepare our programs for the scene.
        Shader::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");

        mpComputePass->getProgram()->addDefines(defines);
        mpComputePass->setVars(nullptr); // Trigger recompile
    }
}
