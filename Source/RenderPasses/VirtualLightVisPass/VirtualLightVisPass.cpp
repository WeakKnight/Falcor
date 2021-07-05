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
#include "VirtualLightVisPass.h"
#include "Utils/VirtualLight/VirtualLightContainer.h"

namespace
{
    const char kDummy[] = "dummy";
    const char kDesc[] = "Insert pass description here";
    const char kRadius[] = "radius";
    
    const char kPosChannel[] = "pos";
    const char kOutputChannel[] = "output";

    const std::string kDicInitialVirtualLights = "initialVirtualLights";
    const std::string kDicCurVirtualLights = "curVirtualLights";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("VirtualLightVisPass", kDesc, VirtualLightVisPass::create);
}

VirtualLightVisPass::SharedPtr VirtualLightVisPass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new VirtualLightVisPass);
    for (const auto& [key, value] : dict)
    {
        if (key == kRadius)
        {
            pPass->mRadius = value;
        }
    }
    pPass->mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    Program::Desc desc;
    desc.addShaderLibrary("RenderPasses/VirtualLightVisPass/VirtualLightVis.cs.slang").csEntry("main").setShaderModel("6_5");
    pPass->mpComputePass = ComputePass::create(desc, Program::DefineList(), false);

    return pPass;
}

std::string VirtualLightVisPass::getDesc()
{
    return kDesc;
}

Dictionary VirtualLightVisPass::getScriptingDictionary()
{
    Dictionary d;
    d[kRadius] = mRadius;
    return d;
}

RenderPassReflection VirtualLightVisPass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput(kDummy, "");
    reflector.addInput(kPosChannel, "pos texture").bindFlags(Falcor::ResourceBindFlags::UnorderedAccess | Falcor::ResourceBindFlags::ShaderResource);
    reflector.addOutput(kOutputChannel, "output texture").bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource).format(ResourceFormat::RGBA32Float);
    return reflector;
}

void VirtualLightVisPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    Texture::SharedPtr pPos = renderData[kPosChannel]->asTexture();
    Texture::SharedPtr pDst = renderData[kOutputChannel]->asTexture();
    VirtualLightContainer::SharedPtr curVirtualLights = renderData.getDictionary()[kDicCurVirtualLights];

    if (curVirtualLights == nullptr)
    {
        debugBreak(); // should not be nullptr here
        return;
    }

    ShaderVar cb = mpComputePass["CB"];
    cb["gViewportDims"] = uint2(mpScene->getCamera()->getFrameWidth(), mpScene->getCamera()->getFrameHeight());
    cb["gFrameIndex"] = gpFramework->getGlobalClock().getFrame();
    cb["gRadius"] = 0.1f;
    curVirtualLights->setShaderData(cb["gVirtualLightContainer"]);
    mpComputePass["gPos"] = pPos;
    mpComputePass["gOutput"] = pDst;

    mpComputePass->execute(pRenderContext, uint3(mpScene->getCamera()->getFrameWidth(), mpScene->getCamera()->getFrameHeight(), 1));
}

void VirtualLightVisPass::renderUI(Gui::Widgets& widget)
{
    widget.var("Vis Radius", mRadius, 0.0f, 1.0f);
}

void VirtualLightVisPass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
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
