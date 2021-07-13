from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    loadRenderPassLibrary('SceneDebugger.dll')
    loadRenderPassLibrary('BSDFViewer.dll')
    loadRenderPassLibrary('AccumulatePass.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('Antialiasing.dll')
    loadRenderPassLibrary('BlitPass.dll')
    loadRenderPassLibrary('DebugPasses.dll')
    loadRenderPassLibrary('CSM.dll')
    loadRenderPassLibrary('ErrorMeasurePass.dll')
    loadRenderPassLibrary('ForwardLightingPass.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('ImageLoader.dll')
    loadRenderPassLibrary('MegakernelPathTracer.dll')
    loadRenderPassLibrary('MinimalPathTracer.dll')
    loadRenderPassLibrary('PassLibraryTemplate.dll')
    loadRenderPassLibrary('PixelInspectorPass.dll')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    loadRenderPassLibrary('SSAO.dll')
    loadRenderPassLibrary('SkyBox.dll')
    loadRenderPassLibrary('SVGFPass.dll')
    loadRenderPassLibrary('TemporalDelayPass.dll')
    loadRenderPassLibrary('ToneMapper.dll')
    loadRenderPassLibrary('Utils.dll')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    loadRenderPassLibrary('WhittedRayTracer.dll')
    GBufferRT = createPass('GBufferRT', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'disableAlphaTest': False, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'texLOD': LODMode.UseMip0})
    g.addPass(GBufferRT, 'GBufferRT')
    VirtualLightGeneratePass = createPass('VirtualLightGeneratePass', {'raySampleNum': 360000, 'boundBoxRadius': 0.006000000052154064})
    g.addPass(VirtualLightGeneratePass, 'VirtualLightGeneratePass')
    VirtualLightVisPass = createPass('VirtualLightVisPass', {'radius': 0.004000000189989805, 'visMode': 0})
    g.addPass(VirtualLightVisPass, 'VirtualLightVisPass')
    SampleEliminatePass = createPass('SampleEliminatePass', {'ratio': 0.17000000178813934, 'radiusSearchRange': 0.3700000047683716, 'radiusSearchCount': 350, 'radius': 0.05000000074505806, 'radiusScalerForASBuilding': 1.5, 'useDMaxForASBuilding': False})
    g.addPass(SampleEliminatePass, 'SampleEliminatePass')
    g.addEdge('VirtualLightGeneratePass.dummy', 'SampleEliminatePass.input')
    g.addEdge('SampleEliminatePass.output', 'VirtualLightVisPass.dummy')
    g.addEdge('GBufferRT.posW', 'VirtualLightVisPass.pos')
    g.markOutput('VirtualLightVisPass.output')
    return g

DefaultRenderGraph = render_graph_DefaultRenderGraph()
try: m.addGraph(DefaultRenderGraph)
except NameError: None
