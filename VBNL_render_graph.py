from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    GBufferRT = createPass('GBufferRT', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'disableAlphaTest': False, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'texLOD': LODMode.UseMip0})
    g.addPass(GBufferRT, 'GBufferRT')
    VirtualLightGeneratePass = createPass('VirtualLightGeneratePass', {'raySampleNum': 360000, 'boundBoxRadius': 0.006000000052154064})
    g.addPass(VirtualLightGeneratePass, 'VirtualLightGeneratePass')
    VirtualLightVisPass = createPass('VirtualLightVisPass', {'radius': 0.004000000189989805})
    g.addPass(VirtualLightVisPass, 'VirtualLightVisPass')
    SampleEliminatePass = createPass('SampleEliminatePass', {'ratio': 0.07999999821186066, 'radiusSearchRange': 0.20000000298023224, 'radiusSearchCount': 350, 'radius': 0.05000000074505806, 'uniformSE': False, 'radiusScalerForASBuilding': 1.5, 'useDMaxForASBuilding': False})
    g.addPass(SampleEliminatePass, 'SampleEliminatePass')
    g.addEdge('VirtualLightGeneratePass.dummy', 'SampleEliminatePass.input')
    g.addEdge('SampleEliminatePass.output', 'VirtualLightVisPass.dummy')
    g.addEdge('GBufferRT.posW', 'VirtualLightVisPass.pos')
    g.markOutput('VirtualLightVisPass.output')
    return g

DefaultRenderGraph = render_graph_DefaultRenderGraph()
try: m.addGraph(DefaultRenderGraph)
except NameError: None
