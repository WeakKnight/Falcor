# Graphs
from falcor import *

def render_graph_DefaultRenderGraph():
    g = RenderGraph('DefaultRenderGraph')
    loadRenderPassLibrary('VirtualLightVisPass.dll')
    loadRenderPassLibrary('DepthPass.dll')
    loadRenderPassLibrary('VirtualLightGeneratePass.dll')
    loadRenderPassLibrary('GBuffer.dll')
    loadRenderPassLibrary('SampleEliminatePass.dll')
    GBufferRT = createPass('GBufferRT', {'samplePattern': SamplePattern.Center, 'sampleCount': 16, 'disableAlphaTest': False, 'adjustShadingNormals': True, 'forceCullMode': False, 'cull': CullMode.CullBack, 'texLOD': LODMode.UseMip0})
    g.addPass(GBufferRT, 'GBufferRT')
    VirtualLightGeneratePass = createPass('VirtualLightGeneratePass', {'raySampleNum': 360000, 'boundBoxRadius': 0.006000000052154064})
    g.addPass(VirtualLightGeneratePass, 'VirtualLightGeneratePass')
    VirtualLightVisPass = createPass('VirtualLightVisPass', {'radius': 0.004000000189989805, 'visMode': 0})
    g.addPass(VirtualLightVisPass, 'VirtualLightVisPass')
    SampleEliminatePass = createPass('SampleEliminatePass', {'ratio': 0.07999999821186066, 'radiusSearchRange': 0.20000000298023224, 'radiusSearchCount': 350, 'radius': 0.05000000074505806, 'uniformSE': False, 'radiusScalerForASBuilding': 1.5, 'useDMaxForASBuilding': False})
    g.addPass(SampleEliminatePass, 'SampleEliminatePass')
    g.addEdge('VirtualLightGeneratePass.dummy', 'SampleEliminatePass.input')
    g.addEdge('SampleEliminatePass.output', 'VirtualLightVisPass.dummy')
    g.addEdge('GBufferRT.posW', 'VirtualLightVisPass.pos')
    g.markOutput('VirtualLightVisPass.output')
    return g
m.addGraph(render_graph_DefaultRenderGraph())

# Scene
m.loadScene('VPLMedia/CornellBox/CornellBox.pyscene')
m.scene.renderSettings = SceneRenderSettings(useEnvLight=True, useAnalyticLights=True, useEmissiveLights=True, useVolumes=True)
m.scene.camera.position = float3(-0.124434,1.056123,2.594023)
m.scene.camera.target = float3(-0.100168,1.000824,1.595849)
m.scene.camera.up = float3(-0.000045,0.999998,0.001848)
m.scene.cameraSpeed = 1.0

# Window Configuration
m.resizeSwapChain(1920, 1080)
m.ui = True

# Clock Settings
m.clock.time = 0
m.clock.framerate = 0
# If framerate is not zero, you can use the frame property to set the start frame
# m.clock.frame = 0

# Frame Capture
m.frameCapture.outputDir = '.'
m.frameCapture.baseFilename = 'Mogwai'

