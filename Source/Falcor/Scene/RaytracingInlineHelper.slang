import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Lights.Lights;
import Scene.Shading;
import Utils.Helpers;
#include "Utils/Math/MathConstants.slangh"
import Utils.Math.PackedFormats;
import Utils.Math.MathHelpers;

bool traceScene(float3 rayOrigin, float3 rayDir, out ShadingData sd, out HitInfo hit)
{
    RayDesc ray;
	ray.Origin = rayOrigin;
	ray.Direction = rayDir;
	ray.TMin = 0.0f;
	ray.TMax = 1.0e38f;

    RayQuery<RAY_FLAG_NONE> rayQuery;

    rayQuery.TraceRayInline(
        gRtScene,
        RAY_FLAG_NONE,
        0xff,
        ray);

    while (rayQuery.Proceed())
    {
        if (rayQuery.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            // Alpha test for non-opaque geometry.
            hit = getCandidateTriangleHit(rayQuery);
            VertexData v = gScene.getVertexData(hit);
            uint materialID = gScene.getMaterialID(hit.instanceID);

            if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) 
            {
                continue;
            }

            rayQuery.CommitNonOpaqueTriangleHit();
        }
    }

    if(rayQuery.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        hit = getCommittedTriangleHit(rayQuery);

        uint meshInstanceID = hit.instanceID;
        uint materialID = gScene.getMaterialID(meshInstanceID);

        float3 barycentrics = float3(1.f - hit.barycentrics.x - hit.barycentrics.y, hit.barycentrics.x, hit.barycentrics.y);
        VertexData v = gScene.getVertexData(hit.instanceID, hit.primitiveIndex, barycentrics);
        sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayQuery.WorldRayDirection(), 0.f);
        return true;
    }

    return false;
}

bool traceScene(float3 rayOrigin, float3 rayDir, out ShadingData sd)
{
    HitInfo hit;
    return traceScene(rayOrigin, rayDir, sd, hit);
}
