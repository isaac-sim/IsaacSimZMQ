// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: MIT

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdUtils/stageCache.h>

#include <carb/logging/Log.h>

#include <omni/graph/core/PreUsdInclude.h>
#include <omni/graph/core/PostUsdInclude.h>
#include <omni/fabric/FabricUSD.h>

#include <OgnIsaacBridgeZMQCameraDatabase.h>


using omni::graph::core::Type;
using omni::graph::core::BaseDataType;


namespace isaacsim {
namespace zmq {
namespace bridge {


class OgnIsaacBridgeZMQCamera
{
    // int m_evaluationCount{ 0 };

public:
    OgnIsaacBridgeZMQCamera()
    {
        CARB_LOG_INFO("OgnIsaacBridgeZMQCamera::constructor\n");
    }
    ~OgnIsaacBridgeZMQCamera()
    {
        CARB_LOG_INFO("OgnIsaacBridgeZMQCamera::destructor\n");
    }


    static bool compute(OgnIsaacBridgeZMQCameraDatabase& db);

    static pxr::GfMatrix4d get_view_matrix_ros(const pxr::UsdPrim& cameraPrim);

    static pxr::GfMatrix3d get_intrinsics_matrix(const pxr::UsdPrim& cameraPrim, uint32_t width, uint32_t height);

};


pxr::GfMatrix4d OgnIsaacBridgeZMQCamera::get_view_matrix_ros(const pxr::UsdPrim& cameraPrim)
{
    // c++ implementation omni.isaac.sensor.camera.Camera.get_view_matrix_ros() by ai :)

    // Step 1: Get the world-to-camera transformation matrix
    pxr::UsdGeomXformable xformable(cameraPrim);
    pxr::GfMatrix4d local_to_world_tf = xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());

    // Convert the matrix type from double to float and transpose
    pxr::GfMatrix4d world_w_cam_u_T = local_to_world_tf.GetTranspose();

    // Step 2: Define R_U_TRANSFORM as a fixed transformation matrix
    static const pxr::GfMatrix4d r_u_transform_converted(
        1,  0,  0, 0,
        0, -1,  0, 0,
        0,  0, -1, 0,
        0,  0,  0, 1
    );

    // Step 3: Perform matrix inversion using GfMatrix4f's Inverse() method
    pxr::GfMatrix4d inverse_world_w_cam_u_T = world_w_cam_u_T.GetInverse();

    // Step 4: Perform matrix multiplication of r_u_transform_converted and inverse_world_w_cam_u_T
    pxr::GfMatrix4d result_matrix = r_u_transform_converted * inverse_world_w_cam_u_T;

    // Return the final view matrix
    return result_matrix;


    // results from a python call:
    // [[ 8.8442159e-01 -4.1348362e-01 -2.1640186e-01  0.0000000e+00]
    // [-4.0163556e-01 -9.1051155e-01  9.8272912e-02  0.0000000e+00]
    // [-2.3767065e-01 -3.2561711e-09 -9.7134584e-01  0.0000000e+00]
    // [ 0.0000000e+00  0.0000000e+00  0.0000000e+00  1.0000000e+00]]

    //results from this call - appear to be 10x smaller...
    // 0.884422 -0.413484 -0.216402 0.000000
    // -0.401636 -0.910512 0.098273 0.000000
    // -0.237671 0.000000 -0.971346 0.000000
    // 0.000000 0.000000 0.000000 1.000000
}


pxr::GfMatrix3d OgnIsaacBridgeZMQCamera::get_intrinsics_matrix(const pxr::UsdPrim& cameraPrim, uint32_t width, uint32_t height)
{
    // c++ implementation omni.isaac.sensor.camera.Camera.get_intrinsics_matrix() by ai :)

    // Get attributes for focal length and horizontal aperture
    pxr::UsdGeomCamera camera(cameraPrim);

    pxr::VtValue focal_length_value;
    camera.GetFocalLengthAttr().Get(&focal_length_value, pxr::UsdTimeCode::Default());

    static constexpr float scale_factor = 10.0f;  // Used in multiple places
    float focal_length = focal_length_value.Get<float>() / scale_factor;

    pxr::VtValue horizontal_aperture_value;
    camera.GetHorizontalApertureAttr().Get(&horizontal_aperture_value, pxr::UsdTimeCode::Default());
    float horizontal_aperture = horizontal_aperture_value.Get<float>() / scale_factor;

    float vertical_aperture = horizontal_aperture * (static_cast<float>(height) / width);

    // Calculate intrinsic parameters
    float fx = width * focal_length / horizontal_aperture;
    float fy = height * focal_length / vertical_aperture;
    float cx = width * 0.5f;
    float cy = height * 0.5f;

    // Return the intrinsic matrix as pxr::GfMatrix3f
    return pxr::GfMatrix3d(fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);
}


bool OgnIsaacBridgeZMQCamera::compute(OgnIsaacBridgeZMQCameraDatabase& db)
{
    // Get USD Stage
    const GraphContextObj& context = db.abi_context();
    long stageId = context.iContext->getStageId(context);
    pxr::UsdStageRefPtr stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

    // // // Get Camera Prim
    std::string cameraPrimPathStr = db.inputs.cameraPrimPath();
    if (cameraPrimPathStr.empty())
    {
        db.logWarning("No target prim path specified");
        return true;
    }
    pxr::SdfPath cameraPrimPath(cameraPrimPathStr.c_str());
    pxr::UsdPrim cameraPrim = stage->GetPrimAtPath(cameraPrimPath);

    // // Get the local-to-world transform of the camera prim
    pxr::UsdGeomXformable xformable(cameraPrim);
    pxr::GfMatrix4d prim_tf = xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
    pxr::GfTransform transform;
    transform.SetMatrix(prim_tf);
    pxr::GfVec3d scale = transform.GetScale();
    db.outputs.cameraWorldScale() = scale;

    uint32_t width = db.inputs.width();
    uint32_t height = db.inputs.height();

    // // Get the view matrix corrected for ROS conventions
    pxr::GfMatrix4d view_matrix_ros = get_view_matrix_ros(cameraPrim);
    db.outputs.cameraViewTransform() = view_matrix_ros;

    // // Get the camera intrinsics matrix
    pxr::GfMatrix3d intrinsics_matrix = get_intrinsics_matrix(cameraPrim, width, height);
    db.outputs.cameraIntrinsics() = intrinsics_matrix;

    // Returning true tells Omnigraph that the compute was successful and the output value is now valid.
    return true;
}

// This macro provides the information necessary to OmniGraph that lets it automatically register and deregister
// your node type definition.
REGISTER_OGN_NODE()


} // bridge
} // zmq
} // isaacsim
