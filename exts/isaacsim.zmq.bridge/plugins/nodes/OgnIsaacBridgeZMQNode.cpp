// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: MIT

#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cuda/include/cuda_runtime_api.h>
#include <zmq.hpp>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3d.h>

#include <carb/logging/Log.h>

#include <OgnIsaacBridgeZMQNodeDatabase.h>
#include "client_stream_message.pb.h"


using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

#define CUDA_CHECK(call)                                                   \
do {                                                                       \
    cudaError_t err = call;                                                \
    if (err != cudaSuccess) {                                              \
        fprintf(stderr, "CUDA error at %s %d: %s\n", __FILE__, __LINE__,   \
                cudaGetErrorString(err));                                  \
        /* Instead of exiting, log the error and continue */               \
        return true;                                                       \
    }                                                                      \
} while (0)

namespace zmq_lib = zmq; // assign namespace to zmq library to avoid conflicts with our library

namespace isaacsim {
namespace zmq {
namespace bridge {

struct InputDataBBox2d {
    uint32_t semanticId;
    int xMin;
    int yMin;
    int xMax;
    int yMax;
    float occlusionRatio;
};

class OgnIsaacBridgeZMQNode {
    std::unique_ptr<zmq_lib::context_t> m_zmqContext;
    std::unique_ptr<zmq_lib::socket_t> m_zmqSocket;
    uint32_t m_port;
    std::string m_ip;
    std::mutex m_mutex;
    cudaStream_t m_cudaStream;
    bool m_cudaStreamNotCreated{ true };
    uint32_t m_zmqFailCount{ 0 };

public:
    OgnIsaacBridgeZMQNode()
        : m_zmqContext(std::make_unique<zmq_lib::context_t>(1)) {
        CARB_LOG_INFO("OgnIsaacBridgeZMQNode::constructor\n");
    }
    ~OgnIsaacBridgeZMQNode() {
        CARB_LOG_INFO("OgnIsaacBridgeZMQNode::destructor\n");
        if (m_zmqSocket) {
            m_zmqSocket->close();
        }
        if (m_zmqContext) {
            m_zmqContext->close();
        }

        // Clean up CUDA stream if it was created
        if (!m_cudaStreamNotCreated) {
            cudaError_t err = cudaStreamDestroy(m_cudaStream);
            if (err != cudaSuccess) {
                // Just log the error instead of using CUDA_CHECK
                CARB_LOG_ERROR("Error destroying CUDA stream in destructor: %s", cudaGetErrorString(err));
            }
        }
    }

    static bool compute(OgnIsaacBridgeZMQNodeDatabase& db);

    bool initializeSocket(uint32_t port, const std::string& ip) {
        std::lock_guard<std::mutex> lock(m_mutex);

        m_port = port;
        m_ip = ip;
        m_zmqFailCount = 0;

        try {
            m_zmqSocket = std::make_unique<zmq_lib::socket_t>(*m_zmqContext, zmq_lib::socket_type::push);

            int linger = 0;
            m_zmqSocket->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

            int hwm = 1;
            m_zmqSocket->setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

            std::string address = "tcp://" + m_ip + ":" + std::to_string(m_port);
            m_zmqSocket->connect(address);
            CARB_LOG_INFO("Connected to %s\n", address.c_str());
            return true;
        } catch (const std::exception& e) {
            CARB_LOG_WARN("Failed to create socket or connect to %s:%d: %s", m_ip.c_str(), m_port, e.what());
            m_zmqSocket.reset();
            return false;
        }
    }
};


bool OgnIsaacBridgeZMQNode::compute(OgnIsaacBridgeZMQNodeDatabase& db) {
    // Static variable to track the last time an error was logged
    // This persists between function calls to limit error message frequency
    static double lastErrorLogTime = 0.0;

    // Get the internal state for this node
    auto& state = db.internalState<OgnIsaacBridgeZMQNode>();

    // Get the port and IP address from the inputs
    uint32_t port = db.inputs.port();
    const omni::graph::core::ogn::const_string& ip = db.inputs.ip();
    std::string std_ip(ip.data(), ip.size());

    // If the socket is not initialized, or the port or IP address has changed, initialize the socket
    if (!state.m_zmqSocket || port != state.m_port || std_ip != state.m_ip) {
        if (!state.initializeSocket(port, std_ip)) {
            return true;
        }
    }

    // Create Protobuf message
    ClientStreamMessage message;

    // Bounding boxes 2d
    const InputDataBBox2d* bbox_data = reinterpret_cast<const InputDataBBox2d*>(db.inputs.dataBBox2d().data());
    size_t num_boxes = db.inputs.dataBBox2d().size() / sizeof(InputDataBBox2d);
    auto& bbox_ids = db.inputs.idsBBox2d();
    auto& bbox_bbox_ids = db.inputs.bboxIdsBBox2d();
    auto& bbox_labels = db.inputs.labelsBBox2d();

    // Populate bbox2d data
    for (size_t i = 0; i < num_boxes; ++i) {
        const InputDataBBox2d& bbox = bbox_data[i];
        BBox2DType* bbox_proto = message.mutable_bbox2d()->add_data();
        bbox_proto->set_semanticid(bbox.semanticId);
        bbox_proto->set_xmin(bbox.xMin);
        bbox_proto->set_ymin(bbox.yMin);
        bbox_proto->set_xmax(bbox.xMax);
        bbox_proto->set_ymax(bbox.yMax);
        bbox_proto->set_occlusionratio(bbox.occlusionRatio);
    }

    // Populate bboxIds
    for (size_t i = 0; i < bbox_bbox_ids.size(); ++i) {
        message.mutable_bbox2d()->mutable_info()->add_bboxids(bbox_bbox_ids[i]);
    }

    // Populate idToLabels
    for (size_t i = 0; i < bbox_ids.size(); ++i) {
        int id = bbox_ids[i];
        std::string label = db.tokenToString(bbox_labels[i]);
        (*message.mutable_bbox2d()->mutable_info()->mutable_idtolabels())[std::to_string(id)] = label;
    }

    // Simulation & System time
    double sim_dt = db.inputs.deltaSimulationTime();
    double sys_dt = db.inputs.deltaSystemTime();
    double sim_time = db.inputs.simulationTime();
    double sys_time = db.inputs.systemTime();

    message.mutable_clock()->set_sim_dt(sim_dt);
    message.mutable_clock()->set_sys_dt(sys_dt);
    message.mutable_clock()->set_sim_time(sim_time);
    message.mutable_clock()->set_sys_time(sys_time);

    // Camera data
    const pxr::GfMatrix4d& view_matrix = db.inputs.cameraViewTransform();
    const pxr::GfVec3d& scale = db.inputs.cameraWorldScale();
    const pxr::GfMatrix3d& intrinsics_matrix = db.inputs.cameraIntrinsics();

    // Flatten and populate view_matrix_ros
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            message.mutable_camera()->add_view_matrix_ros(view_matrix[row][col]);
        }
    }

    // Populate camera_scale
    message.mutable_camera()->add_camera_scale(scale[0]);
    message.mutable_camera()->add_camera_scale(scale[1]);
    message.mutable_camera()->add_camera_scale(scale[2]);

    // Flatten and populate intrinsics_matrix
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            message.mutable_camera()->add_intrinsics_matrix(intrinsics_matrix[row][col]);
        }
    }

    // RGB & DEPTH

    // Copy from Device to Host
    size_t data_size_color = db.inputs.bufferSizeColor();
    uint64_t raw_ptr_color = db.inputs.dataPtrColor();
    auto data_ptr_color = std::make_unique<int8_t[]>(data_size_color);

    size_t data_size_depth = db.inputs.bufferSizeDepth();
    uint64_t raw_ptr_depth = db.inputs.dataPtrDepth();
    auto data_ptr_depth = std::make_unique<float[]>(data_size_depth / sizeof(float));

    // Create CUDA stream if not already created
    if (state.m_cudaStreamNotCreated) {
        CUDA_CHECK(cudaStreamCreate(&state.m_cudaStream));
        state.m_cudaStreamNotCreated = false;
    }

    // If the stream is not created, warn and return true
    if (state.m_cudaStreamNotCreated) {
        CARB_LOG_WARN("CUDA stream not created, will not stream images");
        return true;
    }

    // Use the stream for memory operations
    CUDA_CHECK(cudaMemcpyAsync(data_ptr_color.get(), reinterpret_cast<void*>(raw_ptr_color),
                            data_size_color, cudaMemcpyDeviceToHost, state.m_cudaStream));

    CUDA_CHECK(cudaMemcpyAsync(data_ptr_depth.get(), reinterpret_cast<void*>(raw_ptr_depth),
                        data_size_depth, cudaMemcpyDeviceToHost, state.m_cudaStream));

    CUDA_CHECK(cudaStreamSynchronize(state.m_cudaStream));

    // Add image data to Protobuf message
    message.set_color_image(data_ptr_color.get(), data_size_color);
    message.set_depth_image(reinterpret_cast<const char*>(data_ptr_depth.get()), data_size_depth);

    // Serialize Protobuf message
    std::string serialized_message;
    message.SerializeToString(&serialized_message);

    // ZMQ Data sending
    zmq_lib::message_t zmq_message(serialized_message.size());
    memcpy(zmq_message.data(), serialized_message.data(), serialized_message.size());
    auto message_sent = state.m_zmqSocket->send(zmq_message, zmq_lib::send_flags::dontwait);

    if (!message_sent.has_value()) {
        state.m_zmqFailCount++;
        double currentTime = db.inputs.systemTime();
        // Log the error state every 5 seconds,
        // and only if errors are accumulating.
        if (state.m_zmqFailCount > 20 && currentTime - lastErrorLogTime >= 5.0) {
            CARB_LOG_ERROR("Failed to send message (no server available)");
            lastErrorLogTime = currentTime;
        }
    } else {
        state.m_zmqFailCount = 0;
    }

    return true;
}

// This macro provides the information necessary to OmniGraph that lets it automatically register and deregister
// your node type definition.
REGISTER_OGN_NODE()

} // bridge
} // zmq
} // isaacsim
