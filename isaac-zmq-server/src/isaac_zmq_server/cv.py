# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import cv2
import numpy as np
import torch

np.set_printoptions(precision=4, suppress=True)


class CameraToWorldSpaceTransform:
    """
    Transforms 2D image coordinates to 3D world coordinates using camera parameters.

    This class provides methods to convert a point in a 2D image (such as the center of a
    bounding box) to its corresponding 3D position in world space, using depth information
    and camera intrinsic/extrinsic parameters.
    """

    def __init__(self, dimmentions: tuple):
        """
        Initializes the class with the given dimensions.

        Args:
            dimmentions (tuple): Image dimensions (width, height)
        """
        self.dimmention_x, self.dimmention_y = dimmentions
        self.detection_world_pos = [0, 0, 0]
        self.detection_camera_pos = [0, 0]
        self.depth_scale_factor = 1
        self._gpu_preallocate()

    def get_bbox_center_in_world_coords(
        self,
        bbox_data: dict,
        depth_data: bytes,
        camera_data: dict,
        device: str = "cuda",
    ) -> None:
        """
        Compute the 3D world coordinates of the center of a bounding box.

        Args:
            bbox_data (dict): Bounding box data containing coordinates
            depth_data (bytes): Raw depth image data
            camera_data (dict): Camera parameters including view and intrinsic matrices
            device (str): Computation device ('cuda' or 'cpu')
        """
        # Extract bounding box center coordinates (screen space)
        bbox = bbox_data["data"]
        if bbox:
            u = int((bbox["xMin"] + bbox["xMax"]) / 2)
            v = int((bbox["yMin"] + bbox["yMax"]) / 2)
        else:
            self.detection_world_pos = [0, 0, 0]
            return

        self.detection_camera_pos = [u, v]
        point = [u, v]

        # Convert depth data to numpy array - reshape to height x width
        depth_array = np.frombuffer(depth_data, dtype=np.float32).reshape(self.dimmention_y, self.dimmention_x)
        self.depth_scale_factor = 1 / camera_data["camera_scale"][0]

        # Simplifled implementation of
        # omni.sensor.Camera.get_world_points_from_image_coords()
        # https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.sensor/docs/index.html#omni.isaac.sensor.scripts.camera.Camera.get_world_points_from_image_coords
        if device == "cuda":
            view_matrix_ros = np.array(camera_data["view_matrix_ros"])
            intrinsics_matrix = np.array(camera_data["intrinsics_matrix"])
            self._update_camera_matrices(view_matrix_ros, intrinsics_matrix)
            self._get_bbox_center_in_world_coords_gpu(depth_array, point)
        else:
            self._get_bbox_center_in_world_coords_cpu(depth_array, camera_data, point)

    def _gpu_preallocate(self) -> None:
        """
        Pre-allocate GPU memory for computation.

        This method initializes tensors on the GPU to avoid repeated memory allocations
        during processing.
        """
        # Pre-allocate memory on GPU for depth data - height x width
        self._depth_data_gpu = torch.zeros((self.dimmention_y, self.dimmention_x), device="cuda", dtype=torch.float32)
        # Prepare placeholders for matrices on GPU
        self.view_matrix_ros_gpu = None
        self.intrinsics_matrix_gpu = None
        self.inverse_intrinsics_gpu = None

    def _update_camera_matrices(self, view_matrix_ros, intrinsics_matrix) -> None:
        """
        Update camera matrices on the GPU.

        Args:
            view_matrix_ros: Camera extrinsic matrix (view matrix in ROS convention)
            intrinsics_matrix: Camera intrinsic matrix
        """
        # Update view matrix on GPU
        if self.view_matrix_ros_gpu is None:
            self.view_matrix_ros_gpu = torch.tensor(view_matrix_ros, device="cuda", dtype=torch.float32)
        else:
            self.view_matrix_ros_gpu.copy_(torch.tensor(view_matrix_ros, device="cuda", dtype=torch.float32))

        # Update intrinsics matrix on GPU and calculate its inverse
        if self.intrinsics_matrix_gpu is None:
            self.intrinsics_matrix_gpu = torch.tensor(intrinsics_matrix, device="cuda", dtype=torch.float32)
        else:
            self.intrinsics_matrix_gpu.copy_(torch.tensor(intrinsics_matrix, device="cuda", dtype=torch.float32))

        # Compute the inverse of the intrinsics matrix on GPU
        try:
            self.inverse_intrinsics_gpu = torch.inverse(self.intrinsics_matrix_gpu)
        except RuntimeError as e:
            print(f"[isaac-zmq-server] Error computing inverse intrinsics matrix: {e}")
            self.inverse_intrinsics_gpu = None

    def _get_bbox_center_in_world_coords_gpu(self, depth_array: np.ndarray, point: tuple) -> None:
        """
        Calculate the 3D position of a bounding box in the world coordinates based on screen space and depth.
        (GPU implementation).

        Args:
            depth_array: Depth image as numpy array
            point: 2D point coordinates (u, v) in the image
        """
        # Copy depth data to device
        self._depth_data_gpu.copy_(torch.tensor(depth_array, device="cuda", dtype=torch.float32))

        # Get depth value at the specified point
        u, v = point
        # Copy the depth array to the GPU
        depth_array = np.copy(depth_array)
        self._depth_data_gpu.copy_(torch.from_numpy(depth_array).cuda())

        # Get the depth value at the specified point (u, v)
        # Note: depth array is in (height, width) format, so access as [v, u]
        depth_value = self._depth_data_gpu[v, u] * self.depth_scale_factor

        # Create a homogeneous point (u, v, 1.0) as a tensor on the GPU
        homogenous_point = torch.tensor([u, v, 1.0], device="cuda", dtype=torch.float32)

        # Calculate the point's camera coordinates by multiplying the homogeneous point by the inverse intrinsics matrix and the depth value
        point_camera_coords = torch.mv(self.inverse_intrinsics_gpu, homogenous_point * depth_value)

        # Add a 1.0 to the point's camera coordinates to make it homogeneous
        point_camera_coords_homogenous = torch.cat(
            [
                point_camera_coords,
                torch.tensor([1.0], device="cuda", dtype=torch.float32),
            ]
        )

        # Invert the view matrix on the GPU
        inverse_view_matrix_gpu = torch.inverse(self.view_matrix_ros_gpu)

        # Calculate the point's world coordinates by multiplying the homogeneous point by the inverse view matrix
        point_world_coords_homogenous = torch.mv(inverse_view_matrix_gpu, point_camera_coords_homogenous)

        self.detection_world_pos = point_world_coords_homogenous[:3].cpu().numpy().tolist()

    def _get_bbox_center_in_world_coords_cpu(self, depth_array: np.ndarray, camera_data: dict, point: tuple) -> None:
        """
        Calculate the 3D position of a bounding box in the world coordinates based on screen space and depth.
        (CPU implementation).

        Args:
            depth_array: Depth image as numpy array
            camera_data: Camera parameters including view and intrinsic matrices
            point: 2D point coordinates (u, v) in the image
        """

        u, v = point
        # Note: depth array is in (height, width) format, so access as [v, u]
        depth_value = (depth_array[v, u]) * self.depth_scale_factor

        view_matrix_ros = np.array(camera_data["view_matrix_ros"])
        intrinsics_matrix = np.array(camera_data["intrinsics_matrix"])

        # Create a homogeneous point (u, v, 1.0) as a numpy array
        homogenous_point = np.array([u, v, 1.0])

        # Invert the intrinsics matrix
        inverse_intrinsics = np.linalg.inv(intrinsics_matrix)

        # Calculate the point's camera coordinates by multiplying the homogeneous point by the inverse intrinsics matrix and the depth value
        point_camera_coords = inverse_intrinsics @ (homogenous_point * depth_value)

        # Add a 1.0 to the point's camera coordinates to make it homogeneous
        point_camera_coords_homogenous = np.append(point_camera_coords, 1.0)

        # Invert the view matrix
        inverse_view_matrix = np.linalg.inv(view_matrix_ros)

        # Calculate the point's world coordinates by multiplying the homogeneous point by the inverse view matrix
        point_world_coords_homogenous = inverse_view_matrix @ point_camera_coords_homogenous

        point_world_coords = point_world_coords_homogenous[:3].tolist()

        self.detection_world_pos = point_world_coords


def draw_bounding_boxes(img_array: np.ndarray, bbox_data: dict) -> np.ndarray:
    """
    Draw bounding boxes on an image.

    This function takes an image and bounding box data, and draws rectangles, labels,
    and center points for each detected object.

    Args:
        img_array (np.ndarray): RGB or RGBA image as numpy array
        bbox_data (dict): Dictionary containing bounding box data

    Returns:
        np.ndarray: Image with bounding boxes drawn
    """
    img_with_boxes = img_array.copy()
    bboxes = bbox_data["data"]
    id_to_labels = bbox_data["info"]["idToLabels"]
    color = (118, 185, 0)  # Green color for bounding boxes
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Get image dimensions
    height, width = img_with_boxes.shape[:2]

    for bbox in bboxes:
        semantic_id = bbox["semanticId"]
        x_min, y_min = bbox["xMin"], bbox["yMin"]
        x_max, y_max = bbox["xMax"], bbox["yMax"]

        # Calculate center point
        u = int((bbox["xMin"] + bbox["xMax"]) / 2)
        v = int((bbox["yMin"] + bbox["yMax"]) / 2)

        # Get the label of this bbox
        label = id_to_labels.get(str(semantic_id), "Unknown")

        # Ensure coordinates are within image bounds
        u = max(0, min(u, width-1))
        v = max(0, min(v, height-1))

        # Draw center point, bounding box, and label
        cv2.circle(img_with_boxes, (u, v), 10, color, 2)
        cv2.rectangle(img_with_boxes, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.putText(img_with_boxes, label, (x_min, y_min - 10), font, 0.9, color, 2)

    # Ensure alpha channel is fully opaque if it exists
    if img_with_boxes.shape[2] == 4:
        img_with_boxes[:, :, 3] = 255
    return img_with_boxes


def colorize_depth(depth_data: np.ndarray) -> np.ndarray:
    """
    Convert a depth image to a colorized representation.

    This function takes a raw depth image and converts it to a visually
    interpretable grayscale image by applying logarithmic scaling.

    Args:
        depth_data (np.ndarray): Raw depth image data in shape (height, width, 1)

    Returns:
        np.ndarray: Colorized depth image as uint8 with shape (height, width, 4)
    """
    # https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/programmatic_visualization.html#helper-visualization-functions
    # Set near and far clipping planes
    near = 1.0
    far = 100.0

    # Extract the depth values from the 3D array
    depth_values = depth_data.squeeze()

    # Clip depth values to the specified range
    depth_values = np.clip(depth_values, near, far)

    # Apply logarithmic scaling to better visualize depth
    depth_values = (np.log(depth_values) - np.log(near)) / (np.log(far) - np.log(near))

    # Invert so closer objects are brighter
    depth_values = 1.0 - depth_values

    # Convert to 8-bit for display
    depth_data_uint8 = (depth_values * 255).astype(np.uint8)

    # Create an RGBA image (height, width, 4)
    height, width = depth_values.shape
    rgba_image = np.zeros((height, width, 4), dtype=np.uint8)

    # # Set RGB channels to the grayscale value
    rgba_image[:, :, 0] = depth_data_uint8
    rgba_image[:, :, 1] = depth_data_uint8
    rgba_image[:, :, 2] = depth_data_uint8
    rgba_image[:, :, 3] = 255  # Full opacity

    return rgba_image