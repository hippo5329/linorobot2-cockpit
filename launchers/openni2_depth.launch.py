"""An OpenNI2 depth camera (the Orbbec Astra Pro) for depthimage_to_laserscan.

bringup.launch.py includes this for a depth_camera.model in the `openni2` family
(scripts/depth_camera.py). It replaces openni2_camera's own camera_only.launch.py,
which turns depth registration on -- an Astra Pro's colour camera is a separate UVC
device, so there is no OpenNI colour stream to register to -- and whose tfs.launch.py
joins frame names as tf_prefix + "/" + namespace: with no prefix every frame starts
with "/" and matches nothing in the robot's tree.

Arguments:
  namespace      the driver's namespace (default camera): it publishes
                 <namespace>/depth/image_raw (16UC1, millimetres) and
                 <namespace>/depth/camera_info -- the topics DEPTH_MODELS names
  camera_frame   the camera's root frame, which the robot's URDF parents to base_link
                 at geometry.depth_camera (default camera_link)

The driver opens the depth stream only while something subscribes to it
(depthimage_to_laserscan does), and needs Orbbec's OpenNI2 driver
(depth_camera.ORBBEC_DRIVER) to see an Astra at all.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import depth_camera  # noqa: E402  (the frame names, depth_camera.openni2_frames)


def _actions(context):
    namespace = context.launch_configurations["namespace"]
    camera_frame = context.launch_configurations["camera_frame"]
    depth_frame, depth_optical = depth_camera.openni2_frames(camera_frame)
    return [
        Node(
            package="openni2_camera",
            executable="openni2_camera_driver",
            name="driver",
            namespace=namespace,
            output="screen",
            parameters=[{"depth_registration": False,
                         "use_device_time": True,
                         "depth_frame_id": depth_optical}],
        ),
        # openni2_camera's own offsets (tfs.launch.py): the depth sensor 2 cm to the
        # right of the camera's origin, its optical frame z forward, x right, y down.
        Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
             arguments=["--frame-id", camera_frame, "--child-frame-id", depth_frame, "--y", "-0.02"]),
        Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
             arguments=["--frame-id", depth_frame, "--child-frame-id", depth_optical,
                        "--roll", "-1.5707963267948966", "--yaw", "-1.5707963267948966"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        OpaqueFunction(function=_actions),
    ])
