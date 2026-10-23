"""The Orbbec Astra Pro on Orbbec's ros2_astra_camera, for depthimage_to_laserscan.

bringup.launch.py includes this for a depth_camera.model in the `astra` family
(scripts/depth_camera.py). It replaces the package's own astra_pro.launch.xml, which
names every frame after camera_name (so the tree cannot hang off the robot's
geometry.depth_camera frame), turns on an IR stream and a point cloud that nothing
here reads, and leaves the node on whatever libOpenNI2 the library path finds first.
The parameters are that file's, with those three changed.

The driver is built into the robot image at depth_camera.ASTRA_WS, outside /opt/ros, so
nothing sources it: the node runs from its path, with that install first on its
library path. Its lib/ holds Orbbec's own libOpenNI2.so and the Astra driver;
Debian's OpenNI2 (openni2_camera's dependency) is also in the image, and Orbbec's
driver crashes in that one.

Arguments:
  namespace      the driver's namespace (default camera): it publishes
                 <namespace>/depth/image_raw (16UC1, millimetres) and
                 <namespace>/depth/camera_info -- the topics DEPTH_MODELS names --
                 and <namespace>/color/image_raw (rgb8) with color/camera_info
  camera_frame   the camera's root frame, which the robot's URDF parents to base_link
                 at geometry.depth_camera (default camera_link)
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
import depth_camera  # noqa: E402  (ASTRA_WS, the frame names)


def _path(*parts):
    return ":".join(p for p in parts if p)


def _actions(context):
    namespace = context.launch_configurations["namespace"]
    camera_frame = context.launch_configurations["camera_frame"]
    ws = depth_camera.ASTRA_WS
    frames = {}
    for stream in ("depth", "color", "ir"):
        frame, optical = depth_camera.stream_frames(camera_frame, stream)
        frames[f"{stream}_frame_id"] = frame
        frames[f"{stream}_optical_frame_id"] = optical
    return [Node(
        executable=os.path.join(ws, "lib", "astra_camera", "astra_camera_node"),
        name="camera",
        namespace=namespace,
        output="screen",
        additional_env={
            "LD_LIBRARY_PATH": _path(os.path.join(ws, "lib"), os.environ.get("LD_LIBRARY_PATH", "")),
            "AMENT_PREFIX_PATH": _path(ws, os.environ.get("AMENT_PREFIX_PATH", "")),
        },
        parameters=[{
            # astra_pro.launch.xml's values, as launch types them (its hex ids are ints).
            "camera_name": namespace,
            "depth_registration": False,
            "serial_number": "",
            "device_num": 1,
            "vendor_id": 0,
            "product_id": 0,
            "enable_colored_point_cloud": False,
            "point_cloud_qos": "default",
            "connection_delay": 100,
            "color_width": 640, "color_height": 480, "color_fps": 30,
            "enable_color": True, "flip_color": False,
            "color_qos": "default", "color_camera_info_qos": "default",
            "depth_width": 640, "depth_height": 480, "depth_fps": 30,
            "enable_depth": True, "flip_depth": False,
            "depth_qos": "default", "depth_camera_info_qos": "default",
            "ir_width": 640, "ir_height": 480, "ir_fps": 30, "flip_ir": False,
            "ir_qos": "default", "ir_camera_info_qos": "default",
            "publish_tf": True,
            "ir_info_url": "", "color_info_url": "",
            "color_roi_x": -1, "color_roi_y": -1, "color_roi_width": -1, "color_roi_height": -1,
            "depth_roi_x": -1, "depth_roi_y": -1, "depth_roi_width": -1, "depth_roi_height": -1,
            "depth_scale": 1,
            "color_depth_synchronization": False,
            "use_uvc_camera": True,
            "uvc_vendor_id": 0x2BC5,
            "uvc_product_id": 0x0501,
            "uvc_retry_count": 100,
            "uvc_camera_format": "mjpeg",
            "uvc_flip": False,
            "oni_log_level": "verbose", "oni_log_to_console": False, "oni_log_to_file": False,
            "enable_d2c_viewer": False,
            "enable_publish_extrinsic": False,
            # Changed from the package's file: nothing here reads the IR image or a
            # cloud, and both cost USB 2 bandwidth and CPU on a Pi; a camera on a fixed
            # mount needs its transforms once (static), not at 10 Hz; and the tree hangs
            # off the robot's frame for the camera.
            "enable_ir": False,
            "enable_point_cloud": False,
            "tf_publish_rate": 0.0,
            "camera_link_frame_id": camera_frame,
            **frames,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        OpaqueFunction(function=_actions),
    ])
