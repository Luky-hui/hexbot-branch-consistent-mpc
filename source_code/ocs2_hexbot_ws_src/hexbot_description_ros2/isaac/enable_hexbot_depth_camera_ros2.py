#!/usr/bin/env python3
"""Enable depth ROS2 publishing for Hexbot's front camera mount inside Isaac Sim.

This script follows Isaac Sim's official ROS2 camera publishing workflow:
https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_camera_publishing.html

It tries to reuse an existing camera prim under the front optical frame. If no
camera prim exists there, it authors a new ``UsdGeom.Camera`` child and then
creates a render product plus ROS2 publishers for:

- ``/camera/depth/image_raw``
- ``/camera/depth/camera_info``

The native Isaac Sim ROS2 pointcloud writer is disabled by default here because
it has been observed to crash the current Isaac Sim 4.5.0 environment on this
workspace. Use the ROS-side semantic obstacle fuser to generate
``/semantic_obstacles/points`` from the depth image instead.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from urllib.parse import unquote, urlparse

from pxr import Gf, Usd, UsdGeom

try:
    import omni.graph.core as og  # type: ignore
    import omni.kit.app  # type: ignore
    import omni.replicator.core as rep  # type: ignore
    import omni.syntheticdata._syntheticdata as sd  # type: ignore
    import omni.usd  # type: ignore
    from isaacsim.ros2.bridge import read_camera_info  # type: ignore
except ImportError:  # pragma: no cover - Isaac-only script
    og = None
    rep = None
    sd = None
    omni = None
    read_camera_info = None


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_USD_PATH = (
    THIS_DIR.parent / "urdf" / "hexbot_isaac_rooted_reimport" / "hexbot_isaac_rooted_reimport.usd"
)
PREFERRED_PARENT_NAMES = ("camera_rgb_optical_frame", "camera_link")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Enable front depth camera ROS2 publishers in Isaac Sim.")
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD_PATH)
    parser.add_argument("--camera-name", default="depth_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--freq", type=float, default=15.0)
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--pointcloud-topic", default="/camera/depth/points")
    parser.add_argument("--publish-pointcloud", action="store_true")
    parser.add_argument(
        "--unsafe-native-pointcloud-writer",
        action="store_true",
        help="Create Isaac's native ROS2 pointcloud writer. This is known to be unstable in the current environment.",
    )
    return parser.parse_args()


def normalize_layer_identifier(identifier: str) -> str:
    if identifier.startswith("file:"):
        parsed = urlparse(identifier)
        return str(Path(unquote(parsed.path)).resolve())
    if "://" not in identifier:
        return str(Path(identifier).resolve())
    return identifier


def open_stage(usd_path: Path) -> tuple[Usd.Stage | None, str]:
    if omni is not None:
        try:
            context = omni.usd.get_context()
            live_stage = context.get_stage() if context is not None else None
        except Exception:
            live_stage = None
        if live_stage is not None:
            live_identifier = normalize_layer_identifier(live_stage.GetRootLayer().identifier)
            target_identifier = normalize_layer_identifier(str(usd_path.resolve()))
            if live_identifier == target_identifier:
                return live_stage, "live"
    return Usd.Stage.Open(str(usd_path.resolve())), "file"


def find_front_camera_mount(stage: Usd.Stage) -> Usd.Prim | None:
    candidates: list[Usd.Prim] = []
    for prim in stage.Traverse():
        if prim.GetName() in PREFERRED_PARENT_NAMES:
            candidates.append(prim)
    if not candidates:
        return None
    candidates.sort(key=lambda prim: len(prim.GetPath().pathString))
    for preferred_name in PREFERRED_PARENT_NAMES:
        for prim in candidates:
            if prim.GetName() == preferred_name:
                return prim
    return candidates[0]


def find_existing_camera(parent_prim: Usd.Prim) -> Usd.Prim | None:
    for child in Usd.PrimRange(parent_prim):
        if child.IsA(UsdGeom.Camera):
            return child
    return None


def copy_camera_attributes(source_camera: UsdGeom.Camera, target_camera: UsdGeom.Camera) -> None:
    for getter, setter in (
        (source_camera.GetFocalLengthAttr, target_camera.GetFocalLengthAttr),
        (source_camera.GetFocusDistanceAttr, target_camera.GetFocusDistanceAttr),
        (source_camera.GetFStopAttr, target_camera.GetFStopAttr),
        (source_camera.GetHorizontalApertureAttr, target_camera.GetHorizontalApertureAttr),
        (source_camera.GetVerticalApertureAttr, target_camera.GetVerticalApertureAttr),
        (source_camera.GetClippingRangeAttr, target_camera.GetClippingRangeAttr),
    ):
        source_attr = getter()
        target_attr = setter()
        if source_attr and source_attr.IsValid():
            value = source_attr.Get()
            if value is not None:
                target_attr.Set(value)


def ensure_depth_camera(stage: Usd.Stage, parent_prim: Usd.Prim, camera_name: str) -> tuple[str, bool]:
    existing_camera = find_existing_camera(parent_prim)
    if existing_camera is not None:
        return existing_camera.GetPath().pathString, False

    camera_path = parent_prim.GetPath().AppendChild(camera_name)
    camera = UsdGeom.Camera.Define(stage, camera_path)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

    source_camera = None
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera):
            source_camera = UsdGeom.Camera(prim)
            break
    if source_camera is not None:
        copy_camera_attributes(source_camera, camera)
    else:
        camera.GetFocalLengthAttr().Set(24.0)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetVerticalApertureAttr().Set(15.2908)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))

    return camera_path.pathString, True


def attach_ros2_publishers(
    camera_path: str,
    frame_id: str,
    width: int,
    height: int,
    freq: float,
    depth_topic: str,
    camera_info_topic: str,
    pointcloud_topic: str,
    publish_pointcloud: bool,
    unsafe_native_pointcloud_writer: bool,
) -> str:
    if rep is None or og is None or sd is None or read_camera_info is None or omni is None:
        raise RuntimeError("This script must run inside Isaac Sim with isaacsim.ros2.bridge enabled.")

    render_product = rep.create.render_product(camera_path, (width, height))
    render_product_path = render_product.path if hasattr(render_product, "path") else str(render_product)
    step_size = max(1, int(round(60.0 / max(freq, 1.0))))

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)

    depth_writer = rep.writers.get(rv + "ROS2PublishImage")
    depth_writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=depth_topic,
    )
    depth_writer.attach([render_product_path])
    gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product_path)
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

    camera_info = read_camera_info(render_product_path=render_product_path)
    info_writer = rep.writers.get("ROS2PublishCameraInfo")
    info_writer.initialize(
        frameId=frame_id,
        nodeNamespace="",
        queueSize=1,
        topicName=camera_info_topic,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    info_writer.attach([render_product_path])
    info_gate = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatchIsaacSimulationGate", render_product_path
    )
    og.Controller.attribute(info_gate + ".inputs:step").set(step_size)

    if publish_pointcloud and unsafe_native_pointcloud_writer:
        pc_writer = rep.writers.get(rv + "ROS2PublishPointCloud")
        pc_writer.initialize(
            frameId=frame_id,
            nodeNamespace="",
            queueSize=1,
            topicName=pointcloud_topic,
        )
        pc_writer.attach([render_product_path])
        pc_gate = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product_path)
        og.Controller.attribute(pc_gate + ".inputs:step").set(step_size)

    return render_product_path


def main() -> int:
    args = parse_args()
    stage, stage_source = open_stage(args.usd)
    if stage is None:
        print(f"Failed to open stage: {args.usd}")
        return 1
    if omni is None:
        print("This script must be run from Isaac Sim's Python / Script Editor.")
        return 2

    mount_prim = find_front_camera_mount(stage)
    if mount_prim is None:
        print("Failed to find a front camera mount prim. Expected one of: camera_rgb_optical_frame, camera_link")
        return 3

    camera_path, created_camera = ensure_depth_camera(stage, mount_prim, args.camera_name)

    # Let USD changes settle before creating the render product.
    omni.kit.app.get_app().update()

    if args.publish_pointcloud and not args.unsafe_native_pointcloud_writer:
        print(
            "Warning: --publish-pointcloud requested, but native Isaac ROS2 pointcloud writing is disabled "
            "for stability in this environment. Only depth image + camera_info will be published."
        )

    frame_id = mount_prim.GetName()
    render_product_path = attach_ros2_publishers(
        camera_path=camera_path,
        frame_id=frame_id,
        width=args.width,
        height=args.height,
        freq=args.freq,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        pointcloud_topic=args.pointcloud_topic,
        publish_pointcloud=args.publish_pointcloud,
        unsafe_native_pointcloud_writer=args.unsafe_native_pointcloud_writer,
    )

    stage.GetRootLayer().Save()

    print(f"USD: {args.usd.resolve()}")
    print(f"stageSource={stage_source}")
    print(f"cameraMount={mount_prim.GetPath().pathString}")
    print(f"cameraPath={camera_path}")
    print(f"createdCamera={created_camera}")
    print(f"frameId={frame_id}")
    print(f"renderProduct={render_product_path}")
    print(f"depthTopic={args.depth_topic}")
    print(f"cameraInfoTopic={args.camera_info_topic}")
    if args.publish_pointcloud:
        if args.unsafe_native_pointcloud_writer:
            print(f"pointcloudTopic={args.pointcloud_topic}")
        else:
            print(
                "pointcloudTopic=<disabled>"
                " (native Isaac writer skipped for stability; use semantic_obstacle_fuser on depth image)"
            )
    print("Depth ROS2 publishers enabled. Keep the current Isaac stage running to serve the topics.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
