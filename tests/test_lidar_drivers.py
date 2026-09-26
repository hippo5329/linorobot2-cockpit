"""A real LiDAR is read by its own vendor's driver, with the vendor's settings.

bringup started ldlidar_stl_ros2 for every model: an unknown one fell back to
the LD19, so an RPLIDAR's port was read at the LD19's baud and the log said only
"ldlidar communication is abnormal". The RPLIDAR, YDLIDAR and XV-11 drivers are
now in the image (scripts/prepare_docker_vendor.sh, docker/Dockerfile) and
lidar_drivers.py maps each model to its driver; a model none reads is refused
by name.
"""
import os
import sys

import pytest
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))

import lidar_drivers as ld  # noqa: E402

VENDOR = os.path.join(REPO_ROOT, "docker", "vendor")


def read(*p):
    return open(os.path.join(REPO_ROOT, *p)).read()


@pytest.mark.parametrize("model,fam", [("ld19", "ldlidar"), ("STL27L", "ldlidar"), ("a1", "sllidar"),
                                       ("a2", "sllidar"), ("s3", "sllidar"), ("ydlidar", "ydlidar"),
                                       ("ydlidar_x4", "ydlidar"), ("xv11", "xv11")])
def test_each_model_has_its_family(model, fam):
    assert ld.family(model) == fam


@pytest.mark.parametrize("model", ["rplidar", "ld19x", "", "ld14", "ld14p"])
def test_a_model_no_driver_reads_is_refused(model):
    with pytest.raises(ValueError, match="lidar.model"):
        ld.family(model)


def test_the_ld_driver_knows_every_ld_model():
    """ldlidar_stl_ros2's node exits on a product name it does not know -- LD14 was one."""
    demo = os.path.join(VENDOR, "ldlidar_stl_ros2", "src", "demo.cpp")
    if not os.path.exists(demo):
        pytest.skip("docker/vendor not staged (run scripts/prepare_docker_vendor.sh)")
    src = open(demo).read()
    for product, _ in ld.LDLIDAR_MODELS.values():
        assert f'product_name == "{product}"' in src, product


def test_rplidar_takes_the_vendor_rate_and_mode():
    spec = ld.serial_node("c1", "/dev/rplidar", None, "laser_frame", "scan")
    (p,) = spec["parameters"]
    assert spec["package"] == "sllidar_ros2" and spec["executable"] == "sllidar_node"
    assert p["serial_baudrate"] == 460800 and p["scan_mode"] == "Standard"
    assert p["serial_port"] == "/dev/rplidar" and p["frame_id"] == "laser_frame"
    assert spec["remappings"] == []


def test_a_named_baud_wins_and_a_mask_moves_the_topic():
    spec = ld.serial_node("a1", "/dev/x", "256000", "laser_frame", "scan_raw")
    assert spec["parameters"][0]["serial_baudrate"] == 256000
    assert spec["remappings"] == [("scan", "scan_raw")]


def test_the_rplidar_table_is_the_vendors_launch_files():
    launch = os.path.join(VENDOR, "sllidar_ros2", "launch")
    if not os.path.isdir(launch):
        pytest.skip("docker/vendor not staged")
    for model, (rate, mode) in ld.SLLIDAR_MODELS.items():
        (f,) = [n for n in os.listdir(launch) if n.replace(" ", "") == f"sllidar_{model}_launch.py"]
        src = open(os.path.join(launch, f)).read()
        assert f"'serial_baudrate', default='{rate}'" in src, model
        if mode:
            assert f"'scan_mode', default='{mode}'" in src, model


def test_ydlidar_reads_the_vendors_params_file(tmp_path):
    (tmp_path / "params").mkdir()
    (tmp_path / "params" / "X4.yaml").write_text(yaml.safe_dump(
        {"ydlidar_ros2_driver_node": {"ros__parameters": {"port": "/dev/ttyUSB0", "baudrate": 128000,
                                                          "frame_id": "laser_frame", "range_max": 12.0}}}))
    spec = ld.serial_node("ydlidar_x4", "/dev/ydlidar", None, "base_laser", "scan", share_dir=str(tmp_path))
    (p,) = spec["parameters"]
    assert p["baudrate"] == 128000 and p["range_max"] == 12.0, "the vendor's values stand"
    assert p["port"] == "/dev/ydlidar" and p["frame_id"] == "base_laser"


def test_every_ydlidar_code_has_a_vendor_file():
    params = os.path.join(VENDOR, "ydlidar_ros2_driver", "params")
    if not os.path.isdir(params):
        pytest.skip("docker/vendor not staged")
    for code, f in ld.YDLIDAR_FILES.items():
        assert os.path.isfile(os.path.join(params, f)), code


def test_xv11():
    spec = ld.serial_node("xv11", "/dev/ttyUSB1", None, "laser_frame", "scan")
    assert spec["package"] == "xv_11_driver"
    assert spec["parameters"][0]["baud_rate"] == 115200


def test_the_ui_offers_only_models_a_driver_reads():
    sys.path.insert(0, os.path.join(REPO_ROOT, "web", "backend"))
    import system_utils
    for fam in system_utils.LASER_SENSORS.values():
        for m in fam["models"]:
            ld.family(m["code"])


def test_bringup_dispatches_and_refuses():
    launch = read("launchers", "bringup.launch.py")
    assert "LDLIDAR_MODELS.get(" not in launch, "no silent LD19 fallback"
    assert "lidar_family = lidar_drivers.family(lidar_model) if robot_has_lidar else None" in launch
    assert "**lidar_drivers.serial_node(lidar_model, lidar_port, lidar_baud_named," in launch
    docker = read("docker", "Dockerfile")
    for exe in ("sllidar_ros2/sllidar_node", "ydlidar_ros2_driver/ydlidar_ros2_driver_node",
                "xv_11_driver/xv_11_driver"):
        assert f"test -x /opt/lino_ws/lib/{exe}" in docker, exe
