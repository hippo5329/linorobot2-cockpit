"""The migrator renames use_fake_* to use_sim_*, and the pipeline says so up front.

mcu_env.py refuses a config that still says use_fake_* (an absent use_sim_* flag
is a compiled-in default), and told the user to rename them -- but the migrator,
the tool for renames, did not know this one. On 2026-09-25 a stale
pico_config.yaml stopped the UI's 1-Click at "Flashing or verification failed",
the reason buried in the flash log.
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "scripts"))
import migrate_config_schema as mig  # noqa: E402

CFG = """# my robot -- a comment the user wrote
robot:
  name: myrobot
base_controller:
  name: pico2
  sensors:
    imu: NONE
    use_fake_imu: true     # keep me
    use_fake_wheel: false
kinematics: {base_type: 2wd}
geometry: {}
"""


def _write(tmp_path):
    p = tmp_path / "myrobot_config.yaml"
    p.write_text(CFG)
    return p


def test_renames_keys_and_keeps_comments(tmp_path):
    p = _write(tmp_path)
    assert mig.rename_fake_keys(str(p), dry_run=False)
    out = p.read_text()
    assert "use_fake_" not in out
    assert "use_sim_imu: true     # keep me" in out and "use_sim_wheel: false" in out
    assert out.startswith("# my robot -- a comment the user wrote")


def test_dry_run_writes_nothing(tmp_path):
    p = _write(tmp_path)
    assert mig.rename_fake_keys(str(p), dry_run=True)
    assert p.read_text() == CFG


def test_the_pipeline_refuses_a_stale_config_before_touching_the_board():
    src = open(os.path.join(HERE, "scripts", "one_click_pipeline.py"), encoding="utf-8").read()
    i = src.index("stale = stale_faults(params)")
    assert i < src.index("has_lidar = lidar_fitted(controller_cfg)"), \
        "the stale-config check must come before any hardware step"
    assert "fresh config" in src[i:i + 500]
