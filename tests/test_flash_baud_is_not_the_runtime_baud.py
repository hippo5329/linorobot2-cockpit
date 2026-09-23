"""`baudrate` is two different rates, and they must not be conflated.

The firmware talks micro-ROS at it (env `baud`) and bringup.launch.py hands the
same number to the agent as -b. one_click_pipeline ALSO handed it to esptool as
the upload rate, so raising the GenDrv to the 1.5 Mbaud the README prescribes
would have changed the flashing path at the same time -- and a flash that fails
costs a whole leg of the matrix, which is a poor way to learn that the runtime
rate was fine.

921600 is the rate every ESP32 leg has flashed at to date. The ceiling keeps
that path while the runtime rate moves.
"""
import ast
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIPELINE = os.path.join(ROOT, "scripts", "one_click_pipeline.py")
GENDRV = os.path.join(ROOT, "config", "reference", "gendrv_config.yaml")


def _src():
    with open(PIPELINE, encoding="utf-8") as fh:
        return fh.read()


def _const(name):
    for node in ast.parse(_src()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == name:
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} is gone from one_click_pipeline.py")


def test_the_upload_rate_has_a_ceiling_of_its_own():
    assert _const("FLASH_BAUD_CEILING") == 921600


def test_every_esptool_call_takes_the_capped_rate():
    """Not one of them may take the runtime rate -- a single missed call site
    is the flash that fails."""
    src = _src()
    assert "flash_baud = min(int(baudrate), FLASH_BAUD_CEILING)" in src
    for call in ("probe_board(", "flash_firmware(", "write_env_only("):
        i = src.index(call + "pio_env, serial_port, ")
        arg = src[i + len(call + "pio_env, serial_port, "):].split(",")[0]
        assert arg == "flash_baud", f"{call} uploads at {arg}"


def test_the_runtime_rate_still_reaches_the_board_and_the_agent():
    """The cap must not become a cap on the robot: the env and the agent get
    the config's number, whatever esptool was given."""
    import yaml
    cfg = yaml.safe_load(open(GENDRV, encoding="utf-8"))
    assert cfg["base_controller"]["baudrate"] == 1500000
    import sys
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    import mcu_env
    env = mcu_env.env_from_config(GENDRV, os.path.join(ROOT, "config", "secrets.yaml.example"))
    assert int(env["baud"]) == 1500000, "the firmware would talk at the upload rate"
    launcher = open(os.path.join(ROOT, "launchers", "bringup.launch.py"), encoding="utf-8").read()
    assert 'controller.get("baudrate"' in launcher, "the agent no longer follows the config"


def test_the_gendrv_runs_the_rate_its_own_docs_prescribe():
    """README and mcu_env.py both say this board needs 1.5 Mbaud and that
    921600 is the bare-DevKit setting. It shipped with the DevKit's."""
    import yaml
    cfg = yaml.safe_load(open(GENDRV, encoding="utf-8"))
    bc = cfg["base_controller"]
    assert bc["baudrate"] == 1500000
    assert bc["use_dual_core"] is True
    # and the LiDAR UART is untouched: 230400 is the LD19's protocol rate, not
    # a tunable, and the driver on the host expects exactly it.
    assert cfg["base_controller"]["lidar"]["baudrate"] == 230400
