"""Dead stacks' Fast DDS shared memory is removed at container start and at each 1-Click.

/dev/shm is the host's (ipc: host); a killed stack leaves its fastdds_* segments, and on a
robot computer that is never rebooted they piled up to 2,632 files / 676 MB (Arduino UNO
Q) and 2,782 / 698 MB (Raspberry Pi 5), and a participant created among them outlasted
the micro-ROS client's 1 s. `fastdds shm clean` removes only segments whose owner is dead.
"""
import os
import sys
import types

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))


def read(*p):
    return open(os.path.join(REPO, *p), encoding="utf-8").read()


def test_the_entrypoint_cleans_while_still_root():
    src = read("docker", "entrypoint.sh")
    i_clean = src.index("timeout 300 fastdds shm clean")
    i_drop = src.index("exec setpriv")
    assert i_clean < i_drop, "only root can remove another user's (or root's) dead segment"
    assert "timeout 300 fastdds shm clean" in src and "|| true" in src[i_clean:i_clean + 200]


def test_the_pipeline_cleans_before_the_flash_and_the_first_launch():
    src = read("scripts", "one_click_pipeline.py")
    i_call = src.index("    clean_dds_shm(args.distro)")
    assert src.index('os.environ["ROS_DOMAIN_ID"] = str(domain)') < i_call
    assert i_call < src.index("[3/6] [FLASH] Updating the firmware")


def test_clean_dds_shm_runs_only_when_there_is_something(monkeypatch, capsys):
    import one_click_pipeline as o
    calls = []
    monkeypatch.setattr(o, "run_ros", lambda cmd, timeout=60, distro="jazzy": calls.append(cmd))
    counts = iter([0])
    monkeypatch.setattr(o, "_dds_shm_files", lambda: next(counts))
    o.clean_dds_shm("jazzy")
    assert calls == []
    counts = iter([2632, 12])
    monkeypatch.setattr(o, "_dds_shm_files", lambda: next(counts))
    o.clean_dds_shm("jazzy")
    assert calls == ["fastdds shm clean"]
    assert "removed 2620 dead Fast DDS shared-memory files (2632 -> 12)" in capsys.readouterr().out
