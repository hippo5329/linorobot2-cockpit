"""Linorobot2 Cockpit routes -- Nav2 / EKF / SLAM config editors and AI tuning.

Part of the main.py split: route handlers registered on the shared `app`
as an import side effect (main.py imports this module). Shared state and
helpers come from core.py. See core.py for the split's contract.
"""
import os
import yaml
from fastapi import HTTPException, Request
from core import (
    REPO_ROOT,
    analyze_robotics_ai,
    app,
    gen_robot_description,
    generate_custom_robot_specs,
    get_active_params_path,
    json_body,
    load_params,
    patcher,
    regenerate_firmware_headers,
    regenerate_robot_description,
    save_params,
)


@app.get("/api/nav2_config")
def api_get_nav2_config(distro: str = "jazzy"):
    params = load_params()
    kine = params.get("kinematics", {})
    base = kine.get("base_type", "2wd")
    with open(get_active_params_path(), "r") as f:
        full_text = f.read()
    return {
        "distro": distro,
        "base": base,
        "config": yaml.dump(params.get("nav2", {}), sort_keys=False),
        "full_yaml": full_text,
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
        "supported_distros": ["jazzy", "lyrical", "rolling"],
        "supported_bases": ["2wd", "4wd", "mecanum"],
        "depth_pointcloud_active": patcher.costmap_depth_active(full_text),
    }


@app.post("/api/nav2_config")
async def api_save_nav2_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    nav2_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["nav2"] = nav2_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/nav2_config/patch")
async def api_nav2_config_patch(request: Request):
    data = await json_body(request)
    base = data.get("base", "2wd")
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_nav2_text(cur_text, **patcher.nav2_kwargs(dict(data, base_type=base)))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "base": base, "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/nav2_config/costmap_sources")
async def api_nav2_costmap_sources(request: Request):
    data = await json_body(request)
    depth_enabled = bool(data.get("depth_enabled"))
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_costmap_sources(cur_text, depth_enabled)
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "ok", "depth_pointcloud_active": depth_enabled}


@app.post("/api/nav2_config/reset")
def api_nav2_config_reset():
    # Restore defaults from standard_diff preset
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_nav2_text(cur_text, **patcher.nav2_kwargs(pinfo))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


@app.get("/api/ekf_config")
def api_get_ekf_config(base: str = "2wd"):
    params = load_params()
    return {
        "base": base,
        "config": yaml.dump(params.get("ekf", {}), sort_keys=False),
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
        "supported_bases": ["2wd", "4wd", "mecanum"],
    }


@app.post("/api/ekf_config")
async def api_save_ekf_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    ekf_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["ekf"] = ekf_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/ekf_config/patch")
async def api_ekf_config_patch(request: Request):
    data = await json_body(request)
    base = data.get("base", "2wd")
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_ekf_text(cur_text, **patcher.ekf_kwargs(data, base=base))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "base": base, "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/ekf_config/reset")
def api_ekf_config_reset():
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_ekf_text(cur_text, **patcher.ekf_kwargs(pinfo, base="2wd"))
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


@app.get("/api/slam_config")
def api_get_slam_config():
    params = load_params()
    return {
        "config": yaml.dump(params.get("slam", {}), sort_keys=False),
        "path": os.path.relpath(get_active_params_path(), REPO_ROOT),
        "exists": True,
    }


@app.post("/api/slam_config")
async def api_save_slam_config(request: Request):
    data = await json_body(request)
    cfg_text = data.get("config", "")
    if not cfg_text.strip():
        raise HTTPException(status_code=400, detail="Empty configuration")
    slam_data = yaml.safe_load(cfg_text)
    params = load_params()
    params["slam"] = slam_data
    save_params(params)
    return {"status": "ok", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/slam_config/patch")
async def api_slam_config_patch(request: Request):
    data = await json_body(request)
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_slam_text(
        cur_text,
        resolution=data.get("resolution"),
        max_laser_range=data.get("max_laser_range"),
        minimum_travel_distance=data.get("minimum_travel_distance"),
        minimum_travel_heading=data.get("minimum_travel_heading"),
    )
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "patched", "path": os.path.relpath(get_active_params_path(), REPO_ROOT)}


@app.post("/api/slam_config/reset")
def api_slam_config_reset():
    pinfo = patcher.PRESETS["standard_diff"]
    with open(get_active_params_path(), "r") as f:
        cur_text = f.read()
    patched = patcher.patch_slam_text(cur_text, resolution=pinfo["slam_resolution"], max_laser_range=pinfo["slam_max_range"])
    with open(get_active_params_path(), "w") as f:
        f.write(patched)
    return {"status": "reset"}


# ==============================================================================
# AI Assistant & Custom Robot Builder APIs
# ==============================================================================
@app.post("/api/ai/tune")
async def api_ai_tune(request: Request):
    data = await json_body(request)
    prompt = data.get("prompt", "")
    base = data.get("base", "2wd")
    distro = data.get("distro", "jazzy")
    return analyze_robotics_ai(prompt, base=base, distro=distro)


@app.post("/api/ai/apply")
async def api_ai_apply(request: Request):
    data = await json_body(request)
    distro = data.get("distro", "jazzy")
    nav2_p = data.get("nav2_patch") or {}
    ekf_p = data.get("ekf_patch") or {}
    slam_p = data.get("slam_patch") or {}
    resolved_base = data.get("base") or nav2_p.get("base") or ekf_p.get("base") or "2wd"

    with open(get_active_params_path(), "r") as f:
        text = f.read()

    if nav2_p:
        text = patcher.patch_nav2_text(text, **patcher.nav2_kwargs(dict(nav2_p, base_type=resolved_base)))
    if ekf_p:
        text = patcher.patch_ekf_text(text, **patcher.ekf_kwargs(ekf_p, base=resolved_base))
    if slam_p:
        text = patcher.patch_slam_text(
            text,
            resolution=slam_p.get("resolution"),
            max_laser_range=slam_p.get("max_laser_range"),
            minimum_travel_distance=slam_p.get("minimum_travel_distance"),
            minimum_travel_heading=slam_p.get("minimum_travel_heading"),
        )

    with open(get_active_params_path(), "w") as f:
        f.write(text)

    return {
        "status": "ai_applied",
        "distro": distro,
        "nav2_updated": bool(nav2_p),
        "ekf_updated": bool(ekf_p),
        "slam_updated": bool(slam_p),
    }


@app.post("/api/ai/robot_builder")
async def api_ai_robot_builder(request: Request):
    data = await json_body(request)
    description = data.get("description", "")
    return generate_custom_robot_specs(description)


@app.post("/api/ai/deploy_robot")
async def api_ai_deploy_robot(request: Request):
    data = await json_body(request)
    specs = data.get("specs") or {}
    design = specs.get("design") or {}
    tuning = specs.get("tuning") or {}
    base = design.get("base_type", "2wd")

    params = load_params()
    # The design speaks the config's own vocabulary (system_utils
    # generate_custom_robot_specs): the keys the firmware and the URDF read.
    # This used to write `track_width` and `wheelbase`, which nothing read.
    kine = params.setdefault("kinematics", {})
    kine["base_type"] = base
    for key in ("wheel_diameter", "lr_wheels_distance", "fr_wheels_distance"):
        if design.get(key) is not None:
            kine[key] = design[key]
    # A new chassis gets a body sized to it; the user's explicit geometry, if
    # any, is stale for the new kinematics, so it is derived afresh.
    params.pop("geometry", None)
    params["geometry"] = gen_robot_description.effective_geometry(params)

    save_params(params)
    res = regenerate_firmware_headers()
    regenerate_robot_description(params)

    return {
        "status": "deployed",
        "base": base,
        "message": f"Successfully configured and deployed custom {base.upper()} robot!",
        "generator_stdout": res.stdout,
    }
