"""How a robot's LiDAR reaches the robot computer: `base_controller.lidar.comm_mode`.

    serial  the LiDAR on the robot computer's own USB port (`lidar.serial_port`), beside the
            MCU's: two serial ports on the robot computer, and nothing of the scan on the
            micro-ROS link. The usual robot.
    topic   the LiDAR wired to the MCU (`lidar.rx_pin`), its packets sent as /raw_scan over
            the micro-ROS link: ONE serial port, which must be fast enough -- native USB or a
            UART at depth_camera.RAW_SCAN_MIN_BAUD (the GenDrv's 1.5 Mbaud).
    udp     the LiDAR wired to the MCU, its packets sent straight to the robot computer over
            Wi-Fi, outside micro-ROS: a Wi-Fi robot, with no cable to a robot computer.

So the choices follow the micro-ROS transport and the robot computer's ports (user,
2026-10-08: "it should match wifi, number of serial ports"). Measured the same day on a
GenDrv with a real LD19: serial 1.5 Mbaud + topic -- /raw_scan 37.5 Hz, /scan 9.98 Hz, the
IMU at 40 Hz; Wi-Fi + topic -- /scan 0.77 Hz and the IMU starved; Wi-Fi + udp -- every packet
(3798 of 3798) arrived. The web UI offers what options() allows and the backend refuses
what problems() finds.
"""
import depth_camera

MODES = ("serial", "topic", "udp")
DEFAULT_SERIAL_PORT = "/dev/ttyUSB1"     # bringup.launch.py: lidar.serial_port when unset


def _wifi(controller: dict) -> bool:
    t = str((controller or {}).get("transport", "serial") or "serial").strip().lower()
    return t not in ("serial", "")


def _simulated(controller: dict) -> bool:
    lidar = (controller or {}).get("lidar") or {}
    sensors = (controller or {}).get("sensors") or {}
    v = lidar.get("use_sim_ld19", sensors.get("use_sim_ld19", False))
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def normal(mode) -> str:
    """The mode as this module names it: `udp_server` (the LD driver's own word, the Sim MCU's
    configs) is `udp`; nothing named is `serial`, the LD driver's default."""
    m = str(mode or "serial").strip().lower()
    return "udp" if m == "udp_server" else m


def why_not(controller: dict, mode: str) -> str:
    """Why this robot cannot carry its LiDAR as `mode`, or "" when it can."""
    controller = controller or {}
    mode = normal(mode)
    if mode not in MODES:
        return f"lidar.comm_mode must be one of {', '.join(MODES)}, not {mode!r}"
    wifi = _wifi(controller)
    if mode == "serial" and wifi:
        return ("a Wi-Fi robot has no cable to a robot computer: its LiDAR goes as udp, from the "
                "board's LiDAR pin straight to the robot computer")
    if mode == "udp" and not wifi:
        return ("udp is for a Wi-Fi robot: with micro-ROS on the cable, put the LiDAR on the robot "
                "computer's own port (serial), or send it as raw_scan on a fast link (topic)")
    if mode == "topic":
        reason = depth_camera.raw_scan_link_problem(controller)
        if reason:
            return reason
    return ""


def options(controller: dict) -> list:
    """[{mode, ok, why}] for every mode, in MODES order: what the UI offers and why not."""
    return [{"mode": m, "ok": not why_not(controller, m), "why": why_not(controller, m)} for m in MODES]


def problems(controller: dict) -> list:
    """What is wrong with this robot's LiDAR link, as sentences; [] when nothing is."""
    controller = controller or {}
    lidar = controller.get("lidar") or {}
    if not depth_camera.lidar_fitted(controller):
        return []
    mode = normal(lidar.get("comm_mode"))
    out = []
    reason = why_not(controller, mode)
    if reason:
        out.append(reason)
    if mode == "serial":
        # bringup's default when none is named (launchers/bringup.launch.py)
        port = str(lidar.get("serial_port") or "").strip() or DEFAULT_SERIAL_PORT
        if port == str(controller.get("serial_port") or "").strip():
            out.append(f"lidar.serial_port {port} is the MCU's own port: the LiDAR needs a second "
                       f"port on the robot computer")
    elif not _simulated(controller):
        try:
            rx = int(lidar.get("rx_pin", -1))
        except (TypeError, ValueError):
            rx = -1
        if rx < 0:
            out.append(f"lidar.comm_mode {mode} reads the LiDAR on the MCU: set lidar.rx_pin to the "
                       f"GPIO wired to the LiDAR's TX")
    return out
