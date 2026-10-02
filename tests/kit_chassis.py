"""Reference designs built on a vendor's own robot kit, and what they may differ in.

Every other reference is a board on the one default chassis (test_default_chassis):
nothing about its wheels or motors has been measured, so it ships the shared
numbers and two Nav2 results stay comparable. A kit is the opposite case -- the
chassis is a product with published wheels, gearmotors and encoders, and the
default chassis would be a guess about a robot whose numbers are known. So a
kit ships its vendor's numbers, and exactly these differences are allowed.

Nothing else is: the laser mount, the costmap radii, the frames and the rest of
the Nav2/EKF/SLAM template stay shared, so a kit still runs the same stack.
"""

KIT_CHASSIS = {
    # SparkFun XRP robot kit: 60 mm wheels on a 155 mm track, 140 rpm (4.5 V)
    # gearmotors with 585-count encoders, 0.8 kg. The left encoder is mounted
    # mirrored; the RP2 PIO encoder reads A and A+1, so the pins cannot be
    # swapped and the direction is an invert. All of it is from SparkFun's
    # schematic and XRPLib, and unverified on a kit until one is on the bench.
    "xrp": {
        "inverted": {"encoder1"},
        "simulation": {"robot_mass", "motor_stall_amps"},
    },
}

# What a kit's own motors change. These are the keys
# scripts/drivetrain_report.py derives from the drivetrain, so a kit's values
# are the tool's output for its motors rather than a second set of numbers.
DERIVED_NAV2_KEYS = {
    ("nav2", "velocity_smoother", "ros__parameters", "max_velocity"),
    ("nav2", "velocity_smoother", "ros__parameters", "min_velocity"),
    ("nav2", "velocity_smoother", "ros__parameters", "max_accel"),
    ("nav2", "velocity_smoother", "ros__parameters", "max_decel"),
    ("nav2", "controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"),
    ("nav2", "controller_server", "ros__parameters", "FollowPath", "rotate_to_heading_angular_vel"),
    ("nav2", "controller_server", "ros__parameters", "FollowPath", "max_angular_accel"),
    ("nav2", "behavior_server", "ros__parameters", "max_rotational_vel"),
    ("nav2", "behavior_server", "ros__parameters", "min_rotational_vel"),
    ("nav2", "behavior_server", "ros__parameters", "rotational_acc_lim"),
}


def kit(config_name):
    """The kit entry for 'xrp_config.yaml' or 'xrp', or None for a default-chassis design."""
    name = config_name[: -len("_config.yaml")] if config_name.endswith("_config.yaml") else config_name
    return KIT_CHASSIS.get(name)
