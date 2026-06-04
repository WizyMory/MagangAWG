"""Launch file to publish the robot description and joint states.

This launch file performs two tasks:
- Render the robot URDF/XACRO into a `robot_description` parameter and
    run `robot_state_publisher` so TF and the robot model are available.
- Run `joint_state_publisher` which provides joint state messages (useful
    for visualizing movable joints in RViz).

Files expected in the `amr_description` package:
- `urdf/amr.urdf.xacro` : the robot model expressed as a xacro file
- `config/robot_params.yaml`: optional parameters for robot_state_publisher

To use: `ros2 launch amr_description display.launch.py`
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo
import os
import yaml
import re
import subprocess
import json


def generate_launch_description():
    # Locate package resources (URDF/XACRO and YAML config).
    # Prefer the package source directory in the workspace (so changes in
    # src/ are picked up immediately). If the source files aren't present
    # fall back to the installed package location returned by
    # get_package_share_directory.
    this_file_dir = os.path.dirname(__file__)
    src_pkg_dir = os.path.abspath(os.path.join(this_file_dir, '..'))
    # expected source xacro path
    src_xacro = os.path.join(src_pkg_dir, 'urdf', 'amr.urdf.xacro')
    if os.path.exists(src_xacro):
        pkg_share = src_pkg_dir
    else:
        pkg_share = get_package_share_directory('amr_description')

    # Paths to model and parameter files inside the package. Prefer the
    # source files in the workspace (`src/amr_description/...`) when they
    # exist so edits are reflected immediately.
    src_xacro_path = os.path.join(src_pkg_dir, 'urdf', 'amr.urdf.xacro')
    src_params_path = os.path.join(src_pkg_dir, 'config', 'robot_params.yaml')

    if os.path.exists(src_xacro_path):
        xacro_file = src_xacro_path
    else:
        xacro_file = os.path.join(pkg_share, 'urdf', 'amr.urdf.xacro')

    if os.path.exists(src_params_path):
        params_yaml = src_params_path
    else:
        params_yaml = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    # Read YAML parameters so we can pass them as xacro property overrides.
    # This makes it possible to change values in robot_params.yaml and
    # see the updated geometry after relaunching (no colcon build required).
    params = {}
    try:
        with open(params_yaml, 'r') as f:
            doc = yaml.safe_load(f)
            # Expecting structure: {amr_description: {ros__parameters: { ... }}}
            params = doc.get('amr_description', {}).get('ros__parameters', {}) or {}
    except Exception:
        params = {}

    # Build xacro property overrides from params (e.g. 'base_length:=0.65')
    xacro_props = []
    for k, v in params.items():
        # only simple scalar properties expected
        xacro_props.append(f"{k}:={v}")

    # If we have params, render a temporary xacro file with updated property
    # values and run xacro now to produce the URDF string. We then pass the
    # rendered URDF directly as the `robot_description` parameter. This
    # avoids issues with Command concatenation and ensures YAML edits in
    # `src/` are applied immediately.
    robot_description = None
    info_msgs = []
    try:
        if params:
            # Read original xacro content
            with open(xacro_file, 'r') as f:
                xacro_text = f.read()

            # Replace xacro property value attributes for simple scalar props
            for k, v in params.items():
                # match patterns like: name="base_length" value="0.60"
                pattern = rf'(name="{k}"\s+value=")([^"]+)(")'
                # Use a callable replacement to avoid backreference escape issues
                xacro_text, nsubs = re.subn(pattern, lambda m, val=v: m.group(1) + str(val) + m.group(3), xacro_text)

            # write temp xacro file
            import tempfile
            fd, tmp_path = tempfile.mkstemp(suffix='.xacro', prefix='amr_')
            with os.fdopen(fd, 'w') as tmpf:
                tmpf.write(xacro_text)

            # run xacro on the temp file to get URDF
            rendered = subprocess.check_output(['xacro', tmp_path], text=True, stderr=subprocess.STDOUT)
            robot_description = {'robot_description': rendered}

            # collect debug info
            m = re.search(r'box size="([^"]+)"', rendered)
            if m:
                info_msgs.append(f"rendered box size: {m.group(1)}")
            info_msgs.append(f"rendered from src xacro: {tmp_path}")
        else:
            # No params: fall back to running xacro directly on the file at
            # launch-time via Command substitution
            props_str = '' if not xacro_props else (' ' + ' '.join(xacro_props))
            xacro_cmd_str = f"xacro {xacro_file}{props_str}"
            robot_description = {'robot_description': Command([xacro_cmd_str])}
            info_msgs.append(f"xacro command: {xacro_cmd_str}")
    except Exception as e:
        # On any error, fallback to Command form and log the exception
        props_str = '' if not xacro_props else (' ' + ' '.join(xacro_props))
        xacro_cmd_str = f"xacro {xacro_file}{props_str}"
        robot_description = {'robot_description': Command([xacro_cmd_str])}
        info_msgs.append(f"xacro fallback command: {xacro_cmd_str}")
        info_msgs.append(f"render error: {e}")

    # Add more debug info: show params read and robot_description length
    try:
        info_msgs.append("params: " + json.dumps(params))
    except Exception:
        info_msgs.append("params: (could not serialize)")

    rd_len = 'n/a'
    try:
        if isinstance(robot_description, dict):
            rd = robot_description.get('robot_description')
            if isinstance(rd, str):
                rd_len = len(rd)
        elif isinstance(robot_description, str):
            rd_len = len(robot_description)
    except Exception:
        rd_len = 'error'

    info_msgs.append(f"robot_description length: {rd_len}")

    # LogInfo actions help debugging at launch time
    log_actions = [LogInfo(msg=m) for m in info_msgs]

    return LaunchDescription(
        log_actions + [
        # Publish robot_description and TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            # Pass rendered robot_description and an optional params YAML
            parameters=[robot_description, params_yaml],
        ),

        # Publish joint states (useful for RViz to animate joints)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
    ])
