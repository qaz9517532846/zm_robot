# SPDX-License-Identifier: Apache-2.0
import os

PACKAGE = "zm_robot_omni_policy"
SOURCE_MODEL_DIR = os.path.expanduser("~/catkin_ws/src/zm_robot/zm_robot_omni_policy/models")


def resolve_model_dir(param_value=""):
    """Where the ONNX/engine files live. The models are several hundred MB, so they are NOT
    copied into the colcon install space; order: explicit parameter, $ZM_ROBOT_OMNI_MODEL_DIR,
    <share>/models (only exists with a symlink install), then the source tree."""
    candidates = [param_value, os.environ.get("ZM_ROBOT_OMNI_MODEL_DIR", "")]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory(PACKAGE), "models"))
    except Exception:    # pylint: disable=broad-except
        pass
    candidates.append(SOURCE_MODEL_DIR)
    for c in candidates:
        if c and os.path.isdir(os.path.expanduser(c)):
            return os.path.expanduser(c)
    raise FileNotFoundError(f"no model directory found; tried {[c for c in candidates if c]}")
