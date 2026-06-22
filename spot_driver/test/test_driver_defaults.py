from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_gripper_camera_launch_defaults_disabled_with_1920x1080_10hz_settings():
    launch_text = (REPO_ROOT / "spot_driver/launch/spot_driver.launch.py").read_text()

    assert '"gripper_camera",\n        default_value="false"' in launch_text
    assert '"gripper_camera_rate",\n        default_value="10.0"' in launch_text
    assert '"gripper_camera_resolution",\n        default_value="1920x1080"' in launch_text


def test_gripper_camera_node_default_disabled():
    driver_text = (REPO_ROOT / "spot_driver/spot_driver/spot_driver.py").read_text()

    assert 'self.declare_parameter("gripper_camera", False)' in driver_text


def test_take_lease_launch_argument_defaults_enabled_and_passes_to_node():
    launch_text = (REPO_ROOT / "spot_driver/launch/spot_driver.launch.py").read_text()

    assert '"take_lease",\n        default_value="true"' in launch_text
    assert 'take_lease = LaunchConfiguration("take_lease")' in launch_text
    assert '"take_lease": take_lease' in launch_text


def test_take_lease_node_default_enabled_and_forces_take_before_keepalive():
    driver_text = (REPO_ROOT / "spot_driver/spot_driver/spot_driver.py").read_text()

    assert 'self.declare_parameter("take_lease", True)' in driver_text
    assert 'self.take_lease = self.get_parameter("take_lease").get_parameter_value().bool_value' in driver_text
    assert "must_acquire=not self.take_lease" in driver_text
    take_index = driver_text.index("lease_client.take()")
    keepalive_index = driver_text.index("LeaseKeepAlive(")
    assert take_index < keepalive_index
