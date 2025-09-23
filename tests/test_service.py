from typing import Dict

from control_lib import service, controllers
from control_lib.controllers import (
    DRIVE_NODE_CONFIG,
    STEER_NODE_CONFIG,
)


class FakeDriveController:
    instances = []

    def __init__(self, nodes, status_callback=None):
        self.__class__.instances.append(self)
        self.nodes = nodes
        self.status_callback = status_callback
        self.setup_called = False
        self.speed_calls = []
        self.accel_calls = []
        self.actual_prints = 0
        self.stopped = False

    def setup(self):
        self.setup_called = True

    def set_speed(self, value):
        self.speed_calls.append(("set", value))

    def adjust_speed(self, delta):
        self.speed_calls.append(("adjust", delta))

    def adjust_accel(self, delta):
        self.accel_calls.append(delta)

    def fetch_status(self) -> Dict[str, object]:
        return {
            "target_speed_mps": 0.0,
            "target_accel_mps2": 0.0,
            "actual_velocity_mps": {node: 0.0 for node in self.nodes},
            "actual_position_m": {node: 0.0 for node in self.nodes},
        }

    def print_actual_state(self):
        self.actual_prints += 1

    def emergency_stop(self):
        self.stopped = True


class FakeSteerController:
    instances = []

    def __init__(self, nodes, status_callback=None, offset_path=None):
        self.__class__.instances.append(self)
        self.nodes = nodes
        self.status_callback = status_callback
        self.offset_path = offset_path
        self.setup_called = False
        self.angle_calls = []
        self.velocity_calls = []
        self.accel_calls = []
        self.offset_calls = []
        self.offset_saved = False
        self.offset_loaded = False
        self.push_count = 0
        self.trigger_count = 0
        self.offset_prints = 0
        self.actual_prints = 0

    def setup(self):
        self.setup_called = True

    def set_position(self, value):
        self.angle_calls.append(("set", value))

    def adjust_position(self, delta):
        self.angle_calls.append(("adjust", delta))

    def adjust_velocity(self, delta):
        self.velocity_calls.append(delta)

    def adjust_accel(self, delta):
        self.accel_calls.append(delta)

    def adjust_offset(self, node_id, delta):
        self.offset_calls.append((node_id, delta))

    def save_offsets(self):
        self.offset_saved = True

    def load_offsets(self):
        self.offset_loaded = True

    def _push_position(self):
        self.push_count += 1

    def _trigger_motion(self):
        self.trigger_count += 1

    def fetch_status(self) -> Dict[str, object]:
        return {
            "target_deg": 0.0,
            "profile_vel_dps": 0.0,
            "profile_accel_dps2": 0.0,
            "actual_angle_deg": {node: 0.0 for node in self.nodes},
            "actual_velocity_dps": {node: 0.0 for node in self.nodes},
            "offsets_deg": {node: 0.0 for node in self.nodes},
        }

    def print_offsets(self):
        self.offset_prints += 1

    def print_actual_position(self):
        self.actual_prints += 1


def test_service_wraps_controllers(monkeypatch):
    recorded_channels = set()
    shutdown_called = False

    monkeypatch.setattr(service, "ensure_buses", lambda channels: recorded_channels.update(channels))

    def fake_shutdown():
        nonlocal shutdown_called
        shutdown_called = True

    monkeypatch.setattr(service, "shutdown_buses", fake_shutdown)
    monkeypatch.setattr(service, "DriveController", FakeDriveController)
    monkeypatch.setattr(service, "SteerController", FakeSteerController)

    FakeDriveController.instances.clear()
    FakeSteerController.instances.clear()

    config = service.ControllerConfig(drive_nodes=[1], steer_nodes=[2])
    amr = service.AmrControlService(config)

    amr.start()

    assert recorded_channels == {
        DRIVE_NODE_CONFIG[1],
        STEER_NODE_CONFIG[2],
    }
    assert amr.has_drive()
    assert amr.has_steer()
    assert amr.steer_nodes() == [2]

    amr.set_drive_speed(0.5)
    amr.adjust_drive_speed(0.1)
    amr.adjust_drive_accel(100)
    amr.print_drive_actuals()

    amr.set_steer_angle(10.0)
    amr.adjust_steer_angle(1.0)
    amr.adjust_steer_velocity(2.0)
    amr.adjust_steer_accel(3.0)
    amr.adjust_steer_offset(2, 0.5)
    amr.print_steer_offsets()
    amr.print_steer_actuals()
    amr.save_offsets()
    amr.load_offsets()

    combined = amr.combined_status()
    assert "drive" in combined and "steer" in combined
    assert combined["steps"]["drive"]["speed_step_mps"] == controllers.DRIVE_SPEED_STEP_MPS
    assert combined["steps"]["steer"]["offset_step_deg"] == controllers.STEER_OFFSET_STEP_DEG

    drive = FakeDriveController.instances[0]
    steer = FakeSteerController.instances[0]
    assert drive.speed_calls
    assert drive.accel_calls
    assert drive.actual_prints == 1
    assert steer.angle_calls
    assert steer.velocity_calls
    assert steer.accel_calls
    assert steer.offset_calls
    assert steer.offset_saved
    assert steer.offset_loaded
    assert steer.push_count == 1
    assert steer.trigger_count == 1
    assert steer.offset_prints == 1
    assert steer.actual_prints == 1

    amr.shutdown()
    assert shutdown_called

    assert drive.stopped
