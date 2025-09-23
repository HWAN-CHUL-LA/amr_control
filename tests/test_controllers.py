from pathlib import Path

import pytest

from control_lib import controllers


@pytest.fixture(autouse=True)
def reset_send_functions(monkeypatch):
    # Ensure the controllers use deterministic timing in tests.
    monkeypatch.setattr(controllers.time, "sleep", lambda *_args, **_kwargs: None)


def test_drive_controller_fetch_status(monkeypatch):
    events = []

    def fake_send_sdo_write(*_args, **_kwargs):
        return None

    def fake_send_sdo_read(_bus, _node, index, _sub, _timeout):
        if index == controllers.OD_ACTUAL_VELOCITY:
            return 2000
        if index == controllers.OD_ACTUAL_POSITION:
            return 10000
        return 0

    monkeypatch.setattr(controllers, "send_sdo_write", fake_send_sdo_write)
    monkeypatch.setattr(controllers, "send_sdo_read", fake_send_sdo_read)

    drive = controllers.DriveController([1], status_callback=events.append)
    drive.setup()
    drive.adjust_speed(controllers.DRIVE_SPEED_STEP_MPS)

    assert events, "Status callback should record controller events"

    status = drive.fetch_status()
    assert status["target_speed_mps"] == pytest.approx(controllers.DRIVE_SPEED_STEP_MPS)
    assert status["actual_velocity_mps"][1] == pytest.approx(controllers.pps_to_mps(2000))
    assert status["actual_position_m"][1] == pytest.approx(
        controllers.pulses_to_meters(10000)
    )


def test_steer_controller_offsets(tmp_path: Path, monkeypatch):
    events = []
    writes = []

    def fake_send_sdo_write(_bus, node, index, _sub, value, *_rest, **__rest):
        writes.append((node, index, value))

    def fake_send_sdo_read(_bus, _node, index, _sub, _timeout):
        if index == controllers.OD_ACTUAL_POSITION:
            return controllers.deg_to_pulses(10.0)
        if index == controllers.OD_ACTUAL_VELOCITY:
            return controllers.dps_to_pps(5.0)
        return 0

    monkeypatch.setattr(controllers, "send_sdo_write", fake_send_sdo_write)
    monkeypatch.setattr(controllers, "send_sdo_read", fake_send_sdo_read)

    offset_path = tmp_path / "offsets.json"

    steer = controllers.SteerController(
        [2], status_callback=events.append, offset_path=offset_path
    )
    steer.setup()
    steer.adjust_offset(2, controllers.STEER_OFFSET_STEP_DEG)
    steer.save_offsets()
    assert offset_path.exists()

    status = steer.fetch_status()
    assert status["actual_angle_deg"][2] == pytest.approx(10.0)
    assert status["actual_velocity_dps"][2] == pytest.approx(5.0)
    assert steer.angle_offsets_deg[2] == pytest.approx(controllers.STEER_OFFSET_STEP_DEG)
    assert events, "Status callback should record steer events"

    with offset_path.open() as fh:
        saved = fh.read()
    assert "\"2\"" in saved
