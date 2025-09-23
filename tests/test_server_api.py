import asyncio
from contextlib import asynccontextmanager

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

import importlib

app_module = importlib.import_module("server.app")

from control_lib.controllers import (
    DRIVE_ACCEL_STEP_PPS2,
    DRIVE_NODE_CONFIG,
    DRIVE_SPEED_STEP_MPS,
    STEER_ACCEL_STEP_DPS2,
    STEER_DEG_STEP,
    STEER_NODE_CONFIG,
    STEER_OFFSET_STEP_DEG,
    STEER_VEL_STEP_DPS,
)


class FakeService:
    def __init__(self):
        self.drive_speed = 0.0
        self.drive_accel = 0
        self.steer_angle = 0.0
        self.steer_velocity = 0.0
        self.steer_accel = 0.0
        self.offsets = {}
        self.started = False
        self.stopped = False
        self.steps = {
            "drive": {
                "speed_step_mps": DRIVE_SPEED_STEP_MPS,
                "accel_step_pps2": DRIVE_ACCEL_STEP_PPS2,
            },
            "steer": {
                "angle_step_deg": STEER_DEG_STEP,
                "velocity_step_dps": STEER_VEL_STEP_DPS,
                "accel_step_dps2": STEER_ACCEL_STEP_DPS2,
                "offset_step_deg": STEER_OFFSET_STEP_DEG,
            },
        }

    def start(self):
        self.started = True

    def shutdown(self):
        self.stopped = True

    # Drive operations
    def set_drive_speed(self, value):
        self.drive_speed = value

    def adjust_drive_speed(self, delta):
        self.drive_speed += delta

    def adjust_drive_accel(self, delta):
        self.drive_accel += delta

    def drive_status(self):
        return {
            "target_speed_mps": self.drive_speed,
            "target_accel_mps2": self.drive_accel,
            "actual_velocity_mps": {},
            "actual_position_m": {},
        }

    # Steer operations
    def set_steer_angle(self, value):
        self.steer_angle = value

    def adjust_steer_angle(self, delta):
        self.steer_angle += delta

    def adjust_steer_velocity(self, delta):
        self.steer_velocity += delta

    def adjust_steer_accel(self, delta):
        self.steer_accel += delta

    def adjust_steer_offset(self, node_id, delta):
        self.offsets[node_id] = self.offsets.get(node_id, 0.0) + delta

    def save_offsets(self):
        pass

    def load_offsets(self):
        pass

    def print_drive_actuals(self):
        pass

    def print_steer_offsets(self):
        pass

    def print_steer_actuals(self):
        pass

    def steer_status(self):
        return {
            "target_deg": self.steer_angle,
            "profile_vel_dps": self.steer_velocity,
            "profile_accel_dps2": self.steer_accel,
            "actual_angle_deg": {},
            "actual_velocity_dps": {},
            "offsets_deg": self.offsets.copy(),
        }

    def combined_status(self):
        return {"drive": self.drive_status(), "steer": self.steer_status(), "steps": self.steps}

    def steer_nodes(self):
        return [2]

    def has_drive(self):
        return True

    def has_steer(self):
        return True


class ImmediateBroadcaster:
    """Simplified broadcaster used in tests."""

    def __init__(self, service, interval=0.5):
        self.service = service

    async def register(self, websocket):
        await websocket.send_json(self.service.combined_status())

    async def unregister(self, websocket):
        pass

    async def shutdown(self):
        pass


@pytest.fixture
def client(monkeypatch):
    monkeypatch.delenv("AMR_API_TOKEN", raising=False)
    fake_service = FakeService()

    async def fake_get_service():
        return fake_service

    async def fake_get_broadcaster():
        return ImmediateBroadcaster(fake_service)

    # Override dependency accessors directly
    monkeypatch.setattr(app_module, "create_control_service", lambda: fake_service)
    monkeypatch.setattr(app_module, "create_broadcaster", lambda service: ImmediateBroadcaster(service))

    with TestClient(app_module.app) as test_client:
        yield test_client, fake_service


def test_drive_endpoints(client):
    test_client, service = client

    res = test_client.post("/drive/speed", json={"value": 1.2})
    assert res.status_code == 200
    assert service.drive_speed == pytest.approx(1.2)

    res = test_client.post("/drive/speed/adjust", json={"delta": 0.3})
    assert res.status_code == 200
    assert service.drive_speed == pytest.approx(1.5)

    res = test_client.post("/drive/accel", json={"delta": 100})
    assert res.status_code == 200
    assert service.drive_accel == 100

    res = test_client.post("/drive/emergency_stop")
    assert res.status_code == 200
    assert service.drive_speed == 0.0


def test_steer_endpoints(client):
    test_client, service = client

    res = test_client.post("/steer/angle", json={"value": 10.0})
    assert res.status_code == 200
    assert service.steer_angle == 10.0

    res = test_client.post("/steer/angle/adjust", json={"delta": -2.0})
    assert res.status_code == 200
    assert service.steer_angle == 8.0

    res = test_client.post("/steer/velocity", json={"delta": 1.0})
    assert res.status_code == 200
    assert service.steer_velocity == 1.0

    res = test_client.post("/steer/accel", json={"delta": 2.5})
    assert res.status_code == 200
    assert service.steer_accel == 2.5

    res = test_client.post("/steer/offset", json={"node_id": 2, "delta": 0.1})
    assert res.status_code == 200
    assert service.offsets[2] == pytest.approx(0.1)

    res = test_client.get("/status")
    payload = res.json()
    assert payload["steer"]["target_deg"] == pytest.approx(8.0)
    assert payload["steps"]["drive"]["speed_step_mps"] == pytest.approx(DRIVE_SPEED_STEP_MPS)
    assert payload["steps"]["steer"]["offset_step_deg"] == pytest.approx(STEER_OFFSET_STEP_DEG)


def test_ui_root(client):
    test_client, _ = client
    res = test_client.get("/")
    assert res.status_code == 200
    assert "AMR Control Console" in res.text



def test_status_websocket(client):
    test_client, service = client

    with test_client.websocket_connect("/ws/status") as ws:
        message = ws.receive_json()
        assert message["drive"]["target_speed_mps"] == service.drive_speed


def test_auth_required_when_token_set(monkeypatch):
    monkeypatch.setenv("AMR_API_TOKEN", "secret")
    fake_service = FakeService()
    monkeypatch.setattr(app_module, "create_control_service", lambda: fake_service)
    monkeypatch.setattr(app_module, "create_broadcaster", lambda service: ImmediateBroadcaster(service))
    with TestClient(app_module.app) as test_client:
        res = test_client.get("/status")
        assert res.status_code == 401
        res = test_client.get("/status", headers={"Authorization": "Bearer secret"})
        assert res.status_code == 200
        with pytest.raises(WebSocketDisconnect):
            with test_client.websocket_connect("/ws/status"):
                pass
        with test_client.websocket_connect("/ws/status?token=secret") as ws:
            payload = ws.receive_json()
            assert payload["drive"]

