"""FastAPI app that wraps the AMR control service."""

from __future__ import annotations

import asyncio
import logging
import os
import secrets
from pathlib import Path
from typing import Dict, List, Optional, Set

from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.concurrency import run_in_threadpool
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from contextlib import asynccontextmanager
from pydantic import BaseModel, Field

from control_lib.controllers import (
    DRIVE_NODE_CONFIG,
    STEER_NODE_CONFIG,
)
from control_lib.service import AmrControlService, ControllerConfig

logger = logging.getLogger(__name__)


class StatusBroadcaster:
    """Broadcasts combined status snapshots to connected websocket clients."""

    def __init__(self, service: AmrControlService, interval: float = 0.5) -> None:
        self._service = service
        self._interval = interval
        self._connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()
        self._task: Optional[asyncio.Task[None]] = None
        self._stopped = asyncio.Event()

    async def register(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._connections.add(websocket)
            if not self._task:
                self._stopped.clear()
                self._task = asyncio.create_task(self._run(), name="status-broadcaster")

    async def unregister(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._connections.discard(websocket)
            if not self._connections and self._task:
                self._task.cancel()
                self._task = None
                self._stopped.set()

    async def shutdown(self) -> None:
        async with self._lock:
            connections = list(self._connections)
            self._connections.clear()
            task = self._task
            self._task = None
            self._stopped.set()
        for ws in connections:
            try:
                await ws.close()
            except Exception:  # pragma: no cover - best effort
                pass
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    async def _run(self) -> None:
        try:
            while True:
                await asyncio.sleep(self._interval)
                async with self._lock:
                    connections = list(self._connections)
                if not connections:
                    return
                try:
                    data = await run_in_threadpool(self._service.combined_status)
                except Exception:  # pragma: no cover - logged for diagnostics
                    logger.exception("Failed to fetch combined status")
                    continue
                drop: List[WebSocket] = []
                for ws in connections:
                    try:
                        await ws.send_json(data)
                    except WebSocketDisconnect:
                        drop.append(ws)
                    except Exception:  # pragma: no cover - defensive
                        logger.exception("Failed to push status update")
                        drop.append(ws)
                if drop:
                    async with self._lock:
                        for ws in drop:
                            self._connections.discard(ws)
        except asyncio.CancelledError:
            pass


# --- FastAPI wiring ----------------------------------------------------------

_control_service: Optional[AmrControlService] = None
_broadcaster: Optional[StatusBroadcaster] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _control_service, _broadcaster
    service = create_control_service()
    await run_in_threadpool(service.start)
    broadcaster = create_broadcaster(service)
    _control_service = service
    _broadcaster = broadcaster
    logger.info("Control service started")
    try:
        yield
    finally:
        if _broadcaster:
            await _broadcaster.shutdown()
        if _control_service:
            await run_in_threadpool(_control_service.shutdown)
        _broadcaster = None
        _control_service = None
        logger.info("Control service stopped")


app = FastAPI(title="AMR Control API", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("AMR_API_CORS", "*").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def get_configured_token() -> Optional[str]:
    token = os.getenv("AMR_API_TOKEN")
    return token.strip() if token else None


def _extract_bearer_token(value: Optional[str]) -> Optional[str]:
    if not value:
        return None
    parts = value.split(" ", 1)
    if len(parts) == 2 and parts[0].lower() == "bearer":
        return parts[1]
    return None


def _tokens_match(provided: Optional[str], expected: str) -> bool:
    return bool(provided) and secrets.compare_digest(provided, expected)


async def require_token(request: Request) -> None:
    expected = get_configured_token()
    if not expected:
        return
    header_token = _extract_bearer_token(request.headers.get("Authorization"))
    if _tokens_match(header_token, expected):
        return
    query_token = request.query_params.get("token")
    if _tokens_match(query_token, expected):
        return
    raise HTTPException(status_code=401, detail="Unauthorized")


async def ensure_ws_authorized(websocket: WebSocket) -> bool:
    expected = get_configured_token()
    if not expected:
        return True
    query_token = websocket.query_params.get("token")
    header_token = _extract_bearer_token(websocket.headers.get("Authorization"))
    candidate = query_token or header_token
    if _tokens_match(candidate, expected):
        return True
    await websocket.close(code=4401, reason="Unauthorized")
    return False


def parse_nodes(value: Optional[str], config: Dict[int, str]) -> List[int]:
    if value is None or value.strip().lower() in ("", "all"):
        return sorted(config.keys())
    parts = [item.strip() for item in value.split(",") if item.strip()]
    nodes = []
    for part in parts:
        node = int(part)
        if node not in config:
            raise ValueError(f"Unsupported node {node} for config {sorted(config.keys())}")
        nodes.append(node)
    return sorted(nodes)


def create_control_service() -> AmrControlService:
    drive_nodes = parse_nodes(os.getenv("AMR_DRIVE_NODES"), DRIVE_NODE_CONFIG)
    steer_nodes = parse_nodes(os.getenv("AMR_STEER_NODES"), STEER_NODE_CONFIG)
    logger.info("Initialising control service: drive=%s steer=%s", drive_nodes, steer_nodes)
    config = ControllerConfig(drive_nodes=drive_nodes, steer_nodes=steer_nodes)
    return AmrControlService(config, status_hook=logger.info)


def create_broadcaster(service: AmrControlService) -> StatusBroadcaster:
    interval = float(os.getenv("AMR_STATUS_INTERVAL", "0.5"))
    return StatusBroadcaster(service, interval=interval)


async def get_control_service() -> AmrControlService:
    if _control_service is None:
        raise HTTPException(status_code=503, detail="Control service not ready")
    return _control_service


async def get_broadcaster() -> StatusBroadcaster:
    if _broadcaster is None:
        raise HTTPException(status_code=503, detail="Broadcaster not ready")
    return _broadcaster


@app.get("/", response_class=HTMLResponse, tags=["ui"])
async def ui_root() -> HTMLResponse:
    index_file = STATIC_DIR / "index.html"
    if not index_file.exists():
        raise HTTPException(status_code=404, detail="UI not available")
    return HTMLResponse(index_file.read_text(encoding="utf-8"))


# --- Request models ----------------------------------------------------------


class SpeedRequest(BaseModel):
    value: float = Field(..., description="Target drive speed in m/s")


class AccelRequest(BaseModel):
    delta: int = Field(..., description="Acceleration delta in pps²")


class AngleRequest(BaseModel):
    value: float = Field(..., description="Target steering angle in degrees")


class SpeedDeltaRequest(BaseModel):
    delta: float = Field(..., description="Drive speed delta in m/s")


class AngleDeltaRequest(BaseModel):
    delta: float = Field(..., description="Steering angle delta in degrees")


class VelocityDeltaRequest(BaseModel):
    delta: float = Field(..., description="Steering velocity delta in deg/s")


class AccelDeltaRequest(BaseModel):
    delta: float = Field(..., description="Steering acceleration delta in deg/s²")


class OffsetRequest(BaseModel):
    node_id: int = Field(..., description="CAN node identifier for steer wheel")
    delta: float = Field(..., description="Steering offset delta in degrees")


# --- REST endpoints ----------------------------------------------------------


@app.get("/status", tags=["status"], dependencies=[Depends(require_token)])
async def get_status(service: AmrControlService = Depends(get_control_service)) -> Dict[str, Dict[str, object]]:
    return await run_in_threadpool(service.combined_status)


@app.post("/drive/speed", tags=["drive"], dependencies=[Depends(require_token)])
async def set_drive_speed(
    request: SpeedRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.set_drive_speed, request.value)
    return {"status": "ok"}


@app.post("/drive/accel", tags=["drive"], dependencies=[Depends(require_token)])
async def adjust_drive_accel(
    request: AccelRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_drive_accel, request.delta)
    return {"status": "ok"}


@app.post("/drive/speed/adjust", tags=["drive"], dependencies=[Depends(require_token)])
async def adjust_drive_speed(
    request: SpeedDeltaRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_drive_speed, request.delta)
    return {"status": "ok"}


@app.post("/drive/emergency_stop", tags=["drive"], dependencies=[Depends(require_token)])
async def drive_emergency_stop(service: AmrControlService = Depends(get_control_service)) -> Dict[str, str]:
    await run_in_threadpool(service.set_drive_speed, 0.0)
    return {"status": "ok"}


@app.post("/steer/angle", tags=["steer"], dependencies=[Depends(require_token)])
async def set_steer_angle(
    request: AngleRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.set_steer_angle, request.value)
    return {"status": "ok"}


@app.post("/steer/angle/adjust", tags=["steer"], dependencies=[Depends(require_token)])
async def adjust_steer_angle(
    request: AngleDeltaRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_steer_angle, request.delta)
    return {"status": "ok"}


@app.post("/steer/velocity", tags=["steer"], dependencies=[Depends(require_token)])
async def adjust_steer_velocity(
    request: VelocityDeltaRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_steer_velocity, request.delta)
    return {"status": "ok"}


@app.post("/steer/accel", tags=["steer"], dependencies=[Depends(require_token)])
async def adjust_steer_accel(
    request: AccelDeltaRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_steer_accel, request.delta)
    return {"status": "ok"}


@app.post("/steer/offset", tags=["steer"], dependencies=[Depends(require_token)])
async def adjust_steer_offset(
    request: OffsetRequest, service: AmrControlService = Depends(get_control_service)
) -> Dict[str, str]:
    await run_in_threadpool(service.adjust_steer_offset, request.node_id, request.delta)
    return {"status": "ok"}


@app.post("/steer/save_offsets", tags=["steer"], dependencies=[Depends(require_token)])
async def save_offsets(service: AmrControlService = Depends(get_control_service)) -> Dict[str, str]:
    await run_in_threadpool(service.save_offsets)
    return {"status": "ok"}


@app.post("/steer/load_offsets", tags=["steer"], dependencies=[Depends(require_token)])
async def load_offsets(service: AmrControlService = Depends(get_control_service)) -> Dict[str, str]:
    await run_in_threadpool(service.load_offsets)
    return {"status": "ok"}


# --- WebSocket endpoints -----------------------------------------------------


@app.websocket("/ws/status")
async def status_stream(
    websocket: WebSocket,
    broadcaster: StatusBroadcaster = Depends(get_broadcaster),
) -> None:
    if not await ensure_ws_authorized(websocket):
        return
    await websocket.accept()
    await broadcaster.register(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        await broadcaster.unregister(websocket)


# --- Factory -----------------------------------------------------------------


def create_app() -> FastAPI:
    """Return configured FastAPI application (handy for uvicorn)."""
    return app


__all__ = [
    "app",
    "create_app",
    "StatusBroadcaster",
    "create_control_service",
    "create_broadcaster",
]
