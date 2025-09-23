"""Orchestration layer that coordinates drive and steer controllers."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from .can_bus import ensure_buses, shutdown_buses
from .controllers import (
    DRIVE_ACCEL_STEP_PPS2,
    DRIVE_NODE_CONFIG,
    DRIVE_SPEED_STEP_MPS,
    DriveController,
    STEER_ACCEL_STEP_DPS2,
    STEER_DEG_STEP,
    STEER_NODE_CONFIG,
    STEER_OFFSET_STEP_DEG,
    STEER_VEL_STEP_DPS,
    SteerController,
)

logger = logging.getLogger(__name__)

StatusHook = Callable[[str], None]


@dataclass
class ControllerConfig:
    drive_nodes: List[int]
    steer_nodes: List[int]


class AmrControlService:
    """Facade that exposes high-level drive/steer control methods."""

    def __init__(
        self,
        config: ControllerConfig,
        status_hook: Optional[StatusHook] = None,
    ) -> None:
        self.config = config
        self._status_hook = status_hook or (lambda msg: logger.info(msg))
        self._drive: Optional[DriveController] = None
        self._steer: Optional[SteerController] = None
        self._lock = threading.RLock()

    # --- Lifecycle ---------------------------------------------------------

    def start(self) -> None:
        """Initialise CAN buses and set up the configured controllers."""
        with self._lock:
            if self._drive or self._steer:
                return
            bus_names = set()
            for node in self.config.drive_nodes:
                bus_names.add(DRIVE_NODE_CONFIG[node])
            for node in self.config.steer_nodes:
                bus_names.add(STEER_NODE_CONFIG[node])
            ensure_buses(bus_names)

            if self.config.drive_nodes:
                self._drive = DriveController(
                    self.config.drive_nodes, status_callback=self._status_hook
                )
                self._drive.setup()

            if self.config.steer_nodes:
                self._steer = SteerController(
                    self.config.steer_nodes, status_callback=self._status_hook
                )
                self._steer.setup()

    def shutdown(self) -> None:
        with self._lock:
            try:
                if self._drive:
                    self._drive.emergency_stop()
            finally:
                shutdown_buses()
                self._drive = None
                self._steer = None

    # --- Drive controls ----------------------------------------------------

    def has_drive(self) -> bool:
        with self._lock:
            return self._drive is not None

    def set_drive_speed(self, value_mps: float) -> None:
        with self._lock:
            if not self._drive:
                raise RuntimeError("Drive controller not initialised")
            self._drive.set_speed(value_mps)

    def adjust_drive_speed(self, delta_mps: float) -> None:
        with self._lock:
            if not self._drive:
                raise RuntimeError("Drive controller not initialised")
            self._drive.adjust_speed(delta_mps)

    def adjust_drive_accel(self, delta_pps2: int) -> None:
        with self._lock:
            if not self._drive:
                raise RuntimeError("Drive controller not initialised")
            self._drive.adjust_accel(delta_pps2)

    def drive_status(self) -> Dict[str, object]:
        with self._lock:
            if not self._drive:
                return {}
            return self._drive.fetch_status()

    def print_drive_actuals(self) -> None:
        with self._lock:
            if not self._drive:
                raise RuntimeError("Drive controller not initialised")
            self._drive.print_actual_state()

    # --- Steer controls ----------------------------------------------------

    def has_steer(self) -> bool:
        with self._lock:
            return self._steer is not None

    def steer_nodes(self) -> List[int]:
        with self._lock:
            if not self._steer:
                return []
            return list(self._steer.nodes)

    def set_steer_angle(self, value_deg: float) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.set_position(value_deg)

    def adjust_steer_angle(self, delta_deg: float) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.adjust_position(delta_deg)

    def adjust_steer_velocity(self, delta_dps: float) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.adjust_velocity(delta_dps)

    def adjust_steer_accel(self, delta_dps2: float) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.adjust_accel(delta_dps2)

    def adjust_steer_offset(self, node_id: int, delta_deg: float) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.adjust_offset(node_id, delta_deg)

    def save_offsets(self) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.save_offsets()

    def load_offsets(self) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.load_offsets()
            self._steer._push_position()
            self._steer._trigger_motion()

    def print_steer_offsets(self) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.print_offsets()

    def print_steer_actuals(self) -> None:
        with self._lock:
            if not self._steer:
                raise RuntimeError("Steer controller not initialised")
            self._steer.print_actual_position()

    def steer_status(self) -> Dict[str, object]:
        with self._lock:
            if not self._steer:
                return {}
            return self._steer.fetch_status()

    # --- Combined ----------------------------------------------------------

    def combined_status(self) -> Dict[str, Dict[str, object]]:
        with self._lock:
            drive_status = self._drive.fetch_status() if self._drive else {}
            steer_status = self._steer.fetch_status() if self._steer else {}
        steps = {
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
        return {
            "drive": drive_status,
            "steer": steer_status,
            "steps": steps,
        }


__all__ = ["ControllerConfig", "AmrControlService"]
