"""Shared CAN-bus utilities used by drive and steer controllers."""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Callable, Dict, Iterable, Optional

import can
from can import BusABC

logger = logging.getLogger(__name__)

BusProvider = Callable[[str], BusABC]


BUS_POOL: Dict[str, BusABC] = {}
BUS_LOCKS: Dict[str, threading.Lock] = {}
BUS_POOL_LOCK = threading.Lock()


def _default_bus_provider(channel: str) -> BusABC:
    """Factory used to construct new CAN bus connections."""
    return can.interface.Bus(channel=channel, bustype="socketcan")


_bus_provider: BusProvider = _default_bus_provider


def set_bus_provider(provider: BusProvider) -> None:
    """Configure a custom bus factory (handy for tests)."""
    global _bus_provider
    _bus_provider = provider


def reset_bus_provider() -> None:
    """Restore the default bus factory."""
    global _bus_provider
    _bus_provider = _default_bus_provider


def ensure_buses(channels: Iterable[str]) -> None:
    """Initialise CAN buses for the provided channel names."""
    with BUS_POOL_LOCK:
        for name in channels:
            if name in BUS_POOL:
                continue
            bus = _bus_provider(name)
            BUS_POOL[name] = bus
            BUS_LOCKS[name] = threading.Lock()
            logger.debug("Initialised CAN bus %s", name)


def shutdown_buses() -> None:
    """Shutdown all active CAN buses and clear state."""
    with BUS_POOL_LOCK:
        for name, bus in list(BUS_POOL.items()):
            try:
                bus.shutdown()
                logger.debug("Shutdown CAN bus %s", name)
            except Exception:  # pragma: no cover - defensive
                logger.exception("Failed to shutdown CAN bus %s", name)
        BUS_POOL.clear()
        BUS_LOCKS.clear()


def _require_bus(bus_name: str) -> BusABC:
    if bus_name not in BUS_POOL:
        raise RuntimeError(f"Bus '{bus_name}' not initialised.")
    return BUS_POOL[bus_name]


def send_sdo_write(
    bus_name: str,
    node_id: int,
    index: int,
    sub_index: int,
    value: int,
    size: int,
    sleep: float = 0.01,
) -> None:
    """Send a single SDO write operation."""
    if size == 1:
        cmd, fmt = 0x2F, "<b"
    elif size == 2:
        cmd, fmt = 0x2B, "<H"
    elif size == 4:
        cmd, fmt = 0x23, "<i"
    else:
        raise ValueError("Unsupported SDO size (must be 1/2/4).")

    cob_id = 0x600 + node_id
    index_bytes = struct.pack("<H", index)
    data_bytes = struct.pack(fmt, value)
    payload = [cmd, index_bytes[0], index_bytes[1], sub_index] + list(data_bytes)
    payload.extend([0] * (8 - len(payload)))

    bus = _require_bus(bus_name)
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)

    lock = BUS_LOCKS[bus_name]
    with lock:
        bus.send(msg)
        time.sleep(sleep)


def send_sdo_read(
    bus_name: str,
    node_id: int,
    index: int,
    sub_index: int,
    timeout: float,
) -> int:
    """Perform a blocking SDO read and return the integer payload."""
    cob_id_req = 0x600 + node_id
    cob_id_resp = 0x580 + node_id
    payload = [0x40, index & 0xFF, index >> 8, sub_index, 0, 0, 0, 0]
    request = can.Message(arbitration_id=cob_id_req, data=payload, is_extended_id=False)

    bus = _require_bus(bus_name)
    lock = BUS_LOCKS[bus_name]
    with lock:
        bus.send(request)
        deadline = time.time() + timeout
        while time.time() < deadline:
            response = bus.recv(timeout=timeout)
            if response is None:
                continue
            if response.arbitration_id != cob_id_resp:
                continue
            cmd = response.data[0]
            if (cmd & 0xE0) != 0x40:
                raise RuntimeError(f"Unexpected SDO response 0x{cmd:02X}")
            expedited = bool(cmd & 0x02)
            if not expedited:
                raise RuntimeError("Segmented SDO transfers are not supported.")
            if cmd & 0x01:
                unused = (cmd >> 2) & 0x03
                data_len = 4 - unused
            else:
                data_len = 4
            raw = bytes(response.data[4:4 + data_len]) + b"\x00" * (4 - data_len)
            return struct.unpack("<i", raw)[0]
    raise TimeoutError(f"SDO read timeout 0x{index:04X}:{sub_index} on node {node_id}")


__all__ = [
    "BUS_POOL",
    "BUS_LOCKS",
    "BUS_POOL_LOCK",
    "ensure_buses",
    "shutdown_buses",
    "send_sdo_write",
    "send_sdo_read",
    "set_bus_provider",
    "reset_bus_provider",
]
