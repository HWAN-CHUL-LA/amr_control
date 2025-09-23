"""FastAPI application exposing AMR control interfaces."""

from importlib import import_module
from types import ModuleType
from typing import Any

__all__ = ["app", "create_app"]


def __getattr__(name: str) -> Any:
    if name in {"app", "create_app"}:
        module: ModuleType = import_module("server.app")
        return getattr(module, name)
    raise AttributeError(name)
