"""Unit tests covering the autonomy mode registry and controller edge-cases."""

from __future__ import annotations

import asyncio
import logging
from typing import Dict

import pytest

from autonomy import AutonomyController, AutonomyError
from autonomy.base import AutonomyModeHandler
from autonomy.controller import ModeRegistry


class DummyMode(AutonomyModeHandler):
    def __init__(self, name: str = "dummy", *, logger: logging.Logger | None = None) -> None:
        super().__init__(name, logger=logger)
        self.started_with: Dict[str, object] | None = None

    async def start(self, params):  # type: ignore[override]
        self.started_with = dict(params)


def test_register_requires_callable() -> None:
    registry = ModeRegistry()
    with pytest.raises(TypeError):
        registry.register("demo", object())  # type: ignore[arg-type]


def test_registry_resolves_aliases_and_context() -> None:
    registry = ModeRegistry()
    captured: Dict[str, object] = {}

    def factory(*, context, logger):
        mode = DummyMode("line_follow", logger=logger)
        captured.update(context)
        return mode

    registry.register("line_follow", factory)
    handler = registry.create("Line Track", context={"port": 9000}, logger=logging.getLogger("test"))

    assert isinstance(handler, DummyMode)
    assert handler.name == "line_follow"
    assert captured == {"port": 9000}


def test_registry_rejects_unknown_modes() -> None:
    registry = ModeRegistry()
    with pytest.raises(AutonomyError, match="unknown autonomy mode"):
        registry.create("missing", context={}, logger=logging.getLogger("test"))


def test_controller_start_unknown_mode_raises() -> None:
    async def scenario() -> None:
        controller = AutonomyController()
        with pytest.raises(AutonomyError, match="unknown autonomy mode"):
            await controller.start("missing", {})

    asyncio.run(scenario())


def test_controller_start_uses_registry_factory() -> None:
    async def scenario() -> None:
        registry = ModeRegistry()
        registry.register("demo", lambda **kw: DummyMode("demo", logger=kw.get("logger")))
        controller = AutonomyController(registry)
        status = await controller.start("demo", {"speed": 1})
        assert status["mode"] == "demo"
        assert status["params"]["speed"] == 1

    asyncio.run(scenario())
