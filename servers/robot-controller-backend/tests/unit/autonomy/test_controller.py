from __future__ import annotations

import pathlib
import sys

import asyncio
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from autonomy import AutonomyController, AutonomyError, ModeRegistry, build_default_controller
from autonomy.base import AutonomyModeHandler


class DummyMode(AutonomyModeHandler):
    def __init__(self) -> None:
        super().__init__("dummy")
        self.started_with = {}
        self.updated_with = {}
        self.stopped = False

    async def start(self, params):
        self.started_with = dict(params)

    async def update(self, params):
        self.updated_with.update(dict(params))

    async def stop(self):
        self.stopped = True


def test_start_and_stop_cycle():
    registry = ModeRegistry()
    registry.register("dummy", lambda **_: DummyMode())
    controller = AutonomyController(registry)

    async def scenario():
        status = await controller.start("dummy", {"speed": 42})
        assert status["active"] is True
        assert status["mode"] == "dummy"
        assert status["params"]["speed"] == 42

        status = await controller.update({"speed": 55})
        assert status["params"]["speed"] == 55

        status = await controller.stop()
        assert status["active"] is False

        with pytest.raises(AutonomyError):
            await controller.update({"speed": 99})

    asyncio.run(scenario())


def test_dock_falls_back_to_dock_mode():
    controller = build_default_controller()

    async def scenario():
        # Start patrol first to exercise stop/start sequencing
        await controller.start("patrol", {"loops": 2})
        status = await controller.dock()
        assert status["mode"] == "dock"
        assert status["active"] is True

    asyncio.run(scenario())


def test_waypoint_updates_tracked():
    controller = build_default_controller()

    async def scenario():
        await controller.start("waypoints", {"existing": 1})
        status = await controller.set_waypoint("Home", 12.34, -45.6)
        waypoint = status["params"]["lastWaypoint"]
        assert waypoint["label"] == "Home"
        assert waypoint["lat"] == pytest.approx(12.34)
        assert waypoint["lon"] == pytest.approx(-45.6)

    asyncio.run(scenario())


def test_invalid_mode_raises():
    controller = build_default_controller()

    async def scenario():
        with pytest.raises(AutonomyError):
            await controller.start("", {})

    asyncio.run(scenario())
