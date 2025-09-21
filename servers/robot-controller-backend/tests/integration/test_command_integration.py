"""Integration tests exercising the autonomy controller workflow."""

from __future__ import annotations

import asyncio
from typing import Awaitable, Callable

import pytest

from autonomy import AutonomyController, AutonomyError, build_default_controller

ControllerTest = Callable[[AutonomyController], Awaitable[None]]


def run_controller_test(test: ControllerTest) -> None:
    """Helper that provisions a fresh controller for every test case."""

    async def _wrapper() -> None:
        controller = build_default_controller()
        try:
            await test(controller)
        finally:
            # Ensure we always stop the session even if the assertion fails so the
            # global state is clean for the next scenario.
            await controller.stop()

    asyncio.run(_wrapper())


def test_start_update_and_stop_cycle() -> None:
    async def scenario(controller: AutonomyController) -> None:
        status = await controller.start("patrol", {"speed": 1})
        assert status["active"] is True
        assert status["mode"] == "patrol"
        assert status["params"]["speed"] == 1

        status = await controller.update({"speed": 2})
        assert status["params"]["speed"] == 2

        status = await controller.stop()
        assert status["active"] is False
        assert status["mode"] == "idle"
        assert status["params"] == {}

    run_controller_test(scenario)


def test_dock_switches_modes_and_confirms() -> None:
    async def scenario(controller: AutonomyController) -> None:
        await controller.start("patrol", {})
        status = await controller.dock()
        assert status["active"] is True
        assert status["mode"] == "dock"
        assert status["handler"] == "DockMode"

        # Issuing dock again should hit the active handler rather than
        # instantiating a new one, keeping the mode stable.
        status = await controller.dock()
        assert status["mode"] == "dock"
        assert status["handler"] == "DockMode"

    run_controller_test(scenario)


def test_waypoint_updates_are_tracked() -> None:
    async def scenario(controller: AutonomyController) -> None:
        await controller.start("waypoints", {"speed": 0.25})
        status = await controller.set_waypoint("Home", 41.5, -71.2)
        waypoint = status["params"]["lastWaypoint"]
        assert waypoint["label"] == "Home"
        assert waypoint["lat"] == pytest.approx(41.5)
        assert waypoint["lon"] == pytest.approx(-71.2)
        assert waypoint["ts"] >= status["startedAt"]

    run_controller_test(scenario)


def test_alias_names_are_normalised() -> None:
    async def scenario(controller: AutonomyController) -> None:
        status = await controller.start("Line Track", {})
        assert status["mode"] == "line_follow"

    run_controller_test(scenario)


def test_invalid_parameter_payloads_raise_errors() -> None:
    async def scenario(controller: AutonomyController) -> None:
        await controller.start("patrol", {})
        with pytest.raises(AutonomyError, match="params must be a mapping"):
            await controller.update(["not", "a", "mapping"])  # type: ignore[list-item]

    run_controller_test(scenario)


def test_waypoint_requires_active_session() -> None:
    async def scenario(controller: AutonomyController) -> None:
        with pytest.raises(AutonomyError, match="no autonomy mode is active"):
            await controller.set_waypoint("Home", 1.0, 2.0)

    run_controller_test(scenario)
