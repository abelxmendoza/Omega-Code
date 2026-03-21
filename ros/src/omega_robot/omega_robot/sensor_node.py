#!/usr/bin/env python3
"""
Omega Sensor Node
==================
Publishes all physical sensors to ROS topics alongside the existing
WebSocket servers.  The WebSocket servers continue to run unchanged --
this node is additive.

Published topics
----------------
  /omega/ultrasonic          sensor_msgs/Range      (BEST_EFFORT, ~10 Hz)
  /omega/line_tracking/left  std_msgs/Bool          (BEST_EFFORT, ~20 Hz)
  /omega/line_tracking/center std_msgs/Bool         (BEST_EFFORT, ~20 Hz)
  /omega/line_tracking/right std_msgs/Bool          (BEST_EFFORT, ~20 Hz)
  /omega/line_tracking/state std_msgs/String (JSON) (BEST_EFFORT, ~20 Hz)
  /omega/battery             sensor_msgs/BatteryState (BEST_EFFORT, ~1 Hz)

Parameters
----------
  sim_mode            bool   False   -- return synthetic readings
  ultrasonic_rate_hz  float  10.0    -- ultrasonic publish frequency
  line_rate_hz        float  20.0    -- line tracking publish frequency
  battery_rate_hz     float  1.0     -- battery publish frequency
  ultrasonic_fov      float  0.2618  -- field of view in radians (HC-SR04 ~15 deg)
  ultrasonic_min_m    float  0.02    -- minimum valid range (2 cm)
  ultrasonic_max_m    float  4.00    -- maximum valid range (400 cm)
  ultrasonic_frame    str    'ultrasonic_front'
  pin_left            int    14      -- GPIO pin for left  line sensor
  pin_center          int    15      -- GPIO pin for center line sensor
  pin_right           int    23      -- GPIO pin for right  line sensor
  invert_sensors      bool   False   -- invert all line sensor readings
  ads1115_channel     int    0       -- ADS1115 ADC channel for battery voltage
  battery_cells       int    2       -- number of LiPo cells (affects full_charge)
  battery_cell_full_v float  4.2     -- full charge voltage per cell
  battery_cell_min_v  float  3.0     -- cutoff voltage per cell

Design notes
------------
Each sensor type runs in its own timer callback.  The ultrasonic read is
blocking (lgpio pulse timing) so its timer rate is kept <= sensor's safe
measurement rate (~10 Hz for HC-SR04).

The line tracking GPIO reads are non-blocking, so 20 Hz is safe.

Battery reads go through smbus2 (I2C) and include a 100 ms conversion
delay in the ADS1115 driver; keep the rate at 1 Hz or lower.
"""

from __future__ import annotations

import json
import time
import math
import logging
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from sensor_msgs.msg import Range, BatteryState
from std_msgs.msg import Bool, String

# ---------------------------------------------------------------------------
# Sensor hardware imports (available via: pip install -e /path/to/Omega-Code/)
# ---------------------------------------------------------------------------
_ultrasonic_available = False
try:
    from servers.robot_controller_backend.sensors.ultrasonic_sensor import Ultrasonic
    _ultrasonic_available = True
except ImportError as _e:
    logging.getLogger(__name__).warning('Ultrasonic driver unavailable: %s', _e)

_gpio_available = False
_lgpio = None
try:
    import lgpio as _lgpio  # type: ignore
    _gpio_available = True
except ImportError:
    logging.getLogger(__name__).warning('lgpio unavailable -- line tracking will be simulated')

_adc_available = False
try:
    from smbus2 import SMBus as _SMBus  # type: ignore
    _adc_available = True
except ImportError:
    logging.getLogger(__name__).warning('smbus2 unavailable -- battery readings will be simulated')


# ---------------------------------------------------------------------------
# QoS helper
# ---------------------------------------------------------------------------

def _sensor_qos(depth: int = 5) -> QoSProfile:
    """Best-effort, volatile -- appropriate for high-rate sensor data."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# ADS1115 minimal driver
# Inlined here to avoid importing the read_voltage.py script (which is a CLI
# script with direct console output, not a library).
# ---------------------------------------------------------------------------

class _ADS1115:
    """Minimal ADS1115 single-shot reader for one channel."""

    _CONFIG_REG     = 0x01
    _CONVERSION_REG = 0x00
    _MUX = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}
    _PGA  = 0x0000   # +/- 6.144 V
    _MODE = 0x0100   # single shot
    _DR   = 0x0080   # 128 SPS
    _COMP = 0x0003
    _V_PER_LSB = 6.144 / 32768.0

    def __init__(self, bus_num: int = 1, address: int = 0x48) -> None:
        self._bus     = _SMBus(bus_num)
        self._address = address

    def read_voltage(self, channel: int) -> float:
        """Return voltage on the given channel (0-3)."""
        config = 0x8000 | self._MUX[channel] | self._PGA | self._MODE | self._DR | self._COMP
        self._bus.write_i2c_block_data(
            self._address, self._CONFIG_REG,
            [(config >> 8) & 0xFF, config & 0xFF],
        )
        time.sleep(0.1)   # conversion time for 128 SPS
        data = self._bus.read_i2c_block_data(self._address, self._CONVERSION_REG, 2)
        raw  = (data[0] << 8) | data[1]
        if raw > 32767:
            raw -= 65536
        return round(raw * self._V_PER_LSB, 3)

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class SensorNode(Node):

    def __init__(self) -> None:
        super().__init__('omega_sensor_node')

        # ---- parameters -----------------------------------------------
        self.declare_parameter('sim_mode',           False)
        self.declare_parameter('ultrasonic_rate_hz', 10.0)
        self.declare_parameter('line_rate_hz',       20.0)
        self.declare_parameter('battery_rate_hz',     1.0)
        self.declare_parameter('ultrasonic_fov',     0.2618)   # rad (~15 deg)
        self.declare_parameter('ultrasonic_min_m',   0.02)
        self.declare_parameter('ultrasonic_max_m',   4.00)
        self.declare_parameter('ultrasonic_frame',   'ultrasonic_front')
        self.declare_parameter('pin_left',           14)
        self.declare_parameter('pin_center',         15)
        self.declare_parameter('pin_right',          23)
        self.declare_parameter('invert_sensors',     False)
        self.declare_parameter('ads1115_channel',    0)
        self.declare_parameter('battery_cells',      2)
        self.declare_parameter('battery_cell_full_v', 4.2)
        self.declare_parameter('battery_cell_min_v',  3.0)

        p = self.get_parameter
        self._sim: bool = p('sim_mode').value

        # ---- publishers -----------------------------------------------
        qos = _sensor_qos()
        self._us_pub  = self.create_publisher(Range,        '/omega/ultrasonic',            qos)
        self._lt_l_pub = self.create_publisher(Bool,        '/omega/line_tracking/left',    qos)
        self._lt_c_pub = self.create_publisher(Bool,        '/omega/line_tracking/center',  qos)
        self._lt_r_pub = self.create_publisher(Bool,        '/omega/line_tracking/right',   qos)
        self._lt_state_pub = self.create_publisher(String,  '/omega/line_tracking/state',   qos)
        self._bat_pub = self.create_publisher(BatteryState, '/omega/battery',               qos)

        # ---- hardware setup -------------------------------------------
        self._us_sensor: Optional[Ultrasonic] = None
        self._gpio_handle: Optional[int] = None
        self._adc: Optional[_ADS1115] = None
        self._setup_ultrasonic()
        self._setup_line_tracking()
        self._setup_battery()

        # ---- cached constants -----------------------------------------
        self._us_fov:  float = p('ultrasonic_fov').value
        self._us_min:  float = p('ultrasonic_min_m').value
        self._us_max:  float = p('ultrasonic_max_m').value
        self._us_frame: str  = p('ultrasonic_frame').value
        self._lt_pins  = {
            'left':   p('pin_left').value,
            'center': p('pin_center').value,
            'right':  p('pin_right').value,
        }
        self._invert: bool = p('invert_sensors').value
        self._bat_chan: int = p('ads1115_channel').value
        self._bat_cells: int = p('battery_cells').value
        self._bat_full_v: float = p('battery_cell_full_v').value * self._bat_cells
        self._bat_min_v:  float = p('battery_cell_min_v').value  * self._bat_cells

        # ---- timers ---------------------------------------------------
        us_period   = 1.0 / p('ultrasonic_rate_hz').value
        line_period = 1.0 / p('line_rate_hz').value
        bat_period  = 1.0 / p('battery_rate_hz').value

        self._us_timer   = self.create_timer(us_period,   self._publish_ultrasonic)
        self._line_timer = self.create_timer(line_period, self._publish_line_tracking)
        self._bat_timer  = self.create_timer(bat_period,  self._publish_battery)

        self.get_logger().info(
            f'SensorNode ready [sim={self._sim} '
            f'us={p("ultrasonic_rate_hz").value:.1f}Hz '
            f'line={p("line_rate_hz").value:.1f}Hz '
            f'bat={p("battery_rate_hz").value:.1f}Hz]'
        )

    # ------------------------------------------------------------------
    # Hardware setup
    # ------------------------------------------------------------------

    def _setup_ultrasonic(self) -> None:
        if self._sim or not _ultrasonic_available:
            return
        try:
            self._us_sensor = Ultrasonic()
            self.get_logger().info('Ultrasonic sensor initialised (TRIG=27, ECHO=22)')
        except Exception as exc:
            self.get_logger().error(f'Ultrasonic init failed: {exc} -- using sim')
            self._us_sensor = None

    def _setup_line_tracking(self) -> None:
        if self._sim or not _gpio_available:
            return
        try:
            self._gpio_handle = _lgpio.gpiochip_open(0)
            pins = [
                self.get_parameter('pin_left').value,
                self.get_parameter('pin_center').value,
                self.get_parameter('pin_right').value,
            ]
            for pin in pins:
                _lgpio.gpio_claim_input(self._gpio_handle, pin)
            self.get_logger().info(f'Line tracking GPIO initialised (pins {pins})')
        except Exception as exc:
            self.get_logger().error(f'Line tracking GPIO init failed: {exc} -- using sim')
            self._gpio_handle = None

    def _setup_battery(self) -> None:
        if self._sim or not _adc_available:
            return
        try:
            self._adc = _ADS1115()
            self.get_logger().info(
                f'ADS1115 ADC initialised (channel {self.get_parameter("ads1115_channel").value})'
            )
        except Exception as exc:
            self.get_logger().error(f'ADS1115 init failed: {exc} -- using sim')
            self._adc = None

    # ------------------------------------------------------------------
    # Ultrasonic -- sensor_msgs/Range
    # ------------------------------------------------------------------

    def _publish_ultrasonic(self) -> None:
        distance_m = self._read_ultrasonic_m()
        now = self.get_clock().now().to_msg()

        msg = Range()
        msg.header.stamp    = now
        msg.header.frame_id = self._us_frame
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = self._us_fov
        msg.min_range       = self._us_min
        msg.max_range       = self._us_max

        if distance_m is None or distance_m < 0:
            # Out-of-range sentinel: publish max_range + 1 (per REP 117)
            msg.range = float('inf')
        else:
            msg.range = float(distance_m)

        self._us_pub.publish(msg)

    def _read_ultrasonic_m(self) -> Optional[float]:
        """Return distance in metres, or None on error."""
        if self._us_sensor is not None:
            try:
                cm = self._us_sensor.get_distance()   # returns -1 on timeout
                if cm == -1:
                    return None
                return cm / 100.0
            except Exception as exc:
                self.get_logger().debug(f'Ultrasonic read error: {exc}')
                return None

        # Simulation: sine-wave oscillating between 0.3 m and 3.0 m
        t = time.monotonic()
        return 0.3 + 1.35 * (1.0 + math.sin(t * 0.5))

    # ------------------------------------------------------------------
    # Line tracking -- std_msgs/Bool + state JSON
    # ------------------------------------------------------------------

    def _publish_line_tracking(self) -> None:
        readings = self._read_line_sensors()
        now = self.get_clock().now().to_msg()

        for pub, key in (
            (self._lt_l_pub, 'left'),
            (self._lt_c_pub, 'center'),
            (self._lt_r_pub, 'right'),
        ):
            msg = Bool()
            msg.data = bool(readings[key])
            pub.publish(msg)

        state_msg = String()
        state_msg.data = json.dumps({
            'left':   readings['left'],
            'center': readings['center'],
            'right':  readings['right'],
            'on_line': readings['center'],          # primary guidance signal
            'ts_ms':  int(time.time() * 1000),
        })
        self._lt_state_pub.publish(state_msg)

    def _read_line_sensors(self) -> dict[str, bool]:
        """Return dict with bool values for each sensor channel."""
        if self._gpio_handle is not None:
            raw = {}
            for name, pin in self._lt_pins.items():
                try:
                    val = bool(_lgpio.gpio_read(self._gpio_handle, pin))
                    raw[name] = (not val) if self._invert else val
                except Exception:
                    raw[name] = False
            return raw

        # Simulation: center sensor detects line with a slow blink
        t = time.monotonic()
        on = (int(t * 2) % 4) < 2
        return {'left': False, 'center': on, 'right': False}

    # ------------------------------------------------------------------
    # Battery -- sensor_msgs/BatteryState
    # ------------------------------------------------------------------

    def _publish_battery(self) -> None:
        voltage = self._read_battery_v()
        now = self.get_clock().now().to_msg()

        msg = BatteryState()
        msg.header.stamp    = now
        msg.header.frame_id = 'base_link'
        msg.voltage         = float(voltage)

        # Compute percentage from voltage range
        v_range = self._bat_full_v - self._bat_min_v
        if v_range > 0:
            pct = (voltage - self._bat_min_v) / v_range
            msg.percentage = float(max(0.0, min(1.0, pct)))
        else:
            msg.percentage = 0.0

        # Power supply status
        if msg.percentage > 0.95:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        elif msg.percentage > 0.05:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        msg.power_supply_health     = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present                 = True
        msg.cell_voltage            = [voltage / self._bat_cells] * self._bat_cells

        self._bat_pub.publish(msg)

    def _read_battery_v(self) -> float:
        """Return total battery voltage in Volts."""
        if self._adc is not None:
            try:
                return self._adc.read_voltage(self._bat_chan)
            except Exception as exc:
                self.get_logger().debug(f'Battery read error: {exc}')

        # Simulation: slowly discharging from 8.2V to 6.0V over 10 minutes
        t = time.monotonic()
        return max(6.0, 8.2 - (t / 600.0) * 2.2)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self.get_logger().info('SensorNode shutting down -- releasing GPIO/ADC')
        if self._gpio_handle is not None:
            try:
                _lgpio.gpiochip_close(self._gpio_handle)
            except Exception:
                pass
        if self._adc is not None:
            self._adc.close()
        if self._us_sensor is not None:
            try:
                self._us_sensor.close()
            except Exception:
                pass
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
