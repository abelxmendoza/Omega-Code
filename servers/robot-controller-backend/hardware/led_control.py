"""
LED Strip Control Module
Supports WS2812B (NeoPixel), APA102, and basic RGB LED strips
"""

import asyncio
import logging
import time
import colorsys
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import math

logger = logging.getLogger(__name__)

class LEDStripType(Enum):
    WS2812B = "ws2812b"  # NeoPixel
    APA102 = "apa102"    # DotStar
    RGB_STRIP = "rgb_strip"  # Basic RGB strip

class LEDPattern(Enum):
    STATIC = "static"
    PULSE = "pulse"
    BLINK = "blink"
    RAINBOW = "rainbow"
    CHASE = "chase"
    MUSIC = "music"
    BREATHING = "breathing"

@dataclass
class LEDStripConfig:
    """Configuration for LED strip"""
    strip_type: LEDStripType
    pin: int
    num_leds: int = 50
    brightness: int = 50  # 0-100
    frequency: int = 800000  # Hz for WS2812B
    order: str = "RGB"  # Color order: RGB, GRB, BGR, etc.

@dataclass
class LEDColor:
    """LED color representation"""
    r: int  # 0-255
    g: int  # 0-255
    b: int  # 0-255
    w: int = 0  # White channel for RGBW strips

class LEDStripDriver:
    """Base class for LED strip drivers"""
    
    def __init__(self, strip_id: str, config: LEDStripConfig):
        self.strip_id = strip_id
        self.config = config
        self.initialized = False
        self.current_pattern = LEDPattern.STATIC
        self.current_color = LEDColor(255, 0, 0)  # Red
        self.pattern_task: Optional[asyncio.Task] = None
        self.is_pattern_running = False
    
    async def initialize(self) -> bool:
        """Initialize LED strip driver"""
        raise NotImplementedError
    
    async def cleanup(self) -> None:
        """Cleanup LED strip resources"""
        raise NotImplementedError
    
    async def set_color(self, color: LEDColor) -> bool:
        """Set LED strip color"""
        raise NotImplementedError
    
    async def set_brightness(self, brightness: int) -> bool:
        """Set LED strip brightness (0-100)"""
        raise NotImplementedError
    
    async def set_pattern(self, pattern: LEDPattern, color: LEDColor = None) -> bool:
        """Set LED strip pattern"""
        raise NotImplementedError
    
    async def stop_pattern(self) -> bool:
        """Stop current pattern"""
        raise NotImplementedError
    
    def _hsv_to_rgb(self, h: float, s: float, v: float) -> LEDColor:
        """Convert HSV to RGB"""
        r, g, b = colorsys.hsv_to_rgb(h, s, v)
        return LEDColor(int(r * 255), int(g * 255), int(b * 255))

class WS2812BDriver(LEDStripDriver):
    """WS2812B (NeoPixel) LED strip driver"""
    
    def __init__(self, strip_id: str, config: LEDStripConfig):
        super().__init__(strip_id, config)
        self.strip = None
        
        # Import neopixel library
        try:
            import neopixel
            import board
            self.neopixel = neopixel
            self.board = board
        except ImportError:
            logger.error("neopixel library not available for WS2812B")
            raise
    
    async def initialize(self) -> bool:
        """Initialize WS2812B LED strip"""
        try:
            # Get pin object
            pin = getattr(self.board, f"D{self.config.pin}")
            
            # Create NeoPixel object
            self.strip = self.neopixel.NeoPixel(
                pin,
                self.config.num_leds,
                brightness=self.config.brightness / 100.0,
                auto_write=False,
                pixel_order=getattr(self.neopixel, self.config.order)
            )
            
            # Clear strip
            self.strip.fill((0, 0, 0))
            self.strip.show()
            
            self.initialized = True
            logger.info(f"WS2812B LED strip {self.strip_id} initialized with {self.config.num_leds} LEDs")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize WS2812B LED strip {self.strip_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup WS2812B LED strip resources"""
        try:
            if self.strip:
                # Turn off all LEDs
                self.strip.fill((0, 0, 0))
                self.strip.show()
            
            self.initialized = False
            logger.info(f"WS2812B LED strip {self.strip_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up WS2812B LED strip {self.strip_id}: {e}")
    
    async def set_color(self, color: LEDColor) -> bool:
        """Set WS2812B LED strip color"""
        if not self.initialized:
            return False
        
        try:
            # Apply brightness
            brightness = self.config.brightness / 100.0
            r = int(color.r * brightness)
            g = int(color.g * brightness)
            b = int(color.b * brightness)
            
            # Set all LEDs to the same color
            self.strip.fill((r, g, b))
            self.strip.show()
            
            self.current_color = color
            logger.info(f"WS2812B LED strip {self.strip_id} color set to RGB({r}, {g}, {b})")
            return True
            
        except Exception as e:
            logger.error(f"Error setting WS2812B LED strip color: {e}")
            return False
    
    async def set_brightness(self, brightness: int) -> bool:
        """Set WS2812B LED strip brightness"""
        if not self.initialized:
            return False
        
        try:
            brightness = max(0, min(100, brightness))
            self.config.brightness = brightness
            
            # Update strip brightness
            self.strip.brightness = brightness / 100.0
            self.strip.show()
            
            logger.info(f"WS2812B LED strip {self.strip_id} brightness set to {brightness}%")
            return True
            
        except Exception as e:
            logger.error(f"Error setting WS2812B LED strip brightness: {e}")
            return False
    
    async def set_pattern(self, pattern: LEDPattern, color: LEDColor = None) -> bool:
        """Set WS2812B LED strip pattern"""
        if not self.initialized:
            return False
        
        try:
            # Stop current pattern
            await self.stop_pattern()
            
            self.current_pattern = pattern
            if color:
                self.current_color = color
            
            # Start pattern task
            self.is_pattern_running = True
            self.pattern_task = asyncio.create_task(self._run_pattern())
            
            logger.info(f"WS2812B LED strip {self.strip_id} pattern set to {pattern.value}")
            return True
            
        except Exception as e:
            logger.error(f"Error setting WS2812B LED strip pattern: {e}")
            return False
    
    async def stop_pattern(self) -> bool:
        """Stop WS2812B LED strip pattern"""
        try:
            self.is_pattern_running = False
            
            if self.pattern_task:
                self.pattern_task.cancel()
                try:
                    await self.pattern_task
                except asyncio.CancelledError:
                    pass
            
            # Set to static color
            await self.set_color(self.current_color)
            
            logger.info(f"WS2812B LED strip {self.strip_id} pattern stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping WS2812B LED strip pattern: {e}")
            return False
    
    async def _run_pattern(self) -> None:
        """Run LED pattern"""
        while self.is_pattern_running and self.initialized:
            try:
                if self.current_pattern == LEDPattern.PULSE:
                    await self._pulse_pattern()
                elif self.current_pattern == LEDPattern.BLINK:
                    await self._blink_pattern()
                elif self.current_pattern == LEDPattern.RAINBOW:
                    await self._rainbow_pattern()
                elif self.current_pattern == LEDPattern.CHASE:
                    await self._chase_pattern()
                elif self.current_pattern == LEDPattern.BREATHING:
                    await self._breathing_pattern()
                else:
                    await self.set_color(self.current_color)
                    await asyncio.sleep(1.0)
                
            except Exception as e:
                logger.error(f"Error in LED pattern: {e}")
                await asyncio.sleep(0.1)
    
    async def _pulse_pattern(self) -> None:
        """Pulse pattern - fade in and out"""
        for brightness in range(0, 101, 5):
            if not self.is_pattern_running:
                break
            
            temp_brightness = self.config.brightness
            self.config.brightness = brightness
            await self.set_color(self.current_color)
            self.config.brightness = temp_brightness
            
            # Apply brightness manually
            brightness_factor = brightness / 100.0
            r = int(self.current_color.r * brightness_factor)
            g = int(self.current_color.g * brightness_factor)
            b = int(self.current_color.b * brightness_factor)
            
            self.strip.fill((r, g, b))
            self.strip.show()
            
            await asyncio.sleep(0.05)
        
        for brightness in range(100, -1, -5):
            if not self.is_pattern_running:
                break
            
            temp_brightness = self.config.brightness
            self.config.brightness = brightness
            await self.set_color(self.current_color)
            self.config.brightness = temp_brightness
            
            # Apply brightness manually
            brightness_factor = brightness / 100.0
            r = int(self.current_color.r * brightness_factor)
            g = int(self.current_color.g * brightness_factor)
            b = int(self.current_color.b * brightness_factor)
            
            self.strip.fill((r, g, b))
            self.strip.show()
            
            await asyncio.sleep(0.05)
    
    async def _blink_pattern(self) -> None:
        """Blink pattern - on/off"""
        # On
        await self.set_color(self.current_color)
        await asyncio.sleep(0.5)
        
        if not self.is_pattern_running:
            return
        
        # Off
        self.strip.fill((0, 0, 0))
        self.strip.show()
        await asyncio.sleep(0.5)
    
    async def _rainbow_pattern(self) -> None:
        """Rainbow pattern - cycle through colors"""
        for i in range(self.config.num_leds):
            if not self.is_pattern_running:
                break
            
            # Calculate hue for each LED
            hue = (i / self.config.num_leds) % 1.0
            color = self._hsv_to_rgb(hue, 1.0, 1.0)
            
            # Apply brightness
            brightness_factor = self.config.brightness / 100.0
            r = int(color.r * brightness_factor)
            g = int(color.g * brightness_factor)
            b = int(color.b * brightness_factor)
            
            self.strip[i] = (r, g, b)
        
        self.strip.show()
        await asyncio.sleep(0.1)
    
    async def _chase_pattern(self) -> None:
        """Chase pattern - moving light"""
        for pos in range(self.config.num_leds):
            if not self.is_pattern_running:
                break
            
            # Clear strip
            self.strip.fill((0, 0, 0))
            
            # Set current position
            brightness_factor = self.config.brightness / 100.0
            r = int(self.current_color.r * brightness_factor)
            g = int(self.current_color.g * brightness_factor)
            b = int(self.current_color.b * brightness_factor)
            
            self.strip[pos] = (r, g, b)
            
            # Add trail effect
            for i in range(1, 4):
                trail_pos = (pos - i) % self.config.num_leds
                trail_brightness = brightness_factor * (0.5 ** i)
                trail_r = int(self.current_color.r * trail_brightness)
                trail_g = int(self.current_color.g * trail_brightness)
                trail_b = int(self.current_color.b * trail_brightness)
                self.strip[trail_pos] = (trail_r, trail_g, trail_b)
            
            self.strip.show()
            await asyncio.sleep(0.1)
    
    async def _breathing_pattern(self) -> None:
        """Breathing pattern - smooth fade in/out"""
        while self.is_pattern_running:
            # Fade in
            for brightness in range(0, 101, 2):
                if not self.is_pattern_running:
                    break
                
                brightness_factor = brightness / 100.0
                r = int(self.current_color.r * brightness_factor)
                g = int(self.current_color.g * brightness_factor)
                b = int(self.current_color.b * brightness_factor)
                
                self.strip.fill((r, g, b))
                self.strip.show()
                await asyncio.sleep(0.02)
            
            # Fade out
            for brightness in range(100, -1, -2):
                if not self.is_pattern_running:
                    break
                
                brightness_factor = brightness / 100.0
                r = int(self.current_color.r * brightness_factor)
                g = int(self.current_color.g * brightness_factor)
                b = int(self.current_color.b * brightness_factor)
                
                self.strip.fill((r, g, b))
                self.strip.show()
                await asyncio.sleep(0.02)

class RGBStripDriver(LEDStripDriver):
    """Basic RGB LED strip driver using PWM"""
    
    def __init__(self, strip_id: str, config: LEDStripConfig):
        super().__init__(strip_id, config)
        self.red_pwm = None
        self.green_pwm = None
        self.blue_pwm = None
        self.gpio = None
        
        # Import GPIO library
        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO
        except ImportError:
            logger.error("RPi.GPIO not available for RGB strip")
            raise
    
    async def initialize(self) -> bool:
        """Initialize RGB LED strip"""
        try:
            self.gpio.setmode(self.gpio.BCM)
            self.gpio.setwarnings(False)
            
            # Setup PWM pins (assuming 3 pins for R, G, B)
            red_pin = self.config.pin
            green_pin = self.config.pin + 1
            blue_pin = self.config.pin + 2
            
            self.gpio.setup(red_pin, self.gpio.OUT)
            self.gpio.setup(green_pin, self.gpio.OUT)
            self.gpio.setup(blue_pin, self.gpio.OUT)
            
            # Setup PWM
            self.red_pwm = self.gpio.PWM(red_pin, 1000)
            self.green_pwm = self.gpio.PWM(green_pin, 1000)
            self.blue_pwm = self.gpio.PWM(blue_pin, 1000)
            
            self.red_pwm.start(0)
            self.green_pwm.start(0)
            self.blue_pwm.start(0)
            
            self.initialized = True
            logger.info(f"RGB LED strip {self.strip_id} initialized")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize RGB LED strip {self.strip_id}: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup RGB LED strip resources"""
        try:
            if self.red_pwm:
                self.red_pwm.stop()
            if self.green_pwm:
                self.green_pwm.stop()
            if self.blue_pwm:
                self.blue_pwm.stop()
            
            self.initialized = False
            logger.info(f"RGB LED strip {self.strip_id} cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up RGB LED strip {self.strip_id}: {e}")
    
    async def set_color(self, color: LEDColor) -> bool:
        """Set RGB LED strip color"""
        if not self.initialized:
            return False
        
        try:
            # Apply brightness
            brightness_factor = self.config.brightness / 100.0
            red_duty = (color.r / 255.0) * 100 * brightness_factor
            green_duty = (color.g / 255.0) * 100 * brightness_factor
            blue_duty = (color.b / 255.0) * 100 * brightness_factor
            
            self.red_pwm.ChangeDutyCycle(red_duty)
            self.green_pwm.ChangeDutyCycle(green_duty)
            self.blue_pwm.ChangeDutyCycle(blue_duty)
            
            self.current_color = color
            logger.info(f"RGB LED strip {self.strip_id} color set to RGB({color.r}, {color.g}, {color.b})")
            return True
            
        except Exception as e:
            logger.error(f"Error setting RGB LED strip color: {e}")
            return False
    
    async def set_brightness(self, brightness: int) -> bool:
        """Set RGB LED strip brightness"""
        if not self.initialized:
            return False
        
        try:
            brightness = max(0, min(100, brightness))
            self.config.brightness = brightness
            
            # Update current color with new brightness
            await self.set_color(self.current_color)
            
            logger.info(f"RGB LED strip {self.strip_id} brightness set to {brightness}%")
            return True
            
        except Exception as e:
            logger.error(f"Error setting RGB LED strip brightness: {e}")
            return False
    
    async def set_pattern(self, pattern: LEDPattern, color: LEDColor = None) -> bool:
        """Set RGB LED strip pattern"""
        if not self.initialized:
            return False
        
        try:
            # Stop current pattern
            await self.stop_pattern()
            
            self.current_pattern = pattern
            if color:
                self.current_color = color
            
            # Start pattern task
            self.is_pattern_running = True
            self.pattern_task = asyncio.create_task(self._run_pattern())
            
            logger.info(f"RGB LED strip {self.strip_id} pattern set to {pattern.value}")
            return True
            
        except Exception as e:
            logger.error(f"Error setting RGB LED strip pattern: {e}")
            return False
    
    async def stop_pattern(self) -> bool:
        """Stop RGB LED strip pattern"""
        try:
            self.is_pattern_running = False
            
            if self.pattern_task:
                self.pattern_task.cancel()
                try:
                    await self.pattern_task
                except asyncio.CancelledError:
                    pass
            
            # Set to static color
            await self.set_color(self.current_color)
            
            logger.info(f"RGB LED strip {self.strip_id} pattern stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping RGB LED strip pattern: {e}")
            return False
    
    async def _run_pattern(self) -> None:
        """Run LED pattern for RGB strip"""
        while self.is_pattern_running and self.initialized:
            try:
                if self.current_pattern == LEDPattern.PULSE:
                    await self._pulse_pattern()
                elif self.current_pattern == LEDPattern.BLINK:
                    await self._blink_pattern()
                elif self.current_pattern == LEDPattern.BREATHING:
                    await self._breathing_pattern()
                else:
                    await self.set_color(self.current_color)
                    await asyncio.sleep(1.0)
                
            except Exception as e:
                logger.error(f"Error in RGB LED pattern: {e}")
                await asyncio.sleep(0.1)
    
    async def _pulse_pattern(self) -> None:
        """Pulse pattern for RGB strip"""
        for brightness in range(0, 101, 5):
            if not self.is_pattern_running:
                break
            
            temp_brightness = self.config.brightness
            self.config.brightness = brightness
            await self.set_color(self.current_color)
            self.config.brightness = temp_brightness
            await asyncio.sleep(0.05)
        
        for brightness in range(100, -1, -5):
            if not self.is_pattern_running:
                break
            
            temp_brightness = self.config.brightness
            self.config.brightness = brightness
            await self.set_color(self.current_color)
            self.config.brightness = temp_brightness
            await asyncio.sleep(0.05)
    
    async def _blink_pattern(self) -> None:
        """Blink pattern for RGB strip"""
        # On
        await self.set_color(self.current_color)
        await asyncio.sleep(0.5)
        
        if not self.is_pattern_running:
            return
        
        # Off
        self.red_pwm.ChangeDutyCycle(0)
        self.green_pwm.ChangeDutyCycle(0)
        self.blue_pwm.ChangeDutyCycle(0)
        await asyncio.sleep(0.5)
    
    async def _breathing_pattern(self) -> None:
        """Breathing pattern for RGB strip"""
        while self.is_pattern_running:
            # Fade in
            for brightness in range(0, 101, 2):
                if not self.is_pattern_running:
                    break
                
                temp_brightness = self.config.brightness
                self.config.brightness = brightness
                await self.set_color(self.current_color)
                self.config.brightness = temp_brightness
                await asyncio.sleep(0.02)
            
            # Fade out
            for brightness in range(100, -1, -2):
                if not self.is_pattern_running:
                    break
                
                temp_brightness = self.config.brightness
                self.config.brightness = brightness
                await self.set_color(self.current_color)
                self.config.brightness = temp_brightness
                await asyncio.sleep(0.02)

class LEDManager:
    """Manages all LED strips and provides unified interface"""
    
    def __init__(self):
        self.strips: Dict[str, LEDStripDriver] = {}
        self.initialized = False
    
    async def add_strip(self, strip_id: str, config: LEDStripConfig) -> bool:
        """Add an LED strip to the system"""
        try:
            if config.strip_type == LEDStripType.WS2812B:
                strip = WS2812BDriver(strip_id, config)
            elif config.strip_type == LEDStripType.RGB_STRIP:
                strip = RGBStripDriver(strip_id, config)
            else:
                logger.error(f"Unsupported LED strip type: {config.strip_type}")
                return False
            
            success = await strip.initialize()
            if success:
                self.strips[strip_id] = strip
                logger.info(f"LED strip {strip_id} added to system")
            
            return success
            
        except Exception as e:
            logger.error(f"Failed to add LED strip {strip_id}: {e}")
            return False
    
    async def initialize(self) -> bool:
        """Initialize LED manager"""
        try:
            self.initialized = True
            logger.info("LED manager initialized")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize LED manager: {e}")
            return False
    
    async def cleanup(self) -> None:
        """Cleanup all LED strips"""
        try:
            for strip in self.strips.values():
                await strip.stop_pattern()
                await strip.cleanup()
            
            self.strips.clear()
            self.initialized = False
            logger.info("LED manager cleaned up")
            
        except Exception as e:
            logger.error(f"Error cleaning up LED manager: {e}")
    
    async def set_strip_color(self, strip_id: str, color: LEDColor) -> bool:
        """Set color for specific LED strip"""
        if strip_id in self.strips:
            return await self.strips[strip_id].set_color(color)
        return False
    
    async def set_strip_brightness(self, strip_id: str, brightness: int) -> bool:
        """Set brightness for specific LED strip"""
        if strip_id in self.strips:
            return await self.strips[strip_id].set_brightness(brightness)
        return False
    
    async def set_strip_pattern(self, strip_id: str, pattern: LEDPattern, color: LEDColor = None) -> bool:
        """Set pattern for specific LED strip"""
        if strip_id in self.strips:
            return await self.strips[strip_id].set_pattern(pattern, color)
        return False
    
    async def stop_strip_pattern(self, strip_id: str) -> bool:
        """Stop pattern for specific LED strip"""
        if strip_id in self.strips:
            return await self.strips[strip_id].stop_pattern()
        return False
    
    async def set_all_color(self, color: LEDColor) -> Dict[str, bool]:
        """Set color for all LED strips"""
        results = {}
        for strip_id in self.strips:
            results[strip_id] = await self.set_strip_color(strip_id, color)
        return results
    
    async def set_all_brightness(self, brightness: int) -> Dict[str, bool]:
        """Set brightness for all LED strips"""
        results = {}
        for strip_id in self.strips:
            results[strip_id] = await self.set_strip_brightness(strip_id, brightness)
        return results
    
    async def set_all_pattern(self, pattern: LEDPattern, color: LEDColor = None) -> Dict[str, bool]:
        """Set pattern for all LED strips"""
        results = {}
        for strip_id in self.strips:
            results[strip_id] = await self.set_strip_pattern(strip_id, pattern, color)
        return results
    
    async def stop_all_patterns(self) -> Dict[str, bool]:
        """Stop patterns for all LED strips"""
        results = {}
        for strip_id in self.strips:
            results[strip_id] = await self.stop_strip_pattern(strip_id)
        return results

# Global LED manager instance
led_manager = LEDManager()
