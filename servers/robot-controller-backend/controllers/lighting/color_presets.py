"""
Color Presets and Themes for Omega1 Lighting

Provides predefined color palettes and themes for quick selection.
Optimized for common use cases and robot status indication.
"""

# Popular color presets (hex strings)
COLOR_PRESETS = {
    # Status Colors
    "idle": "#0066ff",          # Soft blue
    "ready": "#00ffff",         # Cyan
    "active": "#00ff00",        # Green
    "warning": "#ff8800",       # Orange
    "error": "#ff0000",         # Red
    "charging": "#ffff00",      # Yellow
    
    # Cool Colors
    "ocean": "#0066cc",         # Ocean blue
    "sky": "#87ceeb",           # Sky blue
    "ice": "#b0e0e6",           # Ice blue
    "cyan": "#00ffff",          # Pure cyan
    "turquoise": "#40e0d0",     # Turquoise
    
    # Warm Colors
    "sunset": "#ff6600",        # Sunset orange
    "fire": "#ff3300",          # Fire red
    "amber": "#ffbf00",         # Amber
    "gold": "#ffd700",          # Gold
    
    # Party Colors
    "neon_pink": "#ff00ff",     # Neon pink
    "neon_green": "#39ff14",    # Neon green
    "neon_blue": "#00ffff",     # Neon blue
    "purple": "#8000ff",        # Purple
    "magenta": "#ff00ff",       # Magenta
    
    # Natural Colors
    "forest": "#228b22",        # Forest green
    "grass": "#7cb342",         # Grass green
    "earth": "#8b4513",         # Earth brown
    "wood": "#daa520",          # Wood tan
    
    # Classic Colors
    "white": "#ffffff",         # White
    "warm_white": "#fff8dc",    # Warm white
    "cool_white": "#f0f8ff",    # Cool white
    "red": "#ff0000",           # Red
    "green": "#00ff00",         # Green
    "blue": "#0000ff",          # Blue
}

# Pattern + Color combinations (themes)
THEMES = {
    "ocean_dream": {
        "color": COLOR_PRESETS["ocean"],
        "pattern": "aurora",
        "brightness": 0.7,
        "interval": 80,
    },
    "fire_ambient": {
        "color": COLOR_PRESETS["fire"],
        "pattern": "fire",
        "brightness": 0.8,
        "interval": 50,
    },
    "cyber_matrix": {
        "color": COLOR_PRESETS["neon_green"],
        "pattern": "matrix",
        "brightness": 0.9,
        "interval": 100,
    },
    "party_rave": {
        "color": COLOR_PRESETS["neon_pink"],
        "pattern": "rave",
        "brightness": 1.0,
        "interval": 50,
    },
    "calm_breathing": {
        "color": COLOR_PRESETS["sky"],
        "pattern": "breathing",
        "brightness": 0.6,
        "interval": 50,
    },
    "rainbow_show": {
        "color": COLOR_PRESETS["white"],
        "pattern": "rainbow",
        "brightness": 1.0,
        "interval": 20,
    },
    "idle_standby": {
        "color": COLOR_PRESETS["idle"],
        "pattern": "breathing",
        "brightness": 0.4,
        "interval": 50,
    },
    "active_working": {
        "color": COLOR_PRESETS["active"],
        "pattern": "pulse",
        "brightness": 0.7,
        "interval": 500,
    },
    "error_alert": {
        "color": COLOR_PRESETS["error"],
        "pattern": "blink",
        "brightness": 1.0,
        "interval": 200,
    },
    "low_power": {
        "color": COLOR_PRESETS["warning"],
        "pattern": "breathing",
        "brightness": 0.3,
        "interval": 100,
    },
}

def get_preset_color(name: str) -> str:
    """Get a preset color by name."""
    return COLOR_PRESETS.get(name.lower(), COLOR_PRESETS["white"])

def get_theme(name: str) -> dict:
    """Get a complete theme configuration by name."""
    return THEMES.get(name.lower(), THEMES["idle_standby"])

def list_presets() -> list:
    """List all available color preset names."""
    return list(COLOR_PRESETS.keys())

def list_themes() -> list:
    """List all available theme names."""
    return list(THEMES.keys())

