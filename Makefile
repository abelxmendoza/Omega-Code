# File: /Omega-Code/Makefile
# Summary:
#   One-stop commands for dev + field ops:
#     - install deps on Mac/Pi
#     - profile-aware endpoint checks (lan|tailscale|local)
#     - run UI (Next.js) with a chosen profile
#     - start backend services (movement, video, etc.)
#     - Bluetooth PAN helpers (Pi + macOS)
#   Usage:
#     make help
#     make check PROFILE=tailscale
#     make ui-dev PROFILE=tailscale
#     make run-movement
#     make pan PHONE_MAC=AA:BB:CC:DD:EE:FF
#
# Notes:
#   - Uses bash; loads env from:
#       scripts/.env (operator values like PHONE_MAC)
#       servers/robot-controller-backend/.env (ports/IPs)
#   - PROFILE defaults to 'local' but you can override per-invocation.

SHELL := /bin/bash
ROOT  := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
UIDIR := $(ROOT)/ui/robot-controller-ui
SRVD  := $(ROOT)/servers/robot-controller-backend
SCRIPTS := $(ROOT)/scripts

# -------- Config knobs --------
PROFILE ?= local            # lan | tailscale | local
PHONE_MAC ?=                # e.g., AA:BB:CC:DD:EE:FF (for PAN target)
PY ?= python3               # python interpreter on Pi
PIP ?= pip3

# Export profile so Next.js picks it up when we run ui-dev
export NEXT_PUBLIC_NETWORK_PROFILE := $(PROFILE)

# Helper: source both env files when running scripts
define SRC_ENVS
set -a; \
  [ -f "$(SCRIPTS)/.env" ] && . "$(SCRIPTS)/.env"; \
  [ -f "$(SRVD)/.env" ]    && . "$(SRVD)/.env"; \
set +a;
endef

# Default target
.DEFAULT_GOAL := help

.PHONY: help
## Show help (this list)
help:
	@awk 'BEGIN{FS":.*## "}/^[a-zA-Z0-9_.-]+:.*## /{printf "\033[36m%-22s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

# ---------------- System deps ----------------

.PHONY: deps-mac
## Install macOS deps (blueutil, curl; optional wscat)
deps-mac:
	@command -v brew >/dev/null || { echo "Homebrew required: https://brew.sh"; exit 1; }
	brew update
	brew install blueutil curl || true
	@command -v npm >/dev/null && npm i -g wscat@8 || true
	@echo "✅ macOS deps installed."

.PHONY: deps-pi
## Install Raspberry Pi (Debian) deps (bluez, net tools)
deps-pi:
	sudo apt update
	sudo apt install -y bluez bluez-tools rfkill curl netcat-openbsd dnsutils iproute2 dhcpcd5 dhclient
	@echo "✅ Pi deps installed."

# ---------------- Profiles & checks ----------------

.PHONY: check
## Print resolved URLs and probe them (PROFILE=$(PROFILE), add SKIP=1 to skip ping)
check:
	@$(SRC_ENVS) \
	SKIP_ARG=""; [[ "$(SKIP)" == "1" ]] && SKIP_ARG="--skip-ping"; \
	"$(SCRIPTS)/check_endpoints.sh" --profile "$(PROFILE)" $$SKIP_ARG

.PHONY: env-print
## Print active profile and key env values (debug)
env-print:
	@echo "Profile: $(PROFILE)"; \
	echo "NEXT_PUBLIC_NETWORK_PROFILE=$(NEXT_PUBLIC_NETWORK_PROFILE)"; \
	$(SRC_ENVS) \
	echo "PI_IP=$$PI_IP  TAILSCALE_IP_PI=$$TAILSCALE_IP_PI  VIDEO_PORT=$$VIDEO_PORT"; \
	echo "Movement=$$PORT_MOVEMENT  Ultrasonic=$$PORT_ULTRASONIC  Lighting=$$PORT_LIGHTING  Line=$$PORT_LINE_TRACKER  Location=$$PORT_LOCATION"

# ---------------- UI (Next.js) ----------------

.PHONY: ui-dev
## Run Next.js dev server (respects PROFILE=lan|tailscale|local; default local)
ui-dev:
	cd "$(UIDIR)" && npm run dev

.PHONY: ui-build
## Build the UI (Next.js)
ui-build:
	cd "$(UIDIR)" && npm run build

.PHONY: ui-start
## Start the built UI
ui-start:
	cd "$(UIDIR)" && npm run start

# ---------------- Backend: Python env & install ----------------

VENVD := $(SRVD)/venv
.PHONY: venv
## Create Python venv for backend
venv:
	test -d "$(VENVD)" || $(PY) -m venv "$(VENVD)"
	@echo "✅ venv at $(VENVD)"

.PHONY: backend-install
## pip install backend requirements (in venv if exists)
backend-install:
	@if [ -d "$(VENVD)" ]; then \
	  . "$(VENVD)/bin/activate"; \
	  $(PIP) install --upgrade pip; \
	  $(PIP) install -r "$(SRVD)/requirements.txt"; \
	else \
	  $(PIP) install --upgrade pip; \
	  $(PIP) install -r "$(SRVD)/requirements.txt"; \
	fi

# ---------------- Backend services (run one-by-one) ----------------

.PHONY: run-video
## Run video server on :$VIDEO_PORT (reads SRV .env)
run-video:
	@$(SRC_ENVS) \
	cd "$(SRVD)" && $(PY) video_server.py

.PHONY: run-movement
## Run movement WS server on :$PORT_MOVEMENT
run-movement:
	@$(SRC_ENVS) \
	cd "$(SRVD)/movement" && $(PY) movement_ws_server.py

.PHONY: run-ultrasonic
## Run ultrasonic WS server on :$PORT_ULTRASONIC
run-ultrasonic:
	@$(SRC_ENVS) \
	cd "$(SRVD)" && $(PY) sensors/ultrasonic_server.py

.PHONY: run-lighting
## Run lighting WS server on :$PORT_LIGHTING
run-lighting:
	@$(SRC_ENVS) \
	cd "$(SRVD)" && $(PY) lighting/ws_server.py

.PHONY: run-line
## Run line tracker WS server on :$PORT_LINE_TRACKER
run-line:
	@$(SRC_ENVS) \
	cd "$(SRVD)" && $(PY) line/ws_server.py

.PHONY: run-location
## Run location WS server on :$PORT_LOCATION
run-location:
	@$(SRC_ENVS) \
	cd "$(SRVD)" && $(PY) location/ws_server.py

# ---------------- Bluetooth PAN helpers ----------------

.PHONY: pan
## (Pi) Connect to iPhone PAN; set PHONE_MAC=AA:BB:... or rely on auto-discovery
pan:
	@[ -n "$(PHONE_MAC)" ] && echo "Using PHONE_MAC=$(PHONE_MAC)" || echo "PHONE_MAC not provided; script will try discovery"
	cd "$(SCRIPTS)" && ./pi_connect_phone_pan.sh $(PHONE_MAC)

.PHONY: mac-pan
## (macOS) Connect Mac to iPhone PAN and trigger Pi helper over SSH (reads scripts/.env)
mac-pan:
	cd "$(SCRIPTS)" && ./connect_robot_bluetooth.sh

# ---------------- Utilities ----------------

.PHONY: wscat-move
## Quick WS test to movement (TS or LAN based on PROFILE)
wscat-move:
	@$(SRC_ENVS) \
	HOST="localhost"; \
	if [ "$(PROFILE)" = "tailscale" ] && [ -n "$$TAILSCALE_IP_PI" ]; then HOST="$$TAILSCALE_IP_PI"; fi; \
	if [ "$(PROFILE)" = "lan" ] && [ -n "$$PI_IP" ]; then HOST="$$PI_IP"; fi; \
	URL="ws://$${HOST}:$${PORT_MOVEMENT:-8081}"; \
	echo "Connecting to $$URL"; \
	npx -y wscat@8 -c "$$URL" || true

.PHONY: mock-video
## Start a tiny mock server on :5000 that returns 200 for /video_feed (dev only)
mock-video:
	$(PY) - <<'PY'
from http.server import BaseHTTPRequestHandler, HTTPServer
class H(BaseHTTPRequestHandler):
    def do_HEAD(self): self.send_response(200); self.end_headers()
    def do_GET(self):  self.do_HEAD()
HTTPServer(('0.0.0.0', 5000), H).serve_forever()
PY
