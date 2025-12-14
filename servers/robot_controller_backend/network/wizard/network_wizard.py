#!/usr/bin/env python3
"""
Omega-1 Network Setup Wizard
============================

Manages Wi-Fi mode switching for Raspberry Pi:
- Access Point Mode (AP mode / Wi-Fi Direct)
- Client Mode (normal Wi-Fi)
- Headless SSH mode with static IP

Usage:
    sudo python3 network_wizard.py
    sudo omega-network ap
    sudo omega-network client
"""

import os
import sys
import subprocess
import shutil
import json
from pathlib import Path
from typing import Dict, Optional, Tuple
from jinja2 import Template, Environment, FileSystemLoader

# Configuration constants
AP_SSID = "Omega1-AP"
AP_PASSWORD = "omegawifi123"
AP_IP = "192.168.4.1"
AP_DHCP_START = "192.168.4.2"
AP_DHCP_END = "192.168.4.20"
AP_SUBNET = "192.168.4.0"
AP_NETMASK = "255.255.255.0"

# File paths
HOSTAPD_CONF = "/etc/hostapd/hostapd.conf"
DNSMASQ_CONF = "/etc/dnsmasq.conf"
DHCPCD_CONF = "/etc/dhcpcd.conf"
HOSTAPD_DEFAULT = "/etc/default/hostapd"
NETWORK_STATE_FILE = "/etc/omega-network/state.json"
LOG_FILE = "/var/log/omega-network.log"

# Template directory
TEMPLATE_DIR = os.path.join(os.path.dirname(__file__), "config")

# Required packages
REQUIRED_PACKAGES = ["hostapd", "dnsmasq", "dhcpcd5"]


class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def log(message: str, level: str = "INFO"):
    """Log message to file and stdout"""
    log_entry = f"[{level}] {message}\n"
    try:
        os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
        with open(LOG_FILE, "a") as f:
            f.write(log_entry)
    except PermissionError:
        pass  # Skip file logging if no permission
    
    if level == "ERROR":
        print(f"{Colors.FAIL}{log_entry.strip()}{Colors.ENDC}")
    elif level == "WARNING":
        print(f"{Colors.WARNING}{log_entry.strip()}{Colors.ENDC}")
    elif level == "SUCCESS":
        print(f"{Colors.OKGREEN}{log_entry.strip()}{Colors.ENDC}")
    else:
        print(log_entry.strip())


def check_root():
    """Ensure script is run as root"""
    if os.geteuid() != 0:
        log("This script must be run as root (use sudo)", "ERROR")
        sys.exit(1)


def run_command(cmd: list, check: bool = True) -> Tuple[bool, str]:
    """Run shell command and return success status and output"""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=check
        )
        return True, result.stdout
    except subprocess.CalledProcessError as e:
        return False, e.stderr


def check_package_installed(package: str) -> bool:
    """Check if a package is installed"""
    success, _ = run_command(["dpkg", "-l", package], check=False)
    return success


def install_packages():
    """Install required packages if not present"""
    log("Checking for required packages...")
    
    missing_packages = []
    for package in REQUIRED_PACKAGES:
        if not check_package_installed(package):
            missing_packages.append(package)
            log(f"Missing package: {package}", "WARNING")
    
    if missing_packages:
        log(f"Installing packages: {', '.join(missing_packages)}")
        success, output = run_command(
            ["apt-get", "update"],
            check=False
        )
        if not success:
            log(f"Failed to update package list: {output}", "ERROR")
            return False
        
        success, output = run_command(
            ["apt-get", "install", "-y"] + missing_packages,
            check=False
        )
        if not success:
            log(f"Failed to install packages: {output}", "ERROR")
            return False
        
        log("Packages installed successfully", "SUCCESS")
    else:
        log("All required packages are installed", "SUCCESS")
    
    return True


def create_directory(path: str):
    """Create directory if it doesn't exist"""
    Path(path).mkdir(parents=True, exist_ok=True)


def backup_file(filepath: str) -> bool:
    """Backup existing configuration file"""
    if os.path.exists(filepath):
        backup_path = f"{filepath}.backup"
        try:
            shutil.copy2(filepath, backup_path)
            log(f"Backed up {filepath} to {backup_path}")
            return True
        except Exception as e:
            log(f"Failed to backup {filepath}: {e}", "WARNING")
            return False
    return True


def render_template(template_name: str, context: Dict) -> str:
    """Render Jinja2 template"""
    try:
        env = Environment(loader=FileSystemLoader(TEMPLATE_DIR))
        template = env.get_template(template_name)
        return template.render(**context)
    except Exception as e:
        log(f"Failed to render template {template_name}: {e}", "ERROR")
        raise


def write_hostapd_config():
    """Create hostapd configuration for AP mode using Jinja template"""
    log("Creating hostapd configuration...")
    
    backup_file(HOSTAPD_CONF)
    create_directory(os.path.dirname(HOSTAPD_CONF))
    
    context = {
        "ssid": AP_SSID,
        "password": AP_PASSWORD,
    }
    
    config = render_template("hostapd.conf.j2", context)
    
    try:
        with open(HOSTAPD_CONF, "w") as f:
            f.write(config)
        log(f"Created {HOSTAPD_CONF}", "SUCCESS")
        return True
    except Exception as e:
        log(f"Failed to write {HOSTAPD_CONF}: {e}", "ERROR")
        return False


def write_dnsmasq_config():
    """Create dnsmasq configuration for DHCP using Jinja template"""
    log("Creating dnsmasq configuration...")
    
    backup_file(DNSMASQ_CONF)
    
    context = {
        "dhcp_start": AP_DHCP_START,
        "dhcp_end": AP_DHCP_END,
        "netmask": AP_NETMASK,
        "ap_ip": AP_IP,
    }
    
    config = render_template("dnsmasq.conf.j2", context)
    
    try:
        with open(DNSMASQ_CONF, "w") as f:
            f.write(config)
        log(f"Created {DNSMASQ_CONF}", "SUCCESS")
        return True
    except Exception as e:
        log(f"Failed to write {DNSMASQ_CONF}: {e}", "ERROR")
        return False


def write_dhcpcd_config(mode: str):
    """Update dhcpcd.conf using Jinja template"""
    log(f"Updating dhcpcd configuration for {mode} mode...")
    
    backup_file(DHCPCD_CONF)
    
    # Read existing config
    existing_config = ""
    if os.path.exists(DHCPCD_CONF):
        with open(DHCPCD_CONF, "r") as f:
            existing_config = f.read()
    
    # Remove any existing Omega-1 AP Mode configuration
    lines = existing_config.split("\n")
    filtered_lines = []
    skip_until_blank = False
    
    for line in lines:
        if "interface wlan0" in line.lower() and "Omega-1 AP Mode" in existing_config:
            skip_until_blank = True
            continue
        if skip_until_blank and line.strip() == "":
            skip_until_blank = False
            continue
        if skip_until_blank:
            continue
        filtered_lines.append(line)
    
    # Render template
    context = {
        "mode": mode,
        "ap_ip": AP_IP,
    }
    
    template_content = render_template("dhcpcd.conf.j2", context)
    
    # Combine existing config with new config
    if mode == "ap":
        new_config = "\n".join(filtered_lines) + "\n\n# Omega-1 AP Mode Configuration (added by network wizard)\n" + template_content
    else:
        new_config = "\n".join(filtered_lines)
    
    try:
        with open(DHCPCD_CONF, "w") as f:
            f.write(new_config)
        log(f"Updated {DHCPCD_CONF} for {mode} mode", "SUCCESS")
        return True
    except Exception as e:
        log(f"Failed to write {DHCPCD_CONF}: {e}", "ERROR")
        return False


def write_hostapd_default():
    """Configure hostapd default settings"""
    log("Configuring hostapd defaults...")
    
    backup_file(HOSTAPD_DEFAULT)
    
    config = f"""# Omega-1 Hostapd Default Configuration
# Generated by Omega Network Wizard

DAEMON_CONF="{HOSTAPD_CONF}"
"""
    
    try:
        with open(HOSTAPD_DEFAULT, "w") as f:
            f.write(config)
        log(f"Created {HOSTAPD_DEFAULT}", "SUCCESS")
        return True
    except Exception as e:
        log(f"Failed to write {HOSTAPD_DEFAULT}: {e}", "ERROR")
        return False


def save_network_state(mode: str):
    """Save current network mode to state file"""
    create_directory(os.path.dirname(NETWORK_STATE_FILE))
    
    state = {
        "mode": mode,
        "ap_ssid": AP_SSID,
        "ap_ip": AP_IP,
        "last_updated": subprocess.check_output(["date", "-Iseconds"]).decode().strip()
    }
    
    try:
        with open(NETWORK_STATE_FILE, "w") as f:
            json.dump(state, f, indent=2)
        log(f"Saved network state: {mode}")
        return True
    except Exception as e:
        log(f"Failed to save network state: {e}", "WARNING")
        return False


def load_network_state() -> Optional[Dict]:
    """Load network state from file"""
    if os.path.exists(NETWORK_STATE_FILE):
        try:
            with open(NETWORK_STATE_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            log(f"Failed to load network state: {e}", "WARNING")
    return None


def stop_services():
    """Stop network services"""
    log("Stopping network services...")
    
    services = ["hostapd", "dnsmasq"]
    for service in services:
        success, _ = run_command(["systemctl", "stop", service], check=False)
        if success:
            log(f"Stopped {service}")
        else:
            log(f"Service {service} was not running or failed to stop", "WARNING")


def enable_ap_mode():
    """Enable Access Point mode"""
    log("=" * 60)
    log("Enabling Access Point Mode", "SUCCESS")
    log("=" * 60)
    
    if not install_packages():
        return False
    
    if not write_hostapd_config():
        return False
    
    if not write_dnsmasq_config():
        return False
    
    if not write_dhcpcd_config("ap"):
        return False
    
    if not write_hostapd_default():
        return False
    
    # Stop wpa_supplicant (interferes with AP mode)
    log("Stopping wpa_supplicant...")
    run_command(["systemctl", "stop", "wpa_supplicant"], check=False)
    run_command(["systemctl", "disable", "wpa_supplicant"], check=False)
    
    # Restart dhcpcd to apply static IP
    log("Restarting dhcpcd...")
    run_command(["systemctl", "restart", "dhcpcd"], check=False)
    
    # Wait for interface to be ready
    import time
    time.sleep(3)
    
    # Start AP services
    log("Starting hostapd and dnsmasq...")
    run_command(["systemctl", "enable", "hostapd"], check=False)
    run_command(["systemctl", "enable", "dnsmasq"], check=False)
    run_command(["systemctl", "start", "hostapd"], check=False)
    run_command(["systemctl", "start", "dnsmasq"], check=False)
    
    save_network_state("ap")
    
    log("=" * 60)
    log("Access Point Mode Enabled!", "SUCCESS")
    log(f"SSID: {AP_SSID}", "SUCCESS")
    log("Password: ***MASKED*** (check config)", "SUCCESS")  # Security: Don't log passwords
    log(f"Pi IP: {AP_IP}", "SUCCESS")
    log("=" * 60)
    log("Connect to Omega1-AP and SSH: omega1@192.168.4.1")
    
    return True


def enable_client_mode():
    """Enable Wi-Fi Client mode"""
    log("=" * 60)
    log("Enabling Wi-Fi Client Mode", "SUCCESS")
    log("=" * 60)
    
    # Stop AP services
    stop_services()
    
    # Disable AP services
    run_command(["systemctl", "disable", "hostapd"], check=False)
    run_command(["systemctl", "disable", "dnsmasq"], check=False)
    
    # Update dhcpcd for client mode
    if not write_dhcpcd_config("client"):
        return False
    
    # Enable wpa_supplicant for client mode
    log("Enabling wpa_supplicant...")
    run_command(["systemctl", "enable", "wpa_supplicant"], check=False)
    run_command(["systemctl", "start", "wpa_supplicant"], check=False)
    
    # Restart dhcpcd to get DHCP IP
    log("Restarting dhcpcd...")
    run_command(["systemctl", "restart", "dhcpcd"], check=False)
    
    save_network_state("client")
    
    log("=" * 60)
    log("Client Mode Enabled!", "SUCCESS")
    log("Pi will connect to your Wi-Fi network", "SUCCESS")
    log("=" * 60)
    
    return True


def get_network_status() -> Dict[str, any]:
    """Get current network status (for API use)"""
    state = load_network_state()
    mode = state.get("mode", "unknown") if state else "unknown"
    
    # Get service statuses
    services = {}
    for service in ["hostapd", "dnsmasq", "dhcpcd", "wpa_supplicant"]:
        success, _ = run_command(["systemctl", "is-active", service], check=False)
        services[service] = "active" if success else "inactive"
    
    # Get wlan0 IP
    wlan0_ip = None
    success, output = run_command(["ip", "addr", "show", "wlan0"], check=False)
    if success:
        for line in output.split("\n"):
            if "inet " in line and "127.0.0.1" not in line:
                parts = line.strip().split()
                if len(parts) >= 2:
                    wlan0_ip = parts[1].split("/")[0]
                    break
    
    result = {
        "mode": mode,
        "services": services,
        "wlan0_ip": wlan0_ip,
    }
    
    if state:
        result["ap_ssid"] = state.get("ap_ssid")
        result["ap_ip"] = state.get("ap_ip")
        result["last_updated"] = state.get("last_updated")
    
    return result


def validate_network_config() -> Dict[str, bool]:
    """Run network configuration validation checks"""
    state = load_network_state()
    mode = state.get("mode", "unknown") if state else "unknown"
    
    results = {}
    
    if mode == "ap":
        # Validate AP config files
        results["ap_config_exists"] = os.path.exists(HOSTAPD_CONF) and os.path.exists(DNSMASQ_CONF)
        
        # Validate hostapd running
        success, _ = run_command(["systemctl", "is-active", "hostapd"], check=False)
        results["hostapd_running"] = success
        
        # Validate wlan0 IP
        wlan0_ip = None
        success, output = run_command(["ip", "addr", "show", "wlan0"], check=False)
        if success:
            for line in output.split("\n"):
                if AP_IP in line:
                    wlan0_ip = AP_IP
                    break
        results["wlan0_static_ip"] = wlan0_ip == AP_IP
        
        # Validate DHCP service
        success, _ = run_command(["systemctl", "is-active", "dnsmasq"], check=False)
        results["dhcp_service_running"] = success
    else:
        results["ap_config_exists"] = True  # Not applicable
        results["hostapd_running"] = True  # Not required
        results["wlan0_static_ip"] = True  # Not applicable
        results["dhcp_service_running"] = True  # Not required
    
    return results


def show_network_status():
    """Display current network status (CLI)"""
    log("=" * 60)
    log("Network Status", "SUCCESS")
    log("=" * 60)
    
    status = get_network_status()
    
    log(f"Current Mode: {status['mode'].upper()}")
    if status.get('mode') == 'ap':
        log(f"AP SSID: {status.get('ap_ssid', AP_SSID)}")
        log(f"AP IP: {status.get('ap_ip', AP_IP)}")
    
    log("\nService Status:")
    for service, service_status in status['services'].items():
        log(f"  {service}: {service_status.upper()}")
    
    if status.get('wlan0_ip'):
        log(f"\nwlan0 IP: {status['wlan0_ip']}")
    else:
        log("\nwlan0: Not configured", "WARNING")
    
    log("=" * 60)


def run_full_validation():
    """Run all validation checks (CLI)"""
    log("=" * 60)
    log("Running Network Validation", "SUCCESS")
    log("=" * 60)
    
    results = validate_network_config()
    
    for check, passed in results.items():
        if passed:
            log(f"✓ {check}", "SUCCESS")
        else:
            log(f"✗ {check}", "ERROR")
    
    if all(results.values()):
        log("\n✓ All validations passed!", "SUCCESS")
        return True
    else:
        log("\n✗ Some validations failed", "ERROR")
        return False


def interactive_menu():
    """Interactive CLI menu"""
    while True:
        print("\n" + "=" * 60)
        print(f"{Colors.BOLD}Omega-1 Network Wizard{Colors.ENDC}")
        print("=" * 60)
        print("1. Enable AP mode (field mode)")
        print("2. Enable Wi-Fi client mode (home mode)")
        print("3. Show network status")
        print("4. Regenerate configs")
        print("5. Run validation")
        print("6. Exit")
        print("=" * 60)
        
        choice = input(f"{Colors.OKCYAN}Select option (1-6): {Colors.ENDC}").strip()
        
        if choice == "1":
            enable_ap_mode()
        elif choice == "2":
            enable_client_mode()
        elif choice == "3":
            show_network_status()
        elif choice == "4":
            log("Regenerating configurations...")
            enable_ap_mode()  # Regenerate AP configs
        elif choice == "5":
            run_full_validation()
        elif choice == "6":
            log("Exiting...")
            break
        else:
            log("Invalid option. Please select 1-6.", "WARNING")


def main():
    """Main entry point"""
    check_root()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "ap":
            enable_ap_mode()
        elif command == "client":
            enable_client_mode()
        elif command == "status":
            show_network_status()
        elif command == "validate":
            run_full_validation()
        else:
            log(f"Unknown command: {command}", "ERROR")
            log("Usage: sudo python3 network_wizard.py [ap|client|status|validate]", "ERROR")
            sys.exit(1)
    else:
        interactive_menu()


if __name__ == "__main__":
    main()

