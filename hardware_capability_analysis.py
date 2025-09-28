"""
Hardware Capability Analysis for Raspberry Pi 4B
- Analyze what optimizations are realistic for Pi 4B
- Identify potential bottlenecks and limitations
- Suggest hardware-appropriate configurations
"""

import psutil
import platform
import subprocess
import json
import time

def get_system_info():
    """Get detailed system information"""
    try:
        # CPU Information
        cpu_count = psutil.cpu_count(logical=False)  # Physical cores
        cpu_count_logical = psutil.cpu_count(logical=True)  # Logical cores
        
        # Memory Information
        memory = psutil.virtual_memory()
        memory_gb = memory.total / (1024**3)
        
        # Disk Information
        disk = psutil.disk_usage('/')
        disk_gb = disk.total / (1024**3)
        
        # System Information
        system_info = {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "architecture": platform.architecture()[0],
            "cpu_cores_physical": cpu_count,
            "cpu_cores_logical": cpu_count_logical,
            "memory_total_gb": round(memory_gb, 2),
            "memory_available_gb": round(memory.available / (1024**3), 2),
            "disk_total_gb": round(disk_gb, 2),
            "disk_free_gb": round(disk.free / (1024**3), 2)
        }
        
        return system_info
        
    except Exception as e:
        print(f"Error getting system info: {e}")
        return {}

def check_pi_specific_info():
    """Check for Raspberry Pi specific information"""
    try:
        pi_info = {}
        
        # Check if running on Raspberry Pi
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpuinfo = f.read()
                if 'BCM' in cpuinfo or 'Raspberry Pi' in cpuinfo:
                    pi_info["is_raspberry_pi"] = True
                    
                    # Extract Pi model
                    for line in cpuinfo.split('\n'):
                        if 'Model' in line:
                            pi_info["pi_model"] = line.split(':')[1].strip()
                            break
                else:
                    pi_info["is_raspberry_pi"] = False
        except:
            pi_info["is_raspberry_pi"] = False
        
        # Check GPU memory split
        try:
            result = subprocess.run(['vcgencmd', 'get_mem', 'gpu'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pi_info["gpu_memory"] = result.stdout.strip()
        except:
            pi_info["gpu_memory"] = "Unknown"
        
        # Check temperature
        try:
            result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pi_info["temperature"] = result.stdout.strip()
        except:
            pi_info["temperature"] = "Unknown"
        
        return pi_info
        
    except Exception as e:
        print(f"Error checking Pi info: {e}")
        return {}

def analyze_optimization_feasibility(system_info, pi_info):
    """Analyze feasibility of each optimization"""
    
    analysis = {
        "hardware_profile": "Raspberry Pi 4B" if pi_info.get("is_raspberry_pi") else "Unknown",
        "optimizations": {}
    }
    
    # AI Predictive Maintenance Analysis
    ai_analysis = {
        "feasible": True,
        "recommendations": [],
        "limitations": [],
        "score": "HIGH"
    }
    
    if system_info.get("memory_total_gb", 0) < 4:
        ai_analysis["limitations"].append("Limited RAM for large ML models")
        ai_analysis["recommendations"].append("Use lightweight ML models")
        ai_analysis["score"] = "MEDIUM"
    
    if system_info.get("cpu_cores_physical", 0) < 4:
        ai_analysis["limitations"].append("Limited CPU cores for parallel processing")
        ai_analysis["recommendations"].append("Use simplified neural networks")
    
    analysis["optimizations"]["ai_predictive"] = ai_analysis
    
    # Quantum Optimization Analysis
    quantum_analysis = {
        "feasible": True,
        "recommendations": [],
        "limitations": [],
        "score": "HIGH"
    }
    
    if system_info.get("cpu_cores_physical", 0) < 4:
        quantum_analysis["limitations"].append("Limited cores for quantum simulation")
        quantum_analysis["recommendations"].append("Reduce particle count in PSO")
    
    if system_info.get("memory_total_gb", 0) < 4:
        quantum_analysis["limitations"].append("Memory constraints for large matrices")
        quantum_analysis["recommendations"].append("Use smaller problem dimensions")
    
    analysis["optimizations"]["quantum_optimization"] = quantum_analysis
    
    # Edge Computing Analysis
    edge_analysis = {
        "feasible": True,
        "recommendations": [],
        "limitations": [],
        "score": "HIGH"
    }
    
    if system_info.get("memory_total_gb", 0) < 4:
        edge_analysis["limitations"].append("Limited memory for multiple tasks")
        edge_analysis["recommendations"].append("Limit concurrent edge tasks")
    
    analysis["optimizations"]["edge_computing"] = edge_analysis
    
    # Blockchain Verification Analysis
    blockchain_analysis = {
        "feasible": True,
        "recommendations": [],
        "limitations": [],
        "score": "MEDIUM"
    }
    
    if system_info.get("disk_free_gb", 0) < 10:
        blockchain_analysis["limitations"].append("Limited storage for blockchain")
        blockchain_analysis["recommendations"].append("Use smaller block sizes")
        blockchain_analysis["score"] = "LOW"
    
    if system_info.get("memory_total_gb", 0) < 4:
        blockchain_analysis["limitations"].append("Memory constraints for blockchain operations")
        blockchain_analysis["recommendations"].append("Limit blockchain history")
    
    analysis["optimizations"]["blockchain_verification"] = blockchain_analysis
    
    # Performance Dashboard Analysis
    dashboard_analysis = {
        "feasible": True,
        "recommendations": [],
        "limitations": [],
        "score": "HIGH"
    }
    
    if system_info.get("memory_total_gb", 0) < 4:
        dashboard_analysis["limitations"].append("Limited memory for real-time visualization")
        dashboard_analysis["recommendations"].append("Reduce update frequency")
    
    analysis["optimizations"]["performance_dashboard"] = dashboard_analysis
    
    return analysis

def generate_recommendations(analysis):
    """Generate hardware-specific recommendations"""
    
    recommendations = {
        "immediate_actions": [],
        "optimization_adjustments": [],
        "hardware_upgrades": [],
        "configuration_changes": []
    }
    
    # Immediate actions
    recommendations["immediate_actions"].extend([
        "Monitor CPU temperature during heavy operations",
        "Set up swap file if memory is limited",
        "Configure GPU memory split appropriately",
        "Enable hardware acceleration where possible"
    ])
    
    # Optimization adjustments
    recommendations["optimization_adjustments"].extend([
        "Reduce quantum PSO particle count to 10-20 (from 30)",
        "Use simplified neural networks for AI predictions",
        "Limit blockchain history to last 100 blocks",
        "Reduce dashboard update frequency to 2-3 seconds",
        "Use smaller problem dimensions for quantum algorithms"
    ])
    
    # Hardware upgrades (if needed)
    recommendations["hardware_upgrades"].extend([
        "Consider Pi 5 for better performance",
        "Add USB SSD for faster storage",
        "Use active cooling for sustained performance",
        "Consider external compute units for heavy AI tasks"
    ])
    
    # Configuration changes
    recommendations["configuration_changes"].extend([
        "Increase GPU memory split to 128MB",
        "Enable hardware acceleration in config.txt",
        "Set CPU governor to 'performance'",
        "Configure appropriate swap size"
    ])
    
    return recommendations

def main():
    """Main analysis function"""
    print("🔍 HARDWARE CAPABILITY ANALYSIS")
    print("=" * 50)
    
    # Get system information
    print("\n📊 System Information:")
    system_info = get_system_info()
    for key, value in system_info.items():
        print(f"  {key}: {value}")
    
    # Get Pi-specific information
    print("\n🍓 Raspberry Pi Information:")
    pi_info = check_pi_specific_info()
    for key, value in pi_info.items():
        print(f"  {key}: {value}")
    
    # Analyze optimization feasibility
    print("\n⚡ Optimization Feasibility Analysis:")
    analysis = analyze_optimization_feasibility(system_info, pi_info)
    
    for opt_name, opt_analysis in analysis["optimizations"].items():
        print(f"\n  {opt_name.upper()}:")
        print(f"    Feasible: {opt_analysis['feasible']}")
        print(f"    Score: {opt_analysis['score']}")
        
        if opt_analysis['limitations']:
            print(f"    Limitations:")
            for limitation in opt_analysis['limitations']:
                print(f"      - {limitation}")
        
        if opt_analysis['recommendations']:
            print(f"    Recommendations:")
            for rec in opt_analysis['recommendations']:
                print(f"      + {rec}")
    
    # Generate recommendations
    print("\n💡 Recommendations:")
    recommendations = generate_recommendations(analysis)
    
    for category, items in recommendations.items():
        print(f"\n  {category.replace('_', ' ').title()}:")
        for item in items:
            print(f"    • {item}")
    
    # Overall assessment
    print("\n🎯 OVERALL ASSESSMENT:")
    total_score = 0
    feasible_count = 0
    
    for opt_analysis in analysis["optimizations"].values():
        if opt_analysis['feasible']:
            feasible_count += 1
            if opt_analysis['score'] == 'HIGH':
                total_score += 3
            elif opt_analysis['score'] == 'MEDIUM':
                total_score += 2
            else:
                total_score += 1
    
    max_score = len(analysis["optimizations"]) * 3
    percentage = (total_score / max_score) * 100
    
    print(f"  Feasible Optimizations: {feasible_count}/{len(analysis['optimizations'])}")
    print(f"  Performance Score: {total_score}/{max_score} ({percentage:.1f}%)")
    
    if percentage >= 80:
        print("  🟢 EXCELLENT: Your hardware can handle most optimizations!")
    elif percentage >= 60:
        print("  🟡 GOOD: Your hardware can handle most optimizations with adjustments")
    else:
        print("  🔴 LIMITED: Consider hardware upgrades for optimal performance")
    
    print(f"\n🏆 RECOMMENDED BEAST MODE LEVEL: ", end="")
    if percentage >= 90:
        print("GODLIKE (100/100)")
    elif percentage >= 80:
        print("LEGENDARY (95/100)")
    elif percentage >= 70:
        print("ENHANCED (90/100)")
    else:
        print("NORMAL (85/100)")

if __name__ == "__main__":
    main()
