"""
Autonomous Decision Engine
AI-powered decision making system for autonomous robot operation
"""

import asyncio
import logging
import numpy as np
import json
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import math
from collections import deque

logger = logging.getLogger(__name__)

class DecisionType(Enum):
    MOVEMENT = "movement"
    SAFETY = "safety"
    EFFICIENCY = "efficiency"
    MAINTENANCE = "maintenance"
    EMERGENCY = "emergency"

class DecisionPriority(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

class ActionType(Enum):
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    STOP = "stop"
    EMERGENCY_STOP = "emergency_stop"
    CHANGE_SPEED = "change_speed"
    ADJUST_CAMERA = "adjust_camera"
    CHANGE_LED_PATTERN = "change_led_pattern"
    SEND_ALERT = "send_alert"
    SCHEDULE_MAINTENANCE = "schedule_maintenance"

@dataclass
class DecisionContext:
    """Context for decision making"""
    timestamp: float
    sensor_data: Dict[str, Any]
    navigation_state: Dict[str, Any]
    system_state: Dict[str, Any]
    predictions: Dict[str, Any]
    user_commands: List[str]
    environment_context: Dict[str, Any]

@dataclass
class Decision:
    """Decision result"""
    decision_type: DecisionType
    action: ActionType
    priority: DecisionPriority
    confidence: float
    reasoning: str
    parameters: Dict[str, Any]
    timestamp: float
    execution_time: float = 0.0

@dataclass
class DecisionRule:
    """Decision rule definition"""
    name: str
    condition: str
    action: ActionType
    priority: DecisionPriority
    confidence: float
    parameters: Dict[str, Any]

class SafetyManager:
    """Safety decision making system"""
    
    def __init__(self):
        self.safety_rules = []
        self.emergency_thresholds = {
            'obstacle_distance': 0.5,  # meters
            'battery_level': 10,  # percentage
            'temperature': 80,  # celsius
            'error_rate': 0.1  # errors per second
        }
        self.safety_history = deque(maxlen=100)
    
    async def evaluate_safety(self, context: DecisionContext) -> List[Decision]:
        """Evaluate safety conditions and make decisions"""
        decisions = []
        
        try:
            # Check emergency conditions
            emergency_decisions = await self._check_emergency_conditions(context)
            decisions.extend(emergency_decisions)
            
            # Check safety rules
            safety_decisions = await self._check_safety_rules(context)
            decisions.extend(safety_decisions)
            
            # Record safety evaluation
            self.safety_history.append({
                'timestamp': context.timestamp,
                'decisions_count': len(decisions),
                'emergency_level': max([d.priority.value for d in decisions], default=0)
            })
            
            return decisions
            
        except Exception as e:
            logger.error(f"Error evaluating safety: {e}")
            return []
    
    async def _check_emergency_conditions(self, context: DecisionContext) -> List[Decision]:
        """Check for emergency conditions"""
        decisions = []
        
        # Check obstacle distance
        obstacle_distance = context.sensor_data.get('ultrasonic', 100)
        if obstacle_distance < self.emergency_thresholds['obstacle_distance']:
            decision = Decision(
                decision_type=DecisionType.EMERGENCY,
                action=ActionType.EMERGENCY_STOP,
                priority=DecisionPriority.CRITICAL,
                confidence=0.95,
                reasoning="Obstacle too close - emergency stop required",
                parameters={'distance': obstacle_distance},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        # Check battery level
        battery_level = context.system_state.get('battery_level', 100)
        if battery_level < self.emergency_thresholds['battery_level']:
            decision = Decision(
                decision_type=DecisionType.EMERGENCY,
                action=ActionType.SEND_ALERT,
                priority=DecisionPriority.HIGH,
                confidence=0.9,
                reasoning="Battery level critically low",
                parameters={'battery_level': battery_level},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        # Check temperature
        temperature = context.sensor_data.get('temperature', 25)
        if temperature > self.emergency_thresholds['temperature']:
            decision = Decision(
                decision_type=DecisionType.EMERGENCY,
                action=ActionType.STOP,
                priority=DecisionPriority.HIGH,
                confidence=0.85,
                reasoning="System temperature too high",
                parameters={'temperature': temperature},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        return decisions
    
    async def _check_safety_rules(self, context: DecisionContext) -> List[Decision]:
        """Check safety rules"""
        decisions = []
        
        # Rule: Slow down near obstacles
        obstacle_distance = context.sensor_data.get('ultrasonic', 100)
        if obstacle_distance < 2.0 and context.navigation_state.get('velocity', 0) > 0.5:
            decision = Decision(
                decision_type=DecisionType.SAFETY,
                action=ActionType.CHANGE_SPEED,
                priority=DecisionPriority.MEDIUM,
                confidence=0.8,
                reasoning="Reducing speed near obstacle",
                parameters={'speed': 0.3},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        # Rule: Avoid high-risk areas
        safety_level = context.predictions.get('behavior', {}).get('safety_level', 'low_risk')
        if safety_level == 'high_risk':
            decision = Decision(
                decision_type=DecisionType.SAFETY,
                action=ActionType.TURN_LEFT,
                priority=DecisionPriority.MEDIUM,
                confidence=0.7,
                reasoning="Avoiding high-risk area",
                parameters={'angle': 45},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        return decisions

class EfficiencyOptimizer:
    """Efficiency optimization decision system"""
    
    def __init__(self):
        self.efficiency_rules = []
        self.performance_history = deque(maxlen=100)
        self.optimization_targets = {
            'battery_efficiency': 0.8,
            'movement_efficiency': 0.9,
            'task_completion_time': 60  # seconds
        }
    
    async def optimize_efficiency(self, context: DecisionContext) -> List[Decision]:
        """Optimize robot efficiency"""
        decisions = []
        
        try:
            # Analyze current performance
            performance_analysis = await self._analyze_performance(context)
            
            # Generate optimization decisions
            optimization_decisions = await self._generate_optimization_decisions(context, performance_analysis)
            decisions.extend(optimization_decisions)
            
            # Record performance
            self.performance_history.append({
                'timestamp': context.timestamp,
                'battery_level': context.system_state.get('battery_level', 100),
                'velocity': context.navigation_state.get('velocity', 0),
                'efficiency_score': performance_analysis.get('efficiency_score', 0.5)
            })
            
            return decisions
            
        except Exception as e:
            logger.error(f"Error optimizing efficiency: {e}")
            return []
    
    async def _analyze_performance(self, context: DecisionContext) -> Dict[str, Any]:
        """Analyze current performance"""
        battery_level = context.system_state.get('battery_level', 100)
        velocity = context.navigation_state.get('velocity', 0)
        obstacle_count = context.sensor_data.get('obstacle_count', 0)
        
        # Calculate efficiency score
        battery_efficiency = battery_level / 100.0
        movement_efficiency = 1.0 - (obstacle_count / 10.0)  # Fewer obstacles = better efficiency
        overall_efficiency = (battery_efficiency + movement_efficiency) / 2.0
        
        return {
            'battery_efficiency': battery_efficiency,
            'movement_efficiency': movement_efficiency,
            'overall_efficiency': overall_efficiency,
            'efficiency_score': overall_efficiency
        }
    
    async def _generate_optimization_decisions(self, context: DecisionContext, performance: Dict[str, Any]) -> List[Decision]:
        """Generate optimization decisions"""
        decisions = []
        
        # Optimize battery usage
        if performance['battery_efficiency'] < 0.6:
            decision = Decision(
                decision_type=DecisionType.EFFICIENCY,
                action=ActionType.CHANGE_SPEED,
                priority=DecisionPriority.MEDIUM,
                confidence=0.8,
                reasoning="Optimizing battery usage by reducing speed",
                parameters={'speed': 0.5},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        # Optimize movement efficiency
        if performance['movement_efficiency'] < 0.7:
            decision = Decision(
                decision_type=DecisionType.EFFICIENCY,
                action=ActionType.ADJUST_CAMERA,
                priority=DecisionPriority.LOW,
                confidence=0.6,
                reasoning="Improving obstacle detection for better path planning",
                parameters={'tilt': 0, 'pan': 0},
                timestamp=context.timestamp
            )
            decisions.append(decision)
        
        return decisions

class MaintenanceScheduler:
    """Maintenance scheduling decision system"""
    
    def __init__(self):
        self.maintenance_schedule = {}
        self.maintenance_history = deque(maxlen=100)
        self.maintenance_thresholds = {
            'motor_temperature': 70,
            'battery_cycles': 100,
            'sensor_accuracy': 0.8,
            'error_count': 50
        }
    
    async def schedule_maintenance(self, context: DecisionContext) -> List[Decision]:
        """Schedule maintenance based on predictions"""
        decisions = []
        
        try:
            # Get maintenance predictions
            maintenance_predictions = context.predictions.get('maintenance', [])
            
            for prediction in maintenance_predictions:
                if prediction['urgency'] == 'critical':
                    decision = Decision(
                        decision_type=DecisionType.MAINTENANCE,
                        action=ActionType.SCHEDULE_MAINTENANCE,
                        priority=DecisionPriority.HIGH,
                        confidence=prediction['confidence'],
                        reasoning=f"Critical maintenance required for {prediction['component']}",
                        parameters={
                            'component': prediction['component'],
                            'action': prediction['recommended_action'],
                            'urgency': prediction['urgency']
                        },
                        timestamp=context.timestamp
                    )
                    decisions.append(decision)
                
                elif prediction['urgency'] == 'high':
                    decision = Decision(
                        decision_type=DecisionType.MAINTENANCE,
                        action=ActionType.SEND_ALERT,
                        priority=DecisionPriority.MEDIUM,
                        confidence=prediction['confidence'],
                        reasoning=f"High priority maintenance for {prediction['component']}",
                        parameters={
                            'component': prediction['component'],
                            'action': prediction['recommended_action']
                        },
                        timestamp=context.timestamp
                    )
                    decisions.append(decision)
            
            return decisions
            
        except Exception as e:
            logger.error(f"Error scheduling maintenance: {e}")
            return []

class AutonomousDecisionEngine:
    """Main autonomous decision engine"""
    
    def __init__(self):
        self.safety_manager = SafetyManager()
        self.efficiency_optimizer = EfficiencyOptimizer()
        self.maintenance_scheduler = MaintenanceScheduler()
        
        self.decision_history = deque(maxlen=1000)
        self.decision_rules = []
        self.initialized = False
        self.running = False
        self.decision_task = None
        
        # Decision weights
        self.decision_weights = {
            DecisionType.EMERGENCY: 1.0,
            DecisionType.SAFETY: 0.8,
            DecisionType.MAINTENANCE: 0.6,
            DecisionType.EFFICIENCY: 0.4,
            DecisionType.MOVEMENT: 0.2
        }
    
    async def initialize(self) -> bool:
        """Initialize autonomous decision engine"""
        try:
            # Load decision rules
            await self._load_decision_rules()
            
            self.initialized = True
            logger.info("Autonomous decision engine initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize decision engine: {e}")
            return False
    
    async def start_decision_making(self):
        """Start autonomous decision making"""
        if not self.initialized:
            return
        
        self.running = True
        self.decision_task = asyncio.create_task(self._decision_loop())
        logger.info("Autonomous decision making started")
    
    async def stop_decision_making(self):
        """Stop autonomous decision making"""
        self.running = False
        if self.decision_task:
            self.decision_task.cancel()
            try:
                await self.decision_task
            except asyncio.CancelledError:
                pass
        logger.info("Autonomous decision making stopped")
    
    async def make_decision(self, context: DecisionContext) -> List[Decision]:
        """Make autonomous decisions based on context"""
        if not self.initialized:
            return []
        
        try:
            all_decisions = []
            
            # Get decisions from all subsystems
            safety_decisions = await self.safety_manager.evaluate_safety(context)
            efficiency_decisions = await self.efficiency_optimizer.optimize_efficiency(context)
            maintenance_decisions = await self.maintenance_scheduler.schedule_maintenance(context)
            
            all_decisions.extend(safety_decisions)
            all_decisions.extend(efficiency_decisions)
            all_decisions.extend(maintenance_decisions)
            
            # Apply decision rules
            rule_decisions = await self._apply_decision_rules(context)
            all_decisions.extend(rule_decisions)
            
            # Prioritize and filter decisions
            final_decisions = await self._prioritize_decisions(all_decisions)
            
            # Record decisions
            self.decision_history.append({
                'timestamp': context.timestamp,
                'decisions': len(final_decisions),
                'context': context
            })
            
            return final_decisions
            
        except Exception as e:
            logger.error(f"Error making decisions: {e}")
            return []
    
    async def _load_decision_rules(self):
        """Load decision rules"""
        # Default decision rules
        self.decision_rules = [
            DecisionRule(
                name="obstacle_avoidance",
                condition="ultrasonic < 1.0",
                action=ActionType.TURN_LEFT,
                priority=DecisionPriority.HIGH,
                confidence=0.9,
                parameters={'angle': 30}
            ),
            DecisionRule(
                name="battery_conservation",
                condition="battery_level < 30",
                action=ActionType.CHANGE_SPEED,
                priority=DecisionPriority.MEDIUM,
                confidence=0.8,
                parameters={'speed': 0.3}
            ),
            DecisionRule(
                name="temperature_control",
                condition="temperature > 60",
                action=ActionType.STOP,
                priority=DecisionPriority.HIGH,
                confidence=0.85,
                parameters={}
            )
        ]
    
    async def _apply_decision_rules(self, context: DecisionContext) -> List[Decision]:
        """Apply decision rules"""
        decisions = []
        
        for rule in self.decision_rules:
            if await self._evaluate_condition(rule.condition, context):
                decision = Decision(
                    decision_type=DecisionType.MOVEMENT,
                    action=rule.action,
                    priority=rule.priority,
                    confidence=rule.confidence,
                    reasoning=f"Applied rule: {rule.name}",
                    parameters=rule.parameters,
                    timestamp=context.timestamp
                )
                decisions.append(decision)
        
        return decisions
    
    async def _evaluate_condition(self, condition: str, context: DecisionContext) -> bool:
        """Evaluate decision rule condition"""
        try:
            # Simple condition evaluation
            if "ultrasonic <" in condition:
                threshold = float(condition.split("<")[1].strip())
                return context.sensor_data.get('ultrasonic', 100) < threshold
            
            elif "battery_level <" in condition:
                threshold = float(condition.split("<")[1].strip())
                return context.system_state.get('battery_level', 100) < threshold
            
            elif "temperature >" in condition:
                threshold = float(condition.split(">")[1].strip())
                return context.sensor_data.get('temperature', 25) > threshold
            
            return False
            
        except Exception as e:
            logger.error(f"Error evaluating condition: {e}")
            return False
    
    async def _prioritize_decisions(self, decisions: List[Decision]) -> List[Decision]:
        """Prioritize and filter decisions"""
        if not decisions:
            return []
        
        # Sort by priority and confidence
        decisions.sort(key=lambda d: (d.priority.value, d.confidence), reverse=True)
        
        # Apply weights
        for decision in decisions:
            weight = self.decision_weights.get(decision.decision_type, 0.5)
            decision.confidence *= weight
        
        # Filter by confidence threshold
        filtered_decisions = [d for d in decisions if d.confidence > 0.5]
        
        # Limit number of decisions
        return filtered_decisions[:5]
    
    async def _decision_loop(self):
        """Main decision making loop"""
        while self.running:
            try:
                # This would be called by the main system with real context
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in decision loop: {e}")
                await asyncio.sleep(0.1)
    
    async def get_decision_summary(self) -> Dict[str, Any]:
        """Get decision making summary"""
        if not self.decision_history:
            return {}
        
        recent_decisions = list(self.decision_history)[-10:]
        
        # Count decision types
        decision_types = {}
        for entry in recent_decisions:
            for decision in entry.get('decisions', []):
                decision_type = decision.decision_type.value
                decision_types[decision_type] = decision_types.get(decision_type, 0) + 1
        
        return {
            'total_decisions': len(recent_decisions),
            'decision_types': decision_types,
            'average_confidence': np.mean([d.confidence for entry in recent_decisions for d in entry.get('decisions', [])]),
            'safety_decisions': decision_types.get('safety', 0),
            'efficiency_decisions': decision_types.get('efficiency', 0),
            'maintenance_decisions': decision_types.get('maintenance', 0)
        }

# Global autonomous decision engine instance
autonomous_engine = AutonomousDecisionEngine()
