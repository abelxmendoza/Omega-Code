"""
Predictive Analytics System
Machine learning for predictive maintenance, behavior analysis, and optimization
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
from sklearn.ensemble import RandomForestRegressor, IsolationForest
from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans
import pickle

logger = logging.getLogger(__name__)

class PredictionType(Enum):
    MAINTENANCE = "maintenance"
    PERFORMANCE = "performance"
    BEHAVIOR = "behavior"
    FAILURE = "failure"
    OPTIMIZATION = "optimization"

class AlertLevel(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

@dataclass
class PredictionResult:
    """Prediction result"""
    prediction_type: PredictionType
    predicted_value: float
    confidence: float
    timestamp: float
    features_used: List[str]
    model_name: str
    alert_level: AlertLevel = AlertLevel.LOW

@dataclass
class MaintenancePrediction:
    """Maintenance prediction result"""
    component: str
    predicted_failure_time: float  # hours
    confidence: float
    recommended_action: str
    urgency: AlertLevel
    cost_estimate: float

@dataclass
class PerformancePrediction:
    """Performance prediction result"""
    metric: str
    predicted_value: float
    trend: str  # "improving", "stable", "declining"
    confidence: float
    recommendations: List[str]

class DataCollector:
    """Collects and preprocesses data for ML models"""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.sensor_data = deque(maxlen=window_size)
        self.motor_data = deque(maxlen=window_size)
        self.navigation_data = deque(maxlen=window_size)
        self.system_data = deque(maxlen=window_size)
        
        self.feature_names = []
        self.scaler = StandardScaler()
        self.scaler_fitted = False
    
    def add_sensor_data(self, data: Dict[str, Any]):
        """Add sensor data"""
        self.sensor_data.append({
            'timestamp': time.time(),
            'ultrasonic': data.get('ultrasonic', 0),
            'temperature': data.get('temperature', 0),
            'humidity': data.get('humidity', 0),
            'light': data.get('light', 0)
        })
    
    def add_motor_data(self, data: Dict[str, Any]):
        """Add motor data"""
        self.motor_data.append({
            'timestamp': time.time(),
            'left_speed': data.get('left_speed', 0),
            'right_speed': data.get('right_speed', 0),
            'left_current': data.get('left_current', 0),
            'right_current': data.get('right_current', 0),
            'left_temperature': data.get('left_temperature', 0),
            'right_temperature': data.get('right_temperature', 0)
        })
    
    def add_navigation_data(self, data: Dict[str, Any]):
        """Add navigation data"""
        self.navigation_data.append({
            'timestamp': time.time(),
            'position_x': data.get('position_x', 0),
            'position_y': data.get('position_y', 0),
            'heading': data.get('heading', 0),
            'velocity': data.get('velocity', 0),
            'obstacle_count': data.get('obstacle_count', 0),
            'path_length': data.get('path_length', 0)
        })
    
    def add_system_data(self, data: Dict[str, Any]):
        """Add system data"""
        self.system_data.append({
            'timestamp': time.time(),
            'cpu_usage': data.get('cpu_usage', 0),
            'memory_usage': data.get('memory_usage', 0),
            'battery_level': data.get('battery_level', 0),
            'uptime': data.get('uptime', 0),
            'error_count': data.get('error_count', 0)
        })
    
    def get_features(self) -> np.ndarray:
        """Extract features from collected data"""
        try:
            features = []
            
            # Sensor features
            if self.sensor_data:
                recent_sensors = list(self.sensor_data)[-10:]  # Last 10 readings
                sensor_features = [
                    np.mean([s['ultrasonic'] for s in recent_sensors]),
                    np.std([s['ultrasonic'] for s in recent_sensors]),
                    np.mean([s['temperature'] for s in recent_sensors]),
                    np.std([s['temperature'] for s in recent_sensors]),
                    np.mean([s['humidity'] for s in recent_sensors]),
                    np.mean([s['light'] for s in recent_sensors])
                ]
                features.extend(sensor_features)
            else:
                features.extend([0] * 6)
            
            # Motor features
            if self.motor_data:
                recent_motors = list(self.motor_data)[-10:]
                motor_features = [
                    np.mean([m['left_speed'] for m in recent_motors]),
                    np.mean([m['right_speed'] for m in recent_motors]),
                    np.mean([m['left_current'] for m in recent_motors]),
                    np.mean([m['right_current'] for m in recent_motors]),
                    np.mean([m['left_temperature'] for m in recent_motors]),
                    np.mean([m['right_temperature'] for m in recent_motors])
                ]
                features.extend(motor_features)
            else:
                features.extend([0] * 6)
            
            # Navigation features
            if self.navigation_data:
                recent_nav = list(self.navigation_data)[-10:]
                nav_features = [
                    np.mean([n['velocity'] for n in recent_nav]),
                    np.std([n['velocity'] for n in recent_nav]),
                    np.mean([n['obstacle_count'] for n in recent_nav]),
                    np.mean([n['path_length'] for n in recent_nav])
                ]
                features.extend(nav_features)
            else:
                features.extend([0] * 4)
            
            # System features
            if self.system_data:
                recent_system = list(self.system_data)[-5:]
                system_features = [
                    np.mean([s['cpu_usage'] for s in recent_system]),
                    np.mean([s['memory_usage'] for s in recent_system]),
                    np.mean([s['battery_level'] for s in recent_system]),
                    np.sum([s['error_count'] for s in recent_system])
                ]
                features.extend(system_features)
            else:
                features.extend([0] * 4)
            
            return np.array(features)
            
        except Exception as e:
            logger.error(f"Error extracting features: {e}")
            return np.zeros(20)  # Default feature vector
    
    def get_feature_names(self) -> List[str]:
        """Get feature names"""
        if not self.feature_names:
            self.feature_names = [
                'ultrasonic_mean', 'ultrasonic_std', 'temperature_mean', 'temperature_std',
                'humidity_mean', 'light_mean',
                'left_speed_mean', 'right_speed_mean', 'left_current_mean', 'right_current_mean',
                'left_temp_mean', 'right_temp_mean',
                'velocity_mean', 'velocity_std', 'obstacle_count_mean', 'path_length_mean',
                'cpu_usage_mean', 'memory_usage_mean', 'battery_level_mean', 'error_count_sum'
            ]
        return self.feature_names

class MaintenancePredictor:
    """Predictive maintenance system"""
    
    def __init__(self):
        self.models = {}
        self.component_thresholds = {
            'motor_left': {'temperature': 80, 'current': 2.0, 'vibration': 5.0},
            'motor_right': {'temperature': 80, 'current': 2.0, 'vibration': 5.0},
            'battery': {'voltage': 10.0, 'current': 1.0, 'temperature': 60},
            'sensors': {'ultrasonic': 0.1, 'temperature': 0.1, 'humidity': 0.1}
        }
        self.maintenance_history = deque(maxlen=1000)
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize maintenance predictor"""
        try:
            # Initialize models for each component
            for component in self.component_thresholds.keys():
                self.models[component] = {
                    'regressor': RandomForestRegressor(n_estimators=100, random_state=42),
                    'anomaly_detector': IsolationForest(contamination=0.1, random_state=42),
                    'trained': False
                }
            
            self.initialized = True
            logger.info("Maintenance predictor initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize maintenance predictor: {e}")
            return False
    
    async def predict_maintenance(self, features: np.ndarray) -> List[MaintenancePrediction]:
        """Predict maintenance needs"""
        if not self.initialized:
            return []
        
        try:
            predictions = []
            
            for component, model_info in self.models.items():
                if not model_info['trained']:
                    continue
                
                # Predict failure time
                failure_time = model_info['regressor'].predict([features])[0]
                
                # Detect anomalies
                anomaly_score = model_info['anomaly_detector'].decision_function([features])[0]
                
                # Calculate confidence
                confidence = max(0, min(1, (anomaly_score + 1) / 2))
                
                # Determine urgency
                urgency = self._determine_urgency(failure_time, anomaly_score)
                
                # Get recommendations
                recommendations = self._get_recommendations(component, features)
                
                prediction = MaintenancePrediction(
                    component=component,
                    predicted_failure_time=failure_time,
                    confidence=confidence,
                    recommended_action=recommendations[0],
                    urgency=urgency,
                    cost_estimate=self._estimate_cost(component, urgency)
                )
                
                predictions.append(prediction)
            
            return predictions
            
        except Exception as e:
            logger.error(f"Error predicting maintenance: {e}")
            return []
    
    def _determine_urgency(self, failure_time: float, anomaly_score: float) -> AlertLevel:
        """Determine urgency level"""
        if failure_time < 24 or anomaly_score < -0.5:
            return AlertLevel.CRITICAL
        elif failure_time < 72 or anomaly_score < -0.3:
            return AlertLevel.HIGH
        elif failure_time < 168 or anomaly_score < -0.1:
            return AlertLevel.MEDIUM
        else:
            return AlertLevel.LOW
    
    def _get_recommendations(self, component: str, features: np.ndarray) -> List[str]:
        """Get maintenance recommendations"""
        recommendations = []
        
        if component == 'motor_left' or component == 'motor_right':
            if features[8] > 1.5:  # High current
                recommendations.append("Check motor load and reduce if possible")
            if features[10] > 70:  # High temperature
                recommendations.append("Clean motor cooling system")
            if features[6] > 80:  # High speed
                recommendations.append("Reduce operating speed")
        
        elif component == 'battery':
            if features[18] < 20:  # Low battery
                recommendations.append("Charge battery immediately")
            if features[17] > 80:  # High memory usage
                recommendations.append("Restart system to free memory")
        
        elif component == 'sensors':
            if features[0] < 5:  # Low ultrasonic reading
                recommendations.append("Clean ultrasonic sensor")
            if features[2] > 40:  # High temperature
                recommendations.append("Check sensor mounting and cooling")
        
        if not recommendations:
            recommendations.append("Continue monitoring")
        
        return recommendations
    
    def _estimate_cost(self, component: str, urgency: AlertLevel) -> float:
        """Estimate maintenance cost"""
        base_costs = {
            'motor_left': 50,
            'motor_right': 50,
            'battery': 100,
            'sensors': 25
        }
        
        urgency_multipliers = {
            AlertLevel.LOW: 1.0,
            AlertLevel.MEDIUM: 1.2,
            AlertLevel.HIGH: 1.5,
            AlertLevel.CRITICAL: 2.0
        }
        
        return base_costs.get(component, 50) * urgency_multipliers[urgency]

class PerformancePredictor:
    """Performance prediction and optimization system"""
    
    def __init__(self):
        self.performance_models = {}
        self.baseline_performance = {}
        self.optimization_history = deque(maxlen=500)
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize performance predictor"""
        try:
            # Initialize models for different performance metrics
            metrics = ['efficiency', 'speed', 'accuracy', 'battery_life']
            
            for metric in metrics:
                self.performance_models[metric] = {
                    'regressor': RandomForestRegressor(n_estimators=50, random_state=42),
                    'trained': False
                }
            
            # Set baseline performance
            self.baseline_performance = {
                'efficiency': 0.8,
                'speed': 1.0,
                'accuracy': 0.9,
                'battery_life': 8.0
            }
            
            self.initialized = True
            logger.info("Performance predictor initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize performance predictor: {e}")
            return False
    
    async def predict_performance(self, features: np.ndarray) -> List[PerformancePrediction]:
        """Predict performance metrics"""
        if not self.initialized:
            return []
        
        try:
            predictions = []
            
            for metric, model_info in self.performance_models.items():
                if not model_info['trained']:
                    # Use baseline if model not trained
                    predicted_value = self.baseline_performance[metric]
                    confidence = 0.5
                else:
                    predicted_value = model_info['regressor'].predict([features])[0]
                    confidence = 0.8
                
                # Determine trend
                trend = self._determine_trend(metric, predicted_value)
                
                # Get recommendations
                recommendations = self._get_performance_recommendations(metric, predicted_value, features)
                
                prediction = PerformancePrediction(
                    metric=metric,
                    predicted_value=predicted_value,
                    trend=trend,
                    confidence=confidence,
                    recommendations=recommendations
                )
                
                predictions.append(prediction)
            
            return predictions
            
        except Exception as e:
            logger.error(f"Error predicting performance: {e}")
            return []
    
    def _determine_trend(self, metric: str, predicted_value: float) -> str:
        """Determine performance trend"""
        baseline = self.baseline_performance[metric]
        
        if predicted_value > baseline * 1.1:
            return "improving"
        elif predicted_value < baseline * 0.9:
            return "declining"
        else:
            return "stable"
    
    def _get_performance_recommendations(self, metric: str, predicted_value: float, features: np.ndarray) -> List[str]:
        """Get performance optimization recommendations"""
        recommendations = []
        
        if metric == 'efficiency':
            if predicted_value < 0.7:
                recommendations.append("Optimize motor control algorithms")
                recommendations.append("Reduce unnecessary movements")
            if features[6] > 80:  # High speed
                recommendations.append("Reduce operating speed for better efficiency")
        
        elif metric == 'speed':
            if predicted_value < 0.8:
                recommendations.append("Check motor performance")
                recommendations.append("Optimize path planning")
            if features[12] > 3:  # High obstacle count
                recommendations.append("Improve obstacle avoidance algorithms")
        
        elif metric == 'accuracy':
            if predicted_value < 0.85:
                recommendations.append("Calibrate sensors")
                recommendations.append("Improve sensor fusion algorithms")
            if features[0] < 10:  # Low ultrasonic reading
                recommendations.append("Clean and calibrate ultrasonic sensor")
        
        elif metric == 'battery_life':
            if predicted_value < 6.0:
                recommendations.append("Reduce power consumption")
                recommendations.append("Optimize sleep modes")
            if features[17] > 70:  # High memory usage
                recommendations.append("Optimize memory usage")
        
        if not recommendations:
            recommendations.append("Performance is within acceptable range")
        
        return recommendations

class BehaviorAnalyzer:
    """Behavior analysis and pattern recognition"""
    
    def __init__(self):
        self.behavior_patterns = {}
        self.clustering_model = KMeans(n_clusters=5, random_state=42)
        self.behavior_history = deque(maxlen=1000)
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize behavior analyzer"""
        try:
            self.initialized = True
            logger.info("Behavior analyzer initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize behavior analyzer: {e}")
            return False
    
    async def analyze_behavior(self, features: np.ndarray) -> Dict[str, Any]:
        """Analyze robot behavior patterns"""
        if not self.initialized:
            return {}
        
        try:
            # Add to behavior history
            self.behavior_history.append(features)
            
            if len(self.behavior_history) < 10:
                return {"status": "insufficient_data"}
            
            # Cluster behavior patterns
            behavior_data = np.array(list(self.behavior_history))
            clusters = self.clustering_model.fit_predict(behavior_data)
            
            # Analyze current behavior
            current_cluster = clusters[-1]
            cluster_count = np.bincount(clusters)
            cluster_percentage = cluster_count[current_cluster] / len(clusters) * 100
            
            # Detect anomalies
            anomalies = self._detect_behavior_anomalies(features)
            
            # Analyze patterns
            patterns = self._analyze_patterns(features)
            
            return {
                'current_cluster': int(current_cluster),
                'cluster_percentage': float(cluster_percentage),
                'anomalies': anomalies,
                'patterns': patterns,
                'behavior_stability': self._calculate_stability(clusters),
                'recommendations': self._get_behavior_recommendations(features, anomalies)
            }
            
        except Exception as e:
            logger.error(f"Error analyzing behavior: {e}")
            return {}
    
    def _detect_behavior_anomalies(self, features: np.ndarray) -> List[str]:
        """Detect behavioral anomalies"""
        anomalies = []
        
        # Check for unusual patterns
        if features[6] > 90 and features[7] < 10:  # High left speed, low right speed
            anomalies.append("asymmetric_motor_usage")
        
        if features[12] > 5:  # High obstacle count
            anomalies.append("high_obstacle_density")
        
        if features[18] < 15:  # Low battery
            anomalies.append("low_battery_level")
        
        if features[17] > 85:  # High memory usage
            anomalies.append("high_memory_usage")
        
        return anomalies
    
    def _analyze_patterns(self, features: np.ndarray) -> Dict[str, Any]:
        """Analyze behavior patterns"""
        return {
            'movement_pattern': 'linear' if features[6] == features[7] else 'curved',
            'obstacle_avoidance': 'active' if features[12] > 2 else 'passive',
            'power_usage': 'high' if features[18] < 30 else 'normal',
            'system_load': 'high' if features[16] > 70 else 'normal'
        }
    
    def _calculate_stability(self, clusters: np.ndarray) -> float:
        """Calculate behavior stability"""
        if len(clusters) < 10:
            return 0.5
        
        # Calculate how often the cluster changes
        cluster_changes = np.sum(np.diff(clusters) != 0)
        stability = 1.0 - (cluster_changes / len(clusters))
        
        return max(0, min(1, stability))
    
    def _get_behavior_recommendations(self, features: np.ndarray, anomalies: List[str]) -> List[str]:
        """Get behavior optimization recommendations"""
        recommendations = []
        
        if 'asymmetric_motor_usage' in anomalies:
            recommendations.append("Check motor calibration and balance")
        
        if 'high_obstacle_density' in anomalies:
            recommendations.append("Improve obstacle avoidance algorithms")
        
        if 'low_battery_level' in anomalies:
            recommendations.append("Implement power-saving mode")
        
        if 'high_memory_usage' in anomalies:
            recommendations.append("Optimize memory usage and restart if necessary")
        
        if not recommendations:
            recommendations.append("Behavior is within normal parameters")
        
        return recommendations

class PredictiveAnalytics:
    """Main predictive analytics system"""
    
    def __init__(self):
        self.data_collector = DataCollector()
        self.maintenance_predictor = MaintenancePredictor()
        self.performance_predictor = PerformancePredictor()
        self.behavior_analyzer = BehaviorAnalyzer()
        
        self.initialized = False
        self.running = False
        self.analysis_task = None
        self.predictions_cache = {}
    
    async def initialize(self) -> bool:
        """Initialize predictive analytics system"""
        try:
            # Initialize all components
            maintenance_ok = await self.maintenance_predictor.initialize()
            performance_ok = await self.performance_predictor.initialize()
            behavior_ok = await self.behavior_analyzer.initialize()
            
            self.initialized = maintenance_ok and performance_ok and behavior_ok
            
            if self.initialized:
                logger.info("Predictive analytics system initialized successfully")
            else:
                logger.error("Failed to initialize some predictive analytics components")
            
            return self.initialized
            
        except Exception as e:
            logger.error(f"Failed to initialize predictive analytics: {e}")
            return False
    
    async def start_analysis(self):
        """Start continuous analysis"""
        if not self.initialized:
            return
        
        self.running = True
        self.analysis_task = asyncio.create_task(self._analysis_loop())
        logger.info("Predictive analytics started")
    
    async def stop_analysis(self):
        """Stop continuous analysis"""
        self.running = False
        if self.analysis_task:
            self.analysis_task.cancel()
            try:
                await self.analysis_task
            except asyncio.CancelledError:
                pass
        logger.info("Predictive analytics stopped")
    
    async def add_data(self, data_type: str, data: Dict[str, Any]):
        """Add data for analysis"""
        try:
            if data_type == 'sensor':
                self.data_collector.add_sensor_data(data)
            elif data_type == 'motor':
                self.data_collector.add_motor_data(data)
            elif data_type == 'navigation':
                self.data_collector.add_navigation_data(data)
            elif data_type == 'system':
                self.data_collector.add_system_data(data)
            
        except Exception as e:
            logger.error(f"Error adding data: {e}")
    
    async def get_predictions(self) -> Dict[str, Any]:
        """Get current predictions"""
        try:
            features = self.data_collector.get_features()
            
            # Get predictions from all systems
            maintenance_predictions = await self.maintenance_predictor.predict_maintenance(features)
            performance_predictions = await self.performance_predictor.predict_performance(features)
            behavior_analysis = await self.behavior_analyzer.analyze_behavior(features)
            
            # Cache predictions
            self.predictions_cache = {
                'timestamp': time.time(),
                'maintenance': [
                    {
                        'component': p.component,
                        'predicted_failure_time': p.predicted_failure_time,
                        'confidence': p.confidence,
                        'recommended_action': p.recommended_action,
                        'urgency': p.urgency.value,
                        'cost_estimate': p.cost_estimate
                    }
                    for p in maintenance_predictions
                ],
                'performance': [
                    {
                        'metric': p.metric,
                        'predicted_value': p.predicted_value,
                        'trend': p.trend,
                        'confidence': p.confidence,
                        'recommendations': p.recommendations
                    }
                    for p in performance_predictions
                ],
                'behavior': behavior_analysis,
                'features_used': self.data_collector.get_feature_names()
            }
            
            return self.predictions_cache
            
        except Exception as e:
            logger.error(f"Error getting predictions: {e}")
            return {}
    
    async def _analysis_loop(self):
        """Continuous analysis loop"""
        while self.running:
            try:
                # Update predictions every 30 seconds
                await self.get_predictions()
                await asyncio.sleep(30)
                
            except Exception as e:
                logger.error(f"Error in analysis loop: {e}")
                await asyncio.sleep(30)

# Global predictive analytics instance
predictive_analytics = PredictiveAnalytics()
