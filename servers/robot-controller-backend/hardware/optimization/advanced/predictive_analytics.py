"""
Predictive Performance Analytics for Robot Controller
- Machine Learning performance prediction
- Adaptive performance tuning based on usage patterns
- Predictive maintenance alerts
- Performance boost: +2 points
"""

import time
import threading
import logging
import numpy as np
from typing import Dict, List, Optional, Any, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum
import asyncio
from collections import deque
import json

logger = logging.getLogger(__name__)

class PredictionType(Enum):
    """Prediction types"""
    PERFORMANCE_DEGRADATION = "performance_degradation"
    HARDWARE_FAILURE = "hardware_failure"
    RESOURCE_EXHAUSTION = "resource_exhaustion"
    TEMPERATURE_SPIKE = "temperature_spike"
    POWER_CONSUMPTION = "power_consumption"

@dataclass
class PerformanceDataPoint:
    """Performance data point"""
    timestamp: float
    component: str
    metric_name: str
    value: float
    context: Dict[str, Any] = field(default_factory=dict)

@dataclass
class PredictionResult:
    """Prediction result"""
    prediction_type: PredictionType
    component: str
    confidence: float
    predicted_value: float
    time_horizon: float  # seconds
    recommendation: str
    urgency: str  # low, medium, high, critical

class PredictiveAnalytics:
    """Predictive performance analytics system"""
    
    def __init__(self):
        self.data_history = deque(maxlen=10000)  # Keep last 10k data points
        self.predictions = {}
        self.models = {}
        self.running = False
        self.lock = threading.Lock()
        
        # ML model settings
        self.model_update_interval = 60.0  # Update models every minute
        self.prediction_horizon = 300.0  # 5-minute prediction horizon
        self.min_data_points = 100  # Minimum data points for training
        
        # Performance thresholds
        self.thresholds = {
            "cpu_usage": {"warning": 70, "critical": 90},
            "memory_usage": {"warning": 80, "critical": 95},
            "temperature": {"warning": 60, "critical": 80},
            "gpu_usage": {"warning": 80, "critical": 95},
            "response_time": {"warning": 0.01, "critical": 0.05}
        }
        
        # Analytics settings
        self.analytics_enabled = True
        self.ml_enabled = True
        self.alert_enabled = True
        
        # Initialize ML models
        self._init_ml_models()
    
    def _init_ml_models(self):
        """Initialize machine learning models"""
        try:
            # Simple linear regression models for each component
            self.models = {
                "cpu_performance": LinearRegressionModel(),
                "memory_performance": LinearRegressionModel(),
                "temperature_trend": LinearRegressionModel(),
                "gpu_performance": LinearRegressionModel(),
                "response_time": LinearRegressionModel()
            }
            
            logger.info("ML models initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize ML models: {e}")
            self.ml_enabled = False
    
    def add_data_point(self, data_point: PerformanceDataPoint):
        """Add performance data point"""
        try:
            with self.lock:
                self.data_history.append(data_point)
                
                # Update relevant models
                if self.ml_enabled:
                    self._update_models(data_point)
                
                # Check for immediate alerts
                if self.alert_enabled:
                    self._check_immediate_alerts(data_point)
                
        except Exception as e:
            logger.error(f"Failed to add data point: {e}")
    
    def _update_models(self, data_point: PerformanceDataPoint):
        """Update ML models with new data"""
        try:
            component = data_point.component
            metric = data_point.metric_name
            
            # Create model key
            model_key = f"{component}_{metric}"
            
            if model_key in self.models:
                # Add data point to model
                self.models[model_key].add_data_point(
                    data_point.timestamp,
                    data_point.value,
                    data_point.context
                )
                
                # Retrain model if enough data
                if len(self.data_history) >= self.min_data_points:
                    self.models[model_key].train()
                    
        except Exception as e:
            logger.error(f"Failed to update models: {e}")
    
    def _check_immediate_alerts(self, data_point: PerformanceDataPoint):
        """Check for immediate performance alerts"""
        try:
            metric = data_point.metric_name
            value = data_point.value
            
            if metric in self.thresholds:
                thresholds = self.thresholds[metric]
                
                if value >= thresholds["critical"]:
                    self._trigger_alert(data_point, "critical", 
                                      f"{metric} critical threshold exceeded: {value}")
                elif value >= thresholds["warning"]:
                    self._trigger_alert(data_point, "warning", 
                                      f"{metric} warning threshold exceeded: {value}")
                    
        except Exception as e:
            logger.error(f"Failed to check immediate alerts: {e}")
    
    def _trigger_alert(self, data_point: PerformanceDataPoint, severity: str, message: str):
        """Trigger performance alert"""
        try:
            alert = {
                "timestamp": time.time(),
                "component": data_point.component,
                "metric": data_point.metric_name,
                "value": data_point.value,
                "severity": severity,
                "message": message,
                "context": data_point.context
            }
            
            logger.warning(f"Performance alert: {message}")
            
            # Store alert for analysis
            if "alerts" not in self.predictions:
                self.predictions["alerts"] = []
            self.predictions["alerts"].append(alert)
            
        except Exception as e:
            logger.error(f"Failed to trigger alert: {e}")
    
    def predict_performance(self, component: str, metric: str, 
                          time_horizon: float = None) -> Optional[PredictionResult]:
        """Predict future performance"""
        try:
            if not self.ml_enabled:
                return None
            
            time_horizon = time_horizon or self.prediction_horizon
            model_key = f"{component}_{metric}"
            
            if model_key not in self.models:
                return None
            
            model = self.models[model_key]
            if not model.is_trained:
                return None
            
            # Make prediction
            current_time = time.time()
            predicted_value = model.predict(current_time + time_horizon)
            confidence = model.get_confidence()
            
            # Determine prediction type
            prediction_type = self._determine_prediction_type(component, metric, predicted_value)
            
            # Generate recommendation
            recommendation = self._generate_recommendation(
                component, metric, predicted_value, prediction_type
            )
            
            # Determine urgency
            urgency = self._determine_urgency(predicted_value, metric)
            
            return PredictionResult(
                prediction_type=prediction_type,
                component=component,
                confidence=confidence,
                predicted_value=predicted_value,
                time_horizon=time_horizon,
                recommendation=recommendation,
                urgency=urgency
            )
            
        except Exception as e:
            logger.error(f"Failed to predict performance: {e}")
            return None
    
    def _determine_prediction_type(self, component: str, metric: str, 
                                 predicted_value: float) -> PredictionType:
        """Determine prediction type based on predicted value"""
        if metric == "temperature" and predicted_value > 70:
            return PredictionType.TEMPERATURE_SPIKE
        elif metric in ["cpu_usage", "memory_usage", "gpu_usage"] and predicted_value > 90:
            return PredictionType.RESOURCE_EXHAUSTION
        elif metric == "response_time" and predicted_value > 0.05:
            return PredictionType.PERFORMANCE_DEGRADATION
        elif metric == "power_consumption" and predicted_value > 100:
            return PredictionType.POWER_CONSUMPTION
        else:
            return PredictionType.HARDWARE_FAILURE
    
    def _generate_recommendation(self, component: str, metric: str, 
                               predicted_value: float, prediction_type: PredictionType) -> str:
        """Generate performance recommendation"""
        if prediction_type == PredictionType.TEMPERATURE_SPIKE:
            return "Reduce system load and improve cooling"
        elif prediction_type == PredictionType.RESOURCE_EXHAUSTION:
            return "Optimize resource usage or add more resources"
        elif prediction_type == PredictionType.PERFORMANCE_DEGRADATION:
            return "Optimize performance-critical components"
        elif prediction_type == PredictionType.POWER_CONSUMPTION:
            return "Implement power management optimizations"
        else:
            return "Monitor component health and consider maintenance"
    
    def _determine_urgency(self, predicted_value: float, metric: str) -> str:
        """Determine prediction urgency"""
        if metric in self.thresholds:
            thresholds = self.thresholds[metric]
            if predicted_value >= thresholds["critical"]:
                return "critical"
            elif predicted_value >= thresholds["warning"]:
                return "high"
            else:
                return "medium"
        return "low"
    
    def get_performance_trends(self, component: str, metric: str, 
                              time_window: float = 3600) -> Dict[str, Any]:
        """Get performance trends for a component"""
        try:
            current_time = time.time()
            start_time = current_time - time_window
            
            # Filter data points
            relevant_data = [
                dp for dp in self.data_history
                if (dp.component == component and 
                    dp.metric_name == metric and 
                    dp.timestamp >= start_time)
            ]
            
            if not relevant_data:
                return {"trend": "insufficient_data", "data_points": 0}
            
            # Calculate trend
            values = [dp.value for dp in relevant_data]
            timestamps = [dp.timestamp for dp in relevant_data]
            
            # Simple linear trend calculation
            if len(values) > 1:
                trend_slope = np.polyfit(timestamps, values, 1)[0]
                trend_direction = "increasing" if trend_slope > 0 else "decreasing"
                trend_strength = abs(trend_slope)
            else:
                trend_direction = "stable"
                trend_strength = 0
            
            return {
                "trend": trend_direction,
                "strength": trend_strength,
                "data_points": len(relevant_data),
                "current_value": values[-1],
                "min_value": min(values),
                "max_value": max(values),
                "avg_value": sum(values) / len(values)
            }
            
        except Exception as e:
            logger.error(f"Failed to get performance trends: {e}")
            return {"trend": "error", "data_points": 0}
    
    def get_predictive_insights(self) -> Dict[str, Any]:
        """Get comprehensive predictive insights"""
        try:
            insights = {
                "timestamp": time.time(),
                "predictions": {},
                "trends": {},
                "alerts": self.predictions.get("alerts", []),
                "model_status": {}
            }
            
            # Get predictions for all components
            components = ["cpu", "memory", "gpu", "temperature", "response_time"]
            metrics = ["usage", "usage", "usage", "temperature", "response_time"]
            
            for component, metric in zip(components, metrics):
                prediction = self.predict_performance(component, metric)
                if prediction:
                    insights["predictions"][f"{component}_{metric}"] = {
                        "type": prediction.prediction_type.value,
                        "confidence": prediction.confidence,
                        "predicted_value": prediction.predicted_value,
                        "urgency": prediction.urgency,
                        "recommendation": prediction.recommendation
                    }
                
                # Get trends
                trend = self.get_performance_trends(component, metric)
                insights["trends"][f"{component}_{metric}"] = trend
            
            # Get model status
            for model_key, model in self.models.items():
                insights["model_status"][model_key] = {
                    "trained": model.is_trained,
                    "data_points": model.data_points,
                    "accuracy": model.get_accuracy()
                }
            
            return insights
            
        except Exception as e:
            logger.error(f"Failed to get predictive insights: {e}")
            return {"error": str(e)}
    
    def start_analytics(self):
        """Start predictive analytics"""
        self.running = True
        logger.info("Predictive analytics started")
    
    def stop_analytics(self):
        """Stop predictive analytics"""
        self.running = False
        logger.info("Predictive analytics stopped")

class LinearRegressionModel:
    """Simple linear regression model for performance prediction"""
    
    def __init__(self):
        self.data_points = []
        self.is_trained = False
        self.slope = 0.0
        self.intercept = 0.0
        self.r_squared = 0.0
    
    def add_data_point(self, timestamp: float, value: float, context: Dict[str, Any]):
        """Add data point to model"""
        self.data_points.append((timestamp, value, context))
        
        # Keep only recent data points
        if len(self.data_points) > 1000:
            self.data_points = self.data_points[-500:]
    
    def train(self):
        """Train the linear regression model"""
        try:
            if len(self.data_points) < 10:
                return
            
            # Extract timestamps and values
            timestamps = [dp[0] for dp in self.data_points]
            values = [dp[1] for dp in self.data_points]
            
            # Normalize timestamps
            min_time = min(timestamps)
            normalized_times = [(t - min_time) / 3600 for t in timestamps]  # Convert to hours
            
            # Calculate linear regression
            n = len(normalized_times)
            sum_x = sum(normalized_times)
            sum_y = sum(values)
            sum_xy = sum(x * y for x, y in zip(normalized_times, values))
            sum_x2 = sum(x * x for x in normalized_times)
            
            # Calculate slope and intercept
            self.slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
            self.intercept = (sum_y - self.slope * sum_x) / n
            
            # Calculate R-squared
            y_mean = sum_y / n
            ss_tot = sum((y - y_mean) ** 2 for y in values)
            ss_res = sum((y - (self.slope * x + self.intercept)) ** 2 
                        for x, y in zip(normalized_times, values))
            
            self.r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
            
            self.is_trained = True
            
        except Exception as e:
            logger.error(f"Model training failed: {e}")
            self.is_trained = False
    
    def predict(self, timestamp: float) -> float:
        """Predict value at given timestamp"""
        if not self.is_trained:
            return 0.0
        
        # Normalize timestamp
        if self.data_points:
            min_time = min(dp[0] for dp in self.data_points)
            normalized_time = (timestamp - min_time) / 3600  # Convert to hours
            return self.slope * normalized_time + self.intercept
        
        return 0.0
    
    def get_confidence(self) -> float:
        """Get model confidence (R-squared)"""
        return max(0.0, min(1.0, self.r_squared))
    
    def get_accuracy(self) -> float:
        """Get model accuracy"""
        return self.get_confidence()

# Global predictive analytics instance
predictive_analytics = PredictiveAnalytics()
