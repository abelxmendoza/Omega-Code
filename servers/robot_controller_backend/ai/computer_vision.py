"""
Computer Vision ML Models
Advanced object detection, face recognition, and scene understanding
"""

import asyncio
import logging
import numpy as np
import cv2
import json
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
import math
from collections import deque

logger = logging.getLogger(__name__)

class DetectionType(Enum):
    PERSON = "person"
    VEHICLE = "vehicle"
    OBSTACLE = "obstacle"
    FACE = "face"
    OBJECT = "object"

class ConfidenceLevel(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

@dataclass
class BoundingBox:
    """Bounding box representation"""
    x: int
    y: int
    width: int
    height: int
    
    def center(self) -> Tuple[int, int]:
        """Get center point of bounding box"""
        return (self.x + self.width // 2, self.y + self.height // 2)
    
    def area(self) -> int:
        """Get area of bounding box"""
        return self.width * self.height
    
    def iou(self, other: 'BoundingBox') -> float:
        """Calculate Intersection over Union with another bounding box"""
        # Calculate intersection
        x1 = max(self.x, other.x)
        y1 = max(self.y, other.y)
        x2 = min(self.x + self.width, other.x + other.width)
        y2 = min(self.y + self.height, other.y + other.height)
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        union = self.area() + other.area() - intersection
        
        return intersection / union if union > 0 else 0.0

@dataclass
class Detection:
    """Object detection result"""
    detection_type: DetectionType
    bounding_box: BoundingBox
    confidence: float
    class_id: int
    class_name: str
    timestamp: float
    features: Optional[np.ndarray] = None

@dataclass
class FaceDetection(Detection):
    """Face detection with additional features"""
    face_id: Optional[str] = None
    name: Optional[str] = None
    age: Optional[int] = None
    gender: Optional[str] = None
    emotions: Optional[Dict[str, float]] = None
    landmarks: Optional[List[Tuple[int, int]]] = None

class ObjectDetector:
    """Advanced object detection using YOLO and custom models"""
    
    def __init__(self, model_path: str = None, confidence_threshold: float = 0.5):
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.net = None
        self.class_names = []
        self.initialized = False
        
        # COCO class names (default)
        self.coco_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
    
    async def initialize(self) -> bool:
        """Initialize object detector"""
        try:
            if self.model_path:
                # Load custom model
                self.net = cv2.dnn.readNet(self.model_path)
            else:
                # Use default YOLO model
                self.net = cv2.dnn.readNetFromDarknet(
                    "yolov3.cfg", "yolov3.weights"
                )
            
            # Set backend and target
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            self.class_names = self.coco_classes
            self.initialized = True
            
            logger.info("Object detector initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize object detector: {e}")
            return False
    
    async def detect_objects(self, frame: np.ndarray) -> List[Detection]:
        """Detect objects in frame"""
        if not self.initialized:
            return []
        
        try:
            # Preprocess frame
            blob = cv2.dnn.blobFromImage(
                frame, 1/255.0, (416, 416), swapRB=True, crop=False
            )
            
            # Run inference
            self.net.setInput(blob)
            outputs = self.net.forward(self._get_output_layers())
            
            # Process detections
            detections = self._process_detections(outputs, frame.shape)
            
            # Apply non-maximum suppression
            detections = self._apply_nms(detections)
            
            return detections
            
        except Exception as e:
            logger.error(f"Error detecting objects: {e}")
            return []
    
    def _get_output_layers(self) -> List[str]:
        """Get output layer names"""
        layer_names = self.net.getLayerNames()
        return [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
    
    def _process_detections(self, outputs: List, frame_shape: Tuple) -> List[Detection]:
        """Process YOLO outputs"""
        detections = []
        height, width = frame_shape[:2]
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > self.confidence_threshold:
                    # Get bounding box coordinates
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # Convert to top-left coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    bounding_box = BoundingBox(x, y, w, h)
                    
                    # Map class ID to detection type
                    class_name = self.class_names[class_id]
                    detection_type = self._map_class_to_type(class_name)
                    
                    detection_obj = Detection(
                        detection_type=detection_type,
                        bounding_box=bounding_box,
                        confidence=float(confidence),
                        class_id=class_id,
                        class_name=class_name,
                        timestamp=time.time()
                    )
                    
                    detections.append(detection_obj)
        
        return detections
    
    def _map_class_to_type(self, class_name: str) -> DetectionType:
        """Map class name to detection type"""
        if class_name == 'person':
            return DetectionType.PERSON
        elif class_name in ['car', 'truck', 'bus', 'motorcycle', 'bicycle']:
            return DetectionType.VEHICLE
        elif class_name in ['chair', 'couch', 'bed', 'table']:
            return DetectionType.OBSTACLE
        else:
            return DetectionType.OBJECT
    
    def _apply_nms(self, detections: List[Detection], nms_threshold: float = 0.4) -> List[Detection]:
        """Apply Non-Maximum Suppression"""
        if not detections:
            return []
        
        # Sort by confidence
        detections.sort(key=lambda x: x.confidence, reverse=True)
        
        # Apply NMS
        keep = []
        while detections:
            current = detections.pop(0)
            keep.append(current)
            
            # Remove detections with high IoU
            detections = [
                det for det in detections
                if current.bounding_box.iou(det.bounding_box) < nms_threshold
            ]
        
        return keep

class FaceRecognizer:
    """Advanced face recognition system"""
    
    def __init__(self):
        self.face_cascade = None
        self.recognizer = None
        self.face_database = {}
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize face recognizer"""
        try:
            # Load Haar cascade for face detection
            self.face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            )
            
            # Initialize LBPH face recognizer
            self.recognizer = cv2.face.LBPHFaceRecognizer_create()
            
            # Load trained model if available
            try:
                self.recognizer.read("face_model.yml")
            except:
                logger.info("No pre-trained face model found, will train from scratch")
            
            self.initialized = True
            logger.info("Face recognizer initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize face recognizer: {e}")
            return False
    
    async def detect_faces(self, frame: np.ndarray) -> List[FaceDetection]:
        """Detect and recognize faces in frame"""
        if not self.initialized:
            return []
        
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )
            
            face_detections = []
            for (x, y, w, h) in faces:
                # Extract face region
                face_roi = gray[y:y+h, x:x+w]
                
                # Recognize face
                face_id, confidence = self.recognizer.predict(face_roi)
                
                # Create face detection
                bounding_box = BoundingBox(x, y, w, h)
                
                face_detection = FaceDetection(
                    detection_type=DetectionType.FACE,
                    bounding_box=bounding_box,
                    confidence=confidence,
                    class_id=face_id,
                    class_name="face",
                    timestamp=time.time(),
                    face_id=str(face_id) if confidence < 100 else None,
                    name=self.face_database.get(str(face_id), "Unknown")
                )
                
                face_detections.append(face_detection)
            
            return face_detections
            
        except Exception as e:
            logger.error(f"Error detecting faces: {e}")
            return []
    
    async def add_face(self, frame: np.ndarray, name: str) -> bool:
        """Add new face to database"""
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 5)
            
            if len(faces) == 1:
                x, y, w, h = faces[0]
                face_roi = gray[y:y+h, x:x+w]
                
                # Add to database
                face_id = len(self.face_database)
                self.face_database[str(face_id)] = name
                
                # Train recognizer
                self.recognizer.update([face_roi], np.array([face_id]))
                
                # Save model
                self.recognizer.write("face_model.yml")
                
                logger.info(f"Added face for {name} with ID {face_id}")
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Error adding face: {e}")
            return False

class SceneAnalyzer:
    """Advanced scene analysis and understanding"""
    
    def __init__(self):
        self.scene_history = deque(maxlen=100)
        self.scene_patterns = {}
        self.initialized = False
    
    async def initialize(self) -> bool:
        """Initialize scene analyzer"""
        try:
            self.initialized = True
            logger.info("Scene analyzer initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize scene analyzer: {e}")
            return False
    
    async def analyze_scene(self, frame: np.ndarray, detections: List[Detection]) -> Dict[str, Any]:
        """Analyze scene and extract semantic information"""
        try:
            scene_info = {
                'timestamp': time.time(),
                'object_count': len(detections),
                'person_count': len([d for d in detections if d.detection_type == DetectionType.PERSON]),
                'vehicle_count': len([d for d in detections if d.detection_type == DetectionType.VEHICLE]),
                'obstacle_count': len([d for d in detections if d.detection_type == DetectionType.OBSTACLE]),
                'scene_type': self._classify_scene(detections),
                'safety_level': self._assess_safety(detections),
                'movement_patterns': self._analyze_movement_patterns(detections),
                'spatial_layout': self._analyze_spatial_layout(detections, frame.shape)
            }
            
            # Update scene history
            self.scene_history.append(scene_info)
            
            # Update patterns
            self._update_scene_patterns(scene_info)
            
            return scene_info
            
        except Exception as e:
            logger.error(f"Error analyzing scene: {e}")
            return {}
    
    def _classify_scene(self, detections: List[Detection]) -> str:
        """Classify scene type based on detections"""
        if not detections:
            return "empty"
        
        person_count = len([d for d in detections if d.detection_type == DetectionType.PERSON])
        vehicle_count = len([d for d in detections if d.detection_type == DetectionType.VEHICLE])
        obstacle_count = len([d for d in detections if d.detection_type == DetectionType.OBSTACLE])
        
        if person_count > 3:
            return "crowded"
        elif vehicle_count > 2:
            return "traffic"
        elif obstacle_count > 5:
            return "cluttered"
        elif person_count > 0:
            return "populated"
        else:
            return "static"
    
    def _assess_safety(self, detections: List[Detection]) -> str:
        """Assess safety level of scene"""
        person_count = len([d for d in detections if d.detection_type == DetectionType.PERSON])
        vehicle_count = len([d for d in detections if d.detection_type == DetectionType.VEHICLE])
        
        if vehicle_count > 3 or person_count > 5:
            return "high_risk"
        elif vehicle_count > 1 or person_count > 2:
            return "medium_risk"
        else:
            return "low_risk"
    
    def _analyze_movement_patterns(self, detections: List[Detection]) -> Dict[str, Any]:
        """Analyze movement patterns in scene"""
        # Simple movement analysis based on detection positions
        if len(detections) < 2:
            return {"movement": "minimal", "direction": "none"}
        
        # Calculate center of mass
        total_x = sum(d.bounding_box.center()[0] for d in detections)
        total_y = sum(d.bounding_box.center()[1] for d in detections)
        center_x = total_x / len(detections)
        center_y = total_y / len(detections)
        
        return {
            "movement": "moderate",
            "direction": "forward" if center_x > 320 else "backward",
            "center_of_mass": (center_x, center_y)
        }
    
    def _analyze_spatial_layout(self, detections: List[Detection], frame_shape: Tuple) -> Dict[str, Any]:
        """Analyze spatial layout of objects"""
        if not detections:
            return {"layout": "empty"}
        
        height, width = frame_shape[:2]
        
        # Analyze object distribution
        left_objects = len([d for d in detections if d.bounding_box.center()[0] < width // 3])
        center_objects = len([d for d in detections if width // 3 <= d.bounding_box.center()[0] <= 2 * width // 3])
        right_objects = len([d for d in detections if d.bounding_box.center()[0] > 2 * width // 3])
        
        return {
            "layout": "distributed",
            "left_density": left_objects,
            "center_density": center_objects,
            "right_density": right_objects,
            "dominant_side": "left" if left_objects > max(center_objects, right_objects) else
                           "center" if center_objects > right_objects else "right"
        }
    
    def _update_scene_patterns(self, scene_info: Dict[str, Any]):
        """Update scene patterns for learning"""
        scene_type = scene_info['scene_type']
        if scene_type not in self.scene_patterns:
            self.scene_patterns[scene_type] = []
        
        self.scene_patterns[scene_type].append(scene_info)
        
        # Keep only recent patterns
        if len(self.scene_patterns[scene_type]) > 50:
            self.scene_patterns[scene_type] = self.scene_patterns[scene_type][-50:]

class ComputerVisionML:
    """Main computer vision ML system"""
    
    def __init__(self):
        self.object_detector = ObjectDetector()
        self.face_recognizer = FaceRecognizer()
        self.scene_analyzer = SceneAnalyzer()
        
        self.initialized = False
        self.detection_history = deque(maxlen=1000)
        self.running = False
        self.processing_task = None
    
    async def initialize(self) -> bool:
        """Initialize computer vision ML system"""
        try:
            # Initialize all components
            detector_ok = await self.object_detector.initialize()
            face_ok = await self.face_recognizer.initialize()
            scene_ok = await self.scene_analyzer.initialize()
            
            self.initialized = detector_ok and face_ok and scene_ok
            
            if self.initialized:
                logger.info("Computer Vision ML system initialized successfully")
            else:
                logger.error("Failed to initialize some CV ML components")
            
            return self.initialized
            
        except Exception as e:
            logger.error(f"Failed to initialize CV ML system: {e}")
            return False
    
    async def start_processing(self):
        """Start continuous processing"""
        if not self.initialized:
            return
        
        self.running = True
        self.processing_task = asyncio.create_task(self._processing_loop())
        logger.info("CV ML processing started")
    
    async def stop_processing(self):
        """Stop continuous processing"""
        self.running = False
        if self.processing_task:
            self.processing_task.cancel()
            try:
                await self.processing_task
            except asyncio.CancelledError:
                pass
        logger.info("CV ML processing stopped")
    
    async def process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """Process single frame and return results"""
        if not self.initialized:
            return {}
        
        try:
            # Detect objects
            detections = await self.object_detector.detect_objects(frame)
            
            # Detect faces
            face_detections = await self.face_recognizer.detect_faces(frame)
            
            # Combine all detections
            all_detections = detections + face_detections
            
            # Analyze scene
            scene_analysis = await self.scene_analyzer.analyze_scene(frame, all_detections)
            
            # Create result
            result = {
                'timestamp': time.time(),
                'detections': [
                    {
                        'type': d.detection_type.value,
                        'bounding_box': {
                            'x': d.bounding_box.x,
                            'y': d.bounding_box.y,
                            'width': d.bounding_box.width,
                            'height': d.bounding_box.height
                        },
                        'confidence': d.confidence,
                        'class_name': d.class_name,
                        'face_id': getattr(d, 'face_id', None),
                        'name': getattr(d, 'name', None)
                    }
                    for d in all_detections
                ],
                'scene_analysis': scene_analysis,
                'frame_info': {
                    'width': frame.shape[1],
                    'height': frame.shape[0],
                    'channels': frame.shape[2] if len(frame.shape) > 2 else 1
                }
            }
            
            # Store in history
            self.detection_history.append(result)
            
            return result
            
        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            return {}
    
    async def _processing_loop(self):
        """Continuous processing loop"""
        while self.running:
            try:
                # This would process frames from camera feed
                # For now, just wait
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Error in CV ML processing loop: {e}")
                await asyncio.sleep(0.1)
    
    async def get_detection_summary(self) -> Dict[str, Any]:
        """Get summary of recent detections"""
        if not self.detection_history:
            return {}
        
        recent_detections = list(self.detection_history)[-10:]  # Last 10 frames
        
        # Count detection types
        type_counts = {}
        for detection in recent_detections:
            for det in detection['detections']:
                det_type = det['type']
                type_counts[det_type] = type_counts.get(det_type, 0) + 1
        
        return {
            'total_frames': len(recent_detections),
            'detection_counts': type_counts,
            'average_objects_per_frame': sum(len(d['detections']) for d in recent_detections) / len(recent_detections),
            'scene_types': [d['scene_analysis'].get('scene_type', 'unknown') for d in recent_detections]
        }

# Global computer vision ML instance
cv_ml = ComputerVisionML()
