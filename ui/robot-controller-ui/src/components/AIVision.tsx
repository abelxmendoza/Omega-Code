import React, { useState, useEffect, useRef, useCallback } from 'react';

interface AIVisionProps {
  onObjectDetected?: (objects: DetectedObject[]) => void;
  onFaceRecognized?: (faces: DetectedFace[]) => void;
  onMotionDetected?: (motion: MotionData) => void;
  enabled?: boolean;
  confidence?: number;
}

interface DetectedObject {
  id: string;
  label: string;
  confidence: number;
  bbox: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
  timestamp: number;
}

interface DetectedFace {
  id: string;
  name?: string;
  confidence: number;
  bbox: {
    x: number;
    y: number;
    width: number;
    height: number;
  };
  emotions?: {
    happy: number;
    sad: number;
    angry: number;
    surprised: number;
  };
  timestamp: number;
}

interface MotionData {
  regions: Array<{
    x: number;
    y: number;
    width: number;
    height: number;
    intensity: number;
  }>;
  timestamp: number;
}

const AIVision: React.FC<AIVisionProps> = ({
  onObjectDetected,
  onFaceRecognized,
  onMotionDetected,
  enabled = true,
  confidence = 0.7
}) => {
  const [isActive, setIsActive] = useState(false);
  const [detectedObjects, setDetectedObjects] = useState<DetectedObject[]>([]);
  const [detectedFaces, setDetectedFaces] = useState<DetectedFace[]>([]);
  const [motionData, setMotionData] = useState<MotionData | null>(null);
  const [aiStatus, setAiStatus] = useState<'idle' | 'processing' | 'error'>('idle');
  
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const streamRef = useRef<MediaStream | null>(null);

  // Simulate AI processing (in real implementation, this would call AI APIs)
  const simulateObjectDetection = useCallback(() => {
    const objects: DetectedObject[] = [
      {
        id: 'obj1',
        label: 'person',
        confidence: 0.95,
        bbox: { x: 100, y: 50, width: 80, height: 120 },
        timestamp: Date.now()
      },
      {
        id: 'obj2',
        label: 'car',
        confidence: 0.87,
        bbox: { x: 200, y: 150, width: 120, height: 60 },
        timestamp: Date.now()
      }
    ];
    
    setDetectedObjects(objects);
    onObjectDetected?.(objects);
  }, [onObjectDetected]);

  const simulateFaceRecognition = useCallback(() => {
    const faces: DetectedFace[] = [
      {
        id: 'face1',
        name: 'John Doe',
        confidence: 0.92,
        bbox: { x: 120, y: 60, width: 60, height: 80 },
        emotions: {
          happy: 0.8,
          sad: 0.1,
          angry: 0.05,
          surprised: 0.05
        },
        timestamp: Date.now()
      }
    ];
    
    setDetectedFaces(faces);
    onFaceRecognized?.(faces);
  }, [onFaceRecognized]);

  const simulateMotionDetection = useCallback(() => {
    const motion: MotionData = {
      regions: [
        {
          x: 150,
          y: 100,
          width: 40,
          height: 40,
          intensity: 0.7
        }
      ],
      timestamp: Date.now()
    };
    
    setMotionData(motion);
    onMotionDetected?.(motion);
  }, [onMotionDetected]);

  const startCamera = async () => {
    try {
      setAiStatus('processing');
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { width: 640, height: 480 }
      });
      
      streamRef.current = stream;
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
      
      setIsActive(true);
      setAiStatus('idle');
    } catch (error) {
      console.error('Error accessing camera:', error);
      setAiStatus('error');
    }
  };

  const stopCamera = () => {
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(track => track.stop());
      streamRef.current = null;
    }
    setIsActive(false);
    setDetectedObjects([]);
    setDetectedFaces([]);
    setMotionData(null);
  };

  const toggleAI = () => {
    if (isActive) {
      stopCamera();
    } else {
      startCamera();
    }
  };

  // Simulate AI processing every 2 seconds
  useEffect(() => {
    if (!isActive || !enabled) return;

    const interval = setInterval(() => {
      simulateObjectDetection();
      simulateFaceRecognition();
      simulateMotionDetection();
    }, 2000);

    return () => clearInterval(interval);
  }, [isActive, enabled, simulateObjectDetection, simulateFaceRecognition, simulateMotionDetection]);

  const drawDetections = useCallback(() => {
    const canvas = canvasRef.current;
    const video = videoRef.current;
    if (!canvas || !video) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size to match video
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw object detections
    detectedObjects.forEach(obj => {
      if (obj.confidence >= confidence) {
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 2;
        ctx.strokeRect(obj.bbox.x, obj.bbox.y, obj.bbox.width, obj.bbox.height);
        
        ctx.fillStyle = '#10b981';
        ctx.font = '12px Arial';
        ctx.fillText(
          `${obj.label} (${(obj.confidence * 100).toFixed(1)}%)`,
          obj.bbox.x,
          obj.bbox.y - 5
        );
      }
    });

    // Draw face detections
    detectedFaces.forEach(face => {
      if (face.confidence >= confidence) {
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 2;
        ctx.strokeRect(face.bbox.x, face.bbox.y, face.bbox.width, face.bbox.height);
        
        ctx.fillStyle = '#3b82f6';
        ctx.font = '12px Arial';
        ctx.fillText(
          `${face.name || 'Unknown'} (${(face.confidence * 100).toFixed(1)}%)`,
          face.bbox.x,
          face.bbox.y - 5
        );
      }
    });

    // Draw motion regions
    if (motionData) {
      motionData.regions.forEach(region => {
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.strokeRect(region.x, region.y, region.width, region.height);
        
        ctx.fillStyle = `rgba(245, 158, 11, ${region.intensity})`;
        ctx.fillRect(region.x, region.y, region.width, region.height);
      });
    }
  }, [detectedObjects, detectedFaces, motionData, confidence]);

  useEffect(() => {
    if (isActive) {
      const interval = setInterval(drawDetections, 100);
      return () => clearInterval(interval);
    }
  }, [isActive, detectedObjects, detectedFaces, motionData, drawDetections]);

  return (
    <div className="p-4 bg-gray-800 rounded-lg">
      <h3 className="text-white font-bold mb-4">AI Computer Vision</h3>
      
      <div className="space-y-4">
        {/* Controls */}
        <div className="flex gap-2">
          <button
            className={`px-4 py-2 rounded font-semibold transition-all duration-200 hover:scale-105 ${
              isActive 
                ? 'bg-red-600 hover:bg-red-700 text-white' 
                : 'bg-green-600 hover:bg-green-700 text-white'
            }`}
            onClick={toggleAI}
          >
            {isActive ? '🛑 Stop AI' : '🤖 Start AI'}
          </button>
          
          <div className={`px-3 py-2 rounded text-sm ${
            aiStatus === 'processing' ? 'bg-yellow-600 text-white' :
            aiStatus === 'error' ? 'bg-red-600 text-white' :
            'bg-gray-600 text-gray-300'
          }`}>
            Status: {aiStatus}
          </div>
        </div>

        {/* Camera Feed */}
        <div className="relative">
          <video
            ref={videoRef}
            autoPlay
            muted
            className="w-full h-64 bg-gray-900 rounded"
            style={{ display: isActive ? 'block' : 'none' }}
          />
          <canvas
            ref={canvasRef}
            className="absolute top-0 left-0 w-full h-64 pointer-events-none"
            style={{ display: isActive ? 'block' : 'none' }}
          />
          {!isActive && (
            <div className="w-full h-64 bg-gray-900 rounded flex items-center justify-center">
              <div className="text-gray-400 text-center">
                <div className="text-4xl mb-2">📹</div>
                <div>Camera not active</div>
              </div>
            </div>
          )}
        </div>

        {/* Detection Results */}
        {isActive && (
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            {/* Objects */}
            <div className="bg-gray-700 p-3 rounded">
              <h4 className="text-white font-semibold mb-2">Objects ({detectedObjects.length})</h4>
              <div className="space-y-1">
                {detectedObjects.map(obj => (
                  <div key={obj.id} className="text-sm text-gray-300">
                    <span className="text-green-400">●</span> {obj.label} ({(obj.confidence * 100).toFixed(1)}%)
                  </div>
                ))}
              </div>
            </div>

            {/* Faces */}
            <div className="bg-gray-700 p-3 rounded">
              <h4 className="text-white font-semibold mb-2">Faces ({detectedFaces.length})</h4>
              <div className="space-y-1">
                {detectedFaces.map(face => (
                  <div key={face.id} className="text-sm text-gray-300">
                    <span className="text-blue-400">●</span> {face.name || 'Unknown'} ({(face.confidence * 100).toFixed(1)}%)
                  </div>
                ))}
              </div>
            </div>

            {/* Motion */}
            <div className="bg-gray-700 p-3 rounded">
              <h4 className="text-white font-semibold mb-2">Motion</h4>
              <div className="text-sm text-gray-300">
                {motionData ? (
                  <div>
                    <span className="text-yellow-400">●</span> {motionData.regions.length} region(s) detected
                  </div>
                ) : (
                  <div>No motion detected</div>
                )}
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default AIVision;
