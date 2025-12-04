import React from 'react';
// FIX: Lucide-react does not export `Gpu`. Replace with a valid icon.
import {
  Info,
  CheckCircle2,
  XCircle,
  AlertTriangle,
  Zap,
  Cpu,
  // Gpu,   <-- removed
  Camera,
  Cpu as GpuIcon // temporary alias if you need a "GPU-like" icon
} from 'lucide-react';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
  DialogTrigger,
} from '@/components/ui/dialog';
import { Badge } from '@/components/ui/badge';
import { useCapabilityContext } from '@/context/CapabilityContext';
import { CapabilityStatus } from './CapabilityStatus';

const PROFILE_INFO = {
  mac: {
    name: 'Light Mode',
    description: 'MacBook + Raspberry Pi',
    color: 'bg-gray-500',
    icon: '💻',
    limitations: [
      'No GPU acceleration',
      'No ML inference',
      'Limited to basic computer vision',
      'No SLAM capabilities',
    ],
    recommendations: [
      'Use Jetson Orin Nano for full autonomy',
      'Upgrade to Lenovo Linux for SLAM development',
    ],
  },
  lenovo: {
    name: 'Dev Mode',
    description: 'Lenovo Linux + Raspberry Pi',
    color: 'bg-blue-500',
    icon: '🖥️',
    limitations: [
      'No GPU acceleration',
      'ML inference is CPU-based (slow)',
      'SLAM available but limited FPS',
    ],
    recommendations: [
      'Use Jetson Orin Nano for real-time ML',
      'Current setup good for development and testing',
    ],
  },
  jetson: {
    name: 'Omega Mode',
    description: 'Jetson Orin Nano + Raspberry Pi',
    color: 'bg-green-500',
    icon: '🚀',
    limitations: [],
    recommendations: [
      'Full autonomy capabilities enabled',
      'All features available at maximum performance',
    ],
  },
};

const FEATURE_COMPARISON = [
  {
    name: 'Motion Detection',
    mac: true,
    lenovo: true,
    jetson: true,
    description: 'Detects movement in camera feed',
  },
  {
    name: 'Object Tracking',
    mac: true,
    lenovo: true,
    jetson: true,
    description: 'Tracks selected objects',
  },
  {
    name: 'ArUco Detection',
    mac: true,
    lenovo: true,
    jetson: true,
    description: 'Detects ArUco markers for navigation',
  },
  {
    name: 'Face Recognition',
    mac: false,
    lenovo: true,
    jetson: true,
    description: 'Recognizes faces (slow on CPU)',
    note: 'CPU-based on Lenovo, GPU-accelerated on Jetson',
  },
  {
    name: 'YOLO Detection',
    mac: false,
    lenovo: false,
    jetson: true,
    description: 'Real-time object detection',
  },
  {
    name: 'SLAM',
    mac: false,
    lenovo: true,
    jetson: true,
    description: 'Simultaneous Localization and Mapping',
    note: 'GPU-accelerated on Jetson',
  },
  {
    name: 'GPU Acceleration',
    mac: false,
    lenovo: false,
    jetson: true,
    description: 'CUDA-accelerated processing',
  },
  {
    name: 'Max Resolution',
    mac: '640x480',
    lenovo: '1280x720',
    jetson: '1920x1080',
    description: 'Maximum camera resolution',
  },
  {
    name: 'Max FPS',
    mac: '20',
    lenovo: '25',
    jetson: '60',
    description: 'Maximum frames per second',
  },
];

export function CapabilityInfoModal() {
  const {
    capabilities,
    profileMode,
    isMLCapable,
    isSLAMCapable,
    maxResolution,
    maxFPS,
  } = useCapabilityContext();

  if (!capabilities) return null;

  const profileInfo = PROFILE_INFO[profileMode as keyof typeof PROFILE_INFO] || PROFILE_INFO.mac;

  return (
    <Dialog>
      <DialogTrigger asChild>
        <button
          className="flex items-center gap-2 text-sm hover:opacity-80 transition-opacity cursor-pointer"
          title="Click for detailed capability information"
        >
          <CapabilityStatus />
          <Info className="h-4 w-4 text-gray-400" />
        </button>
      </DialogTrigger>
      <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto bg-neutral-950 border-neutral-800">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2 text-xl">
            <Zap className="h-5 w-5" />
            System Capabilities & Hardware Information
          </DialogTitle>
          <DialogDescription className="text-neutral-400">
            Current hardware configuration and available features
          </DialogDescription>
        </DialogHeader>

        <div className="space-y-6 mt-4">
          {/* Current Profile Card */}
          <div className={`p-4 rounded-lg border-2 ${profileInfo.color.replace('bg-', 'border-')} bg-neutral-900`}>
            <div className="flex items-center gap-3 mb-2">
              <span className="text-2xl">{profileInfo.icon}</span>
              <div>
                <h3 className="text-lg font-semibold">{profileInfo.name}</h3>
                <p className="text-sm text-neutral-400">{profileInfo.description}</p>
              </div>
              <Badge className={`ml-auto ${profileInfo.color} text-white`}>
                {profileMode.toUpperCase()}
              </Badge>
            </div>

            {/* Hardware Specs */}
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mt-4">
              <div className="bg-neutral-800 p-3 rounded">
                <div className="flex items-center gap-2 text-sm text-neutral-400 mb-1">
                  <Camera className="h-4 w-4" />
                  Resolution
                </div>
                <div className="text-lg font-semibold">{maxResolution}</div>
              </div>
              <div className="bg-neutral-800 p-3 rounded">
                <div className="flex items-center gap-2 text-sm text-neutral-400 mb-1">
                  <Zap className="h-4 w-4" />
                  Max FPS
                </div>
                <div className="text-lg font-semibold">{maxFPS}</div>
              </div>
              <div className="bg-neutral-800 p-3 rounded">
                <div className="flex items-center gap-2 text-sm text-neutral-400 mb-1">
                  <Cpu className="h-4 w-4" />
                  CPU Cores
                </div>
                <div className="text-lg font-semibold">{capabilities.cpu_count}</div>
              </div>
              <div className="bg-neutral-800 p-3 rounded">
                <div className="flex items-center gap-2 text-sm text-neutral-400 mb-1">
                  <GpuIcon className="h-4 w-4" />
                  GPU
                </div>
                <div className="text-lg font-semibold">
                  {capabilities.gpu_available ? (
                    <span className="text-green-400">Available</span>
                  ) : (
                    <span className="text-gray-400">None</span>
                  )}
                </div>
              </div>
            </div>

            {/* Limitations */}
            {profileInfo.limitations.length > 0 && (
              <div className="mt-4">
                <h4 className="text-sm font-semibold mb-2 flex items-center gap-2">
                  <AlertTriangle className="h-4 w-4 text-yellow-500" />
                  Current Limitations
                </h4>
                <ul className="list-disc list-inside text-sm text-neutral-300 space-y-1">
                  {profileInfo.limitations.map((limitation, idx) => (
                    <li key={idx}>{limitation}</li>
                  ))}
                </ul>
              </div>
            )}

            {/* Recommendations */}
            <div className="mt-4">
              <h4 className="text-sm font-semibold mb-2 flex items-center gap-2">
                <Info className="h-4 w-4 text-blue-500" />
                Recommendations
              </h4>
              <ul className="list-disc list-inside text-sm text-neutral-300 space-y-1">
                {profileInfo.recommendations.map((rec, idx) => (
                  <li key={idx}>{rec}</li>
                ))}
              </ul>
            </div>
          </div>

          {/* Feature Comparison Table */}
          <div>
            <h3 className="text-lg font-semibold mb-4">Feature Comparison</h3>
            <div className="overflow-x-auto">
              <table className="w-full border-collapse">
                <thead>
                  <tr className="border-b border-neutral-700">
                    <th className="text-left p-2 text-sm font-semibold">Feature</th>
                    <th className="text-center p-2 text-sm font-semibold">Light Mode</th>
                    <th className="text-center p-2 text-sm font-semibold">Dev Mode</th>
                    <th className="text-center p-2 text-sm font-semibold">Omega Mode</th>
                  </tr>
                </thead>
                <tbody>
                  {FEATURE_COMPARISON.map((feature, idx) => (
                    <tr key={idx} className="border-b border-neutral-800 hover:bg-neutral-900">
                      <td className="p-3">
                        <div className="text-sm font-medium">{feature.name}</div>
                        <div className="text-xs text-neutral-400 mt-1">{feature.description}</div>
                        {feature.note && (
                          <div className="text-xs text-yellow-400 mt-1 italic">{feature.note}</div>
                        )}
                      </td>
                      <td className="text-center p-3">
                        {typeof feature.mac === 'boolean' ? (
                          feature.mac ? (
                            <CheckCircle2 className="h-5 w-5 text-green-500 mx-auto" />
                          ) : (
                            <XCircle className="h-5 w-5 text-red-500 mx-auto" />
                          )
                        ) : (
                          <span className="text-sm">{feature.mac}</span>
                        )}
                      </td>
                      <td className="text-center p-3">
                        {typeof feature.lenovo === 'boolean' ? (
                          feature.lenovo ? (
                            <CheckCircle2 className="h-5 w-5 text-green-500 mx-auto" />
                          ) : (
                            <XCircle className="h-5 w-5 text-red-500 mx-auto" />
                          )
                        ) : (
                          <span className="text-sm">{feature.lenovo}</span>
                        )}
                      </td>
                      <td className="text-center p-3">
                        {typeof feature.jetson === 'boolean' ? (
                          feature.jetson ? (
                            <CheckCircle2 className="h-5 w-5 text-green-500 mx-auto" />
                          ) : (
                            <XCircle className="h-5 w-5 text-red-500 mx-auto" />
                          )
                        ) : (
                          <span className="text-sm">{feature.jetson}</span>
                        )}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>

          {/* Current Capabilities Summary */}
          <div className="bg-neutral-900 p-4 rounded-lg border border-neutral-800">
            <h3 className="text-lg font-semibold mb-3">Your Current Capabilities</h3>
            <div className="grid grid-cols-2 md:grid-cols-3 gap-3">
              {[
                { key: 'motion_detection', label: 'Motion Detection' },
                { key: 'tracking', label: 'Object Tracking' },
                { key: 'aruco', label: 'ArUco Detection' },
                { key: 'face_recognition', label: 'Face Recognition' },
                { key: 'yolo', label: 'YOLO Detection' },
                { key: 'slam_capable', label: 'SLAM' },
              ].map(({ key, label }) => {
                const available = capabilities[key as keyof typeof capabilities] as boolean;
                return (
                  <div
                    key={key}
                    className={`flex items-center gap-2 p-2 rounded ${
                      available ? 'bg-green-900/30' : 'bg-red-900/30'
                    }`}
                  >
                    {available ? (
                      <CheckCircle2 className="h-4 w-4 text-green-500" />
                    ) : (
                      <XCircle className="h-4 w-4 text-red-500" />
                    )}
                    <span className="text-sm">{label}</span>
                  </div>
                );
              })}
            </div>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
}

