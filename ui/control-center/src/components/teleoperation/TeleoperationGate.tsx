import React, { useState, useCallback, useEffect } from 'react'
import { 
  Radio, Shield, AlertTriangle, CheckCircle, Clock, User, Wifi,
  Camera, Gamepad2, Settings, Lock, Unlock, Play, Square, Eye,
  Signal, Battery, Thermometer, Activity, MapPin, Zap
} from 'lucide-react'
import { Button } from '../ui/Button'
import { Badge } from '../ui/Badge'
import { Card } from '../ui/Card'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '../ui/Dialog'
import { Alert, AlertDescription } from '../ui/Alert'
import { Progress } from '../ui/Progress'
import { Switch } from '../ui/Switch'

// Types
interface TeleoperationSession {
  id: string
  vehicleId: string
  vehicleName: string
  operatorId: string
  operatorName: string
  status: 'pending' | 'active' | 'paused' | 'ended' | 'failed'
  startTime: Date
  endTime?: Date
  reason: string
  location: { lat: number; lng: number; address: string }
  quality: {
    latency: number
    bandwidth: number
    packetLoss: number
    signalStrength: number
  }
  controls: {
    steering: boolean
    throttle: boolean
    brake: boolean
    emergency: boolean
    cameras: boolean
  }
}

interface ReadinessCheck {
  id: string
  name: string
  category: 'network' | 'vehicle' | 'operator' | 'safety' | 'regulatory'
  status: 'pass' | 'fail' | 'warning' | 'checking'
  description: string
  details?: string
  required: boolean
  lastChecked: Date
}

interface TeleoperationGateProps {
  vehicleId: string
  vehicleName: string
  onSessionStart?: (session: TeleoperationSession) => void
  onSessionEnd?: (session: TeleoperationSession) => void
  className?: string
}

const TeleoperationGate: React.FC<TeleoperationGateProps> = ({
  vehicleId,
  vehicleName,
  onSessionStart,
  onSessionEnd,
  className = ''
}) => {
  // State
  const [showGateDialog, setShowGateDialog] = useState(false)
  const [showSessionDialog, setShowSessionDialog] = useState(false)
  const [currentSession, setCurrentSession] = useState<TeleoperationSession | null>(null)
  const [readinessChecks, setReadinessChecks] = useState<ReadinessCheck[]>([])
  const [isRunningChecks, setIsRunningChecks] = useState(false)
  const [sessionReason, setSessionReason] = useState('')
  const [operatorPermissions, setOperatorPermissions] = useState({
    canOperate: true,
    certificationExpiry: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000),
    lastTraining: new Date(Date.now() - 60 * 24 * 60 * 60 * 1000),
    hoursThisWeek: 15,
    maxHoursPerWeek: 40
  })

  // Mock readiness checks
  const mockChecks: ReadinessCheck[] = [
    // Network checks
    {
      id: 'net-latency',
      name: 'Network Latency',
      category: 'network',
      status: 'pass',
      description: 'Round-trip latency < 100ms',
      details: 'Current: 45ms',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'net-bandwidth',
      name: 'Bandwidth',
      category: 'network',
      status: 'pass',
      description: 'Available bandwidth > 10 Mbps',
      details: 'Current: 25 Mbps',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'net-redundancy',
      name: 'Network Redundancy',
      category: 'network',
      status: 'warning',
      description: 'Secondary network path available',
      details: 'Primary: LTE, Secondary: Wi-Fi (weak)',
      required: false,
      lastChecked: new Date()
    },
    
    // Vehicle checks
    {
      id: 'veh-health',
      name: 'Vehicle Health',
      category: 'vehicle',
      status: 'pass',
      description: 'All critical systems operational',
      details: 'Battery: 85%, Sensors: OK, Actuators: OK',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'veh-location',
      name: 'GPS Signal',
      category: 'vehicle',
      status: 'pass',
      description: 'GPS accuracy < 1m',
      details: 'Current accuracy: 0.3m, 12 satellites',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'veh-cameras',
      name: 'Camera Systems',
      category: 'vehicle',
      status: 'pass',
      description: 'All cameras operational',
      details: 'Front: OK, Rear: OK, Sides: OK, 360°: OK',
      required: true,
      lastChecked: new Date()
    },
    
    // Operator checks
    {
      id: 'op-cert',
      name: 'Operator Certification',
      category: 'operator',
      status: 'pass',
      description: 'Valid teleoperation certification',
      details: 'Expires: 30 days',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'op-hours',
      name: 'Work Hours Compliance',
      category: 'operator',
      status: 'pass',
      description: 'Within weekly hour limits',
      details: '15/40 hours this week',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'op-training',
      name: 'Recent Training',
      category: 'operator',
      status: 'warning',
      description: 'Training within last 90 days',
      details: 'Last training: 60 days ago',
      required: false,
      lastChecked: new Date()
    },
    
    // Safety checks
    {
      id: 'saf-emergency',
      name: 'Emergency Systems',
      category: 'safety',
      status: 'pass',
      description: 'Emergency stop systems functional',
      details: 'Vehicle E-stop: OK, Remote E-stop: OK',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'saf-monitoring',
      name: 'Safety Monitoring',
      category: 'safety',
      status: 'pass',
      description: 'AI safety monitor active',
      details: 'Collision avoidance: Active, Path validation: Active',
      required: true,
      lastChecked: new Date()
    },
    
    // Regulatory checks
    {
      id: 'reg-permit',
      name: 'Operating Permit',
      category: 'regulatory',
      status: 'pass',
      description: 'Valid for current location',
      details: 'Dubai Municipality Permit: Valid until Dec 2024',
      required: true,
      lastChecked: new Date()
    },
    {
      id: 'reg-insurance',
      name: 'Insurance Coverage',
      category: 'regulatory',
      status: 'pass',
      description: 'Teleoperation coverage active',
      details: 'Policy: AXA-TEL-2024, Coverage: $5M',
      required: true,
      lastChecked: new Date()
    }
  ]

  // Initialize checks
  useEffect(() => {
    setReadinessChecks(mockChecks)
  }, [])

  // Real-time session updates
  useEffect(() => {
    if (!currentSession) return

    const interval = setInterval(() => {
      setCurrentSession(prev => {
        if (!prev) return null
        
        return {
          ...prev,
          quality: {
            latency: Math.max(20, prev.quality.latency + (Math.random() - 0.5) * 10),
            bandwidth: Math.max(5, prev.quality.bandwidth + (Math.random() - 0.5) * 2),
            packetLoss: Math.max(0, Math.min(5, prev.quality.packetLoss + (Math.random() - 0.5) * 0.5)),
            signalStrength: Math.max(1, Math.min(5, prev.quality.signalStrength + (Math.random() - 0.5) * 0.5))
          }
        }
      })
    }, 2000)

    return () => clearInterval(interval)
  }, [currentSession])

  // Handlers
  const runReadinessChecks = useCallback(async () => {
    setIsRunningChecks(true)
    
    // Simulate running checks
    for (let i = 0; i < readinessChecks.length; i++) {
      await new Promise(resolve => setTimeout(resolve, 300))
      
      setReadinessChecks(prev => prev.map((check, index) => {
        if (index === i) {
          // Simulate some randomness in check results
          const rand = Math.random()
          let status: ReadinessCheck['status'] = 'pass'
          
          if (check.required && rand < 0.1) {
            status = 'fail'
          } else if (rand < 0.2) {
            status = 'warning'
          }
          
          return {
            ...check,
            status,
            lastChecked: new Date()
          }
        }
        return check
      }))
    }
    
    setIsRunningChecks(false)
  }, [readinessChecks.length])

  const startTeleoperationSession = useCallback(async () => {
    if (!sessionReason.trim()) return

    const session: TeleoperationSession = {
      id: `tel-${Date.now()}`,
      vehicleId,
      vehicleName,
      operatorId: 'op-001',
      operatorName: 'Current Operator',
      status: 'active',
      startTime: new Date(),
      reason: sessionReason,
      location: {
        lat: 25.2048,
        lng: 55.2708,
        address: 'Dubai Mall Parking Area'
      },
      quality: {
        latency: 45,
        bandwidth: 25,
        packetLoss: 0.1,
        signalStrength: 4
      },
      controls: {
        steering: true,
        throttle: true,
        brake: true,
        emergency: true,
        cameras: true
      }
    }

    setCurrentSession(session)
    setShowGateDialog(false)
    setShowSessionDialog(true)
    onSessionStart?.(session)

    console.info('Teleoperation session started', {
      sessionId: session.id,
      vehicleId,
      operatorId: session.operatorId,
      reason: sessionReason,
      timestamp: new Date().toISOString()
    })
  }, [vehicleId, vehicleName, sessionReason, onSessionStart])

  const endTeleoperationSession = useCallback(() => {
    if (!currentSession) return

    const endedSession: TeleoperationSession = {
      ...currentSession,
      status: 'ended',
      endTime: new Date()
    }

    setCurrentSession(null)
    setShowSessionDialog(false)
    onSessionEnd?.(endedSession)

    console.info('Teleoperation session ended', {
      sessionId: endedSession.id,
      duration: endedSession.endTime!.getTime() - endedSession.startTime.getTime(),
      timestamp: new Date().toISOString()
    })
  }, [currentSession, onSessionEnd])

  const pauseTeleoperationSession = useCallback(() => {
    if (!currentSession) return

    setCurrentSession(prev => prev ? { ...prev, status: 'paused' } : null)
  }, [currentSession])

  const resumeTeleoperationSession = useCallback(() => {
    if (!currentSession) return

    setCurrentSession(prev => prev ? { ...prev, status: 'active' } : null)
  }, [currentSession])

  // Check if teleoperation is allowed
  const canStartTeleoperation = readinessChecks
    .filter(check => check.required)
    .every(check => check.status === 'pass')

  const getStatusColor = (status: ReadinessCheck['status']) => {
    switch (status) {
      case 'pass': return 'text-green-600'
      case 'fail': return 'text-red-600'
      case 'warning': return 'text-yellow-600'
      case 'checking': return 'text-blue-600'
      default: return 'text-gray-600'
    }
  }

  const getStatusIcon = (status: ReadinessCheck['status']) => {
    switch (status) {
      case 'pass': return <CheckCircle className="w-4 h-4 text-green-600" />
      case 'fail': return <AlertTriangle className="w-4 h-4 text-red-600" />
      case 'warning': return <AlertTriangle className="w-4 h-4 text-yellow-600" />
      case 'checking': return <Clock className="w-4 h-4 text-blue-600 animate-spin" />
      default: return <Clock className="w-4 h-4 text-gray-600" />
    }
  }

  const getCategoryIcon = (category: ReadinessCheck['category']) => {
    switch (category) {
      case 'network': return <Wifi className="w-4 h-4" />
      case 'vehicle': return <Activity className="w-4 h-4" />
      case 'operator': return <User className="w-4 h-4" />
      case 'safety': return <Shield className="w-4 h-4" />
      case 'regulatory': return <Lock className="w-4 h-4" />
      default: return <Settings className="w-4 h-4" />
    }
  }

  const getQualityColor = (value: number, thresholds: { good: number; warning: number }) => {
    if (value >= thresholds.good) return 'text-green-600'
    if (value >= thresholds.warning) return 'text-yellow-600'
    return 'text-red-600'
  }

  return (
    <div className={className}>
      {/* Teleoperation Status */}
      {currentSession ? (
        <Card className="p-4 border-blue-200 bg-blue-50">
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-3">
              <div className="p-2 bg-blue-100 rounded-full">
                <Radio className="w-5 h-5 text-blue-600" />
              </div>
              <div>
                <div className="font-medium text-blue-900">Teleoperation Active</div>
                <div className="text-sm text-blue-700">
                  Session: {currentSession.id} • Duration: {Math.floor((new Date().getTime() - currentSession.startTime.getTime()) / 60000)}m
                </div>
              </div>
              <Badge className={
                currentSession.status === 'active' ? 'bg-green-100 text-green-800' :
                currentSession.status === 'paused' ? 'bg-yellow-100 text-yellow-800' :
                'bg-gray-100 text-gray-800'
              }>
                {currentSession.status}
              </Badge>
            </div>
            <div className="flex items-center space-x-2">
              <Button
                variant="outline"
                size="sm"
                onClick={() => setShowSessionDialog(true)}
              >
                <Eye className="w-4 h-4 mr-2" />
                Monitor
              </Button>
              <Button
                variant="destructive"
                size="sm"
                onClick={endTeleoperationSession}
              >
                <Square className="w-4 h-4 mr-2" />
                End Session
              </Button>
            </div>
          </div>
        </Card>
      ) : (
        <Button
          onClick={() => setShowGateDialog(true)}
          className="w-full"
          disabled={!operatorPermissions.canOperate}
        >
          <Radio className="w-4 h-4 mr-2" />
          Request Teleoperation
        </Button>
      )}

      {/* Readiness Gate Dialog */}
      <Dialog open={showGateDialog} onOpenChange={setShowGateDialog}>
        <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Teleoperation Readiness Gate - {vehicleName}</DialogTitle>
          </DialogHeader>

          <div className="space-y-6">
            {/* Operator Status */}
            <Card className="p-4">
              <h3 className="text-lg font-medium text-gray-900 mb-3">Operator Status</h3>
              <div className="grid grid-cols-2 gap-4 text-sm">
                <div>
                  <span className="font-medium">Certification:</span>
                  <span className={`ml-2 ${
                    operatorPermissions.certificationExpiry > new Date() ? 'text-green-600' : 'text-red-600'
                  }`}>
                    {operatorPermissions.certificationExpiry > new Date() ? 'Valid' : 'Expired'}
                  </span>
                </div>
                <div>
                  <span className="font-medium">Work Hours:</span>
                  <span className="ml-2">{operatorPermissions.hoursThisWeek}/{operatorPermissions.maxHoursPerWeek}h this week</span>
                </div>
                <div>
                  <span className="font-medium">Last Training:</span>
                  <span className="ml-2">{Math.floor((new Date().getTime() - operatorPermissions.lastTraining.getTime()) / (24 * 60 * 60 * 1000))} days ago</span>
                </div>
                <div>
                  <span className="font-medium">Status:</span>
                  <span className={`ml-2 ${operatorPermissions.canOperate ? 'text-green-600' : 'text-red-600'}`}>
                    {operatorPermissions.canOperate ? 'Authorized' : 'Not Authorized'}
                  </span>
                </div>
              </div>
            </Card>

            {/* Readiness Checks */}
            <div>
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-medium text-gray-900">Readiness Checks</h3>
                <Button
                  variant="outline"
                  onClick={runReadinessChecks}
                  disabled={isRunningChecks}
                >
                  {isRunningChecks ? 'Running...' : 'Run Checks'}
                </Button>
              </div>

              <div className="space-y-3">
                {['network', 'vehicle', 'operator', 'safety', 'regulatory'].map(category => (
                  <Card key={category} className="p-4">
                    <h4 className="font-medium text-gray-900 mb-3 capitalize flex items-center space-x-2">
                      {getCategoryIcon(category)}
                      <span>{category} Checks</span>
                    </h4>
                    
                    <div className="space-y-2">
                      {readinessChecks
                        .filter(check => check.category === category)
                        .map(check => (
                          <div key={check.id} className="flex items-center justify-between p-2 rounded border">
                            <div className="flex items-center space-x-3">
                              {getStatusIcon(check.status)}
                              <div>
                                <div className="font-medium text-gray-900">{check.name}</div>
                                <div className="text-sm text-gray-600">{check.description}</div>
                                {check.details && (
                                  <div className="text-xs text-gray-500">{check.details}</div>
                                )}
                              </div>
                            </div>
                            <div className="flex items-center space-x-2">
                              {check.required && (
                                <Badge variant="outline" size="sm">Required</Badge>
                              )}
                              <Badge className={
                                check.status === 'pass' ? 'bg-green-100 text-green-800' :
                                check.status === 'fail' ? 'bg-red-100 text-red-800' :
                                check.status === 'warning' ? 'bg-yellow-100 text-yellow-800' :
                                'bg-blue-100 text-blue-800'
                              }>
                                {check.status}
                              </Badge>
                            </div>
                          </div>
                        ))}
                    </div>
                  </Card>
                ))}
              </div>
            </div>

            {/* Session Details */}
            <Card className="p-4">
              <h3 className="text-lg font-medium text-gray-900 mb-3">Session Details</h3>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-2">
                  Reason for Teleoperation *
                </label>
                <textarea
                  value={sessionReason}
                  onChange={(e) => setSessionReason(e.target.value)}
                  placeholder="Describe why teleoperation is needed (e.g., construction zone navigation, emergency response, passenger assistance)"
                  className="w-full p-3 border border-gray-300 rounded-md"
                  rows={3}
                  required
                />
              </div>
            </Card>

            {/* Action Buttons */}
            <div className="flex justify-end space-x-3 pt-4 border-t">
              <Button
                variant="outline"
                onClick={() => setShowGateDialog(false)}
              >
                Cancel
              </Button>
              
              <Button
                onClick={startTeleoperationSession}
                disabled={!canStartTeleoperation || !sessionReason.trim() || isRunningChecks}
                className={canStartTeleoperation ? 'bg-green-600 hover:bg-green-700' : ''}
              >
                <Play className="w-4 h-4 mr-2" />
                Start Teleoperation
              </Button>
            </div>

            {!canStartTeleoperation && (
              <Alert>
                <AlertTriangle className="w-4 h-4" />
                <AlertDescription>
                  Cannot start teleoperation. Please resolve all failed required checks.
                </AlertDescription>
              </Alert>
            )}
          </div>
        </DialogContent>
      </Dialog>

      {/* Active Session Monitor Dialog */}
      <Dialog open={showSessionDialog} onOpenChange={setShowSessionDialog}>
        <DialogContent className="max-w-6xl max-h-[90vh] overflow-y-auto">
          <DialogHeader>
            <DialogTitle>Teleoperation Session Monitor</DialogTitle>
          </DialogHeader>

          {currentSession && (
            <div className="space-y-6">
              {/* Session Info */}
              <div className="grid grid-cols-3 gap-4">
                <Card className="p-4">
                  <div className="text-sm text-gray-600">Session ID</div>
                  <div className="font-medium">{currentSession.id}</div>
                </Card>
                <Card className="p-4">
                  <div className="text-sm text-gray-600">Duration</div>
                  <div className="font-medium">
                    {Math.floor((new Date().getTime() - currentSession.startTime.getTime()) / 60000)}m
                  </div>
                </Card>
                <Card className="p-4">
                  <div className="text-sm text-gray-600">Status</div>
                  <Badge className={
                    currentSession.status === 'active' ? 'bg-green-100 text-green-800' :
                    currentSession.status === 'paused' ? 'bg-yellow-100 text-yellow-800' :
                    'bg-gray-100 text-gray-800'
                  }>
                    {currentSession.status}
                  </Badge>
                </Card>
              </div>

              {/* Quality Metrics */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-4">Connection Quality</h3>
                <div className="grid grid-cols-4 gap-6">
                  <div>
                    <div className="flex items-center space-x-2 mb-2">
                      <Zap className="w-4 h-4 text-gray-500" />
                      <span className="text-sm font-medium">Latency</span>
                    </div>
                    <div className={`text-2xl font-bold ${getQualityColor(
                      100 - currentSession.quality.latency,
                      { good: 50, warning: 25 }
                    )}`}>
                      {Math.round(currentSession.quality.latency)}ms
                    </div>
                    <Progress 
                      value={Math.max(0, 100 - currentSession.quality.latency)} 
                      className="mt-2"
                    />
                  </div>

                  <div>
                    <div className="flex items-center space-x-2 mb-2">
                      <Activity className="w-4 h-4 text-gray-500" />
                      <span className="text-sm font-medium">Bandwidth</span>
                    </div>
                    <div className={`text-2xl font-bold ${getQualityColor(
                      currentSession.quality.bandwidth,
                      { good: 15, warning: 10 }
                    )}`}>
                      {Math.round(currentSession.quality.bandwidth)}Mbps
                    </div>
                    <Progress 
                      value={Math.min(100, currentSession.quality.bandwidth * 4)} 
                      className="mt-2"
                    />
                  </div>

                  <div>
                    <div className="flex items-center space-x-2 mb-2">
                      <Signal className="w-4 h-4 text-gray-500" />
                      <span className="text-sm font-medium">Packet Loss</span>
                    </div>
                    <div className={`text-2xl font-bold ${getQualityColor(
                      100 - currentSession.quality.packetLoss * 20,
                      { good: 80, warning: 60 }
                    )}`}>
                      {currentSession.quality.packetLoss.toFixed(1)}%
                    </div>
                    <Progress 
                      value={Math.max(0, 100 - currentSession.quality.packetLoss * 20)} 
                      className="mt-2"
                    />
                  </div>

                  <div>
                    <div className="flex items-center space-x-2 mb-2">
                      <Wifi className="w-4 h-4 text-gray-500" />
                      <span className="text-sm font-medium">Signal</span>
                    </div>
                    <div className={`text-2xl font-bold ${getQualityColor(
                      currentSession.quality.signalStrength * 20,
                      { good: 60, warning: 40 }
                    )}`}>
                      {Math.round(currentSession.quality.signalStrength)}/5
                    </div>
                    <Progress 
                      value={currentSession.quality.signalStrength * 20} 
                      className="mt-2"
                    />
                  </div>
                </div>
              </Card>

              {/* Controls Status */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-4">Control Systems</h3>
                <div className="grid grid-cols-5 gap-4">
                  {Object.entries(currentSession.controls).map(([control, enabled]) => (
                    <div key={control} className="text-center">
                      <div className={`p-3 rounded-full mb-2 ${
                        enabled ? 'bg-green-100' : 'bg-red-100'
                      }`}>
                        {control === 'steering' && <Gamepad2 className={`w-6 h-6 mx-auto ${enabled ? 'text-green-600' : 'text-red-600'}`} />}
                        {control === 'throttle' && <Zap className={`w-6 h-6 mx-auto ${enabled ? 'text-green-600' : 'text-red-600'}`} />}
                        {control === 'brake' && <Square className={`w-6 h-6 mx-auto ${enabled ? 'text-green-600' : 'text-red-600'}`} />}
                        {control === 'emergency' && <AlertTriangle className={`w-6 h-6 mx-auto ${enabled ? 'text-green-600' : 'text-red-600'}`} />}
                        {control === 'cameras' && <Camera className={`w-6 h-6 mx-auto ${enabled ? 'text-green-600' : 'text-red-600'}`} />}
                      </div>
                      <div className="text-sm font-medium capitalize">{control}</div>
                      <div className={`text-xs ${enabled ? 'text-green-600' : 'text-red-600'}`}>
                        {enabled ? 'Active' : 'Disabled'}
                      </div>
                    </div>
                  ))}
                </div>
              </Card>

              {/* Vehicle Status */}
              <Card className="p-4">
                <h3 className="text-lg font-medium text-gray-900 mb-4">Vehicle Status</h3>
                <div className="grid grid-cols-4 gap-4 text-sm">
                  <div>
                    <div className="flex items-center space-x-2 mb-1">
                      <MapPin className="w-4 h-4 text-gray-500" />
                      <span className="font-medium">Location</span>
                    </div>
                    <div className="text-gray-600">{currentSession.location.address}</div>
                  </div>
                  <div>
                    <div className="flex items-center space-x-2 mb-1">
                      <Battery className="w-4 h-4 text-gray-500" />
                      <span className="font-medium">Battery</span>
                    </div>
                    <div className="text-green-600">85%</div>
                  </div>
                  <div>
                    <div className="flex items-center space-x-2 mb-1">
                      <Thermometer className="w-4 h-4 text-gray-500" />
                      <span className="font-medium">Temperature</span>
                    </div>
                    <div className="text-green-600">42°C</div>
                  </div>
                  <div>
                    <div className="flex items-center space-x-2 mb-1">
                      <Activity className="w-4 h-4 text-gray-500" />
                      <span className="font-medium">Speed</span>
                    </div>
                    <div className="text-blue-600">15 km/h</div>
                  </div>
                </div>
              </Card>

              {/* Action Buttons */}
              <div className="flex justify-between pt-4 border-t">
                <div className="flex space-x-2">
                  {currentSession.status === 'active' ? (
                    <Button
                      variant="outline"
                      onClick={pauseTeleoperationSession}
                    >
                      <Square className="w-4 h-4 mr-2" />
                      Pause
                    </Button>
                  ) : (
                    <Button
                      variant="outline"
                      onClick={resumeTeleoperationSession}
                    >
                      <Play className="w-4 h-4 mr-2" />
                      Resume
                    </Button>
                  )}
                </div>
                
                <div className="flex space-x-2">
                  <Button
                    variant="outline"
                    onClick={() => setShowSessionDialog(false)}
                  >
                    Close Monitor
                  </Button>
                  <Button
                    variant="destructive"
                    onClick={endTeleoperationSession}
                  >
                    <Square className="w-4 h-4 mr-2" />
                    End Session
                  </Button>
                </div>
              </div>
            </div>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
}

export default TeleoperationGate
