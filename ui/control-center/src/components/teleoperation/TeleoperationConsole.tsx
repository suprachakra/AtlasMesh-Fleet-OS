import React, { useState, useEffect, useRef } from 'react'
import { useTranslation } from 'react-i18next'
import {
  VideoCameraIcon,
  MicrophoneIcon,
  SpeakerWaveIcon,
  ArrowUpIcon,
  ArrowDownIcon,
  ArrowLeftIcon,
  ArrowRightIcon,
  StopIcon,
  PlayIcon,
  ExclamationTriangleIcon,
  ClockIcon,
  SignalIcon,
  ShieldCheckIcon,
} from '@heroicons/react/24/outline'

// Components
import { Button } from '@components/ui/Button'
import { Badge } from '@components/ui/Badge'
import { Card, CardContent, CardHeader, CardTitle } from '@components/ui/Card'
import { Slider } from '@components/ui/Slider'
import { Switch } from '@components/ui/Switch'
import { Modal } from '@components/ui/Modal'

// Hooks
import { useAuth } from '@hooks/useAuth'
import { useTeleoperation } from '@hooks/useTeleoperation'
import { useWebRTC } from '@hooks/useWebRTC'

// Types
interface TeleoperationConsoleProps {
  vehicleId: string
  onSessionEnd?: () => void
  className?: string
}

interface TeleoperationSession {
  sessionId: string
  vehicleId: string
  operatorId: string
  startTime: string
  maxDuration: number // minutes
  status: 'connecting' | 'active' | 'ending' | 'ended'
  latency: number // ms
  bandwidth: number // kbps
  quality: 'excellent' | 'good' | 'fair' | 'poor'
}

interface VehicleControls {
  throttle: number // 0-100
  steering: number // -100 to 100
  brake: number // 0-100
  gear: 'park' | 'reverse' | 'neutral' | 'drive'
  hazardLights: boolean
  horn: boolean
}

export function TeleoperationConsole({
  vehicleId,
  onSessionEnd,
  className = '',
}: TeleoperationConsoleProps) {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const videoRef = useRef<HTMLVideoElement>(null)
  
  // State
  const [session, setSession] = useState<TeleoperationSession | null>(null)
  const [readinessChecked, setReadinessChecked] = useState(false)
  const [readinessGateOpen, setReadinessGateOpen] = useState(false)
  const [controls, setControls] = useState<VehicleControls>({
    throttle: 0,
    steering: 0,
    brake: 0,
    gear: 'park',
    hazardLights: false,
    horn: false,
  })
  const [isControlling, setIsControlling] = useState(false)
  const [emergencyStopArmed, setEmergencyStopArmed] = useState(false)
  const [timeRemaining, setTimeRemaining] = useState(0)
  const [connectionQuality, setConnectionQuality] = useState<'excellent' | 'good' | 'fair' | 'poor'>('good')

  // Hooks
  const {
    startSession,
    endSession,
    sendControls,
    isConnected,
    latency,
    bandwidth,
  } = useTeleoperation(vehicleId)

  const {
    localStream,
    remoteStream,
    startCall,
    endCall,
    isCallActive,
  } = useWebRTC()

  // Readiness gate checks
  const readinessChecks = [
    {
      id: 'bandwidth',
      label: t('teleoperation.readiness.bandwidth'),
      status: bandwidth > 5000, // 5 Mbps minimum
      required: true,
    },
    {
      id: 'latency',
      label: t('teleoperation.readiness.latency'),
      status: latency < 100, // 100ms maximum
      required: true,
    },
    {
      id: 'operator_certified',
      label: t('teleoperation.readiness.operatorCertified'),
      status: permissions.includes('teleoperation:certified'),
      required: true,
    },
    {
      id: 'dual_auth',
      label: t('teleoperation.readiness.dualAuth'),
      status: user?.twoFactorEnabled || false,
      required: true,
    },
    {
      id: 'emergency_contact',
      label: t('teleoperation.readiness.emergencyContact'),
      status: true, // Assume emergency services are available
      required: true,
    },
  ]

  const allChecksPass = readinessChecks.filter(check => check.required).every(check => check.status)

  // Session timer
  useEffect(() => {
    if (session?.status === 'active') {
      const interval = setInterval(() => {
        const elapsed = (Date.now() - new Date(session.startTime).getTime()) / 1000 / 60
        const remaining = Math.max(0, session.maxDuration - elapsed)
        setTimeRemaining(remaining)
        
        if (remaining <= 0) {
          handleEndSession()
        }
      }, 1000)
      
      return () => clearInterval(interval)
    }
  }, [session])

  // Connection quality monitoring
  useEffect(() => {
    if (latency < 50 && bandwidth > 10000) {
      setConnectionQuality('excellent')
    } else if (latency < 100 && bandwidth > 5000) {
      setConnectionQuality('good')
    } else if (latency < 200 && bandwidth > 2000) {
      setConnectionQuality('fair')
    } else {
      setConnectionQuality('poor')
    }
  }, [latency, bandwidth])

  // Auto-safe fallback for poor connection
  useEffect(() => {
    if (connectionQuality === 'poor' && isControlling) {
      handleEmergencyStop()
    }
  }, [connectionQuality, isControlling])

  // Start teleoperation session
  const handleStartSession = async () => {
    if (!allChecksPass) return

    try {
      const newSession = await startSession({
        operatorId: user?.id,
        maxDuration: 30, // 30 minutes max
      })
      
      setSession(newSession)
      await startCall()
      setReadinessGateOpen(false)
    } catch (error) {
      console.error('Failed to start teleoperation session:', error)
    }
  }

  // End teleoperation session
  const handleEndSession = async () => {
    try {
      if (session) {
        await endSession(session.sessionId)
        await endCall()
        setSession(null)
        setIsControlling(false)
        onSessionEnd?.()
      }
    } catch (error) {
      console.error('Failed to end teleoperation session:', error)
    }
  }

  // Control handlers
  const handleControlChange = (control: keyof VehicleControls, value: number | boolean) => {
    const newControls = { ...controls, [control]: value }
    setControls(newControls)
    
    if (isControlling && session) {
      sendControls(newControls)
    }
  }

  const handleEmergencyStop = () => {
    const emergencyControls = {
      ...controls,
      throttle: 0,
      brake: 100,
      hazardLights: true,
    }
    setControls(emergencyControls)
    sendControls(emergencyControls)
    setIsControlling(false)
    setEmergencyStopArmed(true)
  }

  // Keyboard controls
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (!isControlling) return

      switch (event.key) {
        case 'ArrowUp':
          event.preventDefault()
          handleControlChange('throttle', Math.min(100, controls.throttle + 10))
          break
        case 'ArrowDown':
          event.preventDefault()
          handleControlChange('brake', Math.min(100, controls.brake + 10))
          break
        case 'ArrowLeft':
          event.preventDefault()
          handleControlChange('steering', Math.max(-100, controls.steering - 10))
          break
        case 'ArrowRight':
          event.preventDefault()
          handleControlChange('steering', Math.min(100, controls.steering + 10))
          break
        case ' ':
          event.preventDefault()
          handleEmergencyStop()
          break
      }
    }

    const handleKeyUp = (event: KeyboardEvent) => {
      if (!isControlling) return

      switch (event.key) {
        case 'ArrowUp':
          handleControlChange('throttle', 0)
          break
        case 'ArrowDown':
          handleControlChange('brake', 0)
          break
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)
    
    return () => {
      window.removeEventListener('keydown', handleKeyDown)
      window.removeEventListener('keyup', handleKeyUp)
    }
  }, [isControlling, controls])

  if (!permissions.includes('teleoperation:access')) {
    return (
      <div className={`text-center py-8 ${className}`}>
        <ShieldCheckIcon className="mx-auto h-12 w-12 text-gray-400" />
        <h3 className="mt-2 text-sm font-medium text-gray-900 dark:text-white">
          {t('teleoperation.accessDenied')}
        </h3>
        <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
          {t('teleoperation.accessDeniedDesc')}
        </p>
      </div>
    )
  }

  return (
    <div className={`space-y-6 ${className}`}>
      {/* Session Status */}
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <CardTitle className="flex items-center space-x-2">
              <VideoCameraIcon className="h-5 w-5" />
              <span>{t('teleoperation.title')}</span>
            </CardTitle>
            
            <div className="flex items-center space-x-2">
              {session && (
                <>
                  <Badge variant={connectionQuality === 'excellent' || connectionQuality === 'good' ? 'success' : 'warning'}>
                    <SignalIcon className="h-3 w-3 mr-1" />
                    {t(`teleoperation.quality.${connectionQuality}`)}
                  </Badge>
                  
                  <Badge variant="secondary">
                    <ClockIcon className="h-3 w-3 mr-1" />
                    {Math.floor(timeRemaining)}m
                  </Badge>
                </>
              )}
              
              <Badge variant={session ? 'success' : 'secondary'}>
                {session ? t('teleoperation.status.active') : t('teleoperation.status.inactive')}
              </Badge>
            </div>
          </div>
        </CardHeader>
        
        <CardContent>
          {!session ? (
            <div className="text-center space-y-4">
              <Button
                variant="primary"
                onClick={() => setReadinessGateOpen(true)}
                disabled={!permissions.includes('teleoperation:certified')}
              >
                {t('teleoperation.startSession')}
              </Button>
              
              <p className="text-sm text-gray-500">
                {t('teleoperation.startSessionDesc')}
              </p>
            </div>
          ) : (
            <div className="space-y-4">
              {/* Connection Stats */}
              <div className="grid grid-cols-3 gap-4 text-sm">
                <div>
                  <span className="text-gray-500">{t('teleoperation.latency')}</span>
                  <div className="font-mono font-medium">{latency}ms</div>
                </div>
                <div>
                  <span className="text-gray-500">{t('teleoperation.bandwidth')}</span>
                  <div className="font-mono font-medium">{Math.round(bandwidth / 1000)}Mbps</div>
                </div>
                <div>
                  <span className="text-gray-500">{t('teleoperation.duration')}</span>
                  <div className="font-mono font-medium">
                    {Math.floor((Date.now() - new Date(session.startTime).getTime()) / 1000 / 60)}m
                  </div>
                </div>
              </div>
              
              <Button
                variant="danger"
                onClick={handleEndSession}
                className="w-full"
              >
                {t('teleoperation.endSession')}
              </Button>
            </div>
          )}
        </CardContent>
      </Card>

      {/* Video Feed */}
      {session && (
        <Card>
          <CardHeader>
            <CardTitle>{t('teleoperation.videoFeed')}</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="relative bg-black rounded-lg overflow-hidden" style={{ aspectRatio: '16/9' }}>
              <video
                ref={videoRef}
                autoPlay
                playsInline
                muted
                className="w-full h-full object-cover"
                srcObject={remoteStream}
              />
              
              {/* Video Overlay */}
              <div className="absolute inset-0 pointer-events-none">
                {/* Connection Quality Indicator */}
                <div className="absolute top-4 left-4">
                  <Badge variant={connectionQuality === 'poor' ? 'danger' : 'success'}>
                    <SignalIcon className="h-3 w-3 mr-1" />
                    {latency}ms
                  </Badge>
                </div>
                
                {/* Emergency Stop Warning */}
                {emergencyStopArmed && (
                  <div className="absolute inset-0 bg-red-500 bg-opacity-20 flex items-center justify-center">
                    <div className="bg-red-600 text-white px-4 py-2 rounded-lg font-medium animate-pulse">
                      {t('teleoperation.emergencyStopActive')}
                    </div>
                  </div>
                )}
                
                {/* Control Indicators */}
                {isControlling && (
                  <div className="absolute bottom-4 left-4 space-y-2">
                    <div className="bg-black bg-opacity-50 text-white px-2 py-1 rounded text-sm">
                      {t('teleoperation.controlActive')}
                    </div>
                    <div className="bg-black bg-opacity-50 text-white px-2 py-1 rounded text-xs font-mono">
                      T:{controls.throttle} S:{controls.steering} B:{controls.brake}
                    </div>
                  </div>
                )}
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Vehicle Controls */}
      {session && (
        <Card>
          <CardHeader>
            <div className="flex items-center justify-between">
              <CardTitle>{t('teleoperation.vehicleControls')}</CardTitle>
              
              <div className="flex items-center space-x-2">
                <Switch
                  checked={isControlling}
                  onChange={setIsControlling}
                  disabled={connectionQuality === 'poor'}
                />
                <span className="text-sm">{t('teleoperation.enableControls')}</span>
              </div>
            </div>
          </CardHeader>
          
          <CardContent>
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* Steering and Throttle */}
              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium mb-2">
                    {t('teleoperation.controls.steering')}
                  </label>
                  <Slider
                    value={[controls.steering]}
                    onValueChange={([value]) => handleControlChange('steering', value)}
                    min={-100}
                    max={100}
                    step={1}
                    disabled={!isControlling}
                    className="w-full"
                  />
                  <div className="flex justify-between text-xs text-gray-500 mt-1">
                    <span>{t('teleoperation.controls.left')}</span>
                    <span>{controls.steering}</span>
                    <span>{t('teleoperation.controls.right')}</span>
                  </div>
                </div>
                
                <div>
                  <label className="block text-sm font-medium mb-2">
                    {t('teleoperation.controls.throttle')}
                  </label>
                  <Slider
                    value={[controls.throttle]}
                    onValueChange={([value]) => handleControlChange('throttle', value)}
                    min={0}
                    max={100}
                    step={1}
                    disabled={!isControlling}
                    className="w-full"
                  />
                  <div className="flex justify-between text-xs text-gray-500 mt-1">
                    <span>0%</span>
                    <span>{controls.throttle}%</span>
                    <span>100%</span>
                  </div>
                </div>
                
                <div>
                  <label className="block text-sm font-medium mb-2">
                    {t('teleoperation.controls.brake')}
                  </label>
                  <Slider
                    value={[controls.brake]}
                    onValueChange={([value]) => handleControlChange('brake', value)}
                    min={0}
                    max={100}
                    step={1}
                    disabled={!isControlling}
                    className="w-full"
                  />
                  <div className="flex justify-between text-xs text-gray-500 mt-1">
                    <span>0%</span>
                    <span>{controls.brake}%</span>
                    <span>100%</span>
                  </div>
                </div>
              </div>
              
              {/* Control Buttons */}
              <div className="space-y-4">
                {/* Directional Controls */}
                <div className="grid grid-cols-3 gap-2">
                  <div></div>
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={ArrowUpIcon}
                    onMouseDown={() => handleControlChange('throttle', 50)}
                    onMouseUp={() => handleControlChange('throttle', 0)}
                    disabled={!isControlling}
                  />
                  <div></div>
                  
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={ArrowLeftIcon}
                    onMouseDown={() => handleControlChange('steering', -50)}
                    onMouseUp={() => handleControlChange('steering', 0)}
                    disabled={!isControlling}
                  />
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={StopIcon}
                    onClick={() => {
                      handleControlChange('throttle', 0)
                      handleControlChange('brake', 0)
                      handleControlChange('steering', 0)
                    }}
                    disabled={!isControlling}
                  />
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={ArrowRightIcon}
                    onMouseDown={() => handleControlChange('steering', 50)}
                    onMouseUp={() => handleControlChange('steering', 0)}
                    disabled={!isControlling}
                  />
                  
                  <div></div>
                  <Button
                    variant="secondary"
                    size="sm"
                    icon={ArrowDownIcon}
                    onMouseDown={() => handleControlChange('brake', 50)}
                    onMouseUp={() => handleControlChange('brake', 0)}
                    disabled={!isControlling}
                  />
                  <div></div>
                </div>
                
                {/* Emergency Stop */}
                <Button
                  variant="danger"
                  size="lg"
                  icon={StopIcon}
                  onClick={handleEmergencyStop}
                  disabled={!isControlling}
                  className="w-full animate-pulse"
                >
                  {t('teleoperation.emergencyStop')}
                </Button>
                
                {/* Auxiliary Controls */}
                <div className="grid grid-cols-2 gap-2">
                  <Button
                    variant={controls.hazardLights ? 'warning' : 'secondary'}
                    size="sm"
                    onClick={() => handleControlChange('hazardLights', !controls.hazardLights)}
                    disabled={!isControlling}
                  >
                    {t('teleoperation.controls.hazardLights')}
                  </Button>
                  
                  <Button
                    variant={controls.horn ? 'warning' : 'secondary'}
                    size="sm"
                    onMouseDown={() => handleControlChange('horn', true)}
                    onMouseUp={() => handleControlChange('horn', false)}
                    disabled={!isControlling}
                  >
                    {t('teleoperation.controls.horn')}
                  </Button>
                </div>
              </div>
            </div>
            
            {/* Keyboard Shortcuts Help */}
            <div className="mt-6 p-4 bg-gray-50 dark:bg-gray-800 rounded-lg">
              <h4 className="text-sm font-medium mb-2">{t('teleoperation.keyboardShortcuts')}</h4>
              <div className="grid grid-cols-2 gap-2 text-xs text-gray-600 dark:text-gray-400">
                <div>↑ {t('teleoperation.shortcuts.throttle')}</div>
                <div>↓ {t('teleoperation.shortcuts.brake')}</div>
                <div>← {t('teleoperation.shortcuts.steerLeft')}</div>
                <div>→ {t('teleoperation.shortcuts.steerRight')}</div>
                <div>Space {t('teleoperation.shortcuts.emergencyStop')}</div>
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Readiness Gate Modal */}
      <Modal
        open={readinessGateOpen}
        onClose={() => setReadinessGateOpen(false)}
        title={t('teleoperation.readinessGate.title')}
        size="lg"
      >
        <div className="space-y-6">
          <div className="bg-yellow-50 dark:bg-yellow-900/20 border border-yellow-200 dark:border-yellow-800 rounded-lg p-4">
            <div className="flex items-start space-x-3">
              <ExclamationTriangleIcon className="h-6 w-6 text-yellow-600 dark:text-yellow-400 mt-0.5" />
              <div>
                <h3 className="font-medium text-yellow-800 dark:text-yellow-200">
                  {t('teleoperation.readinessGate.warning')}
                </h3>
                <p className="text-sm text-yellow-700 dark:text-yellow-300 mt-1">
                  {t('teleoperation.readinessGate.warningDesc')}
                </p>
              </div>
            </div>
          </div>
          
          <div className="space-y-4">
            <h4 className="font-medium">{t('teleoperation.readinessGate.checks')}</h4>
            
            {readinessChecks.map((check) => (
              <div key={check.id} className="flex items-center justify-between p-3 border rounded-lg">
                <div className="flex items-center space-x-3">
                  <div className={`w-4 h-4 rounded-full ${
                    check.status ? 'bg-green-500' : 'bg-red-500'
                  }`} />
                  <span className="text-sm">{check.label}</span>
                  {check.required && (
                    <Badge variant="secondary" size="sm">
                      {t('common.required')}
                    </Badge>
                  )}
                </div>
                
                <Badge variant={check.status ? 'success' : 'danger'}>
                  {check.status ? t('common.pass') : t('common.fail')}
                </Badge>
              </div>
            ))}
          </div>
          
          <div className="flex justify-end space-x-3 pt-4 border-t">
            <Button
              variant="secondary"
              onClick={() => setReadinessGateOpen(false)}
            >
              {t('actions.cancel')}
            </Button>
            
            <Button
              variant="primary"
              onClick={handleStartSession}
              disabled={!allChecksPass}
            >
              {t('teleoperation.startSession')}
            </Button>
          </div>
        </div>
      </Modal>
    </div>
  )
}
