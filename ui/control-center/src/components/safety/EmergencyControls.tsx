import React, { useState } from 'react'
import { useTranslation } from 'react-i18next'
import {
  StopIcon,
  ExclamationTriangleIcon,
  ShieldExclamationIcon,
  PhoneIcon,
  DocumentTextIcon,
  ClockIcon,
} from '@heroicons/react/24/outline'

// Components
import { Button } from '@components/ui/Button'
import { Modal } from '@components/ui/Modal'
import { Input } from '@components/ui/Input'
import { Select } from '@components/ui/Select'
import { Textarea } from '@components/ui/Textarea'
import { Badge } from '@components/ui/Badge'

// Hooks
import { useAuth } from '@hooks/useAuth'
import { useEmergencyProtocols } from '@hooks/useEmergencyProtocols'

// Types
interface EmergencyControlsProps {
  vehicleId?: string
  tripId?: string
  fleetId?: string
  onEmergencyAction?: (action: string, details: Record<string, unknown>) => void
  className?: string
  layout?: 'horizontal' | 'vertical' | 'grid'
}

interface EmergencyAction {
  id: string
  label: string
  icon: any
  variant: 'danger' | 'warning' | 'secondary'
  requiresConfirmation: boolean
  requiresDualAuth: boolean
  requiresReason: boolean
  description: string
  consequences: string[]
}

const emergencyActions: EmergencyAction[] = [
  {
    id: 'emergency_stop',
    label: 'Emergency Stop',
    icon: StopIcon,
    variant: 'danger',
    requiresConfirmation: true,
    requiresDualAuth: true,
    requiresReason: true,
    description: 'Immediately stop the vehicle using emergency braking',
    consequences: [
      'Vehicle will come to an immediate stop',
      'Passengers may experience sudden deceleration',
      'Emergency services may be automatically notified',
      'Incident report will be automatically generated',
    ],
  },
  {
    id: 'safe_stop',
    label: 'Safe Stop',
    icon: ExclamationTriangleIcon,
    variant: 'warning',
    requiresConfirmation: true,
    requiresDualAuth: false,
    requiresReason: true,
    description: 'Bring vehicle to a controlled stop at safe location',
    consequences: [
      'Vehicle will find safe location to stop',
      'Passengers will be notified of delay',
      'Trip may be reassigned to another vehicle',
    ],
  },
  {
    id: 'request_assistance',
    label: 'Request Remote Assistance',
    icon: PhoneIcon,
    variant: 'warning',
    requiresConfirmation: false,
    requiresDualAuth: false,
    requiresReason: true,
    description: 'Connect vehicle to remote assistance operator',
    consequences: [
      'Remote operator will assess situation',
      'Vehicle may switch to teleoperation mode',
      'Response time: 30-60 seconds',
    ],
  },
  {
    id: 'maintenance_mode',
    label: 'Maintenance Mode',
    icon: ShieldExclamationIcon,
    variant: 'secondary',
    requiresConfirmation: true,
    requiresDualAuth: false,
    requiresReason: true,
    description: 'Take vehicle out of service for maintenance',
    consequences: [
      'Vehicle will complete current trip',
      'No new trips will be assigned',
      'Vehicle will return to depot',
    ],
  },
  {
    id: 'create_incident',
    label: 'Create Incident Report',
    icon: DocumentTextIcon,
    variant: 'secondary',
    requiresConfirmation: false,
    requiresDualAuth: false,
    requiresReason: true,
    description: 'Document safety incident or near-miss',
    consequences: [
      'Incident will be logged for investigation',
      'Relevant telemetry will be preserved',
      'Safety team will be notified',
    ],
  },
]

export function EmergencyControls({
  vehicleId,
  tripId,
  fleetId,
  onEmergencyAction,
  className = '',
  layout = 'horizontal',
}: EmergencyControlsProps) {
  const { t } = useTranslation()
  const { user, permissions } = useAuth()
  const { executeEmergencyAction, isExecuting } = useEmergencyProtocols()

  // State
  const [selectedAction, setSelectedAction] = useState<EmergencyAction | null>(null)
  const [confirmationOpen, setConfirmationOpen] = useState(false)
  const [reason, setReason] = useState('')
  const [reasonCategory, setReasonCategory] = useState('')
  const [additionalDetails, setAdditionalDetails] = useState('')
  const [dualAuthCode, setDualAuthCode] = useState('')
  const [countdown, setCountdown] = useState(0)

  // Reason categories
  const reasonCategories = [
    { value: 'safety_concern', label: t('emergency.reasons.safetyConcern') },
    { value: 'technical_issue', label: t('emergency.reasons.technicalIssue') },
    { value: 'passenger_request', label: t('emergency.reasons.passengerRequest') },
    { value: 'traffic_incident', label: t('emergency.reasons.trafficIncident') },
    { value: 'weather_conditions', label: t('emergency.reasons.weatherConditions') },
    { value: 'regulatory_compliance', label: t('emergency.reasons.regulatoryCompliance') },
    { value: 'other', label: t('emergency.reasons.other') },
  ]

  // Handle action initiation
  const handleActionClick = (action: EmergencyAction) => {
    if (!permissions.includes('emergency:execute')) {
      return
    }

    setSelectedAction(action)
    setReason('')
    setReasonCategory('')
    setAdditionalDetails('')
    setDualAuthCode('')
    
    if (action.requiresConfirmation || action.requiresReason) {
      setConfirmationOpen(true)
    } else {
      executeAction(action)
    }
  }

  // Execute emergency action
  const executeAction = async (action: EmergencyAction) => {
    try {
      const actionDetails = {
        actionId: action.id,
        vehicleId,
        tripId,
        fleetId,
        reason,
        reasonCategory,
        additionalDetails,
        executedBy: user?.id,
        executedAt: new Date().toISOString(),
        dualAuthCode: action.requiresDualAuth ? dualAuthCode : undefined,
      }

      await executeEmergencyAction(actionDetails)
      onEmergencyAction?.(action.id, actionDetails)
      
      setConfirmationOpen(false)
      setSelectedAction(null)
    } catch (error) {
      console.error('Emergency action failed:', error)
    }
  }

  // Confirmation countdown for critical actions
  React.useEffect(() => {
    if (selectedAction?.id === 'emergency_stop' && confirmationOpen && countdown > 0) {
      const timer = setTimeout(() => setCountdown(countdown - 1), 1000)
      return () => clearTimeout(timer)
    }
  }, [selectedAction, confirmationOpen, countdown])

  // Layout classes
  const layoutClasses = {
    horizontal: 'flex flex-wrap gap-2',
    vertical: 'flex flex-col space-y-2',
    grid: 'grid grid-cols-2 lg:grid-cols-3 gap-2',
  }

  return (
    <>
      <div className={`${layoutClasses[layout]} ${className}`}>
        {emergencyActions.map((action) => {
          const hasPermission = permissions.includes('emergency:execute') || 
                               (action.id === 'create_incident' && permissions.includes('incidents:create'))
          
          if (!hasPermission) return null

          return (
            <Button
              key={action.id}
              variant={action.variant}
              size={layout === 'grid' ? 'md' : 'sm'}
              icon={action.icon}
              onClick={() => handleActionClick(action)}
              disabled={isExecuting}
              loading={isExecuting && selectedAction?.id === action.id}
              className={`${layout === 'grid' ? 'flex-col h-20' : ''} ${
                action.variant === 'danger' ? 'animate-pulse' : ''
              }`}
            >
              {t(`emergency.actions.${action.id}`)}
            </Button>
          )
        })}
      </div>

      {/* Confirmation Modal */}
      <Modal
        open={confirmationOpen}
        onClose={() => setConfirmationOpen(false)}
        title={selectedAction ? t(`emergency.confirm.${selectedAction.id}`) : ''}
        size="lg"
      >
        {selectedAction && (
          <div className="space-y-6">
            {/* Action Description */}
            <div className="bg-yellow-50 dark:bg-yellow-900/20 border border-yellow-200 dark:border-yellow-800 rounded-lg p-4">
              <div className="flex items-start space-x-3">
                <selectedAction.icon className="h-6 w-6 text-yellow-600 dark:text-yellow-400 mt-0.5" />
                <div>
                  <h3 className="font-medium text-yellow-800 dark:text-yellow-200">
                    {selectedAction.label}
                  </h3>
                  <p className="text-sm text-yellow-700 dark:text-yellow-300 mt-1">
                    {selectedAction.description}
                  </p>
                </div>
              </div>
            </div>

            {/* Consequences */}
            <div>
              <h4 className="font-medium text-gray-900 dark:text-white mb-2">
                {t('emergency.consequences')}
              </h4>
              <ul className="space-y-1 text-sm text-gray-600 dark:text-gray-400">
                {selectedAction.consequences.map((consequence, index) => (
                  <li key={index} className="flex items-start space-x-2">
                    <span className="text-gray-400 mt-1">•</span>
                    <span>{consequence}</span>
                  </li>
                ))}
              </ul>
            </div>

            {/* Reason Selection */}
            {selectedAction.requiresReason && (
              <div className="space-y-4">
                <div>
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    {t('emergency.reasonCategory')} *
                  </label>
                  <Select
                    value={reasonCategory}
                    onChange={setReasonCategory}
                    options={reasonCategories}
                    placeholder={t('emergency.selectReason')}
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    {t('emergency.reasonDetails')} *
                  </label>
                  <Input
                    value={reason}
                    onChange={(e) => setReason(e.target.value)}
                    placeholder={t('emergency.reasonPlaceholder')}
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                    {t('emergency.additionalDetails')}
                  </label>
                  <Textarea
                    value={additionalDetails}
                    onChange={(e) => setAdditionalDetails(e.target.value)}
                    placeholder={t('emergency.additionalDetailsPlaceholder')}
                    rows={3}
                  />
                </div>
              </div>
            )}

            {/* Dual Authentication */}
            {selectedAction.requiresDualAuth && (
              <div>
                <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                  {t('emergency.dualAuthCode')} *
                </label>
                <Input
                  type="password"
                  value={dualAuthCode}
                  onChange={(e) => setDualAuthCode(e.target.value)}
                  placeholder={t('emergency.enterAuthCode')}
                />
                <p className="text-xs text-gray-500 mt-1">
                  {t('emergency.dualAuthHelp')}
                </p>
              </div>
            )}

            {/* Emergency Stop Countdown */}
            {selectedAction.id === 'emergency_stop' && countdown > 0 && (
              <div className="bg-red-50 dark:bg-red-900/20 border border-red-200 dark:border-red-800 rounded-lg p-4">
                <div className="flex items-center justify-center space-x-2">
                  <ClockIcon className="h-5 w-5 text-red-600" />
                  <span className="text-red-800 dark:text-red-200 font-medium">
                    {t('emergency.countdown', { seconds: countdown })}
                  </span>
                </div>
              </div>
            )}

            {/* Action Buttons */}
            <div className="flex justify-end space-x-3 pt-4 border-t">
              <Button
                variant="secondary"
                onClick={() => setConfirmationOpen(false)}
                disabled={isExecuting}
              >
                {t('actions.cancel')}
              </Button>
              
              <Button
                variant={selectedAction.variant}
                onClick={() => executeAction(selectedAction)}
                disabled={
                  isExecuting ||
                  (selectedAction.requiresReason && (!reason || !reasonCategory)) ||
                  (selectedAction.requiresDualAuth && !dualAuthCode) ||
                  (selectedAction.id === 'emergency_stop' && countdown > 0)
                }
                loading={isExecuting}
              >
                {selectedAction.id === 'emergency_stop' && countdown === 0
                  ? t('emergency.executeNow')
                  : t('emergency.confirm')
                }
              </Button>
            </div>
          </div>
        )}
      </Modal>
    </>
  )
}
