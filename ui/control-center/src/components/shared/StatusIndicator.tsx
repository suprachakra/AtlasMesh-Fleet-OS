import React from 'react'
import { cn } from '../../lib/utils'

// Unified status indicator component for AtlasMesh Fleet OS
// Reduces redundancy across vehicle status, system status, and operational status displays

export type StatusType = 
  | 'operational' 
  | 'warning' 
  | 'critical' 
  | 'offline' 
  | 'charging'
  | 'maintenance'
  | 'emergency'
  | 'unknown'

export type StatusSize = 'sm' | 'md' | 'lg' | 'xl'

export interface StatusIndicatorProps {
  status: StatusType
  size?: StatusSize
  label?: string
  showLabel?: boolean
  showIcon?: boolean
  showPulse?: boolean
  className?: string
  onClick?: () => void
  tooltip?: string
}

const statusConfig = {
  operational: {
    color: 'bg-green-500',
    textColor: 'text-green-700',
    borderColor: 'border-green-200',
    bgColor: 'bg-green-50',
    icon: '✓',
    label: 'Operational',
    description: 'System is functioning normally'
  },
  warning: {
    color: 'bg-yellow-500',
    textColor: 'text-yellow-700',
    borderColor: 'border-yellow-200',
    bgColor: 'bg-yellow-50',
    icon: '⚠',
    label: 'Warning',
    description: 'Attention required'
  },
  critical: {
    color: 'bg-red-500',
    textColor: 'text-red-700',
    borderColor: 'border-red-200',
    bgColor: 'bg-red-50',
    icon: '⚠',
    label: 'Critical',
    description: 'Immediate action required'
  },
  offline: {
    color: 'bg-gray-500',
    textColor: 'text-gray-700',
    borderColor: 'border-gray-200',
    bgColor: 'bg-gray-50',
    icon: '○',
    label: 'Offline',
    description: 'System is not responding'
  },
  charging: {
    color: 'bg-blue-500',
    textColor: 'text-blue-700',
    borderColor: 'border-blue-200',
    bgColor: 'bg-blue-50',
    icon: '⚡',
    label: 'Charging',
    description: 'Vehicle is charging'
  },
  maintenance: {
    color: 'bg-orange-500',
    textColor: 'text-orange-700',
    borderColor: 'border-orange-200',
    bgColor: 'bg-orange-50',
    icon: '🔧',
    label: 'Maintenance',
    description: 'Under maintenance'
  },
  emergency: {
    color: 'bg-red-600',
    textColor: 'text-red-800',
    borderColor: 'border-red-300',
    bgColor: 'bg-red-100',
    icon: '🚨',
    label: 'Emergency',
    description: 'Emergency situation'
  },
  unknown: {
    color: 'bg-gray-400',
    textColor: 'text-gray-600',
    borderColor: 'border-gray-200',
    bgColor: 'bg-gray-50',
    icon: '?',
    label: 'Unknown',
    description: 'Status unknown'
  }
}

const sizeConfig = {
  sm: {
    dot: 'w-2 h-2',
    container: 'text-xs',
    icon: 'text-xs',
    padding: 'px-2 py-1'
  },
  md: {
    dot: 'w-3 h-3',
    container: 'text-sm',
    icon: 'text-sm',
    padding: 'px-3 py-1'
  },
  lg: {
    dot: 'w-4 h-4',
    container: 'text-base',
    icon: 'text-base',
    padding: 'px-4 py-2'
  },
  xl: {
    dot: 'w-6 h-6',
    container: 'text-lg',
    icon: 'text-lg',
    padding: 'px-6 py-3'
  }
}

export const StatusIndicator: React.FC<StatusIndicatorProps> = ({
  status,
  size = 'md',
  label,
  showLabel = true,
  showIcon = true,
  showPulse = false,
  className,
  onClick,
  tooltip
}) => {
  const config = statusConfig[status]
  const sizeStyles = sizeConfig[size]
  
  const displayLabel = label || config.label
  const isClickable = !!onClick

  const containerClasses = cn(
    'inline-flex items-center gap-2 rounded-full transition-all duration-200',
    sizeStyles.container,
    sizeStyles.padding,
    config.bgColor,
    config.borderColor,
    'border',
    isClickable && 'cursor-pointer hover:shadow-sm hover:scale-105',
    className
  )

  const dotClasses = cn(
    'rounded-full flex-shrink-0',
    sizeStyles.dot,
    config.color,
    showPulse && 'animate-pulse'
  )

  const iconClasses = cn(
    'flex-shrink-0',
    sizeStyles.icon
  )

  const labelClasses = cn(
    'font-medium',
    config.textColor
  )

  const content = (
    <div className={containerClasses} onClick={onClick}>
      {showIcon ? (
        <span className={iconClasses} role="img" aria-label={config.description}>
          {config.icon}
        </span>
      ) : (
        <div className={dotClasses} />
      )}
      
      {showLabel && (
        <span className={labelClasses}>
          {displayLabel}
        </span>
      )}
    </div>
  )

  if (tooltip) {
    return (
      <div title={tooltip} className="inline-block">
        {content}
      </div>
    )
  }

  return content
}

// Specialized status indicators for common use cases
export const VehicleStatusIndicator: React.FC<{
  status: 'operational' | 'warning' | 'critical' | 'offline' | 'charging'
  vehicleId?: string
  size?: StatusSize
  showVehicleId?: boolean
  onClick?: () => void
}> = ({ status, vehicleId, size = 'md', showVehicleId = false, onClick }) => {
  const label = showVehicleId && vehicleId ? `${vehicleId} - ${statusConfig[status].label}` : undefined
  
  return (
    <StatusIndicator
      status={status}
      size={size}
      label={label}
      showPulse={status === 'critical' || status === 'warning'}
      onClick={onClick}
      tooltip={`Vehicle ${vehicleId || ''} is ${statusConfig[status].label.toLowerCase()}`}
    />
  )
}

export const SystemStatusIndicator: React.FC<{
  status: 'operational' | 'warning' | 'critical' | 'maintenance' | 'offline'
  systemName?: string
  size?: StatusSize
  onClick?: () => void
}> = ({ status, systemName, size = 'md', onClick }) => {
  const label = systemName ? `${systemName}` : undefined
  
  return (
    <StatusIndicator
      status={status}
      size={size}
      label={label}
      showPulse={status === 'critical'}
      onClick={onClick}
      tooltip={`${systemName || 'System'} status: ${statusConfig[status].description}`}
    />
  )
}

export const EmergencyStatusIndicator: React.FC<{
  active: boolean
  size?: StatusSize
  onClick?: () => void
}> = ({ active, size = 'lg', onClick }) => {
  return (
    <StatusIndicator
      status={active ? 'emergency' : 'operational'}
      size={size}
      label={active ? 'EMERGENCY ACTIVE' : 'Normal Operations'}
      showPulse={active}
      onClick={onClick}
      className={active ? 'animate-pulse' : ''}
    />
  )
}

// Status indicator with count (for aggregated displays)
export const StatusIndicatorWithCount: React.FC<{
  status: StatusType
  count: number
  total?: number
  size?: StatusSize
  onClick?: () => void
}> = ({ status, count, total, size = 'md', onClick }) => {
  const percentage = total ? Math.round((count / total) * 100) : null
  const label = `${count}${total ? ` of ${total}` : ''}${percentage ? ` (${percentage}%)` : ''}`
  
  return (
    <StatusIndicator
      status={status}
      size={size}
      label={label}
      onClick={onClick}
      tooltip={`${count} items with ${statusConfig[status].label} status`}
    />
  )
}

export default StatusIndicator
