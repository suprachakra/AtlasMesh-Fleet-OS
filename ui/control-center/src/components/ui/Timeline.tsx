import React from 'react'
import { clsx } from 'clsx'
import { CheckCircleIcon, ClockIcon, ExclamationCircleIcon } from '@heroicons/react/24/outline'

interface TimelineEvent {
  id: string
  timestamp: string
  title: string
  description?: string
  actor?: string
  status?: 'completed' | 'in_progress' | 'pending' | 'error'
  icon?: React.ComponentType<{ className?: string }>
  metadata?: Record<string, unknown>
}

interface TimelineProps {
  events: TimelineEvent[]
  className?: string
}

export function Timeline({ events, className = '' }: TimelineProps) {
  const getStatusIcon = (status?: string) => {
    switch (status) {
      case 'completed':
        return CheckCircleIcon
      case 'in_progress':
        return ClockIcon
      case 'error':
        return ExclamationCircleIcon
      default:
        return ClockIcon
    }
  }

  const getStatusColor = (status?: string) => {
    switch (status) {
      case 'completed':
        return 'text-green-600 bg-green-100 dark:bg-green-900/20'
      case 'in_progress':
        return 'text-blue-600 bg-blue-100 dark:bg-blue-900/20'
      case 'error':
        return 'text-red-600 bg-red-100 dark:bg-red-900/20'
      default:
        return 'text-gray-600 bg-gray-100 dark:bg-gray-700'
    }
  }

  const getLineColor = (status?: string) => {
    switch (status) {
      case 'completed':
        return 'bg-green-200 dark:bg-green-800'
      case 'in_progress':
        return 'bg-blue-200 dark:bg-blue-800'
      case 'error':
        return 'bg-red-200 dark:bg-red-800'
      default:
        return 'bg-gray-200 dark:bg-gray-600'
    }
  }

  return (
    <div className={clsx('flow-root', className)}>
      <ul className="-mb-8">
        {events.map((event, eventIdx) => {
          const StatusIcon = event.icon || getStatusIcon(event.status)
          const isLast = eventIdx === events.length - 1

          return (
            <li key={event.id}>
              <div className="relative pb-8">
                {!isLast && (
                  <span
                    className={clsx(
                      'absolute left-4 top-4 -ml-px h-full w-0.5',
                      getLineColor(event.status)
                    )}
                    aria-hidden="true"
                  />
                )}
                <div className="relative flex space-x-3">
                  <div>
                    <span
                      className={clsx(
                        'h-8 w-8 rounded-full flex items-center justify-center ring-8 ring-white dark:ring-gray-900',
                        getStatusColor(event.status)
                      )}
                    >
                      <StatusIcon className="h-4 w-4" aria-hidden="true" />
                    </span>
                  </div>
                  <div className="flex min-w-0 flex-1 justify-between space-x-4 pt-1.5">
                    <div className="min-w-0 flex-1">
                      <p className="text-sm font-medium text-gray-900 dark:text-white">
                        {event.title}
                      </p>
                      {event.description && (
                        <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
                          {event.description}
                        </p>
                      )}
                      {event.actor && (
                        <p className="mt-1 text-xs text-gray-400 dark:text-gray-500">
                          by {event.actor}
                        </p>
                      )}
                      {event.metadata && Object.keys(event.metadata).length > 0 && (
                        <div className="mt-2 space-y-1">
                          {Object.entries(event.metadata).map(([key, value]) => (
                            <div key={key} className="text-xs text-gray-500 dark:text-gray-400">
                              <span className="font-medium">{key}:</span> {String(value)}
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                    <div className="whitespace-nowrap text-right text-sm text-gray-500 dark:text-gray-400">
                      <time dateTime={event.timestamp}>
                        {new Date(event.timestamp).toLocaleString()}
                      </time>
                    </div>
                  </div>
                </div>
              </div>
            </li>
          )
        })}
      </ul>
    </div>
  )
}
