import React, { useState, useEffect, useCallback, useRef, useMemo } from 'react'

// Performance monitoring hook for AtlasMesh Fleet OS UI
// Real-time performance tracking for Abu Dhabi operations

export interface PerformanceBudget {
  operation: string
  budget: number
  warning: number
  critical: number
  description: string
}

export interface PerformanceMeasurement {
  id: string
  operation: string
  duration: number
  timestamp: number
  metadata?: Record<string, unknown>
}

export interface PerformanceViolation {
  severity: 'warning' | 'critical'
  operation: string
  duration: number
  budget: number
  overage: number
  percentage: number
  timestamp: number
}

export interface PerformanceStats {
  totalMeasurements: number
  averageResponseTime: number
  violations: PerformanceViolation[]
  budgetCompliance: number
  slowestOperations: Array<{
    operation: string
    averageDuration: number
    violationCount: number
  }>
}

// Abu Dhabi specific performance budgets (milliseconds)
const UI_PERFORMANCE_BUDGETS: Record<string, PerformanceBudget> = {
  component_render: {
    operation: 'component_render',
    budget: 16, // 60fps target
    warning: 12,
    critical: 16,
    description: 'React component render time'
  },
  map_update: {
    operation: 'map_update',
    budget: 100,
    warning: 80,
    critical: 100,
    description: 'Map update with vehicle positions'
  },
  api_call: {
    operation: 'api_call',
    budget: 200,
    warning: 150,
    critical: 200,
    description: 'API request response time'
  },
  route_calculation: {
    operation: 'route_calculation',
    budget: 500,
    warning: 400,
    critical: 500,
    description: 'Route calculation and display'
  },
  vehicle_command: {
    operation: 'vehicle_command',
    budget: 100,
    warning: 80,
    critical: 100,
    description: 'Vehicle command execution'
  },
  telemetry_update: {
    operation: 'telemetry_update',
    budget: 50,
    warning: 40,
    critical: 50,
    description: 'Telemetry data processing and display'
  }
}

export const usePerformanceMonitor = () => {
  const [measurements, setMeasurements] = useState<PerformanceMeasurement[]>([])
  const [violations, setViolations] = useState<PerformanceViolation[]>([])
  const [isEnabled, setIsEnabled] = useState(true)
  
  const activeMeasurements = useRef<Map<string, { operation: string; startTime: number; metadata?: Record<string, unknown> }>>(new Map())

  // Start a performance measurement
  const startMeasurement = useCallback((operation: string, metadata?: Record<string, unknown>): string => {
    if (!isEnabled) return ''
    
    const id = `${operation}_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
    const startTime = performance.now()
    
    activeMeasurements.current.set(id, {
      operation,
      startTime,
      metadata: {
        ...metadata,
        region: 'abu_dhabi',
        timestamp: new Date().toISOString()
      }
    })

    return id
  }, [isEnabled])

  // End a performance measurement
  const endMeasurement = useCallback((id: string): PerformanceMeasurement | null => {
    if (!isEnabled || !id) return null
    
    const measurement = activeMeasurements.current.get(id)
    if (!measurement) return null

    const endTime = performance.now()
    const duration = endTime - measurement.startTime
    
    const result: PerformanceMeasurement = {
      id,
      operation: measurement.operation,
      duration,
      timestamp: Date.now(),
      metadata: measurement.metadata
    }

    // Check against budget
    const budget = UI_PERFORMANCE_BUDGETS[measurement.operation]
    if (budget) {
      checkBudget(result, budget)
    }

    // Store measurement
    setMeasurements(prev => [...prev.slice(-99), result]) // Keep last 100 measurements

    // Clean up
    activeMeasurements.current.delete(id)

    return result
  }, [isEnabled])

  // Check if measurement violates performance budget
  const checkBudget = useCallback((measurement: PerformanceMeasurement, budget: PerformanceBudget) => {
    const { duration, operation, timestamp } = measurement
    
    let severity: 'warning' | 'critical' | null = null
    
    if (duration > budget.critical) {
      severity = 'critical'
    } else if (duration > budget.warning) {
      severity = 'warning'
    }

    if (severity) {
      const violation: PerformanceViolation = {
        severity,
        operation,
        duration,
        budget: budget.budget,
        overage: duration - budget.budget,
        percentage: ((duration - budget.budget) / budget.budget) * 100,
        timestamp
      }

      setViolations(prev => [...prev.slice(-49), violation]) // Keep last 50 violations

      // Log violation for debugging
      const emoji = severity === 'critical' ? '🚨' : '⚠️'
      console.warn(`${emoji} Performance violation: ${operation} took ${duration.toFixed(2)}ms (budget: ${budget.budget}ms)`)
      
      // For critical UI operations in Abu Dhabi fleet management
      if (severity === 'critical' && ['vehicle_command', 'map_update'].includes(operation)) {
        console.error('🚨 Critical UI performance issue affecting Abu Dhabi fleet operations!')
      }
    }
  }, [])

  // Measure a function execution
  const measureFunction = useCallback(async <T>(
    operation: string,
    fn: () => T | Promise<T>,
    metadata?: Record<string, unknown>
  ): Promise<T> => {
    const id = startMeasurement(operation, metadata)
    
    try {
      const result = await fn()
      endMeasurement(id)
      return result
    } catch (error) {
      endMeasurement(id)
      throw error
    }
  }, [startMeasurement, endMeasurement])

  // Measure React component render time
  const measureRender = useCallback((componentName: string, metadata?: Record<string, unknown>) => {
    const id = startMeasurement('component_render', { component: componentName, ...metadata })
    
    return () => endMeasurement(id)
  }, [startMeasurement, endMeasurement])

  // Calculate performance statistics
  const stats: PerformanceStats = useMemo(() => {
    if (measurements.length === 0) {
      return {
        totalMeasurements: 0,
        averageResponseTime: 0,
        violations: [],
        budgetCompliance: 100,
        slowestOperations: []
      }
    }

    const totalDuration = measurements.reduce((sum, m) => sum + m.duration, 0)
    const averageResponseTime = totalDuration / measurements.length

    // Calculate budget compliance
    const totalBudgetChecks = measurements.filter(m => UI_PERFORMANCE_BUDGETS[m.operation]).length
    const violationCount = violations.length
    const budgetCompliance = totalBudgetChecks > 0 ? ((totalBudgetChecks - violationCount) / totalBudgetChecks) * 100 : 100

    // Find slowest operations
    const operationStats: Record<string, { totalDuration: number; count: number; violations: number }> = {}
    
    measurements.forEach(m => {
      if (!operationStats[m.operation]) {
        operationStats[m.operation] = { totalDuration: 0, count: 0, violations: 0 }
      }
      operationStats[m.operation].totalDuration += m.duration
      operationStats[m.operation].count++
    })

    violations.forEach(v => {
      if (operationStats[v.operation]) {
        operationStats[v.operation].violations++
      }
    })

    const slowestOperations = Object.entries(operationStats)
      .map(([operation, stats]) => ({
        operation,
        averageDuration: stats.totalDuration / stats.count,
        violationCount: stats.violations
      }))
      .sort((a, b) => b.averageDuration - a.averageDuration)
      .slice(0, 5)

    return {
      totalMeasurements: measurements.length,
      averageResponseTime,
      violations: violations.slice(-10), // Last 10 violations
      budgetCompliance,
      slowestOperations
    }
  }, [measurements, violations])

  // Clear performance data
  const clearData = useCallback(() => {
    setMeasurements([])
    setViolations([])
    activeMeasurements.current.clear()
  }, [])

  // Export performance data
  const exportData = useCallback(() => {
    const data = {
      timestamp: new Date().toISOString(),
      region: 'abu_dhabi',
      measurements: measurements.slice(-100),
      violations: violations.slice(-50),
      stats,
      budgets: UI_PERFORMANCE_BUDGETS
    }

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `atlasmesh-performance-${Date.now()}.json`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }, [measurements, violations, stats])

  // Monitor Web Vitals
  useEffect(() => {
    if (!isEnabled) return

    // Monitor Largest Contentful Paint (LCP)
    const observer = new PerformanceObserver((list) => {
      for (const entry of list.getEntries()) {
        if (entry.entryType === 'largest-contentful-paint') {
          const measurement: PerformanceMeasurement = {
            id: `lcp_${Date.now()}`,
            operation: 'largest_contentful_paint',
            duration: entry.startTime,
            timestamp: Date.now(),
            metadata: { type: 'web_vital' }
          }
          
          setMeasurements(prev => [...prev.slice(-99), measurement])
          
          // Check against LCP budget (2.5s)
          if (entry.startTime > 2500) {
            const violation: PerformanceViolation = {
              severity: 'critical',
              operation: 'largest_contentful_paint',
              duration: entry.startTime,
              budget: 2500,
              overage: entry.startTime - 2500,
              percentage: ((entry.startTime - 2500) / 2500) * 100,
              timestamp: Date.now()
            }
            setViolations(prev => [...prev.slice(-49), violation])
          }
        }
      }
    })

    try {
      observer.observe({ entryTypes: ['largest-contentful-paint'] })
    } catch (e) {
      console.warn('Performance Observer not supported')
    }

    return () => observer.disconnect()
  }, [isEnabled])

  return {
    // Measurement functions
    startMeasurement,
    endMeasurement,
    measureFunction,
    measureRender,
    
    // Data
    measurements: measurements.slice(-20), // Last 20 for UI display
    violations: violations.slice(-10), // Last 10 for UI display
    stats,
    budgets: UI_PERFORMANCE_BUDGETS,
    
    // Controls
    isEnabled,
    setIsEnabled,
    clearData,
    exportData
  }
}

export default usePerformanceMonitor
