#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Performance Budget Monitor
 * Real-time performance monitoring and budget enforcement
 * Abu Dhabi autonomous vehicle fleet operations
 */

const fs = require('fs')
const path = require('path')
const { performance } = require('perf_hooks')

// Performance budgets for Abu Dhabi operations (milliseconds)
const PERFORMANCE_BUDGETS = {
  // Critical control loop latencies
  policy_evaluation: {
    budget: 25,
    warning: 20,
    critical: 25,
    description: 'Policy evaluation response time'
  },
  vehicle_command: {
    budget: 50,
    warning: 40,
    critical: 50,
    description: 'Vehicle command execution time'
  },
  route_calculation: {
    budget: 200,
    warning: 150,
    critical: 200,
    description: 'Route calculation and optimization'
  },
  ui_render: {
    budget: 30,
    warning: 25,
    critical: 30,
    description: 'UI component render time'
  },
  
  // Abu Dhabi specific operations
  map_tile_load: {
    budget: 500,
    warning: 400,
    critical: 500,
    description: 'Map tile loading (OpenStreetMap/Google)'
  },
  weather_fusion: {
    budget: 100,
    warning: 80,
    critical: 100,
    description: 'Weather data fusion from multiple sources'
  },
  emergency_response: {
    budget: 10,
    warning: 8,
    critical: 10,
    description: 'Emergency stop command propagation'
  },
  
  // System operations
  database_query: {
    budget: 100,
    warning: 80,
    critical: 100,
    description: 'Database query execution'
  },
  api_response: {
    budget: 200,
    warning: 150,
    critical: 200,
    description: 'API endpoint response time'
  },
  telemetry_ingestion: {
    budget: 50,
    warning: 40,
    critical: 50,
    description: 'Telemetry data ingestion per message'
  }
}

class PerformanceMonitor {
  constructor() {
    this.measurements = new Map()
    this.violations = []
    this.isMonitoring = false
    this.metricsInterval = null
  }

  start() {
    if (this.isMonitoring) {
      console.warn('Performance monitor already running')
      return
    }

    console.info('🚀 Starting AtlasMesh Fleet OS Performance Monitor')
    console.info('📍 Monitoring Abu Dhabi autonomous vehicle operations')
    
    this.isMonitoring = true
    this.startMetricsCollection()
    this.setupProcessMonitoring()
    
    console.info('✅ Performance monitoring active')
  }

  stop() {
    if (!this.isMonitoring) {
      return
    }

    console.info('🛑 Stopping performance monitor')
    this.isMonitoring = false
    
    if (this.metricsInterval) {
      clearInterval(this.metricsInterval)
      this.metricsInterval = null
    }

    this.generateReport()
  }

  // Start a performance measurement
  startMeasurement(operation, metadata = {}) {
    const measurementId = `${operation}_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
    
    this.measurements.set(measurementId, {
      operation,
      startTime: performance.now(),
      metadata: {
        ...metadata,
        timestamp: new Date().toISOString(),
        region: 'abu_dhabi'
      }
    })

    return measurementId
  }

  // End a performance measurement
  endMeasurement(measurementId) {
    const measurement = this.measurements.get(measurementId)
    if (!measurement) {
      console.warn(`Measurement not found: ${measurementId}`)
      return null
    }

    const endTime = performance.now()
    const duration = endTime - measurement.startTime
    
    measurement.endTime = endTime
    measurement.duration = duration

    // Check against budget
    const budget = PERFORMANCE_BUDGETS[measurement.operation]
    if (budget) {
      this.checkBudget(measurement, budget)
    }

    this.measurements.delete(measurementId)
    return measurement
  }

  // Check if measurement violates performance budget
  checkBudget(measurement, budget) {
    const { duration, operation, metadata } = measurement
    
    if (duration > budget.critical) {
      this.recordViolation('critical', measurement, budget)
    } else if (duration > budget.warning) {
      this.recordViolation('warning', measurement, budget)
    }

    // Log all measurements for analysis
    console.debug(`Performance: ${operation} took ${duration.toFixed(2)}ms (budget: ${budget.budget}ms)`)
  }

  // Record a performance budget violation
  recordViolation(severity, measurement, budget) {
    const violation = {
      severity,
      operation: measurement.operation,
      duration: measurement.duration,
      budget: budget.budget,
      overage: measurement.duration - budget.budget,
      percentage: ((measurement.duration - budget.budget) / budget.budget * 100).toFixed(1),
      timestamp: measurement.metadata.timestamp,
      metadata: measurement.metadata
    }

    this.violations.push(violation)

    // Log violation
    const emoji = severity === 'critical' ? '🚨' : '⚠️'
    console.warn(`${emoji} Performance violation: ${measurement.operation}`)
    console.warn(`   Duration: ${measurement.duration.toFixed(2)}ms (budget: ${budget.budget}ms)`)
    console.warn(`   Overage: +${violation.overage.toFixed(2)}ms (+${violation.percentage}%)`)

    // For critical violations in Abu Dhabi operations, trigger alerts
    if (severity === 'critical' && this.isCriticalOperation(measurement.operation)) {
      this.triggerAlert(violation)
    }
  }

  // Check if operation is critical for Abu Dhabi fleet operations
  isCriticalOperation(operation) {
    const criticalOps = [
      'policy_evaluation',
      'vehicle_command',
      'emergency_response',
      'route_calculation'
    ]
    return criticalOps.includes(operation)
  }

  // Trigger alert for critical performance violations
  triggerAlert(violation) {
    console.error('🚨 CRITICAL PERFORMANCE ALERT 🚨')
    console.error(`Operation: ${violation.operation}`)
    console.error(`Duration: ${violation.duration.toFixed(2)}ms`)
    console.error(`Budget exceeded by: ${violation.percentage}%`)
    console.error('Immediate attention required for Abu Dhabi fleet safety!')

    // In production, this would integrate with alerting systems
    // - PagerDuty for on-call engineers
    // - Slack notifications
    // - SMS alerts for critical operations
  }

  // Start collecting system metrics
  startMetricsCollection() {
    this.metricsInterval = setInterval(() => {
      this.collectSystemMetrics()
    }, 5000) // Collect every 5 seconds
  }

  // Collect system performance metrics
  collectSystemMetrics() {
    const memUsage = process.memoryUsage()
    const cpuUsage = process.cpuUsage()
    
    const metrics = {
      timestamp: new Date().toISOString(),
      memory: {
        rss: Math.round(memUsage.rss / 1024 / 1024), // MB
        heapUsed: Math.round(memUsage.heapUsed / 1024 / 1024), // MB
        heapTotal: Math.round(memUsage.heapTotal / 1024 / 1024), // MB
        external: Math.round(memUsage.external / 1024 / 1024) // MB
      },
      cpu: {
        user: cpuUsage.user,
        system: cpuUsage.system
      },
      uptime: Math.round(process.uptime()),
      violations: this.violations.length
    }

    // Check memory usage against budgets
    if (metrics.memory.heapUsed > 512) { // 512MB warning threshold
      console.warn(`⚠️ High memory usage: ${metrics.memory.heapUsed}MB`)
    }

    // Log metrics for monitoring systems
    console.debug('System metrics:', JSON.stringify(metrics))
  }

  // Setup process monitoring for uncaught issues
  setupProcessMonitoring() {
    // Monitor event loop lag
    const start = process.hrtime()
    setImmediate(() => {
      const delta = process.hrtime(start)
      const nanosec = delta[0] * 1e9 + delta[1]
      const millisec = nanosec / 1e6
      
      if (millisec > 10) { // 10ms event loop lag threshold
        console.warn(`⚠️ Event loop lag: ${millisec.toFixed(2)}ms`)
      }
    })

    // Monitor unhandled promise rejections
    process.on('unhandledRejection', (reason, promise) => {
      console.error('🚨 Unhandled Promise Rejection:', reason)
      this.recordViolation('critical', {
        operation: 'unhandled_promise_rejection',
        duration: 0,
        metadata: { reason: String(reason) }
      }, { budget: 0 })
    })
  }

  // Generate performance report
  generateReport() {
    console.info('\n📊 AtlasMesh Fleet OS Performance Report')
    console.info('=' .repeat(50))
    
    // Summary statistics
    const totalViolations = this.violations.length
    const criticalViolations = this.violations.filter(v => v.severity === 'critical').length
    const warningViolations = this.violations.filter(v => v.severity === 'warning').length

    console.info(`Total violations: ${totalViolations}`)
    console.info(`Critical violations: ${criticalViolations}`)
    console.info(`Warning violations: ${warningViolations}`)

    if (totalViolations === 0) {
      console.info('✅ All operations within performance budgets!')
      return
    }

    // Group violations by operation
    const violationsByOperation = {}
    this.violations.forEach(violation => {
      if (!violationsByOperation[violation.operation]) {
        violationsByOperation[violation.operation] = []
      }
      violationsByOperation[violation.operation].push(violation)
    })

    console.info('\n🎯 Performance Budget Analysis:')
    Object.entries(PERFORMANCE_BUDGETS).forEach(([operation, budget]) => {
      const violations = violationsByOperation[operation] || []
      const status = violations.length === 0 ? '✅' : 
                   violations.some(v => v.severity === 'critical') ? '🚨' : '⚠️'
      
      console.info(`${status} ${operation}: ${budget.budget}ms budget (${violations.length} violations)`)
      
      if (violations.length > 0) {
        const avgOverage = violations.reduce((sum, v) => sum + v.overage, 0) / violations.length
        console.info(`   Average overage: +${avgOverage.toFixed(2)}ms`)
      }
    })

    // Abu Dhabi specific recommendations
    console.info('\n💡 Abu Dhabi Fleet Optimization Recommendations:')
    
    if (violationsByOperation.policy_evaluation?.length > 0) {
      console.info('• Optimize policy evaluation for UAE traffic regulations')
    }
    
    if (violationsByOperation.route_calculation?.length > 0) {
      console.info('• Consider caching common Abu Dhabi routes')
    }
    
    if (violationsByOperation.map_tile_load?.length > 0) {
      console.info('• Implement local map tile caching for Abu Dhabi region')
    }

    // Save detailed report
    const reportPath = path.join(__dirname, 'performance-report.json')
    const detailedReport = {
      timestamp: new Date().toISOString(),
      summary: {
        totalViolations,
        criticalViolations,
        warningViolations
      },
      budgets: PERFORMANCE_BUDGETS,
      violations: this.violations,
      violationsByOperation
    }

    fs.writeFileSync(reportPath, JSON.stringify(detailedReport, null, 2))
    console.info(`\n📄 Detailed report saved: ${reportPath}`)
  }

  // Get current performance statistics
  getStats() {
    return {
      isMonitoring: this.isMonitoring,
      activeMeasurements: this.measurements.size,
      totalViolations: this.violations.length,
      criticalViolations: this.violations.filter(v => v.severity === 'critical').length,
      budgets: PERFORMANCE_BUDGETS
    }
  }
}

// Singleton instance
const performanceMonitor = new PerformanceMonitor()

// CLI interface
if (require.main === module) {
  const command = process.argv[2]
  
  switch (command) {
    case 'start':
      performanceMonitor.start()
      
      // Keep process alive for monitoring
      process.on('SIGINT', () => {
        performanceMonitor.stop()
        process.exit(0)
      })
      
      process.on('SIGTERM', () => {
        performanceMonitor.stop()
        process.exit(0)
      })
      
      break
      
    case 'report':
      performanceMonitor.generateReport()
      break
      
    case 'budgets':
      console.info('AtlasMesh Fleet OS Performance Budgets:')
      Object.entries(PERFORMANCE_BUDGETS).forEach(([operation, budget]) => {
        console.info(`${operation}: ${budget.budget}ms - ${budget.description}`)
      })
      break
      
    default:
      console.info('AtlasMesh Fleet OS Performance Monitor')
      console.info('Usage:')
      console.info('  node performance-monitor.js start   - Start monitoring')
      console.info('  node performance-monitor.js report  - Generate report')
      console.info('  node performance-monitor.js budgets - Show budgets')
  }
}

module.exports = {
  PerformanceMonitor,
  performanceMonitor,
  PERFORMANCE_BUDGETS
}
