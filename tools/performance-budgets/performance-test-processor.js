// AtlasMesh Fleet OS - Performance Test Processor
// Custom functions and validations for Abu Dhabi autonomous vehicle fleet testing

module.exports = {
  // Custom template functions for Artillery.js
  $randomFloat: (min, max) => {
    return (Math.random() * (max - min) + min).toFixed(6)
  },

  $timestamp: () => {
    return new Date().toISOString()
  },

  $randomString: (length = 8) => {
    return Math.random().toString(36).substring(2, 2 + length)
  },

  $pick: (array) => {
    return array[Math.floor(Math.random() * array.length)]
  },

  // Abu Dhabi specific functions
  $abuDhabiVehicleId: () => {
    const prefixes = ['AUH', 'ALD', 'ADH'] // Abu Dhabi, Al Ain, Al Dhafra
    const prefix = prefixes[Math.floor(Math.random() * prefixes.length)]
    const number = Math.floor(Math.random() * 9000) + 1000
    return `${prefix}_${number}`
  },

  $abuDhabiLocation: () => {
    // Generate random location within Abu Dhabi bounds
    const locations = [
      { lat: 24.4539, lng: 54.3773, name: 'Abu Dhabi City Center' },
      { lat: 24.4764, lng: 54.3705, name: 'Corniche' },
      { lat: 24.4267, lng: 54.4344, name: 'Yas Island' },
      { lat: 24.2092, lng: 55.7497, name: 'Al Ain' },
      { lat: 24.5089, lng: 54.3758, name: 'Sheikh Zayed Mosque' }
    ]
    
    const baseLocation = locations[Math.floor(Math.random() * locations.length)]
    
    // Add some random variation (within ~5km radius)
    const latVariation = (Math.random() - 0.5) * 0.1
    const lngVariation = (Math.random() - 0.5) * 0.1
    
    return {
      lat: (baseLocation.lat + latVariation).toFixed(6),
      lng: (baseLocation.lng + lngVariation).toFixed(6),
      area: baseLocation.name
    }
  },

  $uaeTimeContext: () => {
    const now = new Date()
    const uaeTime = new Date(now.getTime() + (4 * 60 * 60 * 1000)) // UTC+4
    
    const hour = uaeTime.getHours()
    const dayOfWeek = uaeTime.getDay()
    
    // Determine context
    let timeContext = 'normal'
    let prayerTimeProximity = Math.floor(Math.random() * 60)
    
    // Peak hours in Abu Dhabi
    if ((hour >= 7 && hour <= 9) || (hour >= 17 && hour <= 19)) {
      timeContext = 'peak_traffic'
    }
    
    // Prayer times (approximate)
    const prayerHours = [5, 12, 15, 18, 19] // Fajr, Dhuhr, Asr, Maghrib, Isha
    if (prayerHours.includes(hour)) {
      prayerTimeProximity = Math.floor(Math.random() * 15) // Within 15 minutes
    }
    
    // Weekend (Friday-Saturday in UAE)
    const isWeekend = dayOfWeek === 5 || dayOfWeek === 6
    
    return {
      time_context: timeContext,
      prayer_time_proximity: prayerTimeProximity,
      is_weekend: isWeekend,
      hour: hour,
      uae_time: uaeTime.toISOString()
    }
  },

  // Weather conditions specific to Abu Dhabi
  $abuDhabiWeather: () => {
    const month = new Date().getMonth() + 1
    let conditions, temperature, humidity, visibility
    
    // Summer months (May-September) - hot and humid
    if (month >= 5 && month <= 9) {
      temperature = Math.floor(Math.random() * 15) + 35 // 35-50°C
      humidity = Math.floor(Math.random() * 30) + 60    // 60-90%
      conditions = ['clear', 'hot', 'humid'][Math.floor(Math.random() * 3)]
      visibility = Math.floor(Math.random() * 10) + 10  // 10-20km
    }
    // Winter months (November-March) - mild
    else if (month >= 11 || month <= 3) {
      temperature = Math.floor(Math.random() * 15) + 20 // 20-35°C
      humidity = Math.floor(Math.random() * 40) + 40    // 40-80%
      conditions = ['clear', 'pleasant', 'cloudy'][Math.floor(Math.random() * 3)]
      visibility = Math.floor(Math.random() * 15) + 15  // 15-30km
    }
    // Transition months - variable with sandstorms
    else {
      temperature = Math.floor(Math.random() * 20) + 25 // 25-45°C
      humidity = Math.floor(Math.random() * 50) + 30    // 30-80%
      conditions = ['clear', 'dusty', 'sandstorm', 'windy'][Math.floor(Math.random() * 4)]
      visibility = Math.floor(Math.random() * 20) + 5   // 5-25km
      
      // Reduce visibility during sandstorms
      if (conditions === 'sandstorm') {
        visibility = Math.floor(Math.random() * 3) + 1  // 1-3km
      }
    }
    
    return {
      temperature,
      humidity,
      conditions,
      visibility,
      wind_speed: Math.floor(Math.random() * 30) + 5, // 5-35 km/h
      uv_index: Math.floor(Math.random() * 6) + 5      // 5-10 (high in UAE)
    }
  },

  // Performance validation hooks
  beforeRequest: (requestParams, context, ee, next) => {
    // Add Abu Dhabi specific headers
    requestParams.headers = requestParams.headers || {}
    requestParams.headers['X-Region'] = 'abu-dhabi'
    requestParams.headers['X-Timezone'] = 'Asia/Dubai'
    requestParams.headers['X-Test-Context'] = 'performance'
    
    // Add request timestamp for latency tracking
    context.vars.requestStartTime = Date.now()
    
    return next()
  },

  afterResponse: (requestParams, response, context, ee, next) => {
    // Calculate request latency
    const latency = Date.now() - context.vars.requestStartTime
    
    // Emit custom metrics
    ee.emit('counter', 'atlasmesh.requests.total', 1)
    ee.emit('histogram', 'atlasmesh.request.latency', latency)
    
    // Check Abu Dhabi specific performance requirements
    if (requestParams.url.includes('/vehicles/') && requestParams.url.includes('/commands')) {
      // Vehicle commands must be under 50ms for safety
      if (latency > 50) {
        ee.emit('counter', 'atlasmesh.vehicle_command.slow', 1)
        console.warn(`⚠️ Slow vehicle command: ${latency}ms (> 50ms budget)`)
      }
    }
    
    if (requestParams.url.includes('/policies/evaluate')) {
      // Policy evaluation must be under 25ms for real-time decisions
      if (latency > 25) {
        ee.emit('counter', 'atlasmesh.policy_evaluation.slow', 1)
        console.warn(`⚠️ Slow policy evaluation: ${latency}ms (> 25ms budget)`)
      }
    }
    
    // Check for UAE-specific error responses
    if (response.statusCode >= 400) {
      ee.emit('counter', 'atlasmesh.errors.total', 1)
      
      if (response.body && response.body.includes('UAE')) {
        ee.emit('counter', 'atlasmesh.errors.uae_specific', 1)
      }
    }
    
    return next()
  },

  // Test completion validation
  validateResults: (stats) => {
    console.log('\n🏁 AtlasMesh Fleet OS Performance Test Results')
    console.log('=' .repeat(60))
    
    // Abu Dhabi specific validations
    const criticalEndpoints = [
      'Vehicle Command',
      'Policy Evaluation',
      'Fleet Overview'
    ]
    
    let allPassed = true
    
    criticalEndpoints.forEach(endpoint => {
      const endpointStats = stats.requestsCompleted[endpoint]
      if (endpointStats) {
        const p95 = endpointStats.p95
        const errorRate = (endpointStats.errors / endpointStats.total) * 100
        
        console.log(`\n📊 ${endpoint}:`)
        console.log(`   P95 Latency: ${p95}ms`)
        console.log(`   Error Rate: ${errorRate.toFixed(2)}%`)
        
        // Validate against Abu Dhabi requirements
        let endpointPassed = true
        
        if (endpoint === 'Vehicle Command' && p95 > 50) {
          console.log(`   ❌ FAILED: P95 latency ${p95}ms > 50ms budget`)
          endpointPassed = false
        } else if (endpoint === 'Policy Evaluation' && p95 > 25) {
          console.log(`   ❌ FAILED: P95 latency ${p95}ms > 25ms budget`)
          endpointPassed = false
        } else if (p95 > 200) {
          console.log(`   ❌ FAILED: P95 latency ${p95}ms > 200ms budget`)
          endpointPassed = false
        }
        
        if (errorRate > 1) {
          console.log(`   ❌ FAILED: Error rate ${errorRate}% > 1% budget`)
          endpointPassed = false
        }
        
        if (endpointPassed) {
          console.log(`   ✅ PASSED: Meets Abu Dhabi performance requirements`)
        } else {
          allPassed = false
        }
      }
    })
    
    console.log('\n' + '=' .repeat(60))
    if (allPassed) {
      console.log('🎉 All performance tests PASSED for Abu Dhabi operations!')
      console.log('✅ System ready for autonomous vehicle fleet deployment')
    } else {
      console.log('❌ Performance tests FAILED - optimization required')
      console.log('⚠️  System not ready for production deployment')
      process.exit(1)
    }
  }
}

// Export functions for Artillery.js template engine
Object.keys(module.exports).forEach(key => {
  if (key.startsWith('$')) {
    global[key] = module.exports[key]
  }
})
