// AtlasMesh Fleet OS - K6 Load Testing Script
// Comprehensive load testing for 100k+ vehicles scenario

import http from 'k6/http';
import { check, sleep, group } from 'k6';
import { Rate, Trend, Counter, Gauge } from 'k6/metrics';
import { randomIntBetween, randomItem } from 'https://jslib.k6.io/k6-utils/1.2.0/index.js';

// Custom metrics
const fleetApiResponseTime = new Trend('fleet_api_response_time');
const vehicleGatewayResponseTime = new Trend('vehicle_gateway_response_time');
const policyEngineResponseTime = new Trend('policy_engine_response_time');
const errorRate = new Rate('error_rate');
const successfulRequests = new Counter('successful_requests');
const failedRequests = new Counter('failed_requests');
const activeVehicles = new Gauge('active_vehicles');

// Test configuration
export const options = {
  scenarios: {
    // Scenario 1: Normal Load Testing
    normal_load: {
      executor: 'ramping-vus',
      startVUs: 10,
      stages: [
        { duration: '5m', target: 100 },   // Ramp up to 100 users
        { duration: '10m', target: 500 },  // Ramp up to 500 users
        { duration: '15m', target: 1000 }, // Ramp up to 1000 users
        { duration: '20m', target: 1000 }, // Stay at 1000 users
        { duration: '5m', target: 0 },     // Ramp down
      ],
      gracefulRampDown: '30s',
    },
    
    // Scenario 2: Stress Testing
    stress_test: {
      executor: 'ramping-vus',
      startVUs: 100,
      stages: [
        { duration: '2m', target: 1000 },  // Fast ramp up
        { duration: '5m', target: 2000 },  // Stress level
        { duration: '5m', target: 5000 },  // High stress
        { duration: '3m', target: 0 },     // Fast ramp down
      ],
      gracefulRampDown: '30s',
      startTime: '45m', // Start after normal load test
    },
    
    // Scenario 3: Spike Testing
    spike_test: {
      executor: 'ramping-vus',
      startVUs: 50,
      stages: [
        { duration: '30s', target: 10000 }, // Sudden spike
        { duration: '2m', target: 10000 },  // Hold spike
        { duration: '30s', target: 50 },    // Drop back
      ],
      startTime: '60m', // Start after stress test
    },
    
    // Scenario 4: Vehicle Telemetry Simulation
    vehicle_telemetry: {
      executor: 'constant-vus',
      vus: 100000, // Simulate 100k vehicles
      duration: '30m',
      exec: 'vehicleTelemetryTest',
      startTime: '0s',
    },
  },
  
  thresholds: {
    // Performance budgets
    'http_req_duration': ['p(95)<500'], // 95% of requests under 500ms
    'http_req_duration{name:fleet_api}': ['p(95)<300'], // Fleet API under 300ms
    'http_req_duration{name:vehicle_gateway}': ['p(95)<200'], // Vehicle Gateway under 200ms
    'http_req_duration{name:policy_engine}': ['p(95)<400'], // Policy Engine under 400ms
    'error_rate': ['rate<0.01'], // Error rate under 1%
    'http_req_failed': ['rate<0.05'], // Failed requests under 5%
  },
};

// Test data
const fleets = ['fleet-001', 'fleet-002', 'fleet-003', 'fleet-004', 'fleet-005'];
const vehicleTypes = ['sedan', 'suv', 'truck', 'van', 'bus'];
const driverIds = Array.from({length: 10000}, (_, i) => `driver-${String(i).padStart(5, '0')}`);
const vehicleIds = Array.from({length: 100000}, (_, i) => `vehicle-${String(i).padStart(6, '0')}`);

// Base URLs
const BASE_URL = __ENV.FLEET_API_BASE_URL || 'http://fleet-manager:8080';
const VEHICLE_GATEWAY_URL = __ENV.VEHICLE_GATEWAY_URL || 'http://vehicle-gateway:8080';
const POLICY_ENGINE_URL = __ENV.POLICY_ENGINE_URL || 'http://policy-engine:8080';

// Authentication token (mock)
const AUTH_TOKEN = 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...';

// Common headers
const headers = {
  'Content-Type': 'application/json',
  'Authorization': AUTH_TOKEN,
  'User-Agent': 'K6-LoadTest/1.0',
};

// Setup function
export function setup() {
  console.log('🚀 Starting AtlasMesh Fleet OS Load Test');
  console.log(`📊 Target: ${vehicleIds.length} vehicles across ${fleets.length} fleets`);
  
  // Warm up the services
  group('Service Warmup', () => {
    http.get(`${BASE_URL}/health`, { headers });
    http.get(`${VEHICLE_GATEWAY_URL}/health`, { headers });
    http.get(`${POLICY_ENGINE_URL}/health`, { headers });
  });
  
  return {
    fleets,
    vehicleTypes,
    driverIds,
    vehicleIds,
  };
}

// Main test function
export default function(data) {
  const vehicleId = randomItem(data.vehicleIds);
  const fleetId = randomItem(data.fleets);
  const driverId = randomItem(data.driverIds);
  
  group('Fleet Management API Tests', () => {
    // Test 1: Get Fleet Status
    group('Get Fleet Status', () => {
      const response = http.get(`${BASE_URL}/api/v1/fleets/${fleetId}`, {
        headers,
        tags: { name: 'fleet_api' },
      });
      
      check(response, {
        'fleet status is 200': (r) => r.status === 200,
        'fleet response time < 300ms': (r) => r.timings.duration < 300,
        'fleet has valid data': (r) => {
          try {
            const data = JSON.parse(r.body);
            return data.fleet_id && data.name;
          } catch {
            return false;
          }
        },
      });
      
      fleetApiResponseTime.add(response.timings.duration);
      
      if (response.status !== 200) {
        errorRate.add(1);
        failedRequests.add(1);
      } else {
        successfulRequests.add(1);
      }
    });
    
    // Test 2: Get Vehicle Details
    group('Get Vehicle Details', () => {
      const response = http.get(`${BASE_URL}/api/v1/vehicles/${vehicleId}`, {
        headers,
        tags: { name: 'fleet_api' },
      });
      
      check(response, {
        'vehicle status is 200 or 404': (r) => [200, 404].includes(r.status),
        'vehicle response time < 250ms': (r) => r.timings.duration < 250,
      });
      
      fleetApiResponseTime.add(response.timings.duration);
    });
    
    // Test 3: Update Vehicle Location
    group('Update Vehicle Location', () => {
      const locationData = {
        vehicle_id: vehicleId,
        latitude: 25.2048 + (Math.random() - 0.5) * 0.1, // Dubai area
        longitude: 55.2708 + (Math.random() - 0.5) * 0.1,
        speed: randomIntBetween(0, 120),
        heading: randomIntBetween(0, 360),
        timestamp: new Date().toISOString(),
      };
      
      const response = http.put(`${BASE_URL}/api/v1/vehicles/${vehicleId}/location`, 
        JSON.stringify(locationData), 
        { 
          headers,
          tags: { name: 'fleet_api' },
        }
      );
      
      check(response, {
        'location update is 200': (r) => r.status === 200,
        'location update time < 200ms': (r) => r.timings.duration < 200,
      });
      
      fleetApiResponseTime.add(response.timings.duration);
    });
  });
  
  group('Vehicle Gateway Tests', () => {
    // Test 4: Vehicle Telemetry Ingestion
    group('Vehicle Telemetry', () => {
      const telemetryData = {
        vehicle_id: vehicleId,
        timestamp: new Date().toISOString(),
        metrics: {
          speed: randomIntBetween(0, 120),
          fuel_level: randomIntBetween(10, 100),
          battery_level: randomIntBetween(20, 100),
          engine_temperature: randomIntBetween(70, 95),
          tire_pressure: [32, 33, 31, 32],
          odometer: randomIntBetween(10000, 200000),
        },
        location: {
          latitude: 25.2048 + (Math.random() - 0.5) * 0.1,
          longitude: 55.2708 + (Math.random() - 0.5) * 0.1,
          altitude: randomIntBetween(0, 100),
        },
        status: randomItem(['active', 'idle', 'maintenance']),
      };
      
      const response = http.post(`${VEHICLE_GATEWAY_URL}/api/v1/telemetry`, 
        JSON.stringify(telemetryData), 
        { 
          headers,
          tags: { name: 'vehicle_gateway' },
        }
      );
      
      check(response, {
        'telemetry ingestion is 200 or 202': (r) => [200, 202].includes(r.status),
        'telemetry response time < 150ms': (r) => r.timings.duration < 150,
      });
      
      vehicleGatewayResponseTime.add(response.timings.duration);
    });
    
    // Test 5: Vehicle Commands
    group('Vehicle Commands', () => {
      const command = {
        vehicle_id: vehicleId,
        command_type: randomItem(['start_engine', 'stop_engine', 'lock_doors', 'unlock_doors', 'honk_horn']),
        parameters: {},
        priority: randomItem(['low', 'medium', 'high']),
        expires_at: new Date(Date.now() + 300000).toISOString(), // 5 minutes
      };
      
      const response = http.post(`${VEHICLE_GATEWAY_URL}/api/v1/commands`, 
        JSON.stringify(command), 
        { 
          headers,
          tags: { name: 'vehicle_gateway' },
        }
      );
      
      check(response, {
        'command is 200 or 202': (r) => [200, 202].includes(r.status),
        'command response time < 100ms': (r) => r.timings.duration < 100,
      });
      
      vehicleGatewayResponseTime.add(response.timings.duration);
    });
  });
  
  group('Policy Engine Tests', () => {
    // Test 6: Policy Evaluation
    group('Policy Evaluation', () => {
      const policyRequest = {
        vehicle_id: vehicleId,
        driver_id: driverId,
        fleet_id: fleetId,
        action: randomItem(['start_trip', 'end_trip', 'emergency_stop', 'maintenance_mode']),
        context: {
          location: {
            latitude: 25.2048 + (Math.random() - 0.5) * 0.1,
            longitude: 55.2708 + (Math.random() - 0.5) * 0.1,
          },
          time_of_day: new Date().getHours(),
          weather: randomItem(['clear', 'rain', 'sandstorm', 'fog']),
          traffic_density: randomItem(['low', 'medium', 'high']),
        },
      };
      
      const response = http.post(`${POLICY_ENGINE_URL}/api/v1/evaluate`, 
        JSON.stringify(policyRequest), 
        { 
          headers,
          tags: { name: 'policy_engine' },
        }
      );
      
      check(response, {
        'policy evaluation is 200': (r) => r.status === 200,
        'policy response time < 400ms': (r) => r.timings.duration < 400,
        'policy has decision': (r) => {
          try {
            const data = JSON.parse(r.body);
            return data.decision && ['allow', 'deny', 'conditional'].includes(data.decision);
          } catch {
            return false;
          }
        },
      });
      
      policyEngineResponseTime.add(response.timings.duration);
    });
  });
  
  // Random sleep between 1-5 seconds to simulate realistic user behavior
  sleep(randomIntBetween(1, 5));
}

// Vehicle Telemetry Test (for 100k vehicles simulation)
export function vehicleTelemetryTest(data) {
  const vehicleId = randomItem(data.vehicleIds);
  
  // Simulate continuous telemetry from vehicles
  const telemetryData = {
    vehicle_id: vehicleId,
    timestamp: new Date().toISOString(),
    metrics: {
      speed: randomIntBetween(0, 120),
      fuel_level: randomIntBetween(10, 100),
      battery_level: randomIntBetween(20, 100),
      engine_temperature: randomIntBetween(70, 95),
      gps_accuracy: randomIntBetween(1, 10),
    },
    location: {
      latitude: 25.2048 + (Math.random() - 0.5) * 0.5, // Wider Dubai area
      longitude: 55.2708 + (Math.random() - 0.5) * 0.5,
    },
    status: randomItem(['active', 'idle']),
  };
  
  const response = http.post(`${VEHICLE_GATEWAY_URL}/api/v1/telemetry`, 
    JSON.stringify(telemetryData), 
    { 
      headers,
      tags: { name: 'vehicle_telemetry' },
    }
  );
  
  check(response, {
    'telemetry is accepted': (r) => [200, 202].includes(r.status),
    'telemetry response < 100ms': (r) => r.timings.duration < 100,
  });
  
  // Update active vehicles gauge
  if (response.status === 200 || response.status === 202) {
    activeVehicles.add(1);
  }
  
  // Sleep for 5 seconds (telemetry frequency)
  sleep(5);
}

// Teardown function
export function teardown(data) {
  console.log('🏁 Load test completed');
  console.log(`📈 Total vehicles simulated: ${data.vehicleIds.length}`);
  console.log('📊 Check Grafana dashboard for detailed metrics');
}

// Helper functions for realistic data generation
function generateRealisticTrip() {
  const dubaiLocations = [
    { name: 'Dubai Mall', lat: 25.1972, lng: 55.2744 },
    { name: 'Burj Khalifa', lat: 25.1963, lng: 55.2744 },
    { name: 'Dubai Marina', lat: 25.0657, lng: 55.1393 },
    { name: 'JBR Beach', lat: 25.0657, lng: 55.1393 },
    { name: 'Dubai Airport', lat: 25.2532, lng: 55.3657 },
    { name: 'Business Bay', lat: 25.1877, lng: 55.2632 },
    { name: 'DIFC', lat: 25.2138, lng: 55.2824 },
    { name: 'Dubai Creek', lat: 25.2297, lng: 55.3414 },
  ];
  
  const pickup = randomItem(dubaiLocations);
  const dropoff = randomItem(dubaiLocations.filter(loc => loc.name !== pickup.name));
  
  return {
    pickup_location: pickup,
    dropoff_location: dropoff,
    estimated_distance: calculateDistance(pickup.lat, pickup.lng, dropoff.lat, dropoff.lng),
  };
}

function calculateDistance(lat1, lng1, lat2, lng2) {
  const R = 6371; // Earth's radius in km
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLng = (lng2 - lng1) * Math.PI / 180;
  const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLng/2) * Math.sin(dLng/2);
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  return R * c;
}

// Performance budget validation
export function handleSummary(data) {
  const budgets = {
    'http_req_duration_p95': 500,
    'fleet_api_response_time_p95': 300,
    'vehicle_gateway_response_time_p95': 200,
    'policy_engine_response_time_p95': 400,
    'error_rate': 0.01,
  };
  
  const results = {
    'http_req_duration_p95': data.metrics.http_req_duration.values.p95,
    'fleet_api_response_time_p95': data.metrics.fleet_api_response_time?.values?.p95 || 0,
    'vehicle_gateway_response_time_p95': data.metrics.vehicle_gateway_response_time?.values?.p95 || 0,
    'policy_engine_response_time_p95': data.metrics.policy_engine_response_time?.values?.p95 || 0,
    'error_rate': data.metrics.error_rate?.values?.rate || 0,
  };
  
  let budgetViolations = [];
  
  for (const [metric, budget] of Object.entries(budgets)) {
    if (results[metric] > budget) {
      budgetViolations.push({
        metric,
        budget,
        actual: results[metric],
        violation: ((results[metric] - budget) / budget * 100).toFixed(2) + '%'
      });
    }
  }
  
  const summary = {
    test_summary: {
      start_time: new Date(data.state.testRunDurationMs).toISOString(),
      duration_ms: data.state.testRunDurationMs,
      total_requests: data.metrics.http_reqs.values.count,
      successful_requests: data.metrics.successful_requests?.values?.count || 0,
      failed_requests: data.metrics.failed_requests?.values?.count || 0,
      error_rate: results.error_rate,
    },
    performance_budgets: {
      violations: budgetViolations,
      passed: budgetViolations.length === 0,
    },
    metrics: results,
  };
  
  return {
    'summary.json': JSON.stringify(summary, null, 2),
    'stdout': `
🎯 Performance Test Summary
==========================
Total Requests: ${summary.test_summary.total_requests}
Success Rate: ${((1 - summary.test_summary.error_rate) * 100).toFixed(2)}%
Performance Budget: ${summary.performance_budgets.passed ? '✅ PASSED' : '❌ FAILED'}

${budgetViolations.length > 0 ? 
  '⚠️  Budget Violations:\n' + 
  budgetViolations.map(v => `   ${v.metric}: ${v.actual}ms (budget: ${v.budget}ms, +${v.violation})`).join('\n')
  : '✅ All performance budgets met'}
    `,
  };
}
