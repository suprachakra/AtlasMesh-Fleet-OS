#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Performance Budget Enforcer
 * 
 * Enforces performance budgets in CI/CD pipeline and production monitoring
 * Critical for autonomous vehicle safety and user experience
 */

const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const axios = require('axios');

class PerformanceBudgetEnforcer {
    constructor() {
        this.budgetsPath = path.join(__dirname, 'budgets.yaml');
        this.reportsDir = path.join(__dirname, 'reports');
        this.baselinesDir = path.join(__dirname, 'baselines');
        
        // Load performance budgets
        this.budgets = this.loadBudgets();
        
        // Configuration
        this.config = {
            environments: {
                development: 'http://localhost:8080',
                staging: 'https://api-staging.atlasmesh.com',
                production: 'https://api.atlasmesh.com'
            },
            prometheus: {
                development: 'http://localhost:9090',
                staging: 'https://prometheus-staging.atlasmesh.com',
                production: 'https://prometheus.atlasmesh.com'
            },
            testDuration: 60, // seconds
            warmupDuration: 10, // seconds
            concurrency: 10,
            timeout: 30000 // ms
        };
    }

    loadBudgets() {
        try {
            return yaml.load(fs.readFileSync(this.budgetsPath, 'utf8'));
        } catch (error) {
            console.error('❌ Failed to load performance budgets:', error.message);
            process.exit(1);
        }
    }

    async enforceAllBudgets(environment = 'development', options = {}) {
        console.log(`🎯 Enforcing performance budgets for ${environment} environment...`);
        
        try {
            // Ensure directories exist
            this.ensureDirectories();
            
            // Run performance tests
            const results = await this.runPerformanceTests(environment, options);
            
            // Validate against budgets
            const violations = this.validateBudgets(results);
            
            // Generate report
            const report = this.generateReport(results, violations, environment);
            this.saveReport(report, environment);
            
            // Check for violations
            if (violations.length > 0) {
                console.log(`❌ ${violations.length} performance budget violations found!`);
                this.logViolations(violations);
                return false;
            } else {
                console.log('✅ All performance budgets passed!');
                return true;
            }
            
        } catch (error) {
            console.error('❌ Performance budget enforcement failed:', error.message);
            return false;
        }
    }

    ensureDirectories() {
        [this.reportsDir, this.baselinesDir].forEach(dir => {
            if (!fs.existsSync(dir)) {
                fs.mkdirSync(dir, { recursive: true });
            }
        });
    }

    async runPerformanceTests(environment, options) {
        console.log('🏃 Running performance tests...');
        
        const baseUrl = this.config.environments[environment];
        const results = {
            environment: environment,
            timestamp: new Date().toISOString(),
            tests: []
        };

        // Test each budget category
        for (const [category, budget] of Object.entries(this.budgets.categories)) {
            console.log(`📊 Testing ${category}...`);
            
            const categoryResults = await this.testCategory(category, budget, baseUrl, options);
            results.tests.push(categoryResults);
        }

        return results;
    }

    async testCategory(category, budget, baseUrl, options) {
        const result = {
            category: category,
            budget: budget,
            measurements: [],
            passed: true,
            violations: []
        };

        try {
            switch (category) {
                case 'control_loop':
                    result.measurements = await this.testControlLoop(baseUrl, budget);
                    break;
                case 'policy_evaluation':
                    result.measurements = await this.testPolicyEvaluation(baseUrl, budget);
                    break;
                case 'route_calculation':
                    result.measurements = await this.testRouteCalculation(baseUrl, budget);
                    break;
                case 'api_response_times':
                    result.measurements = await this.testAPIResponseTimes(baseUrl, budget);
                    break;
                case 'edge_heartbeat':
                    result.measurements = await this.testEdgeHeartbeat(baseUrl, budget);
                    break;
                default:
                    console.warn(`⚠️  Unknown category: ${category}`);
            }

            // Validate measurements against budget
            result.violations = this.validateCategoryBudget(result.measurements, budget);
            result.passed = result.violations.length === 0;

        } catch (error) {
            result.passed = false;
            result.error = error.message;
            console.error(`❌ Error testing ${category}:`, error.message);
        }

        return result;
    }

    async testControlLoop(baseUrl, budget) {
        console.log('🔄 Testing control loop latency...');
        
        const measurements = [];
        const iterations = budget.test_iterations || 100;
        
        for (let i = 0; i < iterations; i++) {
            const startTime = Date.now();
            
            try {
                // Simulate control loop: sensor data -> policy evaluation -> vehicle command
                await this.simulateControlLoopIteration(baseUrl);
                
                const latency = Date.now() - startTime;
                measurements.push({
                    iteration: i + 1,
                    latency_ms: latency,
                    timestamp: new Date().toISOString()
                });
                
                // Small delay to avoid overwhelming the system
                await this.sleep(10);
                
            } catch (error) {
                measurements.push({
                    iteration: i + 1,
                    latency_ms: null,
                    error: error.message,
                    timestamp: new Date().toISOString()
                });
            }
        }
        
        return measurements;
    }

    async simulateControlLoopIteration(baseUrl) {
        // Simulate the critical path: telemetry -> policy -> command
        const vehicleId = 'test-vehicle-001';
        
        // 1. Submit telemetry (edge -> cloud)
        await axios.post(`${baseUrl}/telemetry/v1/vehicles/${vehicleId}/telemetry`, {
            timestamp: new Date().toISOString(),
            location: { latitude: 25.2048, longitude: 55.2708 },
            speed_kmh: 45.5,
            battery_level: 85.2,
            odometer_km: 12345.6
        }, { timeout: this.config.timeout });
        
        // 2. Evaluate policy (policy engine)
        await axios.post(`${baseUrl}/policy/v1/evaluate`, {
            vehicle_id: vehicleId,
            context: {
                location: { latitude: 25.2048, longitude: 55.2708 },
                speed_kmh: 45.5,
                weather: 'clear',
                time_of_day: 'day'
            },
            action: 'continue_route'
        }, { timeout: this.config.timeout });
        
        // 3. Send vehicle command (fleet manager -> edge)
        await axios.post(`${baseUrl}/fleet/v1/vehicles/${vehicleId}/commands`, {
            command_type: 'set_speed',
            target_speed_kmh: 50.0,
            priority: 'normal'
        }, { timeout: this.config.timeout });
    }

    async testPolicyEvaluation(baseUrl, budget) {
        console.log('⚖️  Testing policy evaluation latency...');
        
        const measurements = [];
        const iterations = budget.test_iterations || 200;
        
        // Test various policy scenarios
        const scenarios = [
            { action: 'continue_route', complexity: 'simple' },
            { action: 'change_lane', complexity: 'medium' },
            { action: 'emergency_stop', complexity: 'high' },
            { action: 'reroute', complexity: 'high' }
        ];
        
        for (let i = 0; i < iterations; i++) {
            const scenario = scenarios[i % scenarios.length];
            const startTime = Date.now();
            
            try {
                await axios.post(`${baseUrl}/policy/v1/evaluate`, {
                    vehicle_id: 'test-vehicle-001',
                    context: {
                        location: { latitude: 25.2048, longitude: 55.2708 },
                        speed_kmh: Math.random() * 60,
                        weather: ['clear', 'rain', 'fog'][Math.floor(Math.random() * 3)],
                        time_of_day: ['day', 'night'][Math.floor(Math.random() * 2)],
                        traffic_density: Math.random()
                    },
                    action: scenario.action
                }, { timeout: this.config.timeout });
                
                const latency = Date.now() - startTime;
                measurements.push({
                    iteration: i + 1,
                    scenario: scenario.action,
                    complexity: scenario.complexity,
                    latency_ms: latency,
                    timestamp: new Date().toISOString()
                });
                
            } catch (error) {
                measurements.push({
                    iteration: i + 1,
                    scenario: scenario.action,
                    latency_ms: null,
                    error: error.message,
                    timestamp: new Date().toISOString()
                });
            }
        }
        
        return measurements;
    }

    async testRouteCalculation(baseUrl, budget) {
        console.log('🗺️  Testing route calculation performance...');
        
        const measurements = [];
        const iterations = budget.test_iterations || 50;
        
        // Test routes of varying complexity
        const routeTests = [
            {
                name: 'short_urban',
                origin: { latitude: 25.2048, longitude: 55.2708 },
                destination: { latitude: 25.2100, longitude: 55.2750 },
                complexity: 'low'
            },
            {
                name: 'medium_city',
                origin: { latitude: 25.2048, longitude: 55.2708 },
                destination: { latitude: 25.1972, longitude: 55.2744 },
                complexity: 'medium'
            },
            {
                name: 'long_highway',
                origin: { latitude: 25.2048, longitude: 55.2708 },
                destination: { latitude: 24.4539, longitude: 54.3773 },
                complexity: 'high'
            }
        ];
        
        for (let i = 0; i < iterations; i++) {
            const routeTest = routeTests[i % routeTests.length];
            const startTime = Date.now();
            
            try {
                await axios.post(`${baseUrl}/routing/v1/calculate`, {
                    origin: routeTest.origin,
                    destination: routeTest.destination,
                    vehicle_type: 'ugv_themis',
                    preferences: {
                        optimize_for: 'time',
                        avoid_tolls: false,
                        avoid_highways: false
                    }
                }, { timeout: this.config.timeout });
                
                const latency = Date.now() - startTime;
                measurements.push({
                    iteration: i + 1,
                    route_type: routeTest.name,
                    complexity: routeTest.complexity,
                    latency_ms: latency,
                    timestamp: new Date().toISOString()
                });
                
            } catch (error) {
                measurements.push({
                    iteration: i + 1,
                    route_type: routeTest.name,
                    latency_ms: null,
                    error: error.message,
                    timestamp: new Date().toISOString()
                });
            }
        }
        
        return measurements;
    }

    async testAPIResponseTimes(baseUrl, budget) {
        console.log('🌐 Testing API response times...');
        
        const measurements = [];
        const endpoints = budget.endpoints || [
            { path: '/fleet/v1/vehicles', method: 'GET', name: 'list_vehicles' },
            { path: '/trip/v1/trips', method: 'GET', name: 'list_trips' },
            { path: '/fleet/v1/vehicles/test-vehicle-001', method: 'GET', name: 'get_vehicle' },
            { path: '/trip/v1/trips', method: 'POST', name: 'create_trip' }
        ];
        
        for (const endpoint of endpoints) {
            const iterations = Math.floor((budget.test_iterations || 100) / endpoints.length);
            
            for (let i = 0; i < iterations; i++) {
                const startTime = Date.now();
                
                try {
                    const config = {
                        method: endpoint.method,
                        url: `${baseUrl}${endpoint.path}`,
                        timeout: this.config.timeout
                    };
                    
                    if (endpoint.method === 'POST' && endpoint.name === 'create_trip') {
                        config.data = {
                            origin: { latitude: 25.2048, longitude: 55.2708 },
                            destination: { latitude: 25.2100, longitude: 55.2750 },
                            sector: 'ridehail',
                            priority: 'normal'
                        };
                    }
                    
                    await axios(config);
                    
                    const latency = Date.now() - startTime;
                    measurements.push({
                        iteration: i + 1,
                        endpoint: endpoint.name,
                        method: endpoint.method,
                        path: endpoint.path,
                        latency_ms: latency,
                        timestamp: new Date().toISOString()
                    });
                    
                } catch (error) {
                    measurements.push({
                        iteration: i + 1,
                        endpoint: endpoint.name,
                        method: endpoint.method,
                        path: endpoint.path,
                        latency_ms: null,
                        error: error.message,
                        timestamp: new Date().toISOString()
                    });
                }
            }
        }
        
        return measurements;
    }

    async testEdgeHeartbeat(baseUrl, budget) {
        console.log('💓 Testing edge heartbeat latency...');
        
        const measurements = [];
        const iterations = budget.test_iterations || 60; // 1 minute of heartbeats
        
        for (let i = 0; i < iterations; i++) {
            const startTime = Date.now();
            
            try {
                await axios.post(`${baseUrl}/fleet/v1/vehicles/test-vehicle-001/heartbeat`, {
                    timestamp: new Date().toISOString(),
                    status: 'online',
                    location: { latitude: 25.2048, longitude: 55.2708 },
                    battery_level: 85.0
                }, { timeout: this.config.timeout });
                
                const latency = Date.now() - startTime;
                measurements.push({
                    iteration: i + 1,
                    latency_ms: latency,
                    timestamp: new Date().toISOString()
                });
                
                // Wait 1 second between heartbeats
                await this.sleep(1000);
                
            } catch (error) {
                measurements.push({
                    iteration: i + 1,
                    latency_ms: null,
                    error: error.message,
                    timestamp: new Date().toISOString()
                });
            }
        }
        
        return measurements;
    }

    validateCategoryBudget(measurements, budget) {
        const violations = [];
        
        // Filter out failed measurements
        const validMeasurements = measurements.filter(m => m.latency_ms !== null);
        
        if (validMeasurements.length === 0) {
            violations.push({
                type: 'no_valid_measurements',
                message: 'No valid measurements collected'
            });
            return violations;
        }
        
        // Calculate percentiles
        const latencies = validMeasurements.map(m => m.latency_ms).sort((a, b) => a - b);
        const percentiles = this.calculatePercentiles(latencies);
        
        // Check each budget threshold
        for (const [metric, threshold] of Object.entries(budget.thresholds || {})) {
            const actualValue = percentiles[metric];
            
            if (actualValue > threshold) {
                violations.push({
                    type: 'threshold_exceeded',
                    metric: metric,
                    threshold: threshold,
                    actual: actualValue,
                    message: `${metric} exceeded: ${actualValue}ms > ${threshold}ms`
                });
            }
        }
        
        // Check error rate
        const errorRate = ((measurements.length - validMeasurements.length) / measurements.length) * 100;
        const maxErrorRate = budget.max_error_rate || 1; // 1% default
        
        if (errorRate > maxErrorRate) {
            violations.push({
                type: 'error_rate_exceeded',
                metric: 'error_rate',
                threshold: maxErrorRate,
                actual: errorRate,
                message: `Error rate exceeded: ${errorRate.toFixed(2)}% > ${maxErrorRate}%`
            });
        }
        
        return violations;
    }

    calculatePercentiles(sortedArray) {
        const percentiles = {};
        
        const calculatePercentile = (arr, p) => {
            const index = (p / 100) * (arr.length - 1);
            const lower = Math.floor(index);
            const upper = Math.ceil(index);
            const weight = index % 1;
            
            if (upper >= arr.length) return arr[arr.length - 1];
            return arr[lower] * (1 - weight) + arr[upper] * weight;
        };
        
        percentiles.p50 = calculatePercentile(sortedArray, 50);
        percentiles.p95 = calculatePercentile(sortedArray, 95);
        percentiles.p99 = calculatePercentile(sortedArray, 99);
        percentiles.max = sortedArray[sortedArray.length - 1];
        percentiles.min = sortedArray[0];
        percentiles.avg = sortedArray.reduce((a, b) => a + b, 0) / sortedArray.length;
        
        return percentiles;
    }

    validateBudgets(results) {
        const allViolations = [];
        
        for (const test of results.tests) {
            if (test.violations && test.violations.length > 0) {
                allViolations.push(...test.violations.map(v => ({
                    ...v,
                    category: test.category
                })));
            }
        }
        
        return allViolations;
    }

    generateReport(results, violations, environment) {
        const report = {
            summary: {
                environment: environment,
                timestamp: results.timestamp,
                total_categories: results.tests.length,
                passed_categories: results.tests.filter(t => t.passed).length,
                failed_categories: results.tests.filter(t => !t.passed).length,
                total_violations: violations.length,
                overall_passed: violations.length === 0
            },
            results: results,
            violations: violations,
            recommendations: this.generateRecommendations(violations)
        };
        
        return report;
    }

    generateRecommendations(violations) {
        const recommendations = [];
        
        // Group violations by type
        const violationsByType = violations.reduce((acc, v) => {
            acc[v.type] = acc[v.type] || [];
            acc[v.type].push(v);
            return acc;
        }, {});
        
        // Generate recommendations based on violation patterns
        if (violationsByType.threshold_exceeded) {
            const thresholdViolations = violationsByType.threshold_exceeded;
            
            recommendations.push({
                type: 'performance_optimization',
                priority: 'high',
                message: `${thresholdViolations.length} performance thresholds exceeded`,
                actions: [
                    'Review and optimize slow code paths',
                    'Consider caching strategies for frequently accessed data',
                    'Evaluate database query performance',
                    'Check for resource contention issues',
                    'Consider horizontal scaling for high-load services'
                ]
            });
        }
        
        if (violationsByType.error_rate_exceeded) {
            recommendations.push({
                type: 'reliability_improvement',
                priority: 'critical',
                message: 'Error rates exceeded acceptable thresholds',
                actions: [
                    'Investigate and fix failing requests',
                    'Implement better error handling and retry logic',
                    'Review service dependencies and timeouts',
                    'Check for resource exhaustion issues'
                ]
            });
        }
        
        return recommendations;
    }

    saveReport(report, environment) {
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const reportPath = path.join(this.reportsDir, `performance-budget-${environment}-${timestamp}.json`);
        
        fs.writeFileSync(reportPath, JSON.stringify(report, null, 2));
        
        // Also save as latest
        const latestPath = path.join(this.reportsDir, `performance-budget-${environment}-latest.json`);
        fs.writeFileSync(latestPath, JSON.stringify(report, null, 2));
        
        console.log(`📊 Report saved to: ${reportPath}`);
        
        // Generate HTML report
        this.generateHTMLReport(report, environment);
    }

    generateHTMLReport(report, environment) {
        const htmlContent = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AtlasMesh Performance Budget Report - ${environment}</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .header { text-align: center; margin-bottom: 30px; }
        .summary { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-bottom: 30px; }
        .metric { background: #f8f9fa; padding: 15px; border-radius: 6px; text-align: center; }
        .metric-value { font-size: 2em; font-weight: bold; margin-bottom: 5px; }
        .success { color: #28a745; }
        .warning { color: #ffc107; }
        .error { color: #dc3545; }
        .section { margin-bottom: 30px; }
        .test-result { margin-bottom: 20px; padding: 15px; border-radius: 6px; }
        .test-passed { background: #d4edda; border-left: 4px solid #28a745; }
        .test-failed { background: #f8d7da; border-left: 4px solid #dc3545; }
        .violations { background: #fff3cd; padding: 15px; border-radius: 6px; border-left: 4px solid #ffc107; }
        .chart { width: 100%; height: 300px; margin: 20px 0; }
    </style>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>AtlasMesh Performance Budget Report</h1>
            <p>Environment: <strong>${environment}</strong> | Generated: ${report.summary.timestamp}</p>
        </div>
        
        <div class="summary">
            <div class="metric">
                <div class="metric-value ${report.summary.overall_passed ? 'success' : 'error'}">
                    ${report.summary.overall_passed ? '✅' : '❌'}
                </div>
                <div>Overall Status</div>
            </div>
            <div class="metric">
                <div class="metric-value">${report.summary.passed_categories}/${report.summary.total_categories}</div>
                <div>Categories Passed</div>
            </div>
            <div class="metric">
                <div class="metric-value ${report.summary.total_violations === 0 ? 'success' : 'error'}">
                    ${report.summary.total_violations}
                </div>
                <div>Violations</div>
            </div>
        </div>
        
        <div class="section">
            <h2>Performance Test Results</h2>
            ${report.results.tests.map(test => `
                <div class="test-result ${test.passed ? 'test-passed' : 'test-failed'}">
                    <h3>${test.category.replace('_', ' ').toUpperCase()}</h3>
                    <p>Status: ${test.passed ? 'PASSED' : 'FAILED'} | Measurements: ${test.measurements.length}</p>
                    ${test.violations.length > 0 ? `
                        <div class="violations">
                            <h4>Violations:</h4>
                            <ul>
                                ${test.violations.map(v => `<li>${v.message}</li>`).join('')}
                            </ul>
                        </div>
                    ` : ''}
                </div>
            `).join('')}
        </div>
        
        ${report.recommendations.length > 0 ? `
            <div class="section">
                <h2>Recommendations</h2>
                <div class="violations">
                    ${report.recommendations.map(rec => `
                        <h4>${rec.type.replace('_', ' ').toUpperCase()} (${rec.priority})</h4>
                        <p>${rec.message}</p>
                        <ul>
                            ${rec.actions.map(action => `<li>${action}</li>`).join('')}
                        </ul>
                    `).join('')}
                </div>
            </div>
        ` : ''}
    </div>
</body>
</html>`;

        const htmlPath = path.join(this.reportsDir, `performance-budget-${environment}-latest.html`);
        fs.writeFileSync(htmlPath, htmlContent);
        console.log(`📊 HTML report saved to: ${htmlPath}`);
    }

    logViolations(violations) {
        console.log('\n🚨 Performance Budget Violations:');
        violations.forEach((violation, index) => {
            console.log(`${index + 1}. [${violation.category}] ${violation.message}`);
        });
        console.log('');
    }

    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

// CLI interface
if (require.main === module) {
    const args = process.argv.slice(2);
    const environment = args[0] || 'development';
    const options = {
        skipWarmup: args.includes('--skip-warmup'),
        verbose: args.includes('--verbose')
    };
    
    const enforcer = new PerformanceBudgetEnforcer();
    enforcer.enforceAllBudgets(environment, options)
        .then(success => {
            process.exit(success ? 0 : 1);
        })
        .catch(error => {
            console.error('❌ Performance budget enforcement failed:', error);
            process.exit(1);
        });
}

module.exports = PerformanceBudgetEnforcer;
