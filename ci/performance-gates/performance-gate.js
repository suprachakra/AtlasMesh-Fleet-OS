#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Performance Gate for CI/CD
 * 
 * Enforces performance budgets in CI/CD pipeline
 * Blocks deployments if performance regressions are detected
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

class PerformanceGate {
    constructor() {
        this.projectRoot = path.join(__dirname, '../..');
        this.budgetEnforcerPath = path.join(this.projectRoot, 'tools/performance-budgets/budget-enforcer.js');
        this.reportsDir = path.join(this.projectRoot, 'tools/performance-budgets/reports');
        this.baselinesDir = path.join(this.projectRoot, 'tools/performance-budgets/baselines');
        
        // CI/CD configuration
        this.config = {
            environment: process.env.CI_ENVIRONMENT || 'development',
            branch: process.env.CI_BRANCH || 'main',
            commitSha: process.env.CI_COMMIT_SHA || 'unknown',
            buildNumber: process.env.CI_BUILD_NUMBER || 'local',
            pullRequestNumber: process.env.CI_PULL_REQUEST_NUMBER || null,
            
            // Performance gate settings
            blockOnFailure: process.env.PERFORMANCE_GATE_BLOCK !== 'false',
            regressionThreshold: parseFloat(process.env.PERFORMANCE_REGRESSION_THRESHOLD) || 20, // 20% increase
            baselineBranch: process.env.PERFORMANCE_BASELINE_BRANCH || 'main',
            
            // Notification settings
            slackWebhook: process.env.SLACK_WEBHOOK_URL,
            notifyOnSuccess: process.env.NOTIFY_ON_SUCCESS === 'true',
            notifyOnFailure: process.env.NOTIFY_ON_FAILURE !== 'false'
        };
    }

    async runPerformanceGate() {
        console.log('🎯 Starting Performance Gate...');
        console.log(`Environment: ${this.config.environment}`);
        console.log(`Branch: ${this.config.branch}`);
        console.log(`Commit: ${this.config.commitSha}`);
        console.log(`Build: ${this.config.buildNumber}`);
        
        try {
            // Step 1: Run performance budget enforcement
            console.log('\n📊 Running performance budget tests...');
            const budgetResults = await this.runBudgetEnforcement();
            
            // Step 2: Compare against baseline (if available)
            console.log('\n📈 Analyzing performance trends...');
            const regressionResults = await this.analyzeRegressions(budgetResults);
            
            // Step 3: Generate comprehensive report
            console.log('\n📋 Generating performance gate report...');
            const gateReport = this.generateGateReport(budgetResults, regressionResults);
            
            // Step 4: Save results and artifacts
            this.saveGateResults(gateReport);
            
            // Step 5: Send notifications
            await this.sendNotifications(gateReport);
            
            // Step 6: Determine gate outcome
            const gateOutcome = this.determineGateOutcome(gateReport);
            
            if (gateOutcome.passed) {
                console.log('\n✅ Performance Gate PASSED');
                
                // Update baseline if this is the main branch
                if (this.config.branch === this.config.baselineBranch) {
                    await this.updateBaseline(budgetResults);
                }
                
                return true;
            } else {
                console.log('\n❌ Performance Gate FAILED');
                console.log('Reasons:');
                gateOutcome.failures.forEach(failure => {
                    console.log(`  - ${failure}`);
                });
                
                if (this.config.blockOnFailure) {
                    console.log('\n🚫 Blocking deployment due to performance gate failure');
                    return false;
                } else {
                    console.log('\n⚠️  Performance gate failed but not blocking (warning mode)');
                    return true;
                }
            }
            
        } catch (error) {
            console.error('❌ Performance gate execution failed:', error.message);
            
            // Send failure notification
            await this.sendFailureNotification(error);
            
            // Fail the gate on errors
            return false;
        }
    }

    async runBudgetEnforcement() {
        console.log('Running performance budget enforcer...');
        
        try {
            // Run the budget enforcer
            const command = `node "${this.budgetEnforcerPath}" ${this.config.environment}`;
            const output = execSync(command, { 
                encoding: 'utf8',
                cwd: this.projectRoot,
                timeout: 300000 // 5 minutes timeout
            });
            
            console.log('Budget enforcer output:', output);
            
            // Load the latest report
            const latestReportPath = path.join(this.reportsDir, `performance-budget-${this.config.environment}-latest.json`);
            
            if (!fs.existsSync(latestReportPath)) {
                throw new Error('Performance budget report not found');
            }
            
            const report = JSON.parse(fs.readFileSync(latestReportPath, 'utf8'));
            
            return {
                passed: report.summary.overall_passed,
                report: report,
                timestamp: report.summary.timestamp
            };
            
        } catch (error) {
            // If the command failed, try to get the report anyway
            const latestReportPath = path.join(this.reportsDir, `performance-budget-${this.config.environment}-latest.json`);
            
            if (fs.existsSync(latestReportPath)) {
                const report = JSON.parse(fs.readFileSync(latestReportPath, 'utf8'));
                return {
                    passed: false,
                    report: report,
                    timestamp: report.summary.timestamp,
                    error: error.message
                };
            } else {
                throw new Error(`Performance budget enforcement failed: ${error.message}`);
            }
        }
    }

    async analyzeRegressions(budgetResults) {
        console.log('Analyzing performance regressions...');
        
        const regressionResults = {
            hasBaseline: false,
            regressions: [],
            improvements: [],
            summary: {
                totalRegressions: 0,
                significantRegressions: 0,
                totalImprovements: 0
            }
        };
        
        try {
            // Load baseline data
            const baselinePath = path.join(this.baselinesDir, `${this.config.environment}-baseline.json`);
            
            if (!fs.existsSync(baselinePath)) {
                console.log('No baseline found, skipping regression analysis');
                return regressionResults;
            }
            
            const baseline = JSON.parse(fs.readFileSync(baselinePath, 'utf8'));
            regressionResults.hasBaseline = true;
            
            // Compare current results with baseline
            for (const currentTest of budgetResults.report.results.tests) {
                const baselineTest = baseline.results.tests.find(t => t.category === currentTest.category);
                
                if (!baselineTest) {
                    console.log(`No baseline data for category: ${currentTest.category}`);
                    continue;
                }
                
                const comparison = this.compareTestResults(currentTest, baselineTest);
                
                if (comparison.isRegression) {
                    regressionResults.regressions.push({
                        category: currentTest.category,
                        metric: comparison.metric,
                        baselineValue: comparison.baselineValue,
                        currentValue: comparison.currentValue,
                        percentageIncrease: comparison.percentageIncrease,
                        isSignificant: comparison.percentageIncrease > this.config.regressionThreshold
                    });
                    
                    if (comparison.percentageIncrease > this.config.regressionThreshold) {
                        regressionResults.summary.significantRegressions++;
                    }
                    regressionResults.summary.totalRegressions++;
                }
                
                if (comparison.isImprovement) {
                    regressionResults.improvements.push({
                        category: currentTest.category,
                        metric: comparison.metric,
                        baselineValue: comparison.baselineValue,
                        currentValue: comparison.currentValue,
                        percentageDecrease: comparison.percentageDecrease
                    });
                    regressionResults.summary.totalImprovements++;
                }
            }
            
        } catch (error) {
            console.error('Error analyzing regressions:', error.message);
            regressionResults.error = error.message;
        }
        
        return regressionResults;
    }

    compareTestResults(currentTest, baselineTest) {
        // Calculate percentiles for both tests
        const currentPercentiles = this.calculatePercentiles(
            currentTest.measurements.filter(m => m.latency_ms !== null).map(m => m.latency_ms)
        );
        
        const baselinePercentiles = this.calculatePercentiles(
            baselineTest.measurements.filter(m => m.latency_ms !== null).map(m => m.latency_ms)
        );
        
        // Compare P95 as the primary metric
        const currentP95 = currentPercentiles.p95;
        const baselineP95 = baselinePercentiles.p95;
        
        const percentageChange = ((currentP95 - baselineP95) / baselineP95) * 100;
        
        return {
            metric: 'p95',
            currentValue: currentP95,
            baselineValue: baselineP95,
            percentageIncrease: percentageChange > 0 ? percentageChange : 0,
            percentageDecrease: percentageChange < 0 ? Math.abs(percentageChange) : 0,
            isRegression: percentageChange > 5, // 5% threshold for regression
            isImprovement: percentageChange < -5 // 5% threshold for improvement
        };
    }

    calculatePercentiles(sortedArray) {
        if (sortedArray.length === 0) return { p95: 0, p99: 0, avg: 0 };
        
        sortedArray.sort((a, b) => a - b);
        
        const calculatePercentile = (arr, p) => {
            const index = (p / 100) * (arr.length - 1);
            const lower = Math.floor(index);
            const upper = Math.ceil(index);
            const weight = index % 1;
            
            if (upper >= arr.length) return arr[arr.length - 1];
            return arr[lower] * (1 - weight) + arr[upper] * weight;
        };
        
        return {
            p95: calculatePercentile(sortedArray, 95),
            p99: calculatePercentile(sortedArray, 99),
            avg: sortedArray.reduce((a, b) => a + b, 0) / sortedArray.length
        };
    }

    generateGateReport(budgetResults, regressionResults) {
        const report = {
            metadata: {
                timestamp: new Date().toISOString(),
                environment: this.config.environment,
                branch: this.config.branch,
                commit: this.config.commitSha,
                buildNumber: this.config.buildNumber,
                pullRequestNumber: this.config.pullRequestNumber
            },
            budgetResults: budgetResults,
            regressionResults: regressionResults,
            summary: {
                budgetsPassed: budgetResults.passed,
                hasRegressions: regressionResults.summary.significantRegressions > 0,
                totalViolations: budgetResults.report.summary.total_violations,
                significantRegressions: regressionResults.summary.significantRegressions,
                improvements: regressionResults.summary.totalImprovements
            },
            recommendations: []
        };
        
        // Generate recommendations
        if (!budgetResults.passed) {
            report.recommendations.push({
                type: 'budget_violations',
                priority: 'high',
                message: 'Performance budget violations detected',
                actions: budgetResults.report.recommendations || []
            });
        }
        
        if (regressionResults.summary.significantRegressions > 0) {
            report.recommendations.push({
                type: 'performance_regressions',
                priority: 'high',
                message: `${regressionResults.summary.significantRegressions} significant performance regressions detected`,
                actions: [
                    'Review recent code changes for performance impact',
                    'Profile affected code paths',
                    'Consider reverting changes if critical',
                    'Optimize performance before deployment'
                ]
            });
        }
        
        return report;
    }

    determineGateOutcome(gateReport) {
        const failures = [];
        
        // Check budget violations
        if (!gateReport.budgetResults.passed) {
            failures.push(`Performance budgets failed with ${gateReport.summary.totalViolations} violations`);
        }
        
        // Check significant regressions
        if (gateReport.summary.hasRegressions) {
            failures.push(`${gateReport.summary.significantRegressions} significant performance regressions detected`);
        }
        
        return {
            passed: failures.length === 0,
            failures: failures
        };
    }

    saveGateResults(gateReport) {
        // Save gate report
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const reportPath = path.join(this.reportsDir, `performance-gate-${this.config.environment}-${timestamp}.json`);
        
        fs.writeFileSync(reportPath, JSON.stringify(gateReport, null, 2));
        
        // Save as latest
        const latestPath = path.join(this.reportsDir, `performance-gate-${this.config.environment}-latest.json`);
        fs.writeFileSync(latestPath, JSON.stringify(gateReport, null, 2));
        
        console.log(`Performance gate report saved: ${reportPath}`);
        
        // Generate CI artifacts
        this.generateCIArtifacts(gateReport);
    }

    generateCIArtifacts(gateReport) {
        // Generate JUnit XML for CI integration
        const junitXml = this.generateJUnitXML(gateReport);
        const junitPath = path.join(this.reportsDir, 'performance-gate-junit.xml');
        fs.writeFileSync(junitPath, junitXml);
        
        // Generate summary for GitHub/GitLab comments
        const summary = this.generateSummaryMarkdown(gateReport);
        const summaryPath = path.join(this.reportsDir, 'performance-gate-summary.md');
        fs.writeFileSync(summaryPath, summary);
        
        console.log('CI artifacts generated');
    }

    generateJUnitXML(gateReport) {
        const testCases = [];
        
        // Add budget test cases
        for (const test of gateReport.budgetResults.report.results.tests) {
            const testCase = {
                name: `performance_budget_${test.category}`,
                classname: 'PerformanceBudgets',
                time: '0',
                passed: test.passed
            };
            
            if (!test.passed) {
                testCase.failure = {
                    message: `Performance budget failed for ${test.category}`,
                    content: test.violations.map(v => v.message).join('\n')
                };
            }
            
            testCases.push(testCase);
        }
        
        // Add regression test cases
        if (gateReport.regressionResults.hasBaseline) {
            for (const regression of gateReport.regressionResults.regressions) {
                if (regression.isSignificant) {
                    testCases.push({
                        name: `performance_regression_${regression.category}`,
                        classname: 'PerformanceRegressions',
                        time: '0',
                        passed: false,
                        failure: {
                            message: `Significant performance regression in ${regression.category}`,
                            content: `${regression.metric} increased by ${regression.percentageIncrease.toFixed(2)}% (${regression.baselineValue}ms → ${regression.currentValue}ms)`
                        }
                    });
                }
            }
        }
        
        // Generate XML
        const totalTests = testCases.length;
        const failures = testCases.filter(tc => !tc.passed).length;
        
        let xml = `<?xml version="1.0" encoding="UTF-8"?>\n`;
        xml += `<testsuite name="PerformanceGate" tests="${totalTests}" failures="${failures}" time="0">\n`;
        
        for (const testCase of testCases) {
            xml += `  <testcase name="${testCase.name}" classname="${testCase.classname}" time="${testCase.time}"`;
            
            if (testCase.passed) {
                xml += `/>\n`;
            } else {
                xml += `>\n`;
                xml += `    <failure message="${testCase.failure.message}">${testCase.failure.content}</failure>\n`;
                xml += `  </testcase>\n`;
            }
        }
        
        xml += `</testsuite>\n`;
        return xml;
    }

    generateSummaryMarkdown(gateReport) {
        let summary = `# 🎯 Performance Gate Report\n\n`;
        
        // Overall status
        const status = gateReport.summary.budgetsPassed && !gateReport.summary.hasRegressions ? '✅ PASSED' : '❌ FAILED';
        summary += `**Status:** ${status}\n`;
        summary += `**Environment:** ${gateReport.metadata.environment}\n`;
        summary += `**Branch:** ${gateReport.metadata.branch}\n`;
        summary += `**Commit:** ${gateReport.metadata.commit}\n\n`;
        
        // Budget results
        summary += `## 📊 Performance Budget Results\n\n`;
        summary += `- **Budget Status:** ${gateReport.budgetResults.passed ? '✅ Passed' : '❌ Failed'}\n`;
        summary += `- **Total Violations:** ${gateReport.summary.totalViolations}\n`;
        summary += `- **Categories Tested:** ${gateReport.budgetResults.report.results.tests.length}\n\n`;
        
        // Regression analysis
        if (gateReport.regressionResults.hasBaseline) {
            summary += `## 📈 Regression Analysis\n\n`;
            summary += `- **Significant Regressions:** ${gateReport.summary.significantRegressions}\n`;
            summary += `- **Total Regressions:** ${gateReport.regressionResults.summary.totalRegressions}\n`;
            summary += `- **Improvements:** ${gateReport.summary.improvements}\n\n`;
            
            if (gateReport.regressionResults.regressions.length > 0) {
                summary += `### Regressions Detected\n\n`;
                for (const regression of gateReport.regressionResults.regressions) {
                    const icon = regression.isSignificant ? '🚨' : '⚠️';
                    summary += `${icon} **${regression.category}**: ${regression.metric} increased by ${regression.percentageIncrease.toFixed(2)}% (${regression.baselineValue.toFixed(2)}ms → ${regression.currentValue.toFixed(2)}ms)\n`;
                }
                summary += `\n`;
            }
        }
        
        // Recommendations
        if (gateReport.recommendations.length > 0) {
            summary += `## 💡 Recommendations\n\n`;
            for (const rec of gateReport.recommendations) {
                summary += `### ${rec.type.replace('_', ' ').toUpperCase()} (${rec.priority})\n`;
                summary += `${rec.message}\n\n`;
                if (rec.actions && rec.actions.length > 0) {
                    for (const action of rec.actions) {
                        summary += `- ${action}\n`;
                    }
                    summary += `\n`;
                }
            }
        }
        
        return summary;
    }

    async updateBaseline(budgetResults) {
        console.log('Updating performance baseline...');
        
        try {
            const baselinePath = path.join(this.baselinesDir, `${this.config.environment}-baseline.json`);
            
            const baseline = {
                timestamp: new Date().toISOString(),
                branch: this.config.branch,
                commit: this.config.commitSha,
                buildNumber: this.config.buildNumber,
                results: budgetResults.report.results
            };
            
            // Ensure baselines directory exists
            if (!fs.existsSync(this.baselinesDir)) {
                fs.mkdirSync(this.baselinesDir, { recursive: true });
            }
            
            fs.writeFileSync(baselinePath, JSON.stringify(baseline, null, 2));
            console.log(`Baseline updated: ${baselinePath}`);
            
        } catch (error) {
            console.error('Failed to update baseline:', error.message);
        }
    }

    async sendNotifications(gateReport) {
        if (!this.config.slackWebhook) {
            console.log('No Slack webhook configured, skipping notifications');
            return;
        }
        
        const shouldNotify = (gateReport.summary.budgetsPassed && this.config.notifyOnSuccess) ||
                           (!gateReport.summary.budgetsPassed && this.config.notifyOnFailure);
        
        if (!shouldNotify) {
            return;
        }
        
        try {
            const message = this.createSlackMessage(gateReport);
            
            await axios.post(this.config.slackWebhook, message, {
                headers: { 'Content-Type': 'application/json' }
            });
            
            console.log('Notification sent to Slack');
            
        } catch (error) {
            console.error('Failed to send Slack notification:', error.message);
        }
    }

    createSlackMessage(gateReport) {
        const status = gateReport.summary.budgetsPassed && !gateReport.summary.hasRegressions;
        const color = status ? 'good' : 'danger';
        const emoji = status ? '✅' : '❌';
        
        return {
            attachments: [{
                color: color,
                title: `${emoji} Performance Gate - ${gateReport.metadata.environment}`,
                fields: [
                    {
                        title: 'Branch',
                        value: gateReport.metadata.branch,
                        short: true
                    },
                    {
                        title: 'Build',
                        value: gateReport.metadata.buildNumber,
                        short: true
                    },
                    {
                        title: 'Budget Status',
                        value: gateReport.budgetResults.passed ? 'Passed' : 'Failed',
                        short: true
                    },
                    {
                        title: 'Violations',
                        value: gateReport.summary.totalViolations.toString(),
                        short: true
                    },
                    {
                        title: 'Regressions',
                        value: gateReport.summary.significantRegressions.toString(),
                        short: true
                    },
                    {
                        title: 'Improvements',
                        value: gateReport.summary.improvements.toString(),
                        short: true
                    }
                ],
                footer: 'AtlasMesh Performance Gate',
                ts: Math.floor(Date.now() / 1000)
            }]
        };
    }

    async sendFailureNotification(error) {
        if (!this.config.slackWebhook) {
            return;
        }
        
        try {
            const message = {
                attachments: [{
                    color: 'danger',
                    title: '🚨 Performance Gate Execution Failed',
                    text: `Performance gate failed to execute: ${error.message}`,
                    fields: [
                        {
                            title: 'Environment',
                            value: this.config.environment,
                            short: true
                        },
                        {
                            title: 'Branch',
                            value: this.config.branch,
                            short: true
                        }
                    ],
                    footer: 'AtlasMesh Performance Gate',
                    ts: Math.floor(Date.now() / 1000)
                }]
            };
            
            await axios.post(this.config.slackWebhook, message);
            
        } catch (notificationError) {
            console.error('Failed to send failure notification:', notificationError.message);
        }
    }
}

// CLI interface
if (require.main === module) {
    const gate = new PerformanceGate();
    gate.runPerformanceGate()
        .then(success => {
            process.exit(success ? 0 : 1);
        })
        .catch(error => {
            console.error('❌ Performance gate failed:', error);
            process.exit(1);
        });
}

module.exports = PerformanceGate;
