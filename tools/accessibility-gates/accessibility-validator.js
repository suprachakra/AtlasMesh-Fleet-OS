#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Accessibility & UX Gates Validator
 * 
 * Validates WCAG 2.2 AA compliance and UX metrics in CI/CD pipeline
 * Ensures accessibility and usability standards for autonomous vehicle operations
 */

const fs = require('fs');
const path = require('path');
const puppeteer = require('puppeteer');
const axeCore = require('@axe-core/puppeteer');
const lighthouse = require('lighthouse');
const chromeLauncher = require('chrome-launcher');

class AccessibilityValidator {
    constructor() {
        this.reportsDir = path.join(__dirname, 'reports');
        this.configPath = path.join(__dirname, 'accessibility-config.yaml');
        this.baselinesDir = path.join(__dirname, 'baselines');
        
        // Load configuration
        this.config = this.loadConfig();
        
        // Validation settings
        this.settings = {
            wcagLevel: 'AA',
            wcagVersion: '2.2',
            viewports: [
                { name: 'desktop', width: 1920, height: 1080 },
                { name: 'tablet', width: 768, height: 1024 },
                { name: 'mobile', width: 375, height: 667 }
            ],
            timeout: 30000,
            maxRetries: 3
        };
    }

    loadConfig() {
        try {
            const yaml = require('js-yaml');
            return yaml.load(fs.readFileSync(this.configPath, 'utf8'));
        } catch (error) {
            console.warn('⚠️  Using default configuration');
            return this.getDefaultConfig();
        }
    }

    getDefaultConfig() {
        return {
            urls: [
                { name: 'Dashboard', url: 'http://localhost:3000', critical: true },
                { name: 'Fleet View', url: 'http://localhost:3000/fleet', critical: true },
                { name: 'Trip Management', url: 'http://localhost:3000/trips', critical: true },
                { name: 'Settings', url: 'http://localhost:3000/settings', critical: false }
            ],
            wcag: {
                level: 'AA',
                version: '2.2',
                tags: ['wcag2a', 'wcag2aa', 'wcag21aa', 'wcag22aa'],
                rules: {
                    'color-contrast': { enabled: true, impact: 'serious' },
                    'keyboard-navigation': { enabled: true, impact: 'critical' },
                    'focus-management': { enabled: true, impact: 'serious' },
                    'screen-reader': { enabled: true, impact: 'critical' },
                    'aria-labels': { enabled: true, impact: 'serious' }
                }
            },
            ux: {
                sus: {
                    enabled: true,
                    target_score: 75, // Industry standard for good usability
                    questions: [
                        'I think that I would like to use this system frequently',
                        'I found the system unnecessarily complex',
                        'I thought the system was easy to use',
                        'I think that I would need the support of a technical person to be able to use this system',
                        'I found the various functions in this system were well integrated',
                        'I thought there was too much inconsistency in this system',
                        'I would imagine that most people would learn to use this system very quickly',
                        'I found the system very cumbersome to use',
                        'I felt very confident using the system',
                        'I needed to learn a lot of things before I could get going with this system'
                    ]
                },
                ttfa: {
                    enabled: true,
                    target_ms: 3000, // 3 seconds to first meaningful action
                    actions: [
                        { name: 'view_fleet_status', selector: '[data-testid="fleet-status"]' },
                        { name: 'create_trip', selector: '[data-testid="create-trip-button"]' },
                        { name: 'emergency_stop', selector: '[data-testid="emergency-stop"]' },
                        { name: 'vehicle_details', selector: '[data-testid="vehicle-card"]' }
                    ]
                },
                performance: {
                    fcp_target_ms: 1500, // First Contentful Paint
                    lcp_target_ms: 2500, // Largest Contentful Paint
                    cls_target: 0.1,     // Cumulative Layout Shift
                    fid_target_ms: 100   // First Input Delay
                }
            },
            thresholds: {
                accessibility_score: 95,  // Minimum accessibility score (%)
                performance_score: 90,    // Minimum performance score (%)
                best_practices_score: 90, // Minimum best practices score (%)
                seo_score: 80,           // Minimum SEO score (%)
                max_violations: {
                    critical: 0,   // Zero critical violations allowed
                    serious: 2,    // Maximum 2 serious violations
                    moderate: 5,   // Maximum 5 moderate violations
                    minor: 10      // Maximum 10 minor violations
                }
            }
        };
    }

    async validateAllPages(environment = 'development') {
        console.info(`Starting accessibility & UX validation for ${environment}`, { 
            environment, 
            timestamp: new Date().toISOString() 
        });
        
        try {
            // Ensure directories exist
            this.ensureDirectories();
            
            // Launch browser
            const browser = await puppeteer.launch({
                headless: true,
                args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-dev-shm-usage']
            });
            
            const results = {
                environment: environment,
                timestamp: new Date().toISOString(),
                summary: {
                    total_pages: this.config.urls.length,
                    passed_pages: 0,
                    failed_pages: 0,
                    critical_violations: 0,
                    total_violations: 0
                },
                pages: [],
                overall_passed: true
            };
            
            // Validate each page
            for (const urlConfig of this.config.urls) {
                console.log(`📄 Validating ${urlConfig.name}...`);
                
                const pageResult = await this.validatePage(browser, urlConfig, environment);
                results.pages.push(pageResult);
                
                if (pageResult.passed) {
                    results.summary.passed_pages++;
                } else {
                    results.summary.failed_pages++;
                    if (urlConfig.critical) {
                        results.overall_passed = false;
                    }
                }
                
                results.summary.critical_violations += pageResult.accessibility.violations.critical || 0;
                results.summary.total_violations += pageResult.accessibility.violations.total || 0;
            }
            
            await browser.close();
            
            // Generate comprehensive report
            const report = this.generateReport(results);
            this.saveReport(report, environment);
            
            // Check overall pass/fail
            if (results.overall_passed && results.summary.critical_violations === 0) {
                console.log('✅ All accessibility & UX validations passed!');
                return true;
            } else {
                console.log('❌ Accessibility & UX validation failed!');
                this.logFailures(results);
                return false;
            }
            
        } catch (error) {
            console.error('❌ Accessibility validation failed:', error.message);
            return false;
        }
    }

    async validatePage(browser, urlConfig, environment) {
        const result = {
            name: urlConfig.name,
            url: urlConfig.url,
            critical: urlConfig.critical,
            timestamp: new Date().toISOString(),
            passed: true,
            accessibility: {},
            ux: {},
            performance: {},
            errors: []
        };

        try {
            const page = await browser.newPage();
            
            // Test multiple viewports
            for (const viewport of this.settings.viewports) {
                console.log(`  📱 Testing ${viewport.name} viewport (${viewport.width}x${viewport.height})`);
                
                await page.setViewport({
                    width: viewport.width,
                    height: viewport.height
                });
                
                // Navigate to page
                await page.goto(urlConfig.url, { 
                    waitUntil: 'networkidle0',
                    timeout: this.settings.timeout 
                });
                
                // Run accessibility tests
                const accessibilityResult = await this.runAccessibilityTests(page, viewport.name);
                result.accessibility[viewport.name] = accessibilityResult;
                
                // Run UX tests
                const uxResult = await this.runUXTests(page, viewport.name);
                result.ux[viewport.name] = uxResult;
                
                // Run performance tests (only for desktop)
                if (viewport.name === 'desktop') {
                    const performanceResult = await this.runPerformanceTests(urlConfig.url);
                    result.performance = performanceResult;
                }
            }
            
            await page.close();
            
            // Determine overall pass/fail for this page
            result.passed = this.evaluatePageResult(result);
            
        } catch (error) {
            result.passed = false;
            result.errors.push({
                type: 'page_error',
                message: error.message,
                timestamp: new Date().toISOString()
            });
        }

        return result;
    }

    async runAccessibilityTests(page, viewportName) {
        const result = {
            viewport: viewportName,
            wcag_level: this.settings.wcagLevel,
            wcag_version: this.settings.wcagVersion,
            violations: { critical: 0, serious: 0, moderate: 0, minor: 0, total: 0 },
            passes: 0,
            incomplete: 0,
            score: 0,
            details: []
        };

        try {
            // Inject axe-core
            await axeCore.injectIntoPage(page);
            
            // Configure axe for WCAG 2.2 AA
            const axeResults = await page.evaluate(() => {
                return axe.run({
                    tags: ['wcag2a', 'wcag2aa', 'wcag21aa', 'wcag22aa'],
                    rules: {
                        'color-contrast': { enabled: true },
                        'keyboard': { enabled: true },
                        'focus-order-semantics': { enabled: true },
                        'aria-allowed-attr': { enabled: true },
                        'aria-required-attr': { enabled: true },
                        'aria-valid-attr-value': { enabled: true },
                        'aria-valid-attr': { enabled: true },
                        'button-name': { enabled: true },
                        'bypass': { enabled: true },
                        'document-title': { enabled: true },
                        'duplicate-id': { enabled: true },
                        'form-field-multiple-labels': { enabled: true },
                        'frame-title': { enabled: true },
                        'html-has-lang': { enabled: true },
                        'html-lang-valid': { enabled: true },
                        'image-alt': { enabled: true },
                        'input-image-alt': { enabled: true },
                        'label': { enabled: true },
                        'link-name': { enabled: true },
                        'list': { enabled: true },
                        'listitem': { enabled: true },
                        'meta-refresh': { enabled: true },
                        'meta-viewport': { enabled: true },
                        'object-alt': { enabled: true },
                        'role-img-alt': { enabled: true },
                        'scrollable-region-focusable': { enabled: true },
                        'select-name': { enabled: true },
                        'server-side-image-map': { enabled: true },
                        'svg-img-alt': { enabled: true },
                        'td-headers-attr': { enabled: true },
                        'th-has-data-cells': { enabled: true },
                        'valid-lang': { enabled: true },
                        'video-caption': { enabled: true }
                    }
                });
            });

            // Process violations
            for (const violation of axeResults.violations) {
                const impact = violation.impact || 'minor';
                result.violations[impact]++;
                result.violations.total++;
                
                result.details.push({
                    id: violation.id,
                    impact: impact,
                    description: violation.description,
                    help: violation.help,
                    helpUrl: violation.helpUrl,
                    nodes: violation.nodes.length,
                    tags: violation.tags
                });
            }

            result.passes = axeResults.passes.length;
            result.incomplete = axeResults.incomplete.length;
            
            // Calculate accessibility score
            const totalTests = result.passes + result.violations.total + result.incomplete;
            result.score = totalTests > 0 ? Math.round((result.passes / totalTests) * 100) : 100;

        } catch (error) {
            result.error = error.message;
        }

        return result;
    }

    async runUXTests(page, viewportName) {
        const result = {
            viewport: viewportName,
            ttfa: {},
            interactions: {},
            navigation: {},
            errors: []
        };

        try {
            // Test Time to First Action (TtFA)
            result.ttfa = await this.measureTtFA(page);
            
            // Test keyboard navigation
            result.navigation = await this.testKeyboardNavigation(page);
            
            // Test critical interactions
            result.interactions = await this.testCriticalInteractions(page);

        } catch (error) {
            result.errors.push({
                type: 'ux_test_error',
                message: error.message
            });
        }

        return result;
    }

    async measureTtFA(page) {
        const ttfaResults = {
            measurements: [],
            average_ms: 0,
            passed: true
        };

        try {
            for (const action of this.config.ux.ttfa.actions) {
                const startTime = Date.now();
                
                try {
                    // Wait for the element to be visible and interactable
                    await page.waitForSelector(action.selector, { 
                        visible: true, 
                        timeout: this.config.ux.ttfa.target_ms 
                    });
                    
                    const endTime = Date.now();
                    const duration = endTime - startTime;
                    
                    ttfaResults.measurements.push({
                        action: action.name,
                        duration_ms: duration,
                        passed: duration <= this.config.ux.ttfa.target_ms
                    });
                    
                } catch (error) {
                    ttfaResults.measurements.push({
                        action: action.name,
                        duration_ms: this.config.ux.ttfa.target_ms + 1000, // Timeout + buffer
                        passed: false,
                        error: 'Element not found or not interactive'
                    });
                }
            }
            
            // Calculate average
            const validMeasurements = ttfaResults.measurements.filter(m => !m.error);
            if (validMeasurements.length > 0) {
                ttfaResults.average_ms = validMeasurements.reduce((sum, m) => sum + m.duration_ms, 0) / validMeasurements.length;
            }
            
            // Check if all critical actions passed
            ttfaResults.passed = ttfaResults.measurements.every(m => m.passed);

        } catch (error) {
            ttfaResults.error = error.message;
            ttfaResults.passed = false;
        }

        return ttfaResults;
    }

    async testKeyboardNavigation(page) {
        const navResult = {
            tab_order: { passed: true, issues: [] },
            focus_visible: { passed: true, issues: [] },
            escape_key: { passed: true, issues: [] },
            enter_key: { passed: true, issues: [] }
        };

        try {
            // Test tab order
            await page.keyboard.press('Tab');
            const focusedElement = await page.evaluate(() => {
                const focused = document.activeElement;
                return {
                    tagName: focused.tagName,
                    id: focused.id,
                    className: focused.className,
                    tabIndex: focused.tabIndex
                };
            });

            if (!focusedElement || focusedElement.tagName === 'BODY') {
                navResult.tab_order.passed = false;
                navResult.tab_order.issues.push('No focusable elements found or focus not visible');
            }

            // Test focus visibility
            const focusStyles = await page.evaluate(() => {
                const focused = document.activeElement;
                const styles = window.getComputedStyle(focused);
                return {
                    outline: styles.outline,
                    outlineWidth: styles.outlineWidth,
                    outlineStyle: styles.outlineStyle,
                    outlineColor: styles.outlineColor,
                    boxShadow: styles.boxShadow
                };
            });

            if (focusStyles.outline === 'none' && !focusStyles.boxShadow.includes('inset')) {
                navResult.focus_visible.passed = false;
                navResult.focus_visible.issues.push('Focus indicator not visible');
            }

        } catch (error) {
            navResult.error = error.message;
        }

        return navResult;
    }

    async testCriticalInteractions(page) {
        const interactionResult = {
            emergency_stop: { accessible: false, keyboard_accessible: false },
            main_navigation: { accessible: false, keyboard_accessible: false },
            form_submission: { accessible: false, keyboard_accessible: false }
        };

        try {
            // Test emergency stop accessibility
            const emergencyStop = await page.$('[data-testid="emergency-stop"], [aria-label*="emergency"], [aria-label*="stop"]');
            if (emergencyStop) {
                interactionResult.emergency_stop.accessible = true;
                
                // Test keyboard accessibility
                await emergencyStop.focus();
                const isKeyboardAccessible = await page.evaluate((el) => {
                    return document.activeElement === el;
                }, emergencyStop);
                
                interactionResult.emergency_stop.keyboard_accessible = isKeyboardAccessible;
            }

            // Test main navigation
            const navElements = await page.$$('[role="navigation"], nav, [data-testid*="nav"]');
            if (navElements.length > 0) {
                interactionResult.main_navigation.accessible = true;
                
                // Test keyboard navigation
                await navElements[0].focus();
                const isNavKeyboardAccessible = await page.evaluate((el) => {
                    return document.activeElement === el || el.contains(document.activeElement);
                }, navElements[0]);
                
                interactionResult.main_navigation.keyboard_accessible = isNavKeyboardAccessible;
            }

        } catch (error) {
            interactionResult.error = error.message;
        }

        return interactionResult;
    }

    async runPerformanceTests(url) {
        const result = {
            lighthouse_score: 0,
            metrics: {},
            passed: true,
            error: null
        };

        try {
            // Launch Chrome for Lighthouse
            const chrome = await chromeLauncher.launch({ chromeFlags: ['--headless'] });
            
            // Run Lighthouse
            const lighthouseResult = await lighthouse(url, {
                port: chrome.port,
                onlyCategories: ['accessibility', 'performance', 'best-practices'],
                settings: {
                    formFactor: 'desktop',
                    throttling: {
                        rttMs: 40,
                        throughputKbps: 10240,
                        cpuSlowdownMultiplier: 1,
                        requestLatencyMs: 0,
                        downloadThroughputKbps: 0,
                        uploadThroughputKbps: 0
                    },
                    screenEmulation: {
                        mobile: false,
                        width: 1920,
                        height: 1080,
                        deviceScaleFactor: 1,
                        disabled: false
                    }
                }
            });

            await chrome.kill();

            // Extract scores and metrics
            const lhr = lighthouseResult.lhr;
            result.lighthouse_score = Math.round(lhr.categories.accessibility.score * 100);
            
            result.metrics = {
                accessibility_score: Math.round(lhr.categories.accessibility.score * 100),
                performance_score: Math.round(lhr.categories.performance.score * 100),
                best_practices_score: Math.round(lhr.categories['best-practices'].score * 100),
                first_contentful_paint: lhr.audits['first-contentful-paint'].numericValue,
                largest_contentful_paint: lhr.audits['largest-contentful-paint'].numericValue,
                cumulative_layout_shift: lhr.audits['cumulative-layout-shift'].numericValue,
                first_input_delay: lhr.audits['max-potential-fid']?.numericValue || 0
            };

            // Check thresholds
            const thresholds = this.config.thresholds;
            result.passed = (
                result.metrics.accessibility_score >= thresholds.accessibility_score &&
                result.metrics.performance_score >= thresholds.performance_score &&
                result.metrics.best_practices_score >= thresholds.best_practices_score &&
                result.metrics.first_contentful_paint <= this.config.ux.performance.fcp_target_ms &&
                result.metrics.largest_contentful_paint <= this.config.ux.performance.lcp_target_ms &&
                result.metrics.cumulative_layout_shift <= this.config.ux.performance.cls_target
            );

        } catch (error) {
            result.error = error.message;
            result.passed = false;
        }

        return result;
    }

    evaluatePageResult(pageResult) {
        let passed = true;
        const thresholds = this.config.thresholds;

        // Check accessibility violations
        for (const viewport in pageResult.accessibility) {
            const accessibilityResult = pageResult.accessibility[viewport];
            
            if (accessibilityResult.violations.critical > thresholds.max_violations.critical ||
                accessibilityResult.violations.serious > thresholds.max_violations.serious ||
                accessibilityResult.score < thresholds.accessibility_score) {
                passed = false;
                break;
            }
        }

        // Check UX metrics
        for (const viewport in pageResult.ux) {
            const uxResult = pageResult.ux[viewport];
            
            if (!uxResult.ttfa.passed || !uxResult.navigation.tab_order.passed) {
                passed = false;
                break;
            }
        }

        // Check performance (if available)
        if (pageResult.performance && !pageResult.performance.passed) {
            passed = false;
        }

        return passed;
    }

    generateReport(results) {
        return {
            summary: results.summary,
            timestamp: results.timestamp,
            environment: results.environment,
            overall_passed: results.overall_passed,
            pages: results.pages,
            recommendations: this.generateRecommendations(results),
            compliance: {
                wcag_level: this.settings.wcagLevel,
                wcag_version: this.settings.wcagVersion,
                total_violations: results.summary.total_violations,
                critical_violations: results.summary.critical_violations,
                compliance_rate: this.calculateComplianceRate(results)
            }
        };
    }

    calculateComplianceRate(results) {
        const totalPages = results.pages.length;
        const passedPages = results.pages.filter(p => p.passed).length;
        return totalPages > 0 ? Math.round((passedPages / totalPages) * 100) : 100;
    }

    generateRecommendations(results) {
        const recommendations = [];

        // Accessibility recommendations
        if (results.summary.critical_violations > 0) {
            recommendations.push({
                type: 'critical_accessibility',
                priority: 'high',
                message: `${results.summary.critical_violations} critical accessibility violations found`,
                actions: [
                    'Fix all critical accessibility violations immediately',
                    'Review color contrast ratios',
                    'Ensure all interactive elements are keyboard accessible',
                    'Add proper ARIA labels and descriptions'
                ]
            });
        }

        // UX recommendations
        const slowTtFA = results.pages.some(p => 
            Object.values(p.ux).some(ux => ux.ttfa && ux.ttfa.average_ms > this.config.ux.ttfa.target_ms)
        );

        if (slowTtFA) {
            recommendations.push({
                type: 'ux_performance',
                priority: 'medium',
                message: 'Time to First Action exceeds target threshold',
                actions: [
                    'Optimize page load performance',
                    'Reduce JavaScript bundle size',
                    'Implement progressive loading',
                    'Prioritize critical UI elements'
                ]
            });
        }

        return recommendations;
    }

    ensureDirectories() {
        [this.reportsDir, this.baselinesDir].forEach(dir => {
            if (!fs.existsSync(dir)) {
                fs.mkdirSync(dir, { recursive: true });
            }
        });
    }

    saveReport(report, environment) {
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const reportPath = path.join(this.reportsDir, `accessibility-ux-${environment}-${timestamp}.json`);
        
        fs.writeFileSync(reportPath, JSON.stringify(report, null, 2));
        
        // Save as latest
        const latestPath = path.join(this.reportsDir, `accessibility-ux-${environment}-latest.json`);
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
    <title>AtlasMesh Accessibility & UX Report - ${environment}</title>
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
        .page-result { margin-bottom: 20px; padding: 15px; border-radius: 6px; }
        .page-passed { background: #d4edda; border-left: 4px solid #28a745; }
        .page-failed { background: #f8d7da; border-left: 4px solid #dc3545; }
        .violations { margin-top: 10px; }
        .violation { margin: 5px 0; padding: 10px; background: #fff3cd; border-radius: 4px; }
        .critical { background: #f8d7da; }
        .serious { background: #fff3cd; }
        .moderate { background: #d1ecf1; }
        .minor { background: #e2e3e5; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>AtlasMesh Accessibility & UX Report</h1>
            <p>Environment: <strong>${environment}</strong> | Generated: ${report.timestamp}</p>
        </div>
        
        <div class="summary">
            <div class="metric">
                <div class="metric-value ${report.overall_passed ? 'success' : 'error'}">
                    ${report.overall_passed ? '✅' : '❌'}
                </div>
                <div>Overall Status</div>
            </div>
            <div class="metric">
                <div class="metric-value">${report.compliance.compliance_rate}%</div>
                <div>Compliance Rate</div>
            </div>
            <div class="metric">
                <div class="metric-value ${report.compliance.critical_violations === 0 ? 'success' : 'error'}">
                    ${report.compliance.critical_violations}
                </div>
                <div>Critical Violations</div>
            </div>
            <div class="metric">
                <div class="metric-value">${report.summary.passed_pages}/${report.summary.total_pages}</div>
                <div>Pages Passed</div>
            </div>
        </div>
        
        <div class="section">
            <h2>Page Results</h2>
            ${report.pages.map(page => `
                <div class="page-result ${page.passed ? 'page-passed' : 'page-failed'}">
                    <h3>${page.name} ${page.critical ? '(Critical)' : ''}</h3>
                    <p>URL: ${page.url} | Status: ${page.passed ? 'PASSED' : 'FAILED'}</p>
                    
                    ${Object.entries(page.accessibility).map(([viewport, acc]) => `
                        <h4>Accessibility - ${viewport}</h4>
                        <p>Score: ${acc.score}% | Violations: ${acc.violations.total}</p>
                        ${acc.details.length > 0 ? `
                            <div class="violations">
                                ${acc.details.map(violation => `
                                    <div class="violation ${violation.impact}">
                                        <strong>${violation.impact.toUpperCase()}</strong>: ${violation.description}
                                        <br><small>${violation.help}</small>
                                    </div>
                                `).join('')}
                            </div>
                        ` : ''}
                    `).join('')}
                    
                    ${page.performance.metrics ? `
                        <h4>Performance</h4>
                        <p>Accessibility: ${page.performance.metrics.accessibility_score}% | 
                           Performance: ${page.performance.metrics.performance_score}% |
                           Best Practices: ${page.performance.metrics.best_practices_score}%</p>
                    ` : ''}
                </div>
            `).join('')}
        </div>
        
        ${report.recommendations.length > 0 ? `
            <div class="section">
                <h2>Recommendations</h2>
                ${report.recommendations.map(rec => `
                    <div class="violation ${rec.priority === 'high' ? 'critical' : 'serious'}">
                        <h4>${rec.type.replace('_', ' ').toUpperCase()} (${rec.priority})</h4>
                        <p>${rec.message}</p>
                        <ul>
                            ${rec.actions.map(action => `<li>${action}</li>`).join('')}
                        </ul>
                    </div>
                `).join('')}
            </div>
        ` : ''}
    </div>
</body>
</html>`;

        const htmlPath = path.join(this.reportsDir, `accessibility-ux-${environment}-latest.html`);
        fs.writeFileSync(htmlPath, htmlContent);
        console.log(`📊 HTML report saved to: ${htmlPath}`);
    }

    logFailures(results) {
        console.log('\n🚨 Accessibility & UX Failures:');
        
        results.pages.forEach(page => {
            if (!page.passed) {
                console.log(`❌ ${page.name} (${page.url})`);
                
                Object.entries(page.accessibility).forEach(([viewport, acc]) => {
                    if (acc.violations.critical > 0 || acc.violations.serious > 0) {
                        console.log(`  ${viewport}: ${acc.violations.critical} critical, ${acc.violations.serious} serious violations`);
                    }
                });
            }
        });
        
        console.log('');
    }
}

// CLI interface
if (require.main === module) {
    const args = process.argv.slice(2);
    const environment = args[0] || 'development';
    
    const validator = new AccessibilityValidator();
    validator.validateAllPages(environment)
        .then(success => {
            process.exit(success ? 0 : 1);
        })
        .catch(error => {
            console.error('❌ Accessibility validation failed:', error);
            process.exit(1);
        });
}

module.exports = AccessibilityValidator;
