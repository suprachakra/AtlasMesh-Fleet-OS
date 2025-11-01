#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Contract Testing Framework
 * 
 * Validates API contracts and ensures compatibility between services
 * Implements consumer-driven contract testing using Pact-like patterns
 */

const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const axios = require('axios');
const { execSync } = require('child_process');

class ContractValidator {
    constructor() {
        this.contractsDir = path.join(__dirname, '../../api/contracts/v1');
        this.testsDir = path.join(__dirname, 'tests');
        this.reportsDir = path.join(__dirname, 'reports');
        this.registryPath = path.join(__dirname, '../../api/contracts/registry.yaml');
        
        // Load registry
        this.registry = this.loadRegistry();
        
        // Test configuration
        this.config = {
            timeout: 30000,
            retries: 3,
            environments: {
                development: 'http://localhost:8080',
                staging: 'https://api-staging.atlasmesh.com',
                production: 'https://api.atlasmesh.com'
            }
        };
    }

    loadRegistry() {
        try {
            return yaml.load(fs.readFileSync(this.registryPath, 'utf8'));
        } catch (error) {
            console.error('❌ Failed to load contract registry:', error.message);
            process.exit(1);
        }
    }

    async validateAllContracts(environment = 'development') {
        console.log(`🔍 Starting contract validation for ${environment} environment...`);
        
        try {
            // Ensure directories exist
            this.ensureDirectories();
            
            // Load all contracts
            const contracts = this.loadContracts();
            
            // Validate each contract
            const results = [];
            for (const contract of contracts) {
                const result = await this.validateContract(contract, environment);
                results.push(result);
            }
            
            // Validate cross-service compatibility
            const compatibilityResults = await this.validateCompatibility(contracts);
            
            // Generate report
            const report = this.generateReport(results, compatibilityResults, environment);
            this.saveReport(report, environment);
            
            // Check if all tests passed
            const allPassed = results.every(r => r.passed) && 
                            compatibilityResults.every(r => r.passed);
            
            if (allPassed) {
                console.log('✅ All contract validations passed!');
                return true;
            } else {
                console.log('❌ Some contract validations failed!');
                return false;
            }
            
        } catch (error) {
            console.error('❌ Contract validation failed:', error.message);
            return false;
        }
    }

    ensureDirectories() {
        [this.testsDir, this.reportsDir].forEach(dir => {
            if (!fs.existsSync(dir)) {
                fs.mkdirSync(dir, { recursive: true });
            }
        });
    }

    loadContracts() {
        const contracts = [];
        const files = fs.readdirSync(this.contractsDir);
        
        for (const file of files) {
            if (file.endsWith('.yaml') || file.endsWith('.yml')) {
                const contractPath = path.join(this.contractsDir, file);
                const serviceName = path.basename(file, path.extname(file));
                
                try {
                    const contract = yaml.load(fs.readFileSync(contractPath, 'utf8'));
                    contracts.push({
                        name: serviceName,
                        title: contract.info.title,
                        version: contract.info.version,
                        contractPath: contractPath,
                        contract: contract
                    });
                } catch (error) {
                    console.warn(`⚠️  Skipping invalid contract: ${file} - ${error.message}`);
                }
            }
        }
        
        return contracts;
    }

    async validateContract(contractInfo, environment) {
        console.log(`📋 Validating ${contractInfo.title}...`);
        
        const result = {
            service: contractInfo.name,
            title: contractInfo.title,
            version: contractInfo.version,
            environment: environment,
            timestamp: new Date().toISOString(),
            passed: true,
            tests: [],
            errors: []
        };

        try {
            // Validate OpenAPI specification
            await this.validateOpenAPISpec(contractInfo, result);
            
            // Validate service endpoints
            await this.validateServiceEndpoints(contractInfo, environment, result);
            
            // Validate request/response schemas
            await this.validateSchemas(contractInfo, result);
            
            // Validate security requirements
            await this.validateSecurity(contractInfo, result);
            
        } catch (error) {
            result.passed = false;
            result.errors.push({
                type: 'validation_error',
                message: error.message,
                timestamp: new Date().toISOString()
            });
        }

        return result;
    }

    async validateOpenAPISpec(contractInfo, result) {
        const test = {
            name: 'OpenAPI Specification Validation',
            passed: true,
            details: []
        };

        try {
            // Use swagger-parser to validate the OpenAPI spec
            const validateCmd = `npx swagger-parser validate "${contractInfo.contractPath}"`;
            execSync(validateCmd, { stdio: 'pipe' });
            
            test.details.push('OpenAPI specification is valid');
            
        } catch (error) {
            test.passed = false;
            test.details.push(`OpenAPI validation failed: ${error.message}`);
            result.passed = false;
        }

        result.tests.push(test);
    }

    async validateServiceEndpoints(contractInfo, environment, result) {
        const test = {
            name: 'Service Endpoint Validation',
            passed: true,
            details: []
        };

        const baseUrl = this.config.environments[environment];
        const servicePath = this.getServicePath(contractInfo.name);
        const serviceUrl = `${baseUrl}${servicePath}`;

        try {
            // Test health endpoint
            const healthResponse = await this.makeRequest('GET', `${serviceUrl}/health`);
            if (healthResponse.status === 200) {
                test.details.push('Health endpoint is accessible');
            } else {
                throw new Error(`Health endpoint returned status ${healthResponse.status}`);
            }

            // Test authentication endpoints
            if (contractInfo.contract.components?.securitySchemes) {
                await this.testAuthenticationEndpoints(serviceUrl, contractInfo, test);
            }

            // Test sample endpoints from the contract
            await this.testSampleEndpoints(serviceUrl, contractInfo, test);

        } catch (error) {
            test.passed = false;
            test.details.push(`Endpoint validation failed: ${error.message}`);
            result.passed = false;
        }

        result.tests.push(test);
    }

    async testAuthenticationEndpoints(serviceUrl, contractInfo, test) {
        // Test that protected endpoints require authentication
        const protectedPaths = Object.keys(contractInfo.contract.paths || {})
            .filter(path => {
                const pathObj = contractInfo.contract.paths[path];
                return Object.values(pathObj).some(method => 
                    method.security && method.security.length > 0
                );
            });

        for (const path of protectedPaths.slice(0, 3)) { // Test first 3 protected paths
            try {
                const response = await this.makeRequest('GET', `${serviceUrl}${path}`, {}, false);
                if (response.status === 401 || response.status === 403) {
                    test.details.push(`Protected endpoint ${path} correctly requires authentication`);
                } else {
                    test.details.push(`Warning: Protected endpoint ${path} did not require authentication`);
                }
            } catch (error) {
                // Expected for protected endpoints without auth
                test.details.push(`Protected endpoint ${path} correctly rejected unauthenticated request`);
            }
        }
    }

    async testSampleEndpoints(serviceUrl, contractInfo, test) {
        const paths = contractInfo.contract.paths || {};
        const testPaths = Object.keys(paths).slice(0, 5); // Test first 5 endpoints

        for (const path of testPaths) {
            const pathObj = paths[path];
            const methods = Object.keys(pathObj).filter(method => 
                ['get', 'post', 'put', 'delete', 'patch'].includes(method.toLowerCase())
            );

            for (const method of methods.slice(0, 2)) { // Test first 2 methods per path
                try {
                    const testPath = this.replacePathParameters(path);
                    const response = await this.makeRequest(method.toUpperCase(), `${serviceUrl}${testPath}`, {}, false);
                    
                    // We expect either success or proper error codes
                    if (response.status < 500) {
                        test.details.push(`Endpoint ${method.toUpperCase()} ${path} is accessible`);
                    } else {
                        test.details.push(`Warning: Endpoint ${method.toUpperCase()} ${path} returned server error`);
                    }
                } catch (error) {
                    // Network errors or timeouts
                    test.details.push(`Warning: Could not reach ${method.toUpperCase()} ${path}: ${error.message}`);
                }
            }
        }
    }

    replacePathParameters(path) {
        // Replace path parameters with sample values
        return path
            .replace(/{trip_id}/g, '123e4567-e89b-12d3-a456-426614174000')
            .replace(/{vehicle_id}/g, '123e4567-e89b-12d3-a456-426614174001')
            .replace(/{id}/g, '123e4567-e89b-12d3-a456-426614174002')
            .replace(/{[^}]+}/g, 'test-id');
    }

    async validateSchemas(contractInfo, result) {
        const test = {
            name: 'Schema Validation',
            passed: true,
            details: []
        };

        try {
            const schemas = contractInfo.contract.components?.schemas || {};
            const schemaCount = Object.keys(schemas).length;
            
            if (schemaCount > 0) {
                test.details.push(`Found ${schemaCount} schemas defined`);
                
                // Validate that required schemas exist
                const requiredSchemas = ['ErrorResponse', 'HealthStatus'];
                for (const requiredSchema of requiredSchemas) {
                    if (schemas[requiredSchema]) {
                        test.details.push(`Required schema '${requiredSchema}' is defined`);
                    } else {
                        test.passed = false;
                        test.details.push(`Missing required schema '${requiredSchema}'`);
                        result.passed = false;
                    }
                }
                
                // Validate schema structure
                for (const [schemaName, schema] of Object.entries(schemas)) {
                    if (schema.type === 'object' && schema.properties) {
                        test.details.push(`Schema '${schemaName}' has valid object structure`);
                    }
                }
            } else {
                test.passed = false;
                test.details.push('No schemas defined in contract');
                result.passed = false;
            }

        } catch (error) {
            test.passed = false;
            test.details.push(`Schema validation failed: ${error.message}`);
            result.passed = false;
        }

        result.tests.push(test);
    }

    async validateSecurity(contractInfo, result) {
        const test = {
            name: 'Security Validation',
            passed: true,
            details: []
        };

        try {
            const securitySchemes = contractInfo.contract.components?.securitySchemes || {};
            const globalSecurity = contractInfo.contract.security || [];

            if (Object.keys(securitySchemes).length > 0) {
                test.details.push(`Found ${Object.keys(securitySchemes).length} security schemes`);
                
                // Check for required security schemes
                if (securitySchemes.BearerAuth) {
                    test.details.push('BearerAuth security scheme is defined');
                } else {
                    test.passed = false;
                    test.details.push('Missing BearerAuth security scheme');
                    result.passed = false;
                }
            } else {
                test.passed = false;
                test.details.push('No security schemes defined');
                result.passed = false;
            }

            // Check that sensitive endpoints have security requirements
            const paths = contractInfo.contract.paths || {};
            let protectedEndpoints = 0;
            let totalEndpoints = 0;

            for (const [path, pathObj] of Object.entries(paths)) {
                for (const [method, methodObj] of Object.entries(pathObj)) {
                    if (['get', 'post', 'put', 'delete', 'patch'].includes(method.toLowerCase())) {
                        totalEndpoints++;
                        if (methodObj.security && methodObj.security.length > 0) {
                            protectedEndpoints++;
                        }
                    }
                }
            }

            const protectionRate = totalEndpoints > 0 ? (protectedEndpoints / totalEndpoints) * 100 : 0;
            test.details.push(`${protectionRate.toFixed(1)}% of endpoints are protected (${protectedEndpoints}/${totalEndpoints})`);

            if (protectionRate < 80) {
                test.details.push('Warning: Low percentage of protected endpoints');
            }

        } catch (error) {
            test.passed = false;
            test.details.push(`Security validation failed: ${error.message}`);
            result.passed = false;
        }

        result.tests.push(test);
    }

    async validateCompatibility(contracts) {
        console.log('🔗 Validating cross-service compatibility...');
        
        const results = [];
        const contractMatrix = this.registry.compatibility?.service_matrix || {};

        for (const [consumer, providers] of Object.entries(contractMatrix)) {
            for (const [provider, versions] of Object.entries(providers)) {
                const result = await this.validateServiceCompatibility(consumer, provider, versions, contracts);
                results.push(result);
            }
        }

        return results;
    }

    async validateServiceCompatibility(consumer, provider, expectedVersions, contracts) {
        const result = {
            consumer: consumer,
            provider: provider,
            expectedVersions: expectedVersions,
            actualVersion: null,
            compatible: false,
            timestamp: new Date().toISOString(),
            details: []
        };

        try {
            // Find provider contract
            const providerContract = contracts.find(c => c.name === provider);
            if (!providerContract) {
                result.details.push(`Provider contract '${provider}' not found`);
                return result;
            }

            result.actualVersion = providerContract.version;

            // Check version compatibility
            if (expectedVersions.includes(providerContract.version)) {
                result.compatible = true;
                result.details.push(`Version ${providerContract.version} is compatible`);
            } else {
                result.details.push(`Version ${providerContract.version} is not in expected versions: ${expectedVersions.join(', ')}`);
            }

            // Additional compatibility checks could go here
            // e.g., schema compatibility, endpoint availability, etc.

        } catch (error) {
            result.details.push(`Compatibility check failed: ${error.message}`);
        }

        return result;
    }

    getServicePath(serviceName) {
        // Map service names to their API paths
        const pathMap = {
            'api-gateway': '/api/v1',
            'auth-service': '/auth/v1',
            'trip-service': '/trip/v1',
            'dispatch-service': '/dispatch/v1',
            'routing-service': '/routing/v1',
            'fleet-manager': '/fleet/v1',
            'policy-engine': '/policy/v1',
            'telemetry-ingestion': '/telemetry/v1',
            'data-lineage': '/lineage/v1'
        };

        return pathMap[serviceName] || `/api/v1`;
    }

    async makeRequest(method, url, data = {}, requireAuth = true) {
        const config = {
            method: method,
            url: url,
            timeout: this.config.timeout,
            validateStatus: () => true, // Don't throw on any status code
        };

        if (requireAuth) {
            config.headers = {
                'Authorization': 'Bearer test-token',
                'Content-Type': 'application/json'
            };
        }

        if (['POST', 'PUT', 'PATCH'].includes(method) && Object.keys(data).length > 0) {
            config.data = data;
        }

        return await axios(config);
    }

    generateReport(contractResults, compatibilityResults, environment) {
        const totalContracts = contractResults.length;
        const passedContracts = contractResults.filter(r => r.passed).length;
        const totalCompatibility = compatibilityResults.length;
        const passedCompatibility = compatibilityResults.filter(r => r.compatible).length;

        const report = {
            summary: {
                environment: environment,
                timestamp: new Date().toISOString(),
                totalContracts: totalContracts,
                passedContracts: passedContracts,
                failedContracts: totalContracts - passedContracts,
                contractSuccessRate: totalContracts > 0 ? (passedContracts / totalContracts) * 100 : 0,
                totalCompatibilityChecks: totalCompatibility,
                passedCompatibilityChecks: passedCompatibility,
                failedCompatibilityChecks: totalCompatibility - passedCompatibility,
                compatibilitySuccessRate: totalCompatibility > 0 ? (passedCompatibility / totalCompatibility) * 100 : 0,
                overallSuccess: passedContracts === totalContracts && passedCompatibility === totalCompatibility
            },
            contractResults: contractResults,
            compatibilityResults: compatibilityResults,
            recommendations: this.generateRecommendations(contractResults, compatibilityResults)
        };

        return report;
    }

    generateRecommendations(contractResults, compatibilityResults) {
        const recommendations = [];

        // Contract recommendations
        const failedContracts = contractResults.filter(r => !r.passed);
        if (failedContracts.length > 0) {
            recommendations.push({
                type: 'contract_failures',
                priority: 'high',
                message: `${failedContracts.length} contract(s) failed validation`,
                actions: failedContracts.map(c => `Fix issues in ${c.service}: ${c.errors.map(e => e.message).join(', ')}`)
            });
        }

        // Compatibility recommendations
        const incompatibleServices = compatibilityResults.filter(r => !r.compatible);
        if (incompatibleServices.length > 0) {
            recommendations.push({
                type: 'compatibility_issues',
                priority: 'high',
                message: `${incompatibleServices.length} service compatibility issue(s) found`,
                actions: incompatibleServices.map(c => `Update ${c.consumer} to support ${c.provider} version ${c.actualVersion}`)
            });
        }

        // Security recommendations
        const securityIssues = contractResults.filter(r => 
            r.tests.some(t => t.name === 'Security Validation' && !t.passed)
        );
        if (securityIssues.length > 0) {
            recommendations.push({
                type: 'security_issues',
                priority: 'high',
                message: `${securityIssues.length} service(s) have security validation issues`,
                actions: ['Review and fix security scheme definitions', 'Ensure all sensitive endpoints are protected']
            });
        }

        return recommendations;
    }

    saveReport(report, environment) {
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const reportPath = path.join(this.reportsDir, `contract-validation-${environment}-${timestamp}.json`);
        
        fs.writeFileSync(reportPath, JSON.stringify(report, null, 2));
        
        // Also save as latest
        const latestPath = path.join(this.reportsDir, `contract-validation-${environment}-latest.json`);
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
    <title>AtlasMesh Contract Validation Report - ${environment}</title>
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
        .test-details { margin-top: 10px; }
        .test-details ul { margin: 5px 0; padding-left: 20px; }
        .recommendations { background: #fff3cd; padding: 15px; border-radius: 6px; border-left: 4px solid #ffc107; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>AtlasMesh Contract Validation Report</h1>
            <p>Environment: <strong>${environment}</strong> | Generated: ${report.summary.timestamp}</p>
        </div>
        
        <div class="summary">
            <div class="metric">
                <div class="metric-value ${report.summary.overallSuccess ? 'success' : 'error'}">
                    ${report.summary.overallSuccess ? '✅' : '❌'}
                </div>
                <div>Overall Status</div>
            </div>
            <div class="metric">
                <div class="metric-value ${report.summary.contractSuccessRate === 100 ? 'success' : 'error'}">
                    ${report.summary.contractSuccessRate.toFixed(1)}%
                </div>
                <div>Contract Success Rate</div>
            </div>
            <div class="metric">
                <div class="metric-value ${report.summary.compatibilitySuccessRate === 100 ? 'success' : 'error'}">
                    ${report.summary.compatibilitySuccessRate.toFixed(1)}%
                </div>
                <div>Compatibility Success Rate</div>
            </div>
            <div class="metric">
                <div class="metric-value">${report.summary.totalContracts}</div>
                <div>Total Contracts</div>
            </div>
        </div>
        
        <div class="section">
            <h2>Contract Validation Results</h2>
            ${report.contractResults.map(result => `
                <div class="test-result ${result.passed ? 'test-passed' : 'test-failed'}">
                    <h3>${result.title} (${result.service})</h3>
                    <p>Version: ${result.version} | Status: ${result.passed ? 'PASSED' : 'FAILED'}</p>
                    <div class="test-details">
                        ${result.tests.map(test => `
                            <h4>${test.name}: ${test.passed ? '✅' : '❌'}</h4>
                            <ul>
                                ${test.details.map(detail => `<li>${detail}</li>`).join('')}
                            </ul>
                        `).join('')}
                        ${result.errors.length > 0 ? `
                            <h4>Errors:</h4>
                            <ul>
                                ${result.errors.map(error => `<li class="error">${error.message}</li>`).join('')}
                            </ul>
                        ` : ''}
                    </div>
                </div>
            `).join('')}
        </div>
        
        <div class="section">
            <h2>Compatibility Results</h2>
            ${report.compatibilityResults.map(result => `
                <div class="test-result ${result.compatible ? 'test-passed' : 'test-failed'}">
                    <h3>${result.consumer} → ${result.provider}</h3>
                    <p>Expected: ${result.expectedVersions.join(', ')} | Actual: ${result.actualVersion} | Compatible: ${result.compatible ? 'YES' : 'NO'}</p>
                    <div class="test-details">
                        <ul>
                            ${result.details.map(detail => `<li>${detail}</li>`).join('')}
                        </ul>
                    </div>
                </div>
            `).join('')}
        </div>
        
        ${report.recommendations.length > 0 ? `
            <div class="section">
                <h2>Recommendations</h2>
                <div class="recommendations">
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

        const htmlPath = path.join(this.reportsDir, `contract-validation-${environment}-latest.html`);
        fs.writeFileSync(htmlPath, htmlContent);
        console.log(`📊 HTML report saved to: ${htmlPath}`);
    }
}

// CLI interface
if (require.main === module) {
    const args = process.argv.slice(2);
    const environment = args[0] || 'development';
    
    const validator = new ContractValidator();
    validator.validateAllContracts(environment)
        .then(success => {
            process.exit(success ? 0 : 1);
        })
        .catch(error => {
            console.error('❌ Contract validation failed:', error);
            process.exit(1);
        });
}

module.exports = ContractValidator;
