#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - API Version Manager
 * 
 * Manages API versioning, deprecation, and migration processes
 * Ensures backward compatibility and smooth version transitions
 */

const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const semver = require('semver');
const { execSync } = require('child_process');

class APIVersionManager {
    constructor() {
        this.contractsDir = path.join(__dirname, '../../api/contracts');
        this.registryPath = path.join(__dirname, '../../api/contracts/registry.yaml');
        this.versionsDir = path.join(this.contractsDir, 'versions');
        this.migrationsDir = path.join(__dirname, 'migrations');
        this.changelogPath = path.join(__dirname, '../../CHANGELOG.md');
        
        // Load registry
        this.registry = this.loadRegistry();
        
        // Version management configuration
        this.config = {
            supportedVersions: 3, // Number of versions to support simultaneously
            deprecationPeriodDays: 90,
            supportPeriodDays: 180,
            breakingChangePolicy: 'major_version_only',
            autoMigration: false
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

    saveRegistry() {
        try {
            fs.writeFileSync(this.registryPath, yaml.dump(this.registry, { indent: 2 }));
        } catch (error) {
            console.error('❌ Failed to save contract registry:', error.message);
            throw error;
        }
    }

    async createNewVersion(serviceName, versionType = 'patch', changes = []) {
        console.log(`🚀 Creating new version for ${serviceName}...`);
        
        try {
            // Validate service exists
            if (!this.registry.services[serviceName]) {
                throw new Error(`Service '${serviceName}' not found in registry`);
            }
            
            const service = this.registry.services[serviceName];
            const currentVersion = service.current_version;
            
            // Calculate new version
            const newVersion = semver.inc(currentVersion, versionType);
            if (!newVersion) {
                throw new Error(`Invalid version increment: ${currentVersion} -> ${versionType}`);
            }
            
            console.log(`📈 Incrementing version: ${currentVersion} -> ${newVersion}`);
            
            // Validate breaking changes
            if (versionType === 'major') {
                await this.validateBreakingChanges(serviceName, changes);
            }
            
            // Create version directory structure
            this.createVersionStructure(serviceName, newVersion);
            
            // Copy current contract to new version
            await this.copyContractToNewVersion(serviceName, currentVersion, newVersion);
            
            // Update registry
            this.updateRegistryForNewVersion(serviceName, newVersion);
            
            // Generate migration guide
            await this.generateMigrationGuide(serviceName, currentVersion, newVersion, changes);
            
            // Update changelog
            this.updateChangelog(serviceName, currentVersion, newVersion, changes);
            
            // Validate new version
            await this.validateNewVersion(serviceName, newVersion);
            
            console.log(`✅ Successfully created version ${newVersion} for ${serviceName}`);
            
            return {
                service: serviceName,
                previousVersion: currentVersion,
                newVersion: newVersion,
                changes: changes
            };
            
        } catch (error) {
            console.error(`❌ Failed to create new version for ${serviceName}:`, error.message);
            throw error;
        }
    }

    createVersionStructure(serviceName, version) {
        const versionDir = path.join(this.versionsDir, serviceName, version);
        fs.mkdirSync(versionDir, { recursive: true });
        
        // Create subdirectories
        const subdirs = ['contracts', 'schemas', 'examples', 'tests'];
        subdirs.forEach(subdir => {
            fs.mkdirSync(path.join(versionDir, subdir), { recursive: true });
        });
    }

    async copyContractToNewVersion(serviceName, currentVersion, newVersion) {
        const currentContractPath = path.join(this.contractsDir, 'v1', `${serviceName}.yaml`);
        const newContractPath = path.join(this.versionsDir, serviceName, newVersion, 'contracts', `${serviceName}.yaml`);
        
        if (fs.existsSync(currentContractPath)) {
            // Load and update contract
            const contract = yaml.load(fs.readFileSync(currentContractPath, 'utf8'));
            contract.info.version = newVersion;
            
            // Update server URLs to include version
            if (contract.servers) {
                contract.servers = contract.servers.map(server => ({
                    ...server,
                    url: server.url.replace(/\/v\d+/, `/v${semver.major(newVersion)}`)
                }));
            }
            
            fs.writeFileSync(newContractPath, yaml.dump(contract, { indent: 2 }));
        } else {
            throw new Error(`Current contract not found: ${currentContractPath}`);
        }
    }

    updateRegistryForNewVersion(serviceName, newVersion) {
        const service = this.registry.services[serviceName];
        
        // Update current version
        service.current_version = newVersion;
        
        // Add to supported versions
        if (!service.supported_versions.includes(newVersion)) {
            service.supported_versions.push(newVersion);
        }
        
        // Sort supported versions
        service.supported_versions.sort((a, b) => semver.compare(b, a));
        
        // Limit supported versions
        if (service.supported_versions.length > this.config.supportedVersions) {
            const removedVersions = service.supported_versions.splice(this.config.supportedVersions);
            console.log(`📋 Removed support for versions: ${removedVersions.join(', ')}`);
        }
        
        // Update contract path
        service.contract_path = `/api/contracts/versions/${serviceName}/${newVersion}/contracts/${serviceName}.yaml`;
        
        this.saveRegistry();
    }

    async deprecateVersion(serviceName, version, reason = 'Superseded by newer version') {
        console.log(`⚠️  Deprecating version ${version} of ${serviceName}...`);
        
        try {
            const service = this.registry.services[serviceName];
            if (!service) {
                throw new Error(`Service '${serviceName}' not found`);
            }
            
            if (!service.supported_versions.includes(version)) {
                throw new Error(`Version ${version} is not in supported versions`);
            }
            
            // Add to deprecated versions in lifecycle
            if (!this.registry.versioning.lifecycle.deprecated) {
                this.registry.versioning.lifecycle.deprecated = [];
            }
            
            const deprecationEntry = {
                service: serviceName,
                version: version,
                reason: reason,
                deprecated_date: new Date().toISOString(),
                end_of_support_date: new Date(Date.now() + this.config.supportPeriodDays * 24 * 60 * 60 * 1000).toISOString()
            };
            
            this.registry.versioning.lifecycle.deprecated.push(deprecationEntry);
            
            // Generate deprecation notice
            await this.generateDeprecationNotice(serviceName, version, deprecationEntry);
            
            // Update changelog
            this.updateChangelog(serviceName, version, null, [{
                type: 'deprecation',
                description: `Version ${version} deprecated: ${reason}`
            }]);
            
            this.saveRegistry();
            
            console.log(`✅ Version ${version} of ${serviceName} has been deprecated`);
            console.log(`📅 End of support: ${deprecationEntry.end_of_support_date}`);
            
            return deprecationEntry;
            
        } catch (error) {
            console.error(`❌ Failed to deprecate version ${version} of ${serviceName}:`, error.message);
            throw error;
        }
    }

    async retireVersion(serviceName, version) {
        console.log(`🚫 Retiring version ${version} of ${serviceName}...`);
        
        try {
            const service = this.registry.services[serviceName];
            if (!service) {
                throw new Error(`Service '${serviceName}' not found`);
            }
            
            // Remove from supported versions
            service.supported_versions = service.supported_versions.filter(v => v !== version);
            
            // Add to retired versions
            if (!this.registry.versioning.lifecycle.retired) {
                this.registry.versioning.lifecycle.retired = [];
            }
            
            this.registry.versioning.lifecycle.retired.push({
                service: serviceName,
                version: version,
                retired_date: new Date().toISOString()
            });
            
            // Remove from deprecated list
            if (this.registry.versioning.lifecycle.deprecated) {
                this.registry.versioning.lifecycle.deprecated = 
                    this.registry.versioning.lifecycle.deprecated.filter(
                        d => !(d.service === serviceName && d.version === version)
                    );
            }
            
            // Archive version files
            await this.archiveVersion(serviceName, version);
            
            this.saveRegistry();
            
            console.log(`✅ Version ${version} of ${serviceName} has been retired`);
            
        } catch (error) {
            console.error(`❌ Failed to retire version ${version} of ${serviceName}:`, error.message);
            throw error;
        }
    }

    async validateBreakingChanges(serviceName, changes) {
        const breakingChanges = changes.filter(change => change.breaking === true);
        
        if (breakingChanges.length > 0) {
            console.log(`⚠️  Found ${breakingChanges.length} breaking changes:`);
            breakingChanges.forEach(change => {
                console.log(`  - ${change.description}`);
            });
            
            // Validate that breaking changes are allowed
            if (this.config.breakingChangePolicy === 'major_version_only') {
                console.log('✅ Breaking changes are allowed in major version increments');
            } else {
                throw new Error('Breaking changes are not allowed with current policy');
            }
        }
    }

    async generateMigrationGuide(serviceName, fromVersion, toVersion, changes) {
        console.log(`📝 Generating migration guide: ${fromVersion} -> ${toVersion}...`);
        
        const migrationDir = path.join(this.migrationsDir, serviceName);
        fs.mkdirSync(migrationDir, { recursive: true });
        
        const migrationFile = path.join(migrationDir, `${fromVersion}-to-${toVersion}.md`);
        
        const breakingChanges = changes.filter(c => c.breaking);
        const newFeatures = changes.filter(c => c.type === 'feature');
        const improvements = changes.filter(c => c.type === 'improvement');
        const bugFixes = changes.filter(c => c.type === 'bugfix');
        
        const migrationGuide = `# Migration Guide: ${serviceName} ${fromVersion} → ${toVersion}

## Overview

This guide helps you migrate from ${serviceName} version ${fromVersion} to ${toVersion}.

**Migration Complexity:** ${breakingChanges.length > 0 ? 'HIGH (Breaking Changes)' : 'LOW (Backward Compatible)'}

## Summary of Changes

### 🚨 Breaking Changes
${breakingChanges.length > 0 ? breakingChanges.map(c => `- ${c.description}`).join('\n') : 'None'}

### ✨ New Features
${newFeatures.length > 0 ? newFeatures.map(c => `- ${c.description}`).join('\n') : 'None'}

### 🔧 Improvements
${improvements.length > 0 ? improvements.map(c => `- ${c.description}`).join('\n') : 'None'}

### 🐛 Bug Fixes
${bugFixes.length > 0 ? bugFixes.map(c => `- ${c.description}`).join('\n') : 'None'}

## Migration Steps

${breakingChanges.length > 0 ? `
### Step 1: Review Breaking Changes

${breakingChanges.map((c, i) => `
#### ${i + 1}. ${c.title || c.description}

**Impact:** ${c.impact || 'Not specified'}

**Required Action:** ${c.action || 'Update your code to handle this change'}

**Example:**
\`\`\`
// Before (${fromVersion})
${c.before || '// Old code example'}

// After (${toVersion})
${c.after || '// New code example'}
\`\`\`
`).join('\n')}

### Step 2: Update Dependencies

Update your SDK or client library to version ${toVersion}:

\`\`\`bash
# Go SDK
go get github.com/atlasmesh/fleet-os-sdk-go@v${toVersion}

# TypeScript SDK
npm install @atlasmesh/fleet-os-sdk@${toVersion}

# Python SDK
pip install atlasmesh-fleet-os==${toVersion}
\`\`\`

### Step 3: Update API Endpoints

Update your base URLs to use the new API version:

\`\`\`
Old: https://api.atlasmesh.com/${serviceName}/v${semver.major(fromVersion)}
New: https://api.atlasmesh.com/${serviceName}/v${semver.major(toVersion)}
\`\`\`

### Step 4: Test Your Integration

1. Run your test suite against the new version
2. Verify all API calls work as expected
3. Check for any deprecation warnings
4. Update any hardcoded version references

` : `
### Step 1: Update Dependencies

This is a backward-compatible update. Simply update your dependencies:

\`\`\`bash
# Go SDK
go get github.com/atlasmesh/fleet-os-sdk-go@v${toVersion}

# TypeScript SDK
npm install @atlasmesh/fleet-os-sdk@${toVersion}

# Python SDK
pip install atlasmesh-fleet-os==${toVersion}
\`\`\`

### Step 2: Test Your Integration

1. Run your test suite to ensure everything works
2. Check for any new features you might want to use
3. Review the changelog for improvements
`}

## Rollback Plan

If you encounter issues with the new version:

1. **Immediate Rollback:** Revert to version ${fromVersion}
2. **Gradual Migration:** Use feature flags to gradually adopt new features
3. **Support:** Contact support@atlasmesh.com for assistance

## Timeline

- **Migration Period:** ${this.config.deprecationPeriodDays} days
- **Support End Date:** ${new Date(Date.now() + this.config.supportPeriodDays * 24 * 60 * 60 * 1000).toISOString().split('T')[0]}

## Resources

- [API Documentation](https://docs.atlasmesh.com/api/${serviceName}/${toVersion})
- [SDK Documentation](https://docs.atlasmesh.com/sdk/)
- [Support Portal](https://support.atlasmesh.com)

---

*Generated on ${new Date().toISOString()}*
`;

        fs.writeFileSync(migrationFile, migrationGuide);
        console.log(`📄 Migration guide saved: ${migrationFile}`);
    }

    async generateDeprecationNotice(serviceName, version, deprecationEntry) {
        const noticeDir = path.join(__dirname, 'notices');
        fs.mkdirSync(noticeDir, { recursive: true });
        
        const noticeFile = path.join(noticeDir, `${serviceName}-${version}-deprecation.md`);
        
        const notice = `# Deprecation Notice: ${serviceName} v${version}

## Summary

Version ${version} of the ${serviceName} API has been deprecated and will be retired on ${deprecationEntry.end_of_support_date.split('T')[0]}.

## Reason

${deprecationEntry.reason}

## Action Required

Please migrate to the latest version of the ${serviceName} API before the retirement date.

## Migration Resources

- [Migration Guide](../migrations/${serviceName}/)
- [Latest API Documentation](https://docs.atlasmesh.com/api/${serviceName}/)
- [SDK Updates](https://docs.atlasmesh.com/sdk/)

## Timeline

- **Deprecation Date:** ${deprecationEntry.deprecated_date.split('T')[0]}
- **End of Support:** ${deprecationEntry.end_of_support_date.split('T')[0]}
- **Retirement Date:** ${deprecationEntry.end_of_support_date.split('T')[0]}

## Support

If you need assistance with the migration, please contact:
- Email: support@atlasmesh.com
- Documentation: https://docs.atlasmesh.com
- Support Portal: https://support.atlasmesh.com

---

*This notice was generated automatically on ${new Date().toISOString()}*
`;

        fs.writeFileSync(noticeFile, notice);
        console.log(`📢 Deprecation notice saved: ${noticeFile}`);
    }

    updateChangelog(serviceName, fromVersion, toVersion, changes) {
        const changelogEntry = `
## [${toVersion || fromVersion}] - ${new Date().toISOString().split('T')[0]}

### ${serviceName}

${changes.map(change => {
    const emoji = {
        'feature': '✨',
        'improvement': '🔧',
        'bugfix': '🐛',
        'deprecation': '⚠️',
        'breaking': '🚨'
    }[change.type] || '📝';
    
    return `${emoji} ${change.description}${change.breaking ? ' **[BREAKING]**' : ''}`;
}).join('\n')}
`;

        // Append to changelog
        if (fs.existsSync(this.changelogPath)) {
            const currentChangelog = fs.readFileSync(this.changelogPath, 'utf8');
            const updatedChangelog = currentChangelog.replace(
                '# Changelog',
                `# Changelog${changelogEntry}`
            );
            fs.writeFileSync(this.changelogPath, updatedChangelog);
        } else {
            fs.writeFileSync(this.changelogPath, `# Changelog${changelogEntry}`);
        }
    }

    async validateNewVersion(serviceName, version) {
        console.log(`🔍 Validating new version ${version}...`);
        
        const contractPath = path.join(this.versionsDir, serviceName, version, 'contracts', `${serviceName}.yaml`);
        
        if (!fs.existsSync(contractPath)) {
            throw new Error(`Contract file not found: ${contractPath}`);
        }
        
        // Validate OpenAPI spec
        try {
            const validateCmd = `npx swagger-parser validate "${contractPath}"`;
            execSync(validateCmd, { stdio: 'pipe' });
            console.log('✅ OpenAPI specification is valid');
        } catch (error) {
            throw new Error(`OpenAPI validation failed: ${error.message}`);
        }
        
        // Additional validations could go here
        // - Schema compatibility checks
        // - Endpoint availability checks
        // - Security validation
    }

    async archiveVersion(serviceName, version) {
        const versionDir = path.join(this.versionsDir, serviceName, version);
        const archiveDir = path.join(__dirname, 'archives', serviceName);
        
        fs.mkdirSync(archiveDir, { recursive: true });
        
        if (fs.existsSync(versionDir)) {
            const archivePath = path.join(archiveDir, `${version}.tar.gz`);
            execSync(`tar -czf "${archivePath}" -C "${path.dirname(versionDir)}" "${version}"`);
            console.log(`📦 Version ${version} archived to: ${archivePath}`);
        }
    }

    async getVersionStatus() {
        const status = {
            services: {},
            deprecated: [],
            retired: [],
            summary: {
                totalServices: Object.keys(this.registry.services).length,
                totalVersions: 0,
                deprecatedVersions: 0,
                retiredVersions: 0
            }
        };

        // Collect service version information
        for (const [serviceName, service] of Object.entries(this.registry.services)) {
            status.services[serviceName] = {
                currentVersion: service.current_version,
                supportedVersions: service.supported_versions,
                totalVersions: service.supported_versions.length
            };
            status.summary.totalVersions += service.supported_versions.length;
        }

        // Collect deprecated versions
        if (this.registry.versioning.lifecycle.deprecated) {
            status.deprecated = this.registry.versioning.lifecycle.deprecated;
            status.summary.deprecatedVersions = status.deprecated.length;
        }

        // Collect retired versions
        if (this.registry.versioning.lifecycle.retired) {
            status.retired = this.registry.versioning.lifecycle.retired;
            status.summary.retiredVersions = status.retired.length;
        }

        return status;
    }

    async listVersions(serviceName = null) {
        if (serviceName) {
            const service = this.registry.services[serviceName];
            if (!service) {
                throw new Error(`Service '${serviceName}' not found`);
            }
            
            console.log(`\n📋 Versions for ${serviceName}:`);
            console.log(`Current: ${service.current_version}`);
            console.log(`Supported: ${service.supported_versions.join(', ')}`);
        } else {
            console.log('\n📋 All Service Versions:');
            for (const [name, service] of Object.entries(this.registry.services)) {
                console.log(`${name}: ${service.current_version} (${service.supported_versions.length} supported)`);
            }
        }
    }
}

// CLI interface
if (require.main === module) {
    const args = process.argv.slice(2);
    const command = args[0];
    
    const manager = new APIVersionManager();
    
    switch (command) {
        case 'create':
            const serviceName = args[1];
            const versionType = args[2] || 'patch';
            const changeDescription = args[3] || 'Version update';
            
            if (!serviceName) {
                console.error('Usage: node version-manager.js create <service-name> [patch|minor|major] [description]');
                process.exit(1);
            }
            
            manager.createNewVersion(serviceName, versionType, [{
                type: 'improvement',
                description: changeDescription
            }]).catch(console.error);
            break;
            
        case 'deprecate':
            const depServiceName = args[1];
            const depVersion = args[2];
            const depReason = args[3] || 'Superseded by newer version';
            
            if (!depServiceName || !depVersion) {
                console.error('Usage: node version-manager.js deprecate <service-name> <version> [reason]');
                process.exit(1);
            }
            
            manager.deprecateVersion(depServiceName, depVersion, depReason).catch(console.error);
            break;
            
        case 'retire':
            const retServiceName = args[1];
            const retVersion = args[2];
            
            if (!retServiceName || !retVersion) {
                console.error('Usage: node version-manager.js retire <service-name> <version>');
                process.exit(1);
            }
            
            manager.retireVersion(retServiceName, retVersion).catch(console.error);
            break;
            
        case 'status':
            manager.getVersionStatus().then(status => {
                console.log('\n📊 API Version Status:');
                console.log(JSON.stringify(status, null, 2));
            }).catch(console.error);
            break;
            
        case 'list':
            const listServiceName = args[1];
            manager.listVersions(listServiceName).catch(console.error);
            break;
            
        default:
            console.log(`
AtlasMesh API Version Manager

Usage:
  node version-manager.js create <service-name> [patch|minor|major] [description]
  node version-manager.js deprecate <service-name> <version> [reason]
  node version-manager.js retire <service-name> <version>
  node version-manager.js status
  node version-manager.js list [service-name]

Examples:
  node version-manager.js create trip-service minor "Added new trip status"
  node version-manager.js deprecate trip-service 1.0.0 "Security vulnerability fixed in 1.1.0"
  node version-manager.js retire trip-service 0.9.0
  node version-manager.js status
  node version-manager.js list trip-service
            `);
    }
}

module.exports = APIVersionManager;
