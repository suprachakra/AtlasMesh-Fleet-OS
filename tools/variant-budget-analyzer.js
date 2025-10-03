#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Variant Budget Analyzer
 * 
 * Enforces the 7-dimensional agnostic architecture by monitoring code variance
 * across vehicle, platform, sector, sensor, map, weather, and comms dimensions.
 * 
 * Fails CI if any single agnostic tenet causes >5% core code delta or >25% test time delta.
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Agnostic dimensions and their associated code paths
const AGNOSTIC_DIMENSIONS = {
  'vehicle': {
    paths: ['adapters/vehicles/', 'configs/vehicles/', 'edge/vehicle-agent/'],
    description: 'Vehicle-agnostic abstraction layer'
  },
  'platform': {
    paths: ['deploy/', 'configs/base/', 'services/'],
    description: 'Platform-agnostic deployment and services'
  },
  'sector': {
    paths: ['configs/sectors/', 'rules/policy/', 'ui/sector-overlays/'],
    description: 'Sector-agnostic overlays and policies'
  },
  'sensor': {
    paths: ['adapters/sensors/', 'ml/models/perception/', 'edge/sensor-fusion/'],
    description: 'Sensor-agnostic perception and fusion'
  },
  'map': {
    paths: ['services/map-service/', 'adapters/maps/', 'tools/map-converter/'],
    description: 'Map-source-agnostic routing and navigation'
  },
  'weather': {
    paths: ['services/weather-fusion/', 'adapters/weather/', 'ml/models/weather/'],
    description: 'Weather-source-agnostic fusion and routing'
  },
  'comms': {
    paths: ['edge/cloud-bridge/', 'services/gateway-vehicle/', 'adapters/comms/'],
    description: 'Communications-agnostic connectivity'
  }
};

// Budget thresholds from environment or defaults
const CORE_DELTA_MAX = parseInt(process.env.VARIANT_BUDGET_CORE_DELTA_MAX) || 5; // 5%
const TEST_TIME_MAX = parseInt(process.env.VARIANT_BUDGET_TEST_TIME_MAX) || 25; // 25%

class VariantBudgetAnalyzer {
  constructor() {
    this.results = {
      dimensions: {},
      overall: {
        core_delta_pct: 0,
        test_time_delta_pct: 0,
        budget_exceeded: false,
        violations: []
      }
    };
  }

  /**
   * Get git diff statistics for specific paths
   */
  getGitDiffStats(paths) {
    try {
      const pathArgs = paths.map(p => `"${p}"`).join(' ');
      const diffCmd = `git diff --stat HEAD~1 HEAD -- ${pathArgs}`;
      const diffOutput = execSync(diffCmd, { encoding: 'utf8' });
      
      // Parse diff output to extract line changes
      const lines = diffOutput.split('\n').filter(line => line.trim());
      let totalInsertions = 0;
      let totalDeletions = 0;
      let filesChanged = 0;

      lines.forEach(line => {
        if (line.includes('insertion') || line.includes('deletion')) {
          const insertMatch = line.match(/(\d+) insertion/);
          const deleteMatch = line.match(/(\d+) deletion/);
          
          if (insertMatch) totalInsertions += parseInt(insertMatch[1]);
          if (deleteMatch) totalDeletions += parseInt(deleteMatch[1]);
          filesChanged++;
        }
      });

      return {
        files_changed: filesChanged,
        insertions: totalInsertions,
        deletions: totalDeletions,
        total_changes: totalInsertions + totalDeletions
      };
    } catch (error) {
      console.warn(`Warning: Could not get git diff stats for paths: ${paths.join(', ')}`);
      return { files_changed: 0, insertions: 0, deletions: 0, total_changes: 0 };
    }
  }

  /**
   * Calculate core code delta percentage for a dimension
   */
  calculateCoreDelta(dimensionStats, baselineStats) {
    if (baselineStats.total_changes === 0) return 0;
    return (dimensionStats.total_changes / baselineStats.total_changes) * 100;
  }

  /**
   * Estimate test time impact (simplified heuristic)
   */
  estimateTestTimeImpact(dimensionStats) {
    // Heuristic: more files changed = more test time
    // This would be replaced with actual test timing data
    const baseTestTime = 100; // seconds baseline
    const timePerFile = 5; // seconds per changed file
    
    const estimatedTime = baseTestTime + (dimensionStats.files_changed * timePerFile);
    const deltaPercent = ((estimatedTime - baseTestTime) / baseTestTime) * 100;
    
    return Math.min(deltaPercent, 50); // Cap at 50% for this heuristic
  }

  /**
   * Get baseline statistics (all changes in this commit)
   */
  getBaselineStats() {
    try {
      const diffCmd = 'git diff --stat HEAD~1 HEAD';
      const diffOutput = execSync(diffCmd, { encoding: 'utf8' });
      
      let totalInsertions = 0;
      let totalDeletions = 0;
      let filesChanged = 0;

      const lines = diffOutput.split('\n').filter(line => line.trim());
      lines.forEach(line => {
        if (line.includes('insertion') || line.includes('deletion')) {
          const insertMatch = line.match(/(\d+) insertion/);
          const deleteMatch = line.match(/(\d+) deletion/);
          
          if (insertMatch) totalInsertions += parseInt(insertMatch[1]);
          if (deleteMatch) totalDeletions += parseInt(deleteMatch[1]);
          filesChanged++;
        }
      });

      return {
        files_changed: filesChanged,
        insertions: totalInsertions,
        deletions: totalDeletions,
        total_changes: totalInsertions + totalDeletions
      };
    } catch (error) {
      console.warn('Warning: Could not get baseline git stats, using defaults');
      return { files_changed: 1, insertions: 10, deletions: 0, total_changes: 10 };
    }
  }

  /**
   * Analyze variant budget for all dimensions
   */
  analyze() {
    console.log('🔍 Analyzing variant budget across agnostic dimensions...\n');
    
    const baselineStats = this.getBaselineStats();
    console.log(`📊 Baseline stats: ${baselineStats.files_changed} files, ${baselineStats.total_changes} total changes\n`);

    // Analyze each agnostic dimension
    Object.entries(AGNOSTIC_DIMENSIONS).forEach(([dimension, config]) => {
      console.log(`Analyzing ${dimension} dimension: ${config.description}`);
      
      const dimensionStats = this.getGitDiffStats(config.paths);
      const coreDelta = this.calculateCoreDelta(dimensionStats, baselineStats);
      const testTimeDelta = this.estimateTestTimeImpact(dimensionStats);
      
      this.results.dimensions[dimension] = {
        description: config.description,
        paths: config.paths,
        stats: dimensionStats,
        core_delta_pct: coreDelta,
        test_time_delta_pct: testTimeDelta,
        core_budget_exceeded: coreDelta > CORE_DELTA_MAX,
        test_time_budget_exceeded: testTimeDelta > TEST_TIME_MAX
      };

      // Update overall results
      this.results.overall.core_delta_pct = Math.max(this.results.overall.core_delta_pct, coreDelta);
      this.results.overall.test_time_delta_pct = Math.max(this.results.overall.test_time_delta_pct, testTimeDelta);

      // Check for violations
      if (coreDelta > CORE_DELTA_MAX) {
        this.results.overall.violations.push(`${dimension}: Core delta ${coreDelta.toFixed(1)}% exceeds ${CORE_DELTA_MAX}% limit`);
      }
      if (testTimeDelta > TEST_TIME_MAX) {
        this.results.overall.violations.push(`${dimension}: Test time delta ${testTimeDelta.toFixed(1)}% exceeds ${TEST_TIME_MAX}% limit`);
      }

      console.log(`  📈 Core delta: ${coreDelta.toFixed(1)}% (limit: ${CORE_DELTA_MAX}%)`);
      console.log(`  ⏱️  Test time delta: ${testTimeDelta.toFixed(1)}% (limit: ${TEST_TIME_MAX}%)`);
      console.log(`  📁 Files changed: ${dimensionStats.files_changed}`);
      console.log('');
    });

    this.results.overall.budget_exceeded = this.results.overall.violations.length > 0;
  }

  /**
   * Generate detailed report
   */
  generateReport() {
    const reportPath = 'ci/variant-budget-report.json';
    
    // Ensure directory exists
    const reportDir = path.dirname(reportPath);
    if (!fs.existsSync(reportDir)) {
      fs.mkdirSync(reportDir, { recursive: true });
    }

    // Write detailed report
    fs.writeFileSync(reportPath, JSON.stringify(this.results, null, 2));
    
    console.log(`📋 Detailed report written to: ${reportPath}`);
    return reportPath;
  }

  /**
   * Print summary and exit with appropriate code
   */
  printSummaryAndExit() {
    console.log('🎯 VARIANT BUDGET ANALYSIS SUMMARY');
    console.log('=====================================');
    console.log(`Overall core delta: ${this.results.overall.core_delta_pct.toFixed(1)}% (limit: ${CORE_DELTA_MAX}%)`);
    console.log(`Overall test time delta: ${this.results.overall.test_time_delta_pct.toFixed(1)}% (limit: ${TEST_TIME_MAX}%)`);
    console.log('');

    if (this.results.overall.budget_exceeded) {
      console.log('❌ VARIANT BUDGET EXCEEDED');
      console.log('Violations:');
      this.results.overall.violations.forEach(violation => {
        console.log(`  • ${violation}`);
      });
      console.log('');
      console.log('🔧 Recommended actions:');
      console.log('  1. Review changes for unnecessary complexity');
      console.log('  2. Consider refactoring to reduce dimension-specific code');
      console.log('  3. Submit to CCB (Change Control Board) if variance is justified');
      console.log('  4. Break changes into smaller, focused commits');
      
      process.exit(1);
    } else {
      console.log('✅ VARIANT BUDGET WITHIN LIMITS');
      console.log('All agnostic dimensions are within acceptable variance thresholds.');
      
      process.exit(0);
    }
  }

  /**
   * Run complete analysis
   */
  run() {
    try {
      this.analyze();
      this.generateReport();
      this.printSummaryAndExit();
    } catch (error) {
      console.error('❌ Variant budget analysis failed:', error.message);
      process.exit(1);
    }
  }
}

// Run analysis if called directly
if (require.main === module) {
  const analyzer = new VariantBudgetAnalyzer();
  analyzer.run();
}

module.exports = VariantBudgetAnalyzer;
