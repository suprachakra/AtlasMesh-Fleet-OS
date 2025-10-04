#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Chaos Engineering Test Runner

This script orchestrates chaos engineering tests to validate system resilience.
"""

import asyncio
import aiohttp
import yaml
import json
import time
import logging
import subprocess
import sys
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class ChaosScenario:
    """Chaos scenario configuration"""
    name: str
    description: str
    scenarios: List[str]
    duration: str
    success_criteria: List[Dict]

@dataclass
class ChaosResult:
    """Chaos test result"""
    scenario_name: str
    passed: bool
    duration: float
    message: str
    metrics: Dict
    details: Optional[Dict] = None

class PrometheusClient:
    """Prometheus client for metrics collection"""
    
    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip('/')
        self.session: Optional[aiohttp.ClientSession] = None
    
    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()
    
    async def query(self, query: str) -> Optional[float]:
        """Execute Prometheus query and return scalar result"""
        try:
            url = f"{self.base_url}/api/v1/query"
            params = {"query": query}
            
            async with self.session.get(url, params=params) as response:
                if response.status != 200:
                    logger.error(f"Prometheus query failed: {response.status}")
                    return None
                
                data = await response.json()
                result = data.get('data', {}).get('result', [])
                
                if not result:
                    return None
                
                # Return the first scalar value
                value = result[0].get('value', [None, None])[1]
                return float(value) if value is not None else None
                
        except Exception as e:
            logger.error(f"Prometheus query error: {str(e)}")
            return None
    
    async def query_range(self, query: str, start: datetime, end: datetime, step: str = "30s") -> List[Tuple[datetime, float]]:
        """Execute Prometheus range query"""
        try:
            url = f"{self.base_url}/api/v1/query_range"
            params = {
                "query": query,
                "start": start.timestamp(),
                "end": end.timestamp(),
                "step": step
            }
            
            async with self.session.get(url, params=params) as response:
                if response.status != 200:
                    logger.error(f"Prometheus range query failed: {response.status}")
                    return []
                
                data = await response.json()
                result = data.get('data', {}).get('result', [])
                
                if not result:
                    return []
                
                # Return time series data
                values = result[0].get('values', [])
                return [(datetime.fromtimestamp(float(ts)), float(val)) for ts, val in values]
                
        except Exception as e:
            logger.error(f"Prometheus range query error: {str(e)}")
            return []

class KubernetesClient:
    """Kubernetes client for chaos operations"""
    
    def __init__(self, namespace: str = "fleet-os-staging"):
        self.namespace = namespace
    
    async def apply_chaos_scenario(self, scenario_file: str) -> bool:
        """Apply chaos scenario to Kubernetes"""
        try:
            cmd = ["kubectl", "apply", "-f", scenario_file, "-n", self.namespace]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                logger.error(f"Failed to apply chaos scenario: {result.stderr}")
                return False
            
            logger.info(f"Applied chaos scenario: {scenario_file}")
            return True
            
        except Exception as e:
            logger.error(f"Error applying chaos scenario: {str(e)}")
            return False
    
    async def delete_chaos_scenario(self, scenario_file: str) -> bool:
        """Delete chaos scenario from Kubernetes"""
        try:
            cmd = ["kubectl", "delete", "-f", scenario_file, "-n", self.namespace, "--ignore-not-found=true"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                logger.error(f"Failed to delete chaos scenario: {result.stderr}")
                return False
            
            logger.info(f"Deleted chaos scenario: {scenario_file}")
            return True
            
        except Exception as e:
            logger.error(f"Error deleting chaos scenario: {str(e)}")
            return False
    
    async def get_chaos_status(self, scenario_name: str) -> Optional[str]:
        """Get status of chaos scenario"""
        try:
            cmd = ["kubectl", "get", "chaos", scenario_name, "-n", self.namespace, "-o", "jsonpath={.status.phase}"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                return None
            
            return result.stdout.strip()
            
        except Exception as e:
            logger.error(f"Error getting chaos status: {str(e)}")
            return None

class ChaosTestRunner:
    """Main chaos engineering test runner"""
    
    def __init__(self, config_file: str, prometheus_url: str, kubernetes_namespace: str = "fleet-os-staging"):
        self.config_file = config_file
        self.prometheus = PrometheusClient(prometheus_url)
        self.k8s = KubernetesClient(kubernetes_namespace)
        self.results: List[ChaosResult] = []
        self.scenarios: List[ChaosScenario] = []
        self.prometheus_queries: Dict[str, str] = {}
        
    def load_config(self):
        """Load chaos test configuration"""
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Load scenarios
            for scenario_config in config.get('chaos_scenarios', []):
                scenario = ChaosScenario(
                    name=scenario_config['name'],
                    description=scenario_config['description'],
                    scenarios=scenario_config['scenarios'],
                    duration=scenario_config['duration'],
                    success_criteria=scenario_config['success_criteria']
                )
                self.scenarios.append(scenario)
            
            # Load Prometheus queries
            self.prometheus_queries = config.get('prometheus_queries', {})
            
            logger.info(f"Loaded {len(self.scenarios)} chaos scenarios")
            
        except Exception as e:
            logger.error(f"Failed to load config: {str(e)}")
            raise
    
    def _parse_duration(self, duration_str: str) -> int:
        """Parse duration string to seconds"""
        if duration_str.endswith('s'):
            return int(duration_str[:-1])
        elif duration_str.endswith('m'):
            return int(duration_str[:-1]) * 60
        elif duration_str.endswith('h'):
            return int(duration_str[:-1]) * 3600
        else:
            return int(duration_str)
    
    async def _collect_baseline_metrics(self) -> Dict[str, float]:
        """Collect baseline metrics before chaos test"""
        metrics = {}
        
        async with self.prometheus as prom:
            for metric_name, query in self.prometheus_queries.items():
                value = await prom.query(query)
                if value is not None:
                    metrics[metric_name] = value
                    logger.debug(f"Baseline {metric_name}: {value}")
        
        return metrics
    
    async def _collect_test_metrics(self, start_time: datetime, end_time: datetime) -> Dict[str, float]:
        """Collect metrics during/after chaos test"""
        metrics = {}
        
        async with self.prometheus as prom:
            for metric_name, query in self.prometheus_queries.items():
                # Get average value over the test period
                range_data = await prom.query_range(query, start_time, end_time)
                if range_data:
                    avg_value = sum(val for _, val in range_data) / len(range_data)
                    metrics[metric_name] = avg_value
                    logger.debug(f"Test {metric_name}: {avg_value}")
        
        return metrics
    
    def _evaluate_success_criteria(self, metrics: Dict[str, float], criteria: List[Dict]) -> Tuple[bool, List[str]]:
        """Evaluate success criteria against collected metrics"""
        passed = True
        messages = []
        
        for criterion in criteria:
            metric_name = criterion['metric']
            threshold = criterion['threshold']
            
            if metric_name not in metrics:
                passed = False
                messages.append(f"Metric {metric_name} not available")
                continue
            
            metric_value = metrics[metric_name]
            
            # Parse threshold (e.g., ">= 99%", "<= 2s", "true")
            if threshold.startswith('>='):
                expected = float(threshold[2:].strip().rstrip('%'))
                if threshold.endswith('%'):
                    expected /= 100
                if metric_value < expected:
                    passed = False
                    messages.append(f"{metric_name}: {metric_value} < {expected} (threshold: {threshold})")
                else:
                    messages.append(f"{metric_name}: {metric_value} >= {expected} ✓")
            
            elif threshold.startswith('<='):
                expected = float(threshold[2:].strip().rstrip('%s'))
                if threshold.endswith('%'):
                    expected /= 100
                if metric_value > expected:
                    passed = False
                    messages.append(f"{metric_name}: {metric_value} > {expected} (threshold: {threshold})")
                else:
                    messages.append(f"{metric_name}: {metric_value} <= {expected} ✓")
            
            elif threshold == "true":
                if metric_value <= 0:
                    passed = False
                    messages.append(f"{metric_name}: {metric_value} is not true (threshold: {threshold})")
                else:
                    messages.append(f"{metric_name}: {metric_value} is true ✓")
            
            else:
                messages.append(f"Unknown threshold format: {threshold}")
        
        return passed, messages
    
    async def run_chaos_scenario(self, scenario: ChaosScenario) -> ChaosResult:
        """Run a single chaos scenario"""
        logger.info(f"Starting chaos scenario: {scenario.name}")
        start_time = time.time()
        test_start = datetime.utcnow()
        
        try:
            # Collect baseline metrics
            baseline_metrics = await self._collect_baseline_metrics()
            logger.info(f"Collected baseline metrics: {len(baseline_metrics)} metrics")
            
            # Apply chaos scenarios
            scenario_files = []
            for scenario_name in scenario.scenarios:
                scenario_file = f"testing/chaos/scenarios/{scenario_name}.yaml"
                if Path(scenario_file).exists():
                    if await self.k8s.apply_chaos_scenario(scenario_file):
                        scenario_files.append(scenario_file)
                    else:
                        raise Exception(f"Failed to apply scenario: {scenario_name}")
                else:
                    logger.warning(f"Scenario file not found: {scenario_file}")
            
            # Wait for chaos duration
            duration_seconds = self._parse_duration(scenario.duration)
            logger.info(f"Running chaos for {duration_seconds} seconds...")
            
            # Monitor system during chaos
            await asyncio.sleep(duration_seconds)
            
            test_end = datetime.utcnow()
            
            # Clean up chaos scenarios
            for scenario_file in scenario_files:
                await self.k8s.delete_chaos_scenario(scenario_file)
            
            # Wait for system to stabilize
            await asyncio.sleep(30)
            
            # Collect test metrics
            test_metrics = await self._collect_test_metrics(test_start, test_end)
            logger.info(f"Collected test metrics: {len(test_metrics)} metrics")
            
            # Evaluate success criteria
            passed, messages = self._evaluate_success_criteria(test_metrics, scenario.success_criteria)
            
            duration = time.time() - start_time
            message = f"Scenario completed. {len(messages)} criteria evaluated."
            
            return ChaosResult(
                scenario_name=scenario.name,
                passed=passed,
                duration=duration,
                message=message,
                metrics=test_metrics,
                details={
                    "baseline_metrics": baseline_metrics,
                    "evaluation_messages": messages,
                    "chaos_scenarios": scenario.scenarios
                }
            )
            
        except Exception as e:
            duration = time.time() - start_time
            logger.error(f"Chaos scenario failed: {str(e)}")
            
            return ChaosResult(
                scenario_name=scenario.name,
                passed=False,
                duration=duration,
                message=f"Scenario failed: {str(e)}",
                metrics={},
                details={"error": str(e)}
            )
    
    async def run_all_scenarios(self) -> List[ChaosResult]:
        """Run all chaos scenarios"""
        logger.info(f"Starting chaos test suite with {len(self.scenarios)} scenarios")
        
        for scenario in self.scenarios:
            result = await self.run_chaos_scenario(scenario)
            self.results.append(result)
            
            status = "✅ PASS" if result.passed else "❌ FAIL"
            logger.info(f"{status} {scenario.name}: {result.message} ({result.duration:.2f}s)")
            
            # Wait between scenarios to allow system recovery
            if scenario != self.scenarios[-1]:  # Not the last scenario
                logger.info("Waiting 60s for system recovery...")
                await asyncio.sleep(60)
        
        return self.results
    
    def generate_report(self) -> Dict:
        """Generate chaos test report"""
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        success_rate = (passed / total * 100) if total > 0 else 0
        
        return {
            "chaos_test_report": {
                "timestamp": datetime.utcnow().isoformat(),
                "summary": {
                    "total_scenarios": total,
                    "passed": passed,
                    "failed": total - passed,
                    "success_rate": f"{success_rate:.1f}%"
                },
                "scenarios": [
                    {
                        "name": r.scenario_name,
                        "passed": r.passed,
                        "duration": f"{r.duration:.2f}s",
                        "message": r.message,
                        "metrics": r.metrics,
                        "details": r.details
                    }
                    for r in self.results
                ]
            }
        }

async def main():
    """Main execution function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='AtlasMesh Fleet OS Chaos Engineering Test Runner')
    parser.add_argument('--config', default='testing/chaos/config.yaml',
                       help='Chaos test configuration file')
    parser.add_argument('--prometheus-url', default='http://prometheus:9090',
                       help='Prometheus server URL')
    parser.add_argument('--namespace', default='fleet-os-staging',
                       help='Kubernetes namespace for chaos tests')
    parser.add_argument('--output', help='Output file for test report')
    parser.add_argument('--scenario', help='Run specific scenario only')
    
    args = parser.parse_args()
    
    # Initialize chaos test runner
    runner = ChaosTestRunner(args.config, args.prometheus_url, args.namespace)
    
    try:
        # Load configuration
        runner.load_config()
        
        # Filter scenarios if specific scenario requested
        if args.scenario:
            runner.scenarios = [s for s in runner.scenarios if s.name == args.scenario]
            if not runner.scenarios:
                logger.error(f"Scenario '{args.scenario}' not found")
                sys.exit(1)
        
        # Run chaos tests
        results = await runner.run_all_scenarios()
        report = runner.generate_report()
        
        # Output report
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(report, f, indent=2)
            logger.info(f"Report saved to {args.output}")
        else:
            print(json.dumps(report, indent=2))
        
        # Exit with appropriate code
        failed_tests = len([r for r in results if not r.passed])
        if failed_tests > 0:
            logger.error(f"❌ {failed_tests} chaos scenarios failed")
            sys.exit(1)
        else:
            logger.info("✅ All chaos scenarios passed")
            sys.exit(0)
            
    except Exception as e:
        logger.error(f"Chaos test execution failed: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    asyncio.run(main())
