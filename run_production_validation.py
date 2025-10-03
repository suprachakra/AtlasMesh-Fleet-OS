#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Production Validation Suite

This script demonstrates the complete production-ready operational framework
by running all validation tools in sequence.
"""

import asyncio
import subprocess
import sys
import time
import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class ProductionValidationSuite:
    """Complete production validation orchestrator"""
    
    def __init__(self, environment: str = "development"):
        self.environment = environment
        self.start_time = datetime.utcnow()
        self.results = {}
        
        # Environment-specific configurations
        self.configs = {
            "development": {
                "api_url": "http://localhost:8080",
                "ui_url": "http://localhost:3000",
                "prometheus_url": "http://localhost:9090",
                "namespace": "fleet-os-dev"
            },
            "staging": {
                "api_url": "https://api-staging.atlasmesh.ae",
                "ui_url": "https://fleet-staging.atlasmesh.ae", 
                "prometheus_url": "https://prometheus-staging.atlasmesh.ae",
                "namespace": "fleet-os-staging"
            },
            "production": {
                "api_url": "https://api.atlasmesh.ae",
                "ui_url": "https://fleet.atlasmesh.ae",
                "prometheus_url": "https://prometheus.atlasmesh.ae",
                "namespace": "fleet-os-production"
            }
        }
        
        self.config = self.configs.get(environment, self.configs["development"])
    
    def print_banner(self):
        """Print validation suite banner"""
        print("=" * 80)
        print("🚀 AtlasMesh Fleet OS - Production Validation Suite")
        print("=" * 80)
        print(f"Environment: {self.environment.upper()}")
        print(f"Started: {self.start_time.strftime('%Y-%m-%d %H:%M:%S UTC')}")
        print(f"API URL: {self.config['api_url']}")
        print(f"UI URL: {self.config['ui_url']}")
        print("=" * 80)
        print()
    
    async def run_command(self, cmd: List[str], description: str, timeout: int = 300) -> Tuple[bool, str, float]:
        """Run command with timeout and capture output"""
        logger.info(f"🔄 {description}")
        start_time = time.time()
        
        try:
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            stdout, stderr = await asyncio.wait_for(
                process.communicate(), 
                timeout=timeout
            )
            
            duration = time.time() - start_time
            
            if process.returncode == 0:
                logger.info(f"✅ {description} - Completed in {duration:.2f}s")
                return True, stdout.decode(), duration
            else:
                logger.error(f"❌ {description} - Failed in {duration:.2f}s")
                logger.error(f"Error: {stderr.decode()}")
                return False, stderr.decode(), duration
                
        except asyncio.TimeoutError:
            duration = time.time() - start_time
            logger.error(f"⏰ {description} - Timed out after {duration:.2f}s")
            return False, "Command timed out", duration
        except Exception as e:
            duration = time.time() - start_time
            logger.error(f"💥 {description} - Exception: {str(e)}")
            return False, str(e), duration
    
    async def validate_environment(self) -> bool:
        """Validate environment prerequisites"""
        logger.info("🔍 Validating environment prerequisites...")
        
        # Check if required tools are available
        tools = ["python3", "kubectl", "helm"]
        
        for tool in tools:
            success, output, duration = await self.run_command(
                ["which", tool], 
                f"Check {tool} availability",
                timeout=10
            )
            
            if not success:
                logger.error(f"❌ {tool} not found in PATH")
                return False
        
        logger.info("✅ Environment prerequisites validated")
        return True
    
    async def run_smoke_tests(self) -> bool:
        """Run smoke test suite"""
        logger.info("💨 Running smoke test suite...")
        
        cmd = [
            "python3", "testing/smoke/smoke-tests.py",
            "--environment", self.environment,
            "--api-url", self.config["api_url"],
            "--ui-url", self.config["ui_url"],
            "--output", f"smoke-test-results-{self.environment}.json"
        ]
        
        success, output, duration = await self.run_command(
            cmd, 
            "Smoke Test Suite",
            timeout=600  # 10 minutes
        )
        
        self.results["smoke_tests"] = {
            "success": success,
            "duration": duration,
            "output_file": f"smoke-test-results-{self.environment}.json"
        }
        
        return success
    
    async def run_e2e_validation(self) -> bool:
        """Run E2E validation suite"""
        logger.info("🧪 Running E2E validation suite...")
        
        cmd = [
            "python3", "testing/e2e/vertical-slices.py",
            "--environment", self.environment,
            "--api-url", self.config["api_url"],
            "--ui-url", self.config["ui_url"],
            "--output", f"e2e-validation-results-{self.environment}.json"
        ]
        
        success, output, duration = await self.run_command(
            cmd,
            "E2E Validation Suite", 
            timeout=1800  # 30 minutes
        )
        
        self.results["e2e_validation"] = {
            "success": success,
            "duration": duration,
            "output_file": f"e2e-validation-results-{self.environment}.json"
        }
        
        return success
    
    async def run_chaos_tests(self) -> bool:
        """Run chaos engineering tests"""
        if self.environment == "production":
            logger.warning("⚠️ Skipping chaos tests in production environment")
            return True
            
        logger.info("💥 Running chaos engineering tests...")
        
        cmd = [
            "python3", "testing/chaos/chaos-runner.py",
            "--config", "testing/chaos/config.yaml",
            "--prometheus-url", self.config["prometheus_url"],
            "--namespace", self.config["namespace"],
            "--output", f"chaos-test-results-{self.environment}.json"
        ]
        
        success, output, duration = await self.run_command(
            cmd,
            "Chaos Engineering Tests",
            timeout=3600  # 1 hour
        )
        
        self.results["chaos_tests"] = {
            "success": success,
            "duration": duration,
            "output_file": f"chaos-test-results-{self.environment}.json",
            "skipped": self.environment == "production"
        }
        
        return success
    
    async def validate_deployment(self) -> bool:
        """Validate Kubernetes deployment"""
        logger.info("🚢 Validating Kubernetes deployment...")
        
        # Check if namespace exists
        success, output, duration = await self.run_command(
            ["kubectl", "get", "namespace", self.config["namespace"]],
            f"Check namespace {self.config['namespace']}",
            timeout=30
        )
        
        if not success:
            logger.warning(f"⚠️ Namespace {self.config['namespace']} not found")
            return True  # Not critical for validation
        
        # Check pod status
        success, output, duration = await self.run_command(
            ["kubectl", "get", "pods", "-n", self.config["namespace"], "-o", "json"],
            f"Check pod status in {self.config['namespace']}",
            timeout=60
        )
        
        if success:
            try:
                pods_data = json.loads(output)
                pods = pods_data.get("items", [])
                
                running_pods = sum(1 for pod in pods 
                                 if pod.get("status", {}).get("phase") == "Running")
                total_pods = len(pods)
                
                logger.info(f"📊 Pods status: {running_pods}/{total_pods} running")
                
                self.results["deployment_validation"] = {
                    "success": True,
                    "total_pods": total_pods,
                    "running_pods": running_pods,
                    "health_percentage": (running_pods / total_pods * 100) if total_pods > 0 else 0
                }
                
                return running_pods > 0
                
            except json.JSONDecodeError:
                logger.error("❌ Failed to parse kubectl output")
                return False
        
        return success
    
    async def validate_monitoring(self) -> bool:
        """Validate monitoring stack"""
        logger.info("📊 Validating monitoring stack...")
        
        # Simple HTTP check for Prometheus
        cmd = [
            "curl", "-f", "-s", "-m", "10",
            f"{self.config['prometheus_url']}/api/v1/query?query=up"
        ]
        
        success, output, duration = await self.run_command(
            cmd,
            "Prometheus connectivity check",
            timeout=30
        )
        
        self.results["monitoring_validation"] = {
            "success": success,
            "prometheus_accessible": success,
            "duration": duration
        }
        
        return success
    
    async def generate_deployment_report(self) -> Dict:
        """Generate comprehensive deployment report"""
        end_time = datetime.utcnow()
        total_duration = (end_time - self.start_time).total_seconds()
        
        # Count successes and failures
        test_results = [
            self.results.get("smoke_tests", {}).get("success", False),
            self.results.get("e2e_validation", {}).get("success", False),
            self.results.get("chaos_tests", {}).get("success", True),  # Default true if skipped
            self.results.get("deployment_validation", {}).get("success", False),
            self.results.get("monitoring_validation", {}).get("success", False)
        ]
        
        passed_tests = sum(test_results)
        total_tests = len(test_results)
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        # Determine deployment readiness
        critical_tests = [
            self.results.get("smoke_tests", {}).get("success", False),
            self.results.get("deployment_validation", {}).get("success", False)
        ]
        
        deployment_ready = all(critical_tests)
        
        report = {
            "validation_report": {
                "environment": self.environment,
                "start_time": self.start_time.isoformat(),
                "end_time": end_time.isoformat(),
                "total_duration": f"{total_duration:.2f}s",
                "summary": {
                    "deployment_ready": deployment_ready,
                    "total_tests": total_tests,
                    "passed_tests": passed_tests,
                    "failed_tests": total_tests - passed_tests,
                    "success_rate": f"{success_rate:.1f}%"
                },
                "test_results": self.results,
                "recommendations": self._generate_recommendations()
            }
        }
        
        return report
    
    def _generate_recommendations(self) -> List[str]:
        """Generate recommendations based on test results"""
        recommendations = []
        
        if not self.results.get("smoke_tests", {}).get("success", False):
            recommendations.append("❌ CRITICAL: Smoke tests failed - Do not deploy to production")
        
        if not self.results.get("e2e_validation", {}).get("success", False):
            recommendations.append("⚠️ WARNING: E2E validation failed - Review business functionality")
        
        if not self.results.get("deployment_validation", {}).get("success", False):
            recommendations.append("❌ CRITICAL: Deployment validation failed - Check Kubernetes resources")
        
        if not self.results.get("monitoring_validation", {}).get("success", False):
            recommendations.append("⚠️ WARNING: Monitoring validation failed - Limited observability")
        
        if not recommendations:
            recommendations.append("✅ All validations passed - System ready for operation")
        
        return recommendations
    
    async def run_full_validation(self) -> bool:
        """Run complete validation suite"""
        self.print_banner()
        
        try:
            # Step 1: Environment validation
            if not await self.validate_environment():
                logger.error("❌ Environment validation failed")
                return False
            
            # Step 2: Deployment validation
            await self.validate_deployment()
            
            # Step 3: Monitoring validation
            await self.validate_monitoring()
            
            # Step 4: Smoke tests (critical)
            smoke_success = await self.run_smoke_tests()
            
            # Step 5: E2E validation (if smoke tests pass)
            if smoke_success:
                await self.run_e2e_validation()
            else:
                logger.warning("⚠️ Skipping E2E validation due to smoke test failures")
            
            # Step 6: Chaos tests (non-production only)
            if self.environment != "production":
                await self.run_chaos_tests()
            
            # Generate final report
            report = await self.generate_deployment_report()
            
            # Save report
            report_file = f"production-validation-report-{self.environment}.json"
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            
            # Print summary
            self.print_summary(report)
            
            return report["validation_report"]["summary"]["deployment_ready"]
            
        except Exception as e:
            logger.error(f"💥 Validation suite failed: {str(e)}")
            return False
    
    def print_summary(self, report: Dict):
        """Print validation summary"""
        summary = report["validation_report"]["summary"]
        recommendations = report["validation_report"]["recommendations"]
        
        print("\n" + "=" * 80)
        print("📋 VALIDATION SUMMARY")
        print("=" * 80)
        print(f"Environment: {self.environment.upper()}")
        print(f"Duration: {report['validation_report']['total_duration']}")
        print(f"Success Rate: {summary['success_rate']}")
        print(f"Deployment Ready: {'✅ YES' if summary['deployment_ready'] else '❌ NO'}")
        print()
        
        print("📊 TEST RESULTS:")
        for test_name, result in self.results.items():
            status = "✅ PASS" if result.get("success", False) else "❌ FAIL"
            duration = result.get("duration", 0)
            print(f"  {status} {test_name.replace('_', ' ').title()}: {duration:.2f}s")
        
        print("\n💡 RECOMMENDATIONS:")
        for rec in recommendations:
            print(f"  {rec}")
        
        print("\n" + "=" * 80)

async def main():
    """Main execution function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='AtlasMesh Fleet OS Production Validation Suite')
    parser.add_argument('--environment', default='development',
                       choices=['development', 'staging', 'production'],
                       help='Target environment for validation')
    parser.add_argument('--skip-chaos', action='store_true',
                       help='Skip chaos engineering tests')
    
    args = parser.parse_args()
    
    # Initialize validation suite
    suite = ProductionValidationSuite(args.environment)
    
    # Run validation
    success = await suite.run_full_validation()
    
    # Exit with appropriate code
    if success:
        logger.info("🎉 Production validation completed successfully")
        sys.exit(0)
    else:
        logger.error("💥 Production validation failed")
        sys.exit(1)

if __name__ == '__main__':
    asyncio.run(main())
