#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Smoke Test Suite

10-minute validation checklist for post-deployment verification.
These tests validate basic functionality across all system components.
"""

import asyncio
import aiohttp
import json
import time
import logging
import sys
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class SmokeTestResult:
    """Smoke test result data structure"""
    name: str
    passed: bool
    duration: float
    message: str
    critical: bool = False
    details: Optional[Dict] = None

@dataclass
class Environment:
    """Environment configuration"""
    name: str
    api_base_url: str
    ui_base_url: str
    auth_token: Optional[str] = None
    timeout: int = 10  # Shorter timeout for smoke tests

class SmokeTestSuite:
    """Smoke test suite for rapid validation"""
    
    def __init__(self, environment: Environment):
        self.env = environment
        self.session: Optional[aiohttp.ClientSession] = None
        self.results: List[SmokeTestResult] = []
        
    async def __aenter__(self):
        """Async context manager entry"""
        connector = aiohttp.TCPConnector(limit=50, limit_per_host=20)
        timeout = aiohttp.ClientTimeout(total=self.env.timeout)
        
        headers = {
            'Content-Type': 'application/json',
            'User-Agent': 'AtlasMesh-Smoke-Tests/1.0'
        }
        
        if self.env.auth_token:
            headers['Authorization'] = f'Bearer {self.env.auth_token}'
            
        self.session = aiohttp.ClientSession(
            connector=connector,
            timeout=timeout,
            headers=headers
        )
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        if self.session:
            await self.session.close()
    
    async def _make_request(self, method: str, url: str, **kwargs) -> Tuple[int, Dict]:
        """Make HTTP request with error handling"""
        try:
            async with self.session.request(method, url, **kwargs) as response:
                try:
                    data = await response.json()
                except:
                    data = {"text": await response.text()[:500]}  # Limit text for smoke tests
                return response.status, data
        except asyncio.TimeoutError:
            return 408, {"error": "Request timeout"}
        except Exception as e:
            return 0, {"error": str(e)}
    
    # Critical Smoke Tests (Must Pass)
    
    async def test_ui_accessibility(self) -> SmokeTestResult:
        """Test: UI loads and is accessible"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request('GET', self.env.ui_base_url)
            
            if status != 200:
                return SmokeTestResult(
                    "UI Accessibility", False, time.time() - start_time,
                    f"UI not accessible: HTTP {status}", critical=True
                )
            
            # Check if response contains expected UI elements
            text = response.get('text', '')
            if 'AtlasMesh' not in text and 'Fleet' not in text:
                return SmokeTestResult(
                    "UI Accessibility", False, time.time() - start_time,
                    "UI content does not appear to be Fleet OS", critical=True
                )
            
            return SmokeTestResult(
                "UI Accessibility", True, time.time() - start_time,
                "UI is accessible and loading correctly", critical=True
            )
            
        except Exception as e:
            return SmokeTestResult(
                "UI Accessibility", False, time.time() - start_time,
                f"Exception: {str(e)}", critical=True
            )
    
    async def test_api_gateway_health(self) -> SmokeTestResult:
        """Test: API Gateway is healthy and responsive"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request('GET', f'{self.env.api_base_url}/health')
            
            if status != 200:
                return SmokeTestResult(
                    "API Gateway Health", False, time.time() - start_time,
                    f"API Gateway unhealthy: HTTP {status}", critical=True
                )
            
            # Check health response format
            if not isinstance(response, dict) or response.get('status') != 'healthy':
                return SmokeTestResult(
                    "API Gateway Health", False, time.time() - start_time,
                    f"Invalid health response: {response}", critical=True
                )
            
            return SmokeTestResult(
                "API Gateway Health", True, time.time() - start_time,
                "API Gateway is healthy", critical=True
            )
            
        except Exception as e:
            return SmokeTestResult(
                "API Gateway Health", False, time.time() - start_time,
                f"Exception: {str(e)}", critical=True
            )
    
    async def test_core_services_health(self) -> SmokeTestResult:
        """Test: Core services are healthy"""
        start_time = time.time()
        
        core_services = [
            'fleet-manager',
            'vehicle-gateway', 
            'policy-engine',
            'telemetry-ingest'
        ]
        
        try:
            failed_services = []
            
            for service in core_services:
                status, response = await self._make_request(
                    'GET', f'{self.env.api_base_url}/health/{service}'
                )
                
                if status != 200:
                    failed_services.append(f"{service}: HTTP {status}")
            
            if failed_services:
                return SmokeTestResult(
                    "Core Services Health", False, time.time() - start_time,
                    f"Failed services: {', '.join(failed_services)}", critical=True
                )
            
            return SmokeTestResult(
                "Core Services Health", True, time.time() - start_time,
                f"All {len(core_services)} core services are healthy", critical=True
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Core Services Health", False, time.time() - start_time,
                f"Exception: {str(e)}", critical=True
            )
    
    async def test_database_connectivity(self) -> SmokeTestResult:
        """Test: Database is accessible"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request(
                'GET', f'{self.env.api_base_url}/health/database'
            )
            
            if status != 200:
                return SmokeTestResult(
                    "Database Connectivity", False, time.time() - start_time,
                    f"Database health check failed: HTTP {status}", critical=True
                )
            
            if response.get('database') != 'connected':
                return SmokeTestResult(
                    "Database Connectivity", False, time.time() - start_time,
                    f"Database not connected: {response}", critical=True
                )
            
            return SmokeTestResult(
                "Database Connectivity", True, time.time() - start_time,
                "Database is connected and accessible", critical=True
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Database Connectivity", False, time.time() - start_time,
                f"Exception: {str(e)}", critical=True
            )
    
    # Non-Critical Smoke Tests
    
    async def test_fleet_data_access(self) -> SmokeTestResult:
        """Test: Fleet data can be retrieved"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request(
                'GET', f'{self.env.api_base_url}/api/v1/fleets?limit=1'
            )
            
            if status not in [200, 404]:  # 404 is OK if no fleets exist
                return SmokeTestResult(
                    "Fleet Data Access", False, time.time() - start_time,
                    f"Fleet API failed: HTTP {status}"
                )
            
            # Check response structure
            if status == 200 and not isinstance(response, dict):
                return SmokeTestResult(
                    "Fleet Data Access", False, time.time() - start_time,
                    f"Invalid fleet API response format"
                )
            
            return SmokeTestResult(
                "Fleet Data Access", True, time.time() - start_time,
                "Fleet API is accessible and responding correctly"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Fleet Data Access", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_vehicle_data_access(self) -> SmokeTestResult:
        """Test: Vehicle data can be retrieved"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request(
                'GET', f'{self.env.api_base_url}/api/v1/vehicles?limit=1'
            )
            
            if status not in [200, 404]:  # 404 is OK if no vehicles exist
                return SmokeTestResult(
                    "Vehicle Data Access", False, time.time() - start_time,
                    f"Vehicle API failed: HTTP {status}"
                )
            
            return SmokeTestResult(
                "Vehicle Data Access", True, time.time() - start_time,
                "Vehicle API is accessible and responding correctly"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Vehicle Data Access", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_trip_creation(self) -> SmokeTestResult:
        """Test: Trip can be created (basic functionality)"""
        start_time = time.time()
        
        try:
            trip_data = {
                "pickup": {"lat": 24.4539, "lng": 54.3773},
                "destination": {"lat": 25.2048, "lng": 55.2708},
                "passenger_count": 1,
                "trip_type": "smoke_test"
            }
            
            status, response = await self._make_request(
                'POST', f'{self.env.api_base_url}/api/v1/trips', json=trip_data
            )
            
            if status not in [201, 202]:  # Accept both created and accepted
                return SmokeTestResult(
                    "Trip Creation", False, time.time() - start_time,
                    f"Trip creation failed: HTTP {status} - {response}"
                )
            
            trip_id = response.get('trip_id')
            if not trip_id:
                return SmokeTestResult(
                    "Trip Creation", False, time.time() - start_time,
                    "No trip_id in response"
                )
            
            # Clean up: Cancel the test trip
            await self._make_request(
                'PATCH', f'{self.env.api_base_url}/api/v1/trips/{trip_id}',
                json={"status": "cancelled", "reason": "smoke_test_cleanup"}
            )
            
            return SmokeTestResult(
                "Trip Creation", True, time.time() - start_time,
                f"Trip created successfully: {trip_id}"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Trip Creation", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_policy_engine_access(self) -> SmokeTestResult:
        """Test: Policy engine is accessible"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request(
                'GET', f'{self.env.api_base_url}/api/v1/policies/health'
            )
            
            if status != 200:
                return SmokeTestResult(
                    "Policy Engine Access", False, time.time() - start_time,
                    f"Policy engine not accessible: HTTP {status}"
                )
            
            return SmokeTestResult(
                "Policy Engine Access", True, time.time() - start_time,
                "Policy engine is accessible"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Policy Engine Access", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_weather_service_access(self) -> SmokeTestResult:
        """Test: Weather service is accessible"""
        start_time = time.time()
        
        try:
            status, response = await self._make_request(
                'GET', f'{self.env.api_base_url}/api/v1/weather/health'
            )
            
            if status != 200:
                return SmokeTestResult(
                    "Weather Service Access", False, time.time() - start_time,
                    f"Weather service not accessible: HTTP {status}"
                )
            
            return SmokeTestResult(
                "Weather Service Access", True, time.time() - start_time,
                "Weather service is accessible"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Weather Service Access", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_telemetry_ingestion(self) -> SmokeTestResult:
        """Test: Telemetry ingestion is working"""
        start_time = time.time()
        
        try:
            # Send a test telemetry message
            telemetry_data = {
                "vehicle_id": "SMOKE_TEST_VEHICLE",
                "timestamp": datetime.utcnow().isoformat(),
                "location": {"lat": 24.4539, "lng": 54.3773},
                "speed": 0,
                "battery": 100,
                "status": "parked"
            }
            
            status, response = await self._make_request(
                'POST', f'{self.env.api_base_url}/api/v1/telemetry', json=telemetry_data
            )
            
            if status not in [200, 202]:  # Accept both OK and Accepted
                return SmokeTestResult(
                    "Telemetry Ingestion", False, time.time() - start_time,
                    f"Telemetry ingestion failed: HTTP {status}"
                )
            
            return SmokeTestResult(
                "Telemetry Ingestion", True, time.time() - start_time,
                "Telemetry ingestion is working"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Telemetry Ingestion", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def test_authentication_flow(self) -> SmokeTestResult:
        """Test: Authentication system is working"""
        start_time = time.time()
        
        try:
            # Test token validation if we have a token
            if self.env.auth_token:
                status, response = await self._make_request(
                    'GET', f'{self.env.api_base_url}/api/v1/auth/validate'
                )
                
                if status != 200:
                    return SmokeTestResult(
                        "Authentication Flow", False, time.time() - start_time,
                        f"Token validation failed: HTTP {status}"
                    )
            else:
                # Test that protected endpoints require authentication
                status, response = await self._make_request(
                    'GET', f'{self.env.api_base_url}/api/v1/admin/users'
                )
                
                if status not in [401, 403]:  # Should be unauthorized
                    return SmokeTestResult(
                        "Authentication Flow", False, time.time() - start_time,
                        f"Protected endpoint accessible without auth: HTTP {status}"
                    )
            
            return SmokeTestResult(
                "Authentication Flow", True, time.time() - start_time,
                "Authentication system is working correctly"
            )
            
        except Exception as e:
            return SmokeTestResult(
                "Authentication Flow", False, time.time() - start_time,
                f"Exception: {str(e)}"
            )
    
    async def run_all_tests(self) -> List[SmokeTestResult]:
        """Run all smoke tests in parallel for speed"""
        logger.info("Starting smoke test suite (target: <10 minutes)")
        
        # Define all tests
        tests = [
            # Critical tests (must pass)
            ("UI Accessibility", self.test_ui_accessibility),
            ("API Gateway Health", self.test_api_gateway_health),
            ("Core Services Health", self.test_core_services_health),
            ("Database Connectivity", self.test_database_connectivity),
            
            # Non-critical tests
            ("Fleet Data Access", self.test_fleet_data_access),
            ("Vehicle Data Access", self.test_vehicle_data_access),
            ("Trip Creation", self.test_trip_creation),
            ("Policy Engine Access", self.test_policy_engine_access),
            ("Weather Service Access", self.test_weather_service_access),
            ("Telemetry Ingestion", self.test_telemetry_ingestion),
            ("Authentication Flow", self.test_authentication_flow),
        ]
        
        # Run all tests concurrently for speed
        start_time = time.time()
        tasks = [test_func() for _, test_func in tests]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        # Process results
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                test_name = tests[i][0]
                self.results.append(SmokeTestResult(
                    test_name, False, 0, f"Test execution failed: {str(result)}"
                ))
            else:
                self.results.append(result)
        
        total_duration = time.time() - start_time
        logger.info(f"Smoke tests completed in {total_duration:.2f} seconds")
        
        # Log results
        for result in self.results:
            status = "✅ PASS" if result.passed else "❌ FAIL"
            critical = " [CRITICAL]" if result.critical else ""
            logger.info(f"{status}{critical} {result.name}: {result.message} ({result.duration:.2f}s)")
        
        return self.results
    
    def generate_report(self) -> Dict:
        """Generate smoke test report"""
        passed = sum(1 for r in self.results if r.passed)
        total = len(self.results)
        critical_failed = sum(1 for r in self.results if not r.passed and r.critical)
        success_rate = (passed / total * 100) if total > 0 else 0
        
        return {
            "smoke_test_report": {
                "environment": self.env.name,
                "timestamp": datetime.utcnow().isoformat(),
                "summary": {
                    "total_tests": total,
                    "passed": passed,
                    "failed": total - passed,
                    "critical_failures": critical_failed,
                    "success_rate": f"{success_rate:.1f}%",
                    "deployment_ready": critical_failed == 0
                },
                "results": [
                    {
                        "name": r.name,
                        "passed": r.passed,
                        "critical": r.critical,
                        "duration": f"{r.duration:.2f}s",
                        "message": r.message,
                        "details": r.details
                    }
                    for r in self.results
                ]
            }
        }

async def main():
    """Main execution function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='AtlasMesh Fleet OS Smoke Test Suite')
    parser.add_argument('--environment', default='development', 
                       choices=['development', 'staging', 'production'],
                       help='Target environment')
    parser.add_argument('--api-url', help='API base URL (overrides environment default)')
    parser.add_argument('--ui-url', help='UI base URL (overrides environment default)')
    parser.add_argument('--auth-token', help='Authentication token')
    parser.add_argument('--output', help='Output file for test report')
    parser.add_argument('--timeout', type=int, default=10, help='Request timeout in seconds')
    
    args = parser.parse_args()
    
    # Environment configurations
    environments = {
        'development': Environment(
            name='development',
            api_base_url='http://localhost:8080',
            ui_base_url='http://localhost:3000'
        ),
        'staging': Environment(
            name='staging',
            api_base_url='https://api-staging.atlasmesh.ae',
            ui_base_url='https://fleet-staging.atlasmesh.ae'
        ),
        'production': Environment(
            name='production',
            api_base_url='https://api.atlasmesh.ae',
            ui_base_url='https://fleet.atlasmesh.ae'
        )
    }
    
    env = environments[args.environment]
    env.timeout = args.timeout
    
    # Override URLs if provided
    if args.api_url:
        env.api_base_url = args.api_url
    if args.ui_url:
        env.ui_base_url = args.ui_url
    if args.auth_token:
        env.auth_token = args.auth_token
    
    # Run smoke tests
    async with SmokeTestSuite(env) as suite:
        results = await suite.run_all_tests()
        report = suite.generate_report()
        
        # Output report
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(report, f, indent=2)
            logger.info(f"Report saved to {args.output}")
        else:
            print(json.dumps(report, indent=2))
        
        # Exit with appropriate code
        critical_failures = len([r for r in results if not r.passed and r.critical])
        total_failures = len([r for r in results if not r.passed])
        
        if critical_failures > 0:
            logger.error(f"❌ {critical_failures} critical smoke tests failed - DEPLOYMENT NOT READY")
            sys.exit(2)  # Critical failure
        elif total_failures > 0:
            logger.warning(f"⚠️ {total_failures} non-critical smoke tests failed - DEPLOYMENT READY WITH WARNINGS")
            sys.exit(1)  # Non-critical failures
        else:
            logger.info("✅ All smoke tests passed - DEPLOYMENT READY")
            sys.exit(0)

if __name__ == '__main__':
    asyncio.run(main())
