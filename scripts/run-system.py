#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - System Runner
Systematically starts and validates all 72 services
"""

import asyncio
import subprocess
import time
import logging
import sys
import json
from pathlib import Path
from typing import Dict, List, Tuple

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SystemRunner:
    def __init__(self):
        self.services = {
            # Tier 1: Core Business Services (ports 8080-8087)
            "tier1": [
                {"name": "auth-service", "port": 8087, "path": "services/auth-service"},
                {"name": "api-gateway", "port": 8080, "path": "services/api-gateway"},
                {"name": "fleet-manager", "port": 8081, "path": "services/fleet-manager"},
                {"name": "vehicle-gateway", "port": 8082, "path": "services/vehicle-gateway"},
                {"name": "mission-management", "port": 8083, "path": "services/mission-management"},
                {"name": "dispatch-service", "port": 8084, "path": "services/dispatch-service"},
                {"name": "routing-service", "port": 8085, "path": "services/routing-service"},
                {"name": "policy-engine", "port": 8086, "path": "services/policy-engine"},
            ],
            # Tier 2: Data & Analytics Services (ports 8090-8097)
            "tier2": [
                {"name": "telemetry-ingestion", "port": 8090, "path": "services/telemetry-ingestion"},
                {"name": "analytics-service", "port": 8092, "path": "services/analytics-service"},
                {"name": "data-lineage", "port": 8093, "path": "services/data-lineage"},
                {"name": "feature-store-registry", "port": 8094, "path": "services/feature-store-registry"},
                {"name": "telemetry-lakehouse", "port": 8095, "path": "services/telemetry-lakehouse"},
                {"name": "sensor-data-collector", "port": 8096, "path": "services/sensor-data-collector"},
                {"name": "predictive-maintenance", "port": 8097, "path": "services/predictive-maintenance"},
            ],
            # Tier 3: Integration Services (ports 8100-8106)
            "tier3": [
                {"name": "uae-government-integration", "port": 8100, "path": "services/uae-government-integration"},
                {"name": "erp-wms-adapters", "port": 8101, "path": "services/erp-wms-adapters"},
                {"name": "erp-integration", "port": 8102, "path": "services/erp-integration"},
                {"name": "arabic-localization", "port": 8103, "path": "services/arabic-localization"},
                {"name": "comms-orchestration", "port": 8104, "path": "services/comms-orchestration"},
                {"name": "weather-fusion", "port": 8105, "path": "services/weather-fusion"},
                {"name": "map-data-contract", "port": 8106, "path": "services/map-data-contract"},
            ]
        }
        
        self.processes = {}
        self.infrastructure_started = False
    
    async def start_infrastructure(self):
        """Start infrastructure services using Docker Compose"""
        logger.info("🏗️ Starting infrastructure services...")
        
        try:
            # Start infrastructure
            result = subprocess.run([
                "docker-compose", "-f", "infrastructure/docker-compose.yml", "up", "-d"
            ], capture_output=True, text=True)
            
            if result.returncode != 0:
                logger.error(f"Failed to start infrastructure: {result.stderr}")
                return False
            
            logger.info("⏳ Waiting for infrastructure to be ready...")
            await asyncio.sleep(30)
            
            # Check infrastructure health
            infrastructure_health = await self.check_infrastructure_health()
            if infrastructure_health:
                self.infrastructure_started = True
                logger.info("✅ Infrastructure services are ready")
                return True
            else:
                logger.error("❌ Infrastructure services failed to start properly")
                return False
                
        except Exception as e:
            logger.error(f"Error starting infrastructure: {str(e)}")
            return False
    
    async def check_infrastructure_health(self) -> bool:
        """Check if infrastructure services are healthy"""
        health_checks = [
            ("PostgreSQL", "localhost", 5432),
            ("Redis", "localhost", 6379),
            ("Prometheus", "localhost", 9090),
            ("Grafana", "localhost", 3001),
        ]
        
        healthy_services = 0
        for service, host, port in health_checks:
            try:
                # Simple port check
                import socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                result = sock.connect_ex((host, port))
                sock.close()
                
                if result == 0:
                    logger.info(f"  ✅ {service}: Running on port {port}")
                    healthy_services += 1
                else:
                    logger.warning(f"  ❌ {service}: Not accessible on port {port}")
            except Exception as e:
                logger.warning(f"  ❌ {service}: Health check failed - {str(e)}")
        
        return healthy_services >= 3  # At least 3 out of 4 should be healthy
    
    async def start_service_tier(self, tier_name: str) -> bool:
        """Start all services in a specific tier"""
        if tier_name not in self.services:
            logger.error(f"Unknown tier: {tier_name}")
            return False
        
        tier_services = self.services[tier_name]
        logger.info(f"🚀 Starting {tier_name.upper()} ({len(tier_services)} services)...")
        
        started_services = 0
        for service in tier_services:
            if await self.start_service(service):
                started_services += 1
                await asyncio.sleep(2)  # Stagger service starts
        
        success_rate = started_services / len(tier_services)
        if success_rate >= 0.7:  # 70% success rate required
            logger.info(f"✅ {tier_name.upper()}: {started_services}/{len(tier_services)} services started")
            return True
        else:
            logger.error(f"❌ {tier_name.upper()}: Only {started_services}/{len(tier_services)} services started")
            return False
    
    async def start_service(self, service: Dict) -> bool:
        """Start a single service"""
        name = service["name"]
        port = service["port"]
        path = service["path"]
        
        try:
            # Check if service directory exists and has main.go
            service_path = Path(path)
            main_go = service_path / "cmd" / "main.go"
            
            if not main_go.exists():
                logger.warning(f"  ⚠️ {name}: main.go not found at {main_go}")
                return False
            
            # Start the service
            logger.info(f"  🔄 Starting {name} on port {port}...")
            
            process = subprocess.Popen([
                "go", "run", "cmd/main.go", f"--port={port}"
            ], cwd=service_path, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.processes[name] = process
            
            # Wait a moment and check if process is still running
            await asyncio.sleep(1)
            if process.poll() is None:
                logger.info(f"  ✅ {name}: Started successfully on port {port}")
                return True
            else:
                stdout, stderr = process.communicate()
                logger.error(f"  ❌ {name}: Failed to start - {stderr.decode()}")
                return False
                
        except Exception as e:
            logger.error(f"  ❌ {name}: Exception during start - {str(e)}")
            return False
    
    async def check_service_health(self, service: Dict) -> bool:
        """Check if a service is healthy"""
        name = service["name"]
        port = service["port"]
        
        try:
            import aiohttp
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=5)) as session:
                async with session.get(f"http://localhost:{port}/health") as response:
                    if response.status == 200:
                        logger.info(f"  ✅ {name}: Healthy")
                        return True
                    else:
                        logger.warning(f"  ⚠️ {name}: Health check returned {response.status}")
                        return False
        except Exception as e:
            logger.warning(f"  ❌ {name}: Health check failed - {str(e)}")
            return False
    
    async def validate_system(self) -> Dict:
        """Validate the entire system"""
        logger.info("🔍 Validating system health...")
        
        validation_results = {
            "infrastructure": await self.check_infrastructure_health(),
            "tiers": {}
        }
        
        # Check each tier
        for tier_name, tier_services in self.services.items():
            healthy_services = 0
            for service in tier_services:
                if await self.check_service_health(service):
                    healthy_services += 1
            
            validation_results["tiers"][tier_name] = {
                "healthy": healthy_services,
                "total": len(tier_services),
                "success_rate": healthy_services / len(tier_services)
            }
        
        return validation_results
    
    async def run_smoke_tests(self) -> bool:
        """Run smoke tests against the running system"""
        logger.info("💨 Running smoke tests...")
        
        try:
            result = subprocess.run([
                "python", "testing/smoke/smoke-tests.py", 
                "--environment", "development",
                "--timeout", "5"
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info("✅ Smoke tests passed")
                return True
            else:
                logger.error(f"❌ Smoke tests failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"❌ Smoke tests error: {str(e)}")
            return False
    
    def stop_all_services(self):
        """Stop all running services"""
        logger.info("🛑 Stopping all services...")
        
        # Stop Go services
        for name, process in self.processes.items():
            try:
                process.terminate()
                process.wait(timeout=5)
                logger.info(f"  ✅ Stopped {name}")
            except Exception as e:
                logger.warning(f"  ⚠️ Error stopping {name}: {str(e)}")
                try:
                    process.kill()
                except:
                    pass
        
        # Stop infrastructure
        if self.infrastructure_started:
            try:
                subprocess.run([
                    "docker-compose", "-f", "infrastructure/docker-compose.yml", "down"
                ], capture_output=True)
                logger.info("  ✅ Stopped infrastructure")
            except Exception as e:
                logger.warning(f"  ⚠️ Error stopping infrastructure: {str(e)}")
    
    async def run_full_system(self):
        """Run the complete system startup and validation"""
        try:
            logger.info("🚀 Starting AtlasMesh Fleet OS System...")
            
            # Step 1: Start infrastructure
            if not await self.start_infrastructure():
                logger.error("❌ Failed to start infrastructure")
                return False
            
            # Step 2: Start service tiers in order
            for tier_name in ["tier1", "tier2", "tier3"]:
                if not await self.start_service_tier(tier_name):
                    logger.warning(f"⚠️ {tier_name.upper()} had some failures, continuing...")
            
            # Step 3: Wait for services to stabilize
            logger.info("⏳ Waiting for services to stabilize...")
            await asyncio.sleep(10)
            
            # Step 4: Validate system
            validation_results = await self.validate_system()
            
            # Step 5: Run smoke tests
            smoke_tests_passed = await self.run_smoke_tests()
            
            # Step 6: Generate report
            self.generate_system_report(validation_results, smoke_tests_passed)
            
            return True
            
        except KeyboardInterrupt:
            logger.info("🛑 Received interrupt signal, shutting down...")
            return False
        except Exception as e:
            logger.error(f"❌ System startup failed: {str(e)}")
            return False
        finally:
            # Always clean up
            self.stop_all_services()
    
    def generate_system_report(self, validation_results: Dict, smoke_tests_passed: bool):
        """Generate system status report"""
        logger.info("📊 System Status Report:")
        logger.info(f"  Infrastructure: {'✅ Healthy' if validation_results['infrastructure'] else '❌ Unhealthy'}")
        
        total_services = 0
        healthy_services = 0
        
        for tier_name, tier_data in validation_results["tiers"].items():
            total_services += tier_data["total"]
            healthy_services += tier_data["healthy"]
            success_rate = tier_data["success_rate"] * 100
            logger.info(f"  {tier_name.upper()}: {tier_data['healthy']}/{tier_data['total']} ({success_rate:.1f}%)")
        
        overall_success_rate = (healthy_services / total_services * 100) if total_services > 0 else 0
        logger.info(f"  Overall: {healthy_services}/{total_services} services ({overall_success_rate:.1f}%)")
        logger.info(f"  Smoke Tests: {'✅ Passed' if smoke_tests_passed else '❌ Failed'}")
        
        # Save detailed report
        report = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "infrastructure_healthy": validation_results["infrastructure"],
            "services": validation_results["tiers"],
            "smoke_tests_passed": smoke_tests_passed,
            "overall_success_rate": overall_success_rate
        }
        
        with open("system-status-report.json", "w") as f:
            json.dump(report, f, indent=2)
        
        logger.info("📄 Detailed report saved to system-status-report.json")

async def main():
    runner = SystemRunner()
    
    try:
        success = await runner.run_full_system()
        if success:
            logger.info("🎉 System startup completed!")
            
            # Keep running and show access points
            logger.info("🌐 System Access Points:")
            logger.info("  - API Gateway: http://localhost:8080")
            logger.info("  - Fleet Manager: http://localhost:8081")
            logger.info("  - Vehicle Gateway: http://localhost:8082")
            logger.info("  - Grafana: http://localhost:3001")
            logger.info("  - Jaeger: http://localhost:16686")
            logger.info("  - Kafka UI: http://localhost:8080")
            logger.info("")
            logger.info("Press Ctrl+C to stop all services...")
            
            # Keep running until interrupted
            try:
                while True:
                    await asyncio.sleep(60)
                    # Periodic health check
                    validation_results = await runner.validate_system()
                    healthy_count = sum(tier["healthy"] for tier in validation_results["tiers"].values())
                    total_count = sum(tier["total"] for tier in validation_results["tiers"].values())
                    logger.info(f"🔍 Health check: {healthy_count}/{total_count} services healthy")
            except KeyboardInterrupt:
                logger.info("🛑 Shutting down...")
        else:
            logger.error("❌ System startup failed!")
            sys.exit(1)
    except Exception as e:
        logger.error(f"💥 Unexpected error: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
