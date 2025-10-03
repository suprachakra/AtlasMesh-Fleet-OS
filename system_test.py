#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - System Integration Test
Comprehensive testing of the mock system and validation of components
"""

import json
import time
import urllib.request
import urllib.error
from typing import Dict, List, Any
import sys

class SystemTester:
    def __init__(self):
        self.api_base = "http://localhost:8080"
        self.ui_base = "http://localhost:3000"
        self.test_results = []
        
    def print_header(self):
        print("🧪 AtlasMesh Fleet OS - System Integration Test")
        print("=" * 60)
        print(f"🎯 Testing API Server: {self.api_base}")
        print(f"🌐 Testing UI Server: {self.ui_base}")
        print(f"⏰ Test Started: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print()

    def test_api_endpoint(self, endpoint: str, expected_keys: List[str] = None) -> Dict[str, Any]:
        """Test a specific API endpoint"""
        test_name = f"API {endpoint}"
        url = f"{self.api_base}{endpoint}"
        
        try:
            print(f"🔍 Testing {test_name}...")
            start_time = time.time()
            
            response = urllib.request.urlopen(url, timeout=10)
            response_time = (time.time() - start_time) * 1000  # Convert to ms
            
            if response.getcode() == 200:
                data = json.loads(response.read().decode())
                
                # Validate expected keys if provided
                validation_passed = True
                if expected_keys:
                    for key in expected_keys:
                        if key not in data:
                            validation_passed = False
                            print(f"  ❌ Missing expected key: {key}")
                
                if validation_passed:
                    print(f"  ✅ {test_name} - OK ({response_time:.1f}ms)")
                    result = {
                        "test": test_name,
                        "status": "PASS",
                        "response_time": response_time,
                        "data": data
                    }
                else:
                    print(f"  ⚠️  {test_name} - Data validation failed")
                    result = {
                        "test": test_name,
                        "status": "PARTIAL",
                        "response_time": response_time,
                        "data": data,
                        "issue": "Data validation failed"
                    }
            else:
                print(f"  ❌ {test_name} - HTTP {response.getcode()}")
                result = {
                    "test": test_name,
                    "status": "FAIL",
                    "issue": f"HTTP {response.getcode()}"
                }
                
        except urllib.error.URLError as e:
            print(f"  ❌ {test_name} - Connection failed: {e}")
            result = {
                "test": test_name,
                "status": "FAIL",
                "issue": f"Connection failed: {e}"
            }
        except Exception as e:
            print(f"  ❌ {test_name} - Error: {e}")
            result = {
                "test": test_name,
                "status": "FAIL",
                "issue": f"Error: {e}"
            }
        
        self.test_results.append(result)
        return result

    def test_ui_server(self) -> Dict[str, Any]:
        """Test the UI server"""
        test_name = "UI Server"
        
        try:
            print(f"🔍 Testing {test_name}...")
            start_time = time.time()
            
            response = urllib.request.urlopen(f"{self.ui_base}/debug_dashboard.html", timeout=10)
            response_time = (time.time() - start_time) * 1000
            
            if response.getcode() == 200:
                content = response.read().decode()
                
                # Basic validation - check for key elements
                required_elements = [
                    "AtlasMesh Fleet OS",
                    "System Status",
                    "Fleet Overview",
                    "Vehicle Status",
                    "API Testing"
                ]
                
                validation_passed = True
                for element in required_elements:
                    if element not in content:
                        validation_passed = False
                        print(f"  ❌ Missing UI element: {element}")
                
                if validation_passed:
                    print(f"  ✅ {test_name} - OK ({response_time:.1f}ms)")
                    result = {
                        "test": test_name,
                        "status": "PASS",
                        "response_time": response_time,
                        "content_length": len(content)
                    }
                else:
                    print(f"  ⚠️  {test_name} - UI validation failed")
                    result = {
                        "test": test_name,
                        "status": "PARTIAL",
                        "response_time": response_time,
                        "issue": "UI validation failed"
                    }
            else:
                print(f"  ❌ {test_name} - HTTP {response.getcode()}")
                result = {
                    "test": test_name,
                    "status": "FAIL",
                    "issue": f"HTTP {response.getcode()}"
                }
                
        except Exception as e:
            print(f"  ❌ {test_name} - Error: {e}")
            result = {
                "test": test_name,
                "status": "FAIL",
                "issue": f"Error: {e}"
            }
        
        self.test_results.append(result)
        return result

    def test_data_consistency(self) -> Dict[str, Any]:
        """Test data consistency across endpoints"""
        test_name = "Data Consistency"
        print(f"🔍 Testing {test_name}...")
        
        try:
            # Get fleet data
            fleet_response = urllib.request.urlopen(f"{self.api_base}/api/v1/fleets")
            fleet_data = json.loads(fleet_response.read().decode())
            
            # Get vehicle data
            vehicle_response = urllib.request.urlopen(f"{self.api_base}/api/v1/vehicles")
            vehicle_data = json.loads(vehicle_response.read().decode())
            
            # Validate consistency
            issues = []
            
            if fleet_data.get("fleets") and vehicle_data.get("vehicles"):
                fleet = fleet_data["fleets"][0]
                vehicle = vehicle_data["vehicles"][0]
                
                # Check if vehicle location matches fleet location (approximately)
                fleet_lat = fleet["location"]["latitude"]
                fleet_lng = fleet["location"]["longitude"]
                vehicle_lat = vehicle["location"]["latitude"]
                vehicle_lng = vehicle["location"]["longitude"]
                
                # Allow for small differences (within 0.1 degrees)
                if abs(fleet_lat - vehicle_lat) > 0.1 or abs(fleet_lng - vehicle_lng) > 0.1:
                    issues.append("Vehicle location doesn't match fleet location")
                
                # Check if vehicle status is consistent
                if vehicle["status"] != "active":
                    issues.append("Vehicle status is not active")
            
            if not issues:
                print(f"  ✅ {test_name} - OK")
                result = {
                    "test": test_name,
                    "status": "PASS"
                }
            else:
                print(f"  ⚠️  {test_name} - Issues found: {', '.join(issues)}")
                result = {
                    "test": test_name,
                    "status": "PARTIAL",
                    "issues": issues
                }
                
        except Exception as e:
            print(f"  ❌ {test_name} - Error: {e}")
            result = {
                "test": test_name,
                "status": "FAIL",
                "issue": f"Error: {e}"
            }
        
        self.test_results.append(result)
        return result

    def test_performance(self) -> Dict[str, Any]:
        """Test API performance with multiple requests"""
        test_name = "Performance Test"
        print(f"🔍 Testing {test_name}...")
        
        try:
            endpoints = ["/health", "/api/v1/fleets", "/api/v1/vehicles"]
            response_times = []
            
            for _ in range(5):  # 5 iterations
                for endpoint in endpoints:
                    start_time = time.time()
                    response = urllib.request.urlopen(f"{self.api_base}{endpoint}")
                    response_time = (time.time() - start_time) * 1000
                    response_times.append(response_time)
            
            avg_response_time = sum(response_times) / len(response_times)
            max_response_time = max(response_times)
            min_response_time = min(response_times)
            
            # Performance thresholds
            if avg_response_time < 100:  # 100ms average
                print(f"  ✅ {test_name} - Excellent (avg: {avg_response_time:.1f}ms)")
                status = "PASS"
            elif avg_response_time < 500:  # 500ms average
                print(f"  ⚠️  {test_name} - Acceptable (avg: {avg_response_time:.1f}ms)")
                status = "PARTIAL"
            else:
                print(f"  ❌ {test_name} - Slow (avg: {avg_response_time:.1f}ms)")
                status = "FAIL"
            
            result = {
                "test": test_name,
                "status": status,
                "avg_response_time": avg_response_time,
                "max_response_time": max_response_time,
                "min_response_time": min_response_time,
                "total_requests": len(response_times)
            }
                
        except Exception as e:
            print(f"  ❌ {test_name} - Error: {e}")
            result = {
                "test": test_name,
                "status": "FAIL",
                "issue": f"Error: {e}"
            }
        
        self.test_results.append(result)
        return result

    def run_comprehensive_test(self):
        """Run all tests"""
        self.print_header()
        
        # Test API endpoints
        print("🔧 Testing API Endpoints...")
        self.test_api_endpoint("/health", ["status", "timestamp", "service"])
        self.test_api_endpoint("/api/v1/fleets", ["fleets"])
        self.test_api_endpoint("/api/v1/vehicles", ["vehicles"])
        
        print()
        
        # Test UI server
        print("🌐 Testing UI Server...")
        self.test_ui_server()
        
        print()
        
        # Test data consistency
        print("🔍 Testing Data Consistency...")
        self.test_data_consistency()
        
        print()
        
        # Test performance
        print("⚡ Testing Performance...")
        self.test_performance()
        
        print()
        
        # Generate summary
        self.generate_summary()

    def generate_summary(self):
        """Generate test summary"""
        print("📊 Test Summary")
        print("-" * 40)
        
        total_tests = len(self.test_results)
        passed_tests = len([r for r in self.test_results if r["status"] == "PASS"])
        partial_tests = len([r for r in self.test_results if r["status"] == "PARTIAL"])
        failed_tests = len([r for r in self.test_results if r["status"] == "FAIL"])
        
        print(f"Total Tests: {total_tests}")
        print(f"✅ Passed: {passed_tests}")
        print(f"⚠️  Partial: {partial_tests}")
        print(f"❌ Failed: {failed_tests}")
        
        success_rate = (passed_tests + partial_tests * 0.5) / total_tests * 100
        print(f"Success Rate: {success_rate:.1f}%")
        
        print()
        
        # Detailed results
        print("📋 Detailed Results:")
        for result in self.test_results:
            status_icon = {
                "PASS": "✅",
                "PARTIAL": "⚠️ ",
                "FAIL": "❌"
            }.get(result["status"], "❓")
            
            print(f"  {status_icon} {result['test']}")
            
            if "response_time" in result:
                print(f"    Response Time: {result['response_time']:.1f}ms")
            
            if "issue" in result:
                print(f"    Issue: {result['issue']}")
            
            if "issues" in result:
                print(f"    Issues: {', '.join(result['issues'])}")
        
        print()
        
        # Recommendations
        print("💡 Recommendations:")
        if failed_tests > 0:
            print("  • Fix failed tests before proceeding to production")
        if partial_tests > 0:
            print("  • Review partial test results and address issues")
        if success_rate >= 80:
            print("  • System is ready for further development")
            print("  • Consider adding more comprehensive tests")
        else:
            print("  • System needs significant improvements")
        
        print()
        print("🎯 Next Steps:")
        print("  1. Open http://localhost:3000/debug_dashboard.html in your browser")
        print("  2. Test the interactive dashboard features")
        print("  3. Install Node.js and Docker for full development environment")
        print("  4. Run the actual Go services when dependencies are available")

def main():
    tester = SystemTester()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "api":
            tester.print_header()
            print("🔧 Testing API Endpoints Only...")
            tester.test_api_endpoint("/health", ["status", "timestamp", "service"])
            tester.test_api_endpoint("/api/v1/fleets", ["fleets"])
            tester.test_api_endpoint("/api/v1/vehicles", ["vehicles"])
            
        elif command == "ui":
            tester.print_header()
            print("🌐 Testing UI Server Only...")
            tester.test_ui_server()
            
        elif command == "performance":
            tester.print_header()
            print("⚡ Testing Performance Only...")
            tester.test_performance()
            
        else:
            print(f"❌ Unknown command: {command}")
            print("Available commands: api, ui, performance")
    else:
        tester.run_comprehensive_test()

if __name__ == "__main__":
    main()
