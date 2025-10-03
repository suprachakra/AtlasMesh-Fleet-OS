#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Debug and Health Check Tool
Provides debugging capabilities without requiring Docker or Node.js
"""

import os
import sys
import json
import time
import subprocess
from pathlib import Path
from typing import Dict, List, Any, Optional
import http.server
import socketserver
from urllib.parse import urlparse, parse_qs
import threading

class FleetOSDebugger:
    def __init__(self):
        self.project_root = Path.cwd()
        self.services_dir = self.project_root / "services"
        self.ui_dir = self.project_root / "ui"
        self.docs_dir = self.project_root / "docs"
        
    def print_header(self):
        print("🚀 AtlasMesh Fleet OS - Debug Tool")
        print("=" * 50)
        print(f"📁 Project Root: {self.project_root}")
        print(f"⏰ Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print()

    def check_project_structure(self) -> Dict[str, Any]:
        """Check the project structure and return status"""
        print("📋 Checking Project Structure...")
        
        structure = {
            "services": {"path": self.services_dir, "exists": False, "count": 0},
            "ui": {"path": self.ui_dir, "exists": False, "count": 0},
            "docs": {"path": self.docs_dir, "exists": False, "count": 0},
            "database": {"path": self.project_root / "database", "exists": False, "count": 0},
            "scripts": {"path": self.project_root / "scripts", "exists": False, "count": 0}
        }
        
        for name, info in structure.items():
            if info["path"].exists():
                info["exists"] = True
                info["count"] = len(list(info["path"].iterdir()))
                print(f"  ✅ {name}/ ({info['count']} items)")
            else:
                print(f"  ❌ {name}/ missing")
        
        return structure

    def list_services(self) -> List[str]:
        """List all available services"""
        print("\n🔧 Available Services:")
        
        if not self.services_dir.exists():
            print("  ❌ Services directory not found")
            return []
        
        services = []
        for service_dir in self.services_dir.iterdir():
            if service_dir.is_dir():
                services.append(service_dir.name)
                
                # Check for main.go or other entry points
                main_go = service_dir / "cmd" / "main.go"
                dockerfile = service_dir / "Dockerfile"
                
                status_indicators = []
                if main_go.exists():
                    status_indicators.append("🐹 Go")
                if dockerfile.exists():
                    status_indicators.append("🐳 Docker")
                
                status = " ".join(status_indicators) if status_indicators else "📄 Config"
                print(f"  • {service_dir.name:<30} {status}")
        
        print(f"\n  📊 Total Services: {len(services)}")
        return services

    def check_service_health(self, service_name: str) -> Dict[str, Any]:
        """Check health of a specific service"""
        service_path = self.services_dir / service_name
        
        health = {
            "name": service_name,
            "exists": service_path.exists(),
            "has_main": False,
            "has_dockerfile": False,
            "has_config": False,
            "dependencies": []
        }
        
        if health["exists"]:
            # Check for Go main file
            main_go = service_path / "cmd" / "main.go"
            if main_go.exists():
                health["has_main"] = True
                
            # Check for Dockerfile
            dockerfile = service_path / "Dockerfile"
            if dockerfile.exists():
                health["has_dockerfile"] = True
                
            # Check for go.mod
            go_mod = service_path / "go.mod"
            if go_mod.exists():
                health["has_config"] = True
                try:
                    with open(go_mod, 'r') as f:
                        content = f.read()
                        # Extract dependencies (simplified)
                        lines = content.split('\n')
                        in_require = False
                        for line in lines:
                            line = line.strip()
                            if line.startswith('require'):
                                in_require = True
                                continue
                            if in_require and line.startswith(')'):
                                break
                            if in_require and line and not line.startswith('//'):
                                health["dependencies"].append(line.split()[0])
                except Exception as e:
                    print(f"    ⚠️  Could not parse go.mod: {e}")
        
        return health

    def analyze_configuration(self) -> Dict[str, Any]:
        """Analyze project configuration files"""
        print("\n⚙️  Analyzing Configuration...")
        
        config_files = {
            "docker-compose.yml": self.project_root / "docker-compose.yml",
            "package.json": self.project_root / "package.json",
            "Makefile": self.project_root / "Makefile",
            "README.md": self.project_root / "README.md"
        }
        
        config_status = {}
        
        for name, path in config_files.items():
            if path.exists():
                config_status[name] = {
                    "exists": True,
                    "size": path.stat().st_size,
                    "modified": time.ctime(path.stat().st_mtime)
                }
                print(f"  ✅ {name} ({config_status[name]['size']} bytes)")
                
                # Parse specific files
                if name == "package.json":
                    try:
                        with open(path, 'r') as f:
                            pkg_data = json.load(f)
                            if "scripts" in pkg_data:
                                print(f"    📜 Scripts: {', '.join(pkg_data['scripts'].keys())}")
                    except Exception as e:
                        print(f"    ⚠️  Could not parse package.json: {e}")
                        
            else:
                config_status[name] = {"exists": False}
                print(f"  ❌ {name} missing")
        
        return config_status

    def create_mock_api_server(self, port: int = 8080):
        """Create a mock API server for testing"""
        print(f"\n🌐 Starting Mock API Server on port {port}...")
        
        class MockAPIHandler(http.server.SimpleHTTPRequestHandler):
            def do_GET(self):
                parsed_path = urlparse(self.path)
                
                # Mock API responses
                if parsed_path.path == "/health":
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    response = {
                        "status": "healthy",
                        "timestamp": time.time(),
                        "service": "mock-api",
                        "version": "1.0.0"
                    }
                    self.wfile.write(json.dumps(response).encode())
                    
                elif parsed_path.path.startswith("/api/v1/fleets"):
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    response = {
                        "fleets": [
                            {
                                "fleet_id": "fleet-001",
                                "name": "Abu Dhabi Downtown Fleet",
                                "status": "active",
                                "total_vehicles": 125,
                                "active_vehicles": 89,
                                "location": {
                                    "latitude": 24.4539,
                                    "longitude": 54.3773,
                                    "address": "Abu Dhabi, UAE"
                                }
                            }
                        ]
                    }
                    self.wfile.write(json.dumps(response).encode())
                    
                elif parsed_path.path.startswith("/api/v1/vehicles"):
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    response = {
                        "vehicles": [
                            {
                                "vehicle_id": "AV-001",
                                "status": "active",
                                "location": {
                                    "latitude": 24.4539,
                                    "longitude": 54.3773,
                                    "speed": 65.5
                                },
                                "battery": {"level": 78.5, "range": 410.2},
                                "health_score": 92
                            }
                        ]
                    }
                    self.wfile.write(json.dumps(response).encode())
                    
                else:
                    # Serve static files or 404
                    super().do_GET()
            
            def do_OPTIONS(self):
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
                self.end_headers()
            
            def log_message(self, format, *args):
                print(f"🌐 {self.address_string()} - {format % args}")
        
        try:
            with socketserver.TCPServer(("", port), MockAPIHandler) as httpd:
                print(f"  ✅ Mock API Server running at http://localhost:{port}")
                print(f"  📡 Health endpoint: http://localhost:{port}/health")
                print(f"  🚗 Fleet API: http://localhost:{port}/api/v1/fleets")
                print(f"  🚙 Vehicle API: http://localhost:{port}/api/v1/vehicles")
                print(f"  🛑 Press Ctrl+C to stop")
                httpd.serve_forever()
        except KeyboardInterrupt:
            print(f"\n  🛑 Mock API Server stopped")
        except Exception as e:
            print(f"  ❌ Failed to start server: {e}")

    def run_diagnostics(self):
        """Run comprehensive diagnostics"""
        self.print_header()
        
        # Check project structure
        structure = self.check_project_structure()
        
        # List services
        services = self.list_services()
        
        # Analyze configuration
        config = self.analyze_configuration()
        
        # Service health checks
        if services:
            print(f"\n🔍 Service Health Check (showing first 5):")
            for service in services[:5]:
                health = self.check_service_health(service)
                status_icons = []
                if health["has_main"]:
                    status_icons.append("🐹")
                if health["has_dockerfile"]:
                    status_icons.append("🐳")
                if health["has_config"]:
                    status_icons.append("⚙️")
                
                status = "".join(status_icons) if status_icons else "❓"
                print(f"  {status} {service}")
                
                if health["dependencies"]:
                    deps = health["dependencies"][:3]  # Show first 3 dependencies
                    print(f"    📦 Dependencies: {', '.join(deps)}")
        
        # Summary
        print(f"\n📊 Summary:")
        print(f"  • Services: {len(services)}")
        print(f"  • UI Components: {len(list(self.ui_dir.iterdir())) if self.ui_dir.exists() else 0}")
        print(f"  • Documentation: {len(list(self.docs_dir.iterdir())) if self.docs_dir.exists() else 0}")
        
        return {
            "structure": structure,
            "services": services,
            "config": config
        }

    def interactive_menu(self):
        """Interactive debugging menu"""
        while True:
            print(f"\n🎛️  AtlasMesh Fleet OS - Debug Menu")
            print("1. Run Full Diagnostics")
            print("2. List All Services")
            print("3. Check Specific Service")
            print("4. Start Mock API Server")
            print("5. View Project Structure")
            print("6. Exit")
            
            try:
                choice = input("\nSelect option (1-6): ").strip()
                
                if choice == "1":
                    self.run_diagnostics()
                    
                elif choice == "2":
                    self.list_services()
                    
                elif choice == "3":
                    service_name = input("Enter service name: ").strip()
                    if service_name:
                        health = self.check_service_health(service_name)
                        print(f"\n🔍 Service: {service_name}")
                        print(f"  Exists: {'✅' if health['exists'] else '❌'}")
                        print(f"  Has Main: {'✅' if health['has_main'] else '❌'}")
                        print(f"  Has Dockerfile: {'✅' if health['has_dockerfile'] else '❌'}")
                        print(f"  Has Config: {'✅' if health['has_config'] else '❌'}")
                        if health['dependencies']:
                            print(f"  Dependencies: {', '.join(health['dependencies'][:5])}")
                    
                elif choice == "4":
                    port = input("Enter port (default 8080): ").strip()
                    port = int(port) if port.isdigit() else 8080
                    self.create_mock_api_server(port)
                    
                elif choice == "5":
                    self.check_project_structure()
                    
                elif choice == "6":
                    print("👋 Goodbye!")
                    break
                    
                else:
                    print("❌ Invalid option. Please select 1-6.")
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def main():
    debugger = FleetOSDebugger()
    
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        
        if command == "diagnostics":
            debugger.run_diagnostics()
        elif command == "services":
            debugger.list_services()
        elif command == "server":
            port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
            debugger.create_mock_api_server(port)
        elif command == "structure":
            debugger.check_project_structure()
        else:
            print(f"❌ Unknown command: {command}")
            print("Available commands: diagnostics, services, server, structure")
    else:
        debugger.interactive_menu()

if __name__ == "__main__":
    main()
