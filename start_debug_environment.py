#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Debug Environment Startup Script
Quick startup script for the debugging environment
"""

import os
import sys
import time
import subprocess
import threading
import webbrowser
from pathlib import Path

def print_banner():
    print("""
🚀 AtlasMesh Fleet OS - Debug Environment
==========================================
Starting mock services and debug dashboard...
""")

def start_api_server():
    """Start the mock API server"""
    print("🌐 Starting Mock API Server on port 8080...")
    try:
        subprocess.run([
            sys.executable, "debug_fleet_os.py", "server", "8080"
        ], cwd=Path.cwd())
    except KeyboardInterrupt:
        print("\n🛑 API Server stopped")

def start_ui_server():
    """Start the UI server"""
    print("📊 Starting UI Server on port 3000...")
    try:
        subprocess.run([
            sys.executable, "-m", "http.server", "3000"
        ], cwd=Path.cwd())
    except KeyboardInterrupt:
        print("\n🛑 UI Server stopped")

def wait_for_servers():
    """Wait for servers to start and open browser"""
    print("⏳ Waiting for servers to start...")
    time.sleep(3)
    
    try:
        import urllib.request
        
        # Test API server
        try:
            urllib.request.urlopen('http://localhost:8080/health', timeout=5)
            print("✅ API Server is ready")
        except:
            print("❌ API Server not responding")
        
        # Test UI server
        try:
            urllib.request.urlopen('http://localhost:3000', timeout=5)
            print("✅ UI Server is ready")
        except:
            print("❌ UI Server not responding")
        
        # Open browser
        print("🌐 Opening debug dashboard in browser...")
        webbrowser.open('http://localhost:3000/debug_dashboard.html')
        
    except Exception as e:
        print(f"⚠️  Could not verify servers: {e}")

def main():
    print_banner()
    
    # Check if files exist
    required_files = ['debug_fleet_os.py', 'debug_dashboard.html']
    for file in required_files:
        if not Path(file).exists():
            print(f"❌ Required file missing: {file}")
            sys.exit(1)
    
    print("📋 Starting services...")
    print("   • Mock API Server: http://localhost:8080")
    print("   • Debug Dashboard: http://localhost:3000/debug_dashboard.html")
    print("   • Press Ctrl+C to stop all services")
    print()
    
    try:
        # Start API server in background thread
        api_thread = threading.Thread(target=start_api_server, daemon=True)
        api_thread.start()
        
        # Wait a moment for API server to start
        time.sleep(2)
        
        # Start UI server in background thread
        ui_thread = threading.Thread(target=start_ui_server, daemon=True)
        ui_thread.start()
        
        # Wait for servers and open browser
        wait_for_servers()
        
        print("\n🎯 Debug Environment Ready!")
        print("   📊 Dashboard: http://localhost:3000/debug_dashboard.html")
        print("   🔧 API Health: http://localhost:8080/health")
        print("   🚗 Fleet API: http://localhost:8080/api/v1/fleets")
        print("   🚙 Vehicle API: http://localhost:8080/api/v1/vehicles")
        print("\n💡 Available Commands:")
        print("   python debug_fleet_os.py diagnostics  # Run diagnostics")
        print("   python system_test.py                 # Run system tests")
        print("\n🛑 Press Ctrl+C to stop all services")
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down debug environment...")
        print("✅ All services stopped")

if __name__ == "__main__":
    main()
