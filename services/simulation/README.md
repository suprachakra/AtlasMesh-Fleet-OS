# Simulation Service

> **TL;DR:** Comprehensive simulation service providing vehicle simulation, scenario testing, and digital twin integration for autonomous vehicle development and testing.

## Overview

The Simulation Service provides comprehensive simulation capabilities for autonomous vehicle development, testing, and validation. It includes vehicle dynamics simulation, sensor simulation, and scenario-based testing.

## Features

- **Vehicle Dynamics**: Realistic vehicle physics simulation
- **Sensor Simulation**: LiDAR, Camera, Radar, GPS simulation
- **Scenario Testing**: Predefined and custom scenario testing
- **Digital Twin Integration**: Real-time digital twin updates
- **Performance Testing**: Load and stress testing capabilities
- **Validation**: Algorithm validation and verification

## API Endpoints

- `POST /api/v1/simulation/start` - Start simulation
- `POST /api/v1/simulation/stop` - Stop simulation
- `GET /api/v1/simulation/status` - Get simulation status
- `POST /api/v1/simulation/scenario` - Load scenario
- `GET /api/v1/simulation/metrics` - Get simulation metrics
