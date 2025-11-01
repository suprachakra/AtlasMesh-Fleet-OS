# Sector-Specific Tests

This directory contains comprehensive tests for all sector-specific configurations and functionality in AtlasMesh Fleet OS.

## Overview

The sector-specific tests validate that each sector (Logistics, Defense, Mining, Ride-Hail) has the correct configuration parameters, service settings, and operational characteristics.

## Test Structure

### Test Files
- `logistics_test.go` - Tests for Logistics sector (warehouse automation)
- `defense_test.go` - Tests for Defense sector (military UGV operations)
- `mining_test.go` - Tests for Mining sector (haul truck automation)
- `ride_hail_test.go` - Tests for Ride-Hail sector (robotaxi operations)
- `test_runner.go` - Main test runner and configuration loader
- `test_config.yaml` - Test configuration file

### Test Categories

#### 1. Configuration Tests
- Sector-specific parameter validation
- Service configuration consistency
- Environment variable handling
- Configuration loading and validation

#### 2. Integration Tests
- Cross-service configuration consistency
- Sector-specific feature integration
- End-to-end configuration validation

#### 3. Performance Tests
- Sector-specific performance characteristics
- Load testing for each sector
- Stress testing and endurance testing

#### 4. Security Tests
- Defense sector security features
- Encrypted communications validation
- Access control and clearance levels

#### 5. Operational Tests
- Sector-specific operational features
- Tactical features (Defense)
- Production optimization (Mining)
- Passenger experience (Ride-Hail)
- Warehouse operations (Logistics)

## Running Tests

### Run All Sector Tests
```bash
go test ./testing/sector-tests/...
```

### Run Specific Sector Tests
```bash
# Test Logistics sector
go test -sector=logistics ./testing/sector-tests/...

# Test Defense sector
go test -sector=defense ./testing/sector-tests/...

# Test Mining sector
go test -sector=mining ./testing/sector-tests/...

# Test Ride-Hail sector
go test -sector=ride_hail ./testing/sector-tests/...
```

### Run with Custom Configuration
```bash
go test -config=custom_test_config.yaml ./testing/sector-tests/...
```

### Run with Verbose Output
```bash
go test -verbose ./testing/sector-tests/...
```

### Run Tests in Parallel
```bash
go test -parallel ./testing/sector-tests/...
```

## Test Configuration

The test configuration is defined in `test_config.yaml` and includes:

### Environment Settings
- Test environment configuration
- Log level settings
- Database connection settings

### Service Settings
- Service-specific test timeouts
- Mock configurations for testing
- Service enable/disable flags

### Sector Settings
- Sector-specific test scenarios
- Test data generation settings
- Performance test parameters

### Performance Settings
- Load testing configuration
- Stress testing parameters
- Endurance testing settings

## Test Data

Each sector test uses appropriate test data:

### Logistics
- 10 test vehicles (forklifts, tuggers, pallet trucks)
- 100 test tasks (warehouse operations)
- Indoor navigation scenarios

### Defense
- 5 test vehicles (UGVs, tactical vehicles)
- 25 test tasks (tactical operations)
- GPS-denied navigation scenarios

### Mining
- 20 test vehicles (haul trucks, bulldozers)
- 200 test tasks (haul cycles)
- Production optimization scenarios

### Ride-Hail
- 50 test vehicles (robotaxis, shuttles)
- 500 test tasks (passenger rides)
- Urban navigation scenarios

## Expected Test Results

### Configuration Tests
- All sector configurations load successfully
- All service configurations are valid
- Environment variables are set correctly
- Configuration validation passes

### Integration Tests
- Cross-service configurations are consistent
- Sector-specific features work together
- End-to-end configuration validation passes

### Performance Tests
- Sector-specific performance targets are met
- Load testing completes successfully
- Stress testing handles peak loads
- Endurance testing runs for specified duration

### Security Tests (Defense)
- Encrypted communications are enabled
- Security clearance levels are enforced
- Threat assessment features are active
- Access control is properly configured

### Operational Tests
- Sector-specific operational features work correctly
- Tactical features are properly configured (Defense)
- Production optimization is enabled (Mining)
- Passenger experience features work (Ride-Hail)
- Warehouse operations are configured (Logistics)

## Troubleshooting

### Common Issues

1. **Configuration Loading Errors**
   - Check that `config/` directory exists
   - Verify sector configuration files are present
   - Ensure environment variables are set correctly

2. **Database Connection Errors**
   - Verify test database is running
   - Check database connection parameters
   - Ensure test database has correct schema

3. **Service Configuration Errors**
   - Check that all required services are enabled
   - Verify service configuration files exist
   - Ensure service-specific parameters are correct

4. **Test Timeout Errors**
   - Increase test timeout values in configuration
   - Check for resource constraints
   - Verify test environment performance

### Debug Mode

Run tests with debug output:
```bash
go test -v -sector=<sector_name> ./testing/sector-tests/...
```

### Log Analysis

Check test logs for detailed error information:
```bash
go test -v 2>&1 | tee test_output.log
```

## Contributing

When adding new sector tests:

1. Create test file following naming convention: `{sector}_test.go`
2. Implement required test functions
3. Add sector-specific test scenarios to `test_config.yaml`
4. Update this README with new test information
5. Ensure all tests pass before submitting

## Dependencies

- Go 1.21+
- AtlasMesh config package
- PostgreSQL (for database tests)
- Prometheus client (for metrics tests)
- Zap logger (for logging tests)
- YAML v2 (for configuration parsing)

