package sector_tests

import (
	"flag"
	"fmt"
	"os"
	"testing"
	"time"
	"gopkg.in/yaml.v2"
)

// TestConfig represents the test configuration
type TestConfig struct {
	TestConfig struct {
		Environment string `yaml:"environment"`
		LogLevel    string `yaml:"log_level"`
		Database    struct {
			Host     string `yaml:"host"`
			Port     int    `yaml:"port"`
			Name     string `yaml:"name"`
			User     string `yaml:"user"`
			Password string `yaml:"password"`
			SSLMode  string `yaml:"ssl_mode"`
		} `yaml:"database"`
		Services map[string]struct {
			Enabled     bool          `yaml:"enabled"`
			TestTimeout time.Duration `yaml:"test_timeout"`
			MockSensors bool          `yaml:"mock_sensors,omitempty"`
			MockVehicles bool         `yaml:"mock_vehicles,omitempty"`
			MockTasks   bool          `yaml:"mock_tasks,omitempty"`
			MockDashboard bool        `yaml:"mock_dashboard,omitempty"`
		} `yaml:"services"`
		Sectors map[string]struct {
			Enabled       bool     `yaml:"enabled"`
			TestScenarios []string `yaml:"test_scenarios"`
		} `yaml:"sectors"`
		TestData struct {
			VehiclesPerSector map[string]int `yaml:"vehicles_per_sector"`
			TasksPerSector    map[string]int `yaml:"tasks_per_sector"`
			TestDuration      time.Duration  `yaml:"test_duration"`
		} `yaml:"test_data"`
		Performance struct {
			Enabled     bool `yaml:"enabled"`
			LoadTest    struct {
				ConcurrentUsers int           `yaml:"concurrent_users"`
				Duration        time.Duration `yaml:"duration"`
				RampUp          time.Duration `yaml:"ramp_up"`
			} `yaml:"load_test"`
			StressTest struct {
				MaxConcurrentUsers int           `yaml:"max_concurrent_users"`
				Duration           time.Duration `yaml:"duration"`
			} `yaml:"stress_test"`
			EnduranceTest struct {
				Duration      time.Duration `yaml:"duration"`
				CheckInterval time.Duration `yaml:"check_interval"`
			} `yaml:"endurance_test"`
		} `yaml:"performance"`
	} `yaml:"test_config"`
}

var (
	configFile = flag.String("config", "test_config.yaml", "Path to test configuration file")
	sector     = flag.String("sector", "", "Specific sector to test (logistics, defense, mining, ride_hail)")
	verbose    = flag.Bool("verbose", false, "Enable verbose output")
	parallel   = flag.Bool("parallel", true, "Run tests in parallel")
)

// LoadTestConfig loads the test configuration from file
func LoadTestConfig() (*TestConfig, error) {
	data, err := os.ReadFile(*configFile)
	if err != nil {
		return nil, fmt.Errorf("failed to read config file: %v", err)
	}

	var config TestConfig
	if err := yaml.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("failed to unmarshal config: %v", err)
	}

	return &config, nil
}

// TestMain is the main test entry point
func TestMain(m *testing.M) {
	flag.Parse()

	// Load test configuration
	config, err := LoadTestConfig()
	if err != nil {
		fmt.Printf("Failed to load test configuration: %v\n", err)
		os.Exit(1)
	}

	// Set up test environment
	if err := setupTestEnvironment(config); err != nil {
		fmt.Printf("Failed to setup test environment: %v\n", err)
		os.Exit(1)
	}

	// Run tests
	code := m.Run()

	// Cleanup test environment
	if err := cleanupTestEnvironment(config); err != nil {
		fmt.Printf("Failed to cleanup test environment: %v\n", err)
	}

	os.Exit(code)
}

// setupTestEnvironment sets up the test environment
func setupTestEnvironment(config *TestConfig) error {
	// Set environment variables
	os.Setenv("ATLASMESH_ENV", config.TestConfig.Environment)
	os.Setenv("ATLASMESH_LOG_LEVEL", config.TestConfig.LogLevel)
	
	// Set database environment variables
	os.Setenv("ATLASMESH_DB_HOST", config.TestConfig.Database.Host)
	os.Setenv("ATLASMESH_DB_PORT", fmt.Sprintf("%d", config.TestConfig.Database.Port))
	os.Setenv("ATLASMESH_DB_NAME", config.TestConfig.Database.Name)
	os.Setenv("ATLASMESH_DB_USER", config.TestConfig.Database.User)
	os.Setenv("ATLASMESH_DB_PASSWORD", config.TestConfig.Database.Password)
	os.Setenv("ATLASMESH_DB_SSL_MODE", config.TestConfig.Database.SSLMode)

	// Set sector if specified
	if *sector != "" {
		os.Setenv("ATLASMESH_SECTOR", *sector)
	}

	// Set verbose mode
	if *verbose {
		os.Setenv("ATLASMESH_VERBOSE", "true")
	}

	// Set parallel mode
	if *parallel {
		os.Setenv("ATLASMESH_PARALLEL", "true")
	}

	return nil
}

// cleanupTestEnvironment cleans up the test environment
func cleanupTestEnvironment(config *TestConfig) error {
	// Unset environment variables
	os.Unsetenv("ATLASMESH_ENV")
	os.Unsetenv("ATLASMESH_LOG_LEVEL")
	os.Unsetenv("ATLASMESH_DB_HOST")
	os.Unsetenv("ATLASMESH_DB_PORT")
	os.Unsetenv("ATLASMESH_DB_NAME")
	os.Unsetenv("ATLASMESH_DB_USER")
	os.Unsetenv("ATLASMESH_DB_PASSWORD")
	os.Unsetenv("ATLASMESH_DB_SSL_MODE")
	os.Unsetenv("ATLASMESH_SECTOR")
	os.Unsetenv("ATLASMESH_VERBOSE")
	os.Unsetenv("ATLASMESH_PARALLEL")

	return nil
}

// RunSectorTests runs tests for a specific sector
func RunSectorTests(t *testing.T, sector string) {
	// Set sector environment variable
	os.Setenv("ATLASMESH_SECTOR", sector)
	defer os.Unsetenv("ATLASMESH_SECTOR")

	// Run sector-specific tests
	switch sector {
	case "logistics":
		t.Run("LogisticsSector", func(t *testing.T) {
			TestLogisticsSectorConfiguration(t)
			TestLogisticsSectorIntegration(t)
			TestLogisticsSectorPerformance(t)
		})
	case "defense":
		t.Run("DefenseSector", func(t *testing.T) {
			TestDefenseSectorConfiguration(t)
			TestDefenseSectorSecurity(t)
			TestDefenseSectorTacticalFeatures(t)
		})
	case "mining":
		t.Run("MiningSector", func(t *testing.T) {
			TestMiningSectorConfiguration(t)
			TestMiningSectorProduction(t)
			TestMiningSectorSafety(t)
		})
	case "ride_hail":
		t.Run("RideHailSector", func(t *testing.T) {
			TestRideHailSectorConfiguration(t)
			TestRideHailSectorPassengerExperience(t)
			TestRideHailSectorUrbanOperations(t)
		})
	default:
		t.Errorf("Unknown sector: %s", sector)
	}
}

// TestAllSectors runs tests for all sectors
func TestAllSectors(t *testing.T) {
	sectors := []string{"logistics", "defense", "mining", "ride_hail"}
	
	for _, sector := range sectors {
		t.Run(sector, func(t *testing.T) {
			RunSectorTests(t, sector)
		})
	}
}

// TestSectorConfiguration tests sector configuration loading
func TestSectorConfiguration(t *testing.T) {
	sectors := []string{"logistics", "defense", "mining", "ride_hail"}
	
	for _, sector := range sectors {
		t.Run(sector, func(t *testing.T) {
			os.Setenv("ATLASMESH_SECTOR", sector)
			defer os.Unsetenv("ATLASMESH_SECTOR")
			
			// Test configuration loading
			config, err := LoadTestConfig()
			if err != nil {
				t.Fatalf("Failed to load test configuration: %v", err)
			}
			
			// Check if sector is enabled
			if sectorConfig, exists := config.TestConfig.Sectors[sector]; !exists || !sectorConfig.Enabled {
				t.Errorf("Sector %s is not enabled in test configuration", sector)
			}
		})
	}
}

