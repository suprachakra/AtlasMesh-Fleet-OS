#!/usr/bin/env node

/**
 * AtlasMesh Fleet OS - Go SDK Generator
 * 
 * Generates Go client SDK from OpenAPI specifications
 * Uses openapi-generator-cli to create type-safe Go clients
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');
const yaml = require('js-yaml');

class GoSDKGenerator {
    constructor() {
        this.contractsDir = path.join(__dirname, '../../api/contracts/v1');
        this.outputDir = path.join(__dirname, '../../sdks/go');
        this.templateDir = path.join(__dirname, 'templates/go');
        
        // SDK Configuration
        this.config = {
            packageName: 'atlasmesh',
            packageVersion: '1.0.0',
            packageUrl: 'github.com/atlasmesh/fleet-os-sdk-go',
            gitUserId: 'atlasmesh',
            gitRepoId: 'fleet-os-sdk-go',
            moduleName: 'github.com/atlasmesh/fleet-os-sdk-go'
        };
    }

    async generateSDK() {
        console.log('🚀 Starting Go SDK generation...');
        
        try {
            // Clean output directory
            this.cleanOutputDir();
            
            // Create base SDK structure
            this.createBaseStructure();
            
            // Generate client for each service
            const services = this.getServiceContracts();
            for (const service of services) {
                await this.generateServiceClient(service);
            }
            
            // Generate unified client
            this.generateUnifiedClient(services);
            
            // Generate configuration and utilities
            this.generateConfiguration();
            this.generateUtilities();
            
            // Generate documentation
            this.generateDocumentation(services);
            
            // Generate go.mod and other files
            this.generateModuleFiles();
            
            console.log('✅ Go SDK generation completed successfully!');
            console.log(`📦 SDK available at: ${this.outputDir}`);
            
        } catch (error) {
            console.error('❌ Go SDK generation failed:', error.message);
            process.exit(1);
        }
    }

    cleanOutputDir() {
        if (fs.existsSync(this.outputDir)) {
            fs.rmSync(this.outputDir, { recursive: true, force: true });
        }
        fs.mkdirSync(this.outputDir, { recursive: true });
    }

    createBaseStructure() {
        const dirs = [
            'client',
            'models',
            'auth',
            'config',
            'utils',
            'examples',
            'docs'
        ];
        
        dirs.forEach(dir => {
            fs.mkdirSync(path.join(this.outputDir, dir), { recursive: true });
        });
    }

    getServiceContracts() {
        const contracts = [];
        const files = fs.readdirSync(this.contractsDir);
        
        for (const file of files) {
            if (file.endsWith('.yaml') || file.endsWith('.yml')) {
                const contractPath = path.join(this.contractsDir, file);
                const serviceName = path.basename(file, path.extname(file));
                
                try {
                    const contract = yaml.load(fs.readFileSync(contractPath, 'utf8'));
                    contracts.push({
                        name: serviceName,
                        title: contract.info.title,
                        version: contract.info.version,
                        contractPath: contractPath,
                        contract: contract
                    });
                } catch (error) {
                    console.warn(`⚠️  Skipping invalid contract: ${file} - ${error.message}`);
                }
            }
        }
        
        return contracts;
    }

    async generateServiceClient(service) {
        console.log(`📝 Generating client for ${service.title}...`);
        
        const clientDir = path.join(this.outputDir, 'client', service.name.replace('-', '_'));
        fs.mkdirSync(clientDir, { recursive: true });
        
        // Generate using openapi-generator-cli
        const tempDir = path.join('/tmp', `atlasmesh-go-${service.name}-${Date.now()}`);
        
        try {
            // Use openapi-generator to generate Go client
            const generateCmd = [
                'npx openapi-generator-cli generate',
                `-i ${service.contractPath}`,
                `-g go`,
                `-o ${tempDir}`,
                `--package-name ${service.name.replace('-', '_')}`,
                `--git-user-id ${this.config.gitUserId}`,
                `--git-repo-id ${this.config.gitRepoId}`,
                '--additional-properties=packageVersion=' + this.config.packageVersion,
                '--additional-properties=packageUrl=' + this.config.packageUrl,
                '--skip-validate-spec'
            ].join(' ');
            
            execSync(generateCmd, { stdio: 'pipe' });
            
            // Copy generated files to our structure
            this.copyGeneratedFiles(tempDir, clientDir, service);
            
            // Clean up temp directory
            fs.rmSync(tempDir, { recursive: true, force: true });
            
        } catch (error) {
            console.error(`❌ Failed to generate client for ${service.name}:`, error.message);
            throw error;
        }
    }

    copyGeneratedFiles(tempDir, clientDir, service) {
        // Copy model files
        const modelsDir = path.join(tempDir, 'model');
        if (fs.existsSync(modelsDir)) {
            const targetModelsDir = path.join(this.outputDir, 'models', service.name.replace('-', '_'));
            fs.mkdirSync(targetModelsDir, { recursive: true });
            this.copyDirectory(modelsDir, targetModelsDir);
        }
        
        // Copy client files
        const clientFiles = ['client.go', 'api_*.go'];
        const tempClientDir = path.join(tempDir);
        
        if (fs.existsSync(tempClientDir)) {
            const files = fs.readdirSync(tempClientDir);
            files.forEach(file => {
                if (file.endsWith('.go') && (file.includes('api_') || file === 'client.go')) {
                    const srcPath = path.join(tempClientDir, file);
                    const destPath = path.join(clientDir, file);
                    fs.copyFileSync(srcPath, destPath);
                }
            });
        }
    }

    copyDirectory(src, dest) {
        if (!fs.existsSync(dest)) {
            fs.mkdirSync(dest, { recursive: true });
        }
        
        const files = fs.readdirSync(src);
        files.forEach(file => {
            const srcPath = path.join(src, file);
            const destPath = path.join(dest, file);
            
            if (fs.statSync(srcPath).isDirectory()) {
                this.copyDirectory(srcPath, destPath);
            } else {
                fs.copyFileSync(srcPath, destPath);
            }
        });
    }

    generateUnifiedClient(services) {
        console.log('📝 Generating unified client...');
        
        const clientCode = this.generateUnifiedClientCode(services);
        fs.writeFileSync(path.join(this.outputDir, 'client.go'), clientCode);
    }

    generateUnifiedClientCode(services) {
        const imports = services.map(service => 
            `\t${service.name.replace('-', '_')} "github.com/atlasmesh/fleet-os-sdk-go/client/${service.name.replace('-', '_')}"`
        ).join('\n');

        const clientFields = services.map(service => 
            `\t${this.toPascalCase(service.name.replace('-', '_'))} *${service.name.replace('-', '_')}.APIClient`
        ).join('\n');

        const clientInit = services.map(service => 
            `\t\tclient.${this.toPascalCase(service.name.replace('-', '_'))} = ${service.name.replace('-', '_')}.NewAPIClient(${service.name.replace('-', '_')}.NewConfiguration())`
        ).join('\n');

        return `package atlasmesh

import (
\t"context"
\t"net/http"
\t
${imports}
\t"github.com/atlasmesh/fleet-os-sdk-go/auth"
\t"github.com/atlasmesh/fleet-os-sdk-go/config"
)

// Client is the unified AtlasMesh Fleet OS client
type Client struct {
${clientFields}
\tconfig *config.Config
\tauth   *auth.Manager
}

// NewClient creates a new AtlasMesh Fleet OS client
func NewClient(cfg *config.Config) *Client {
\tclient := &Client{
\t\tconfig: cfg,
\t\tauth:   auth.NewManager(cfg),
\t}
\t
${clientInit}
\t
\treturn client
}

// NewClientWithAPIKey creates a client with API key authentication
func NewClientWithAPIKey(apiKey string, opts ...config.Option) *Client {
\tcfg := config.NewConfig(opts...)
\tcfg.APIKey = apiKey
\treturn NewClient(cfg)
}

// NewClientWithJWT creates a client with JWT token authentication
func NewClientWithJWT(token string, opts ...config.Option) *Client {
\tcfg := config.NewConfig(opts...)
\tcfg.JWTToken = token
\treturn NewClient(cfg)
}

// SetHTTPClient sets a custom HTTP client
func (c *Client) SetHTTPClient(httpClient *http.Client) {
\tc.config.HTTPClient = httpClient
}

// WithContext returns a new client with the given context
func (c *Client) WithContext(ctx context.Context) *Client {
\tnewClient := *c
\tnewClient.config = c.config.WithContext(ctx)
\treturn &newClient
}

// Health checks the health of all services
func (c *Client) Health(ctx context.Context) (*HealthStatus, error) {
\t// Implementation for checking health of all services
\treturn nil, nil
}

// HealthStatus represents the overall health status
type HealthStatus struct {
\tOverallStatus string            \`json:"overall_status"\`
\tServices      map[string]string \`json:"services"\`
\tTimestamp     string            \`json:"timestamp"\`
}
`;
    }

    generateConfiguration() {
        console.log('📝 Generating configuration...');
        
        const configCode = `package config

import (
\t"context"
\t"net/http"
\t"time"
)

// Config holds the configuration for the AtlasMesh Fleet OS client
type Config struct {
\t// Base URL for the API
\tBaseURL string
\t
\t// Authentication
\tAPIKey   string
\tJWTToken string
\t
\t// HTTP Client configuration
\tHTTPClient *http.Client
\tTimeout    time.Duration
\t
\t// Request context
\tContext context.Context
\t
\t// User Agent
\tUserAgent string
\t
\t// Debug mode
\tDebug bool
\t
\t// Rate limiting
\tRateLimit int
\t
\t// Retry configuration
\tMaxRetries    int
\tRetryDelay    time.Duration
\tRetryMaxDelay time.Duration
}

// Option is a function that configures the Config
type Option func(*Config)

// NewConfig creates a new configuration with default values
func NewConfig(opts ...Option) *Config {
\tcfg := &Config{
\t\tBaseURL:       "https://api.atlasmesh.com",
\t\tHTTPClient:    &http.Client{},
\t\tTimeout:       30 * time.Second,
\t\tContext:       context.Background(),
\t\tUserAgent:     "AtlasMesh-Go-SDK/1.0.0",
\t\tDebug:         false,
\t\tRateLimit:     1000,
\t\tMaxRetries:    3,
\t\tRetryDelay:    1 * time.Second,
\t\tRetryMaxDelay: 30 * time.Second,
\t}
\t
\tfor _, opt := range opts {
\t\topt(cfg)
\t}
\t
\treturn cfg
}

// WithBaseURL sets the base URL
func WithBaseURL(url string) Option {
\treturn func(c *Config) {
\t\tc.BaseURL = url
\t}
}

// WithAPIKey sets the API key
func WithAPIKey(apiKey string) Option {
\treturn func(c *Config) {
\t\tc.APIKey = apiKey
\t}
}

// WithJWTToken sets the JWT token
func WithJWTToken(token string) Option {
\treturn func(c *Config) {
\t\tc.JWTToken = token
\t}
}

// WithHTTPClient sets a custom HTTP client
func WithHTTPClient(client *http.Client) Option {
\treturn func(c *Config) {
\t\tc.HTTPClient = client
\t}
}

// WithTimeout sets the request timeout
func WithTimeout(timeout time.Duration) Option {
\treturn func(c *Config) {
\t\tc.Timeout = timeout
\t}
}

// WithDebug enables debug mode
func WithDebug(debug bool) Option {
\treturn func(c *Config) {
\t\tc.Debug = debug
\t}
}

// WithContext returns a new config with the given context
func (c *Config) WithContext(ctx context.Context) *Config {
\tnewConfig := *c
\tnewConfig.Context = ctx
\treturn &newConfig
}
`;

        fs.writeFileSync(path.join(this.outputDir, 'config', 'config.go'), configCode);

        // Generate auth manager
        const authCode = `package auth

import (
\t"fmt"
\t"net/http"
\t
\t"github.com/atlasmesh/fleet-os-sdk-go/config"
)

// Manager handles authentication for API requests
type Manager struct {
\tconfig *config.Config
}

// NewManager creates a new authentication manager
func NewManager(cfg *config.Config) *Manager {
\treturn &Manager{
\t\tconfig: cfg,
\t}
}

// AddAuth adds authentication headers to the request
func (m *Manager) AddAuth(req *http.Request) error {
\tif m.config.JWTToken != "" {
\t\treq.Header.Set("Authorization", fmt.Sprintf("Bearer %s", m.config.JWTToken))
\t} else if m.config.APIKey != "" {
\t\treq.Header.Set("X-API-Key", m.config.APIKey)
\t}
\t
\treturn nil
}

// RefreshToken refreshes the JWT token if needed
func (m *Manager) RefreshToken() error {
\t// Implementation for token refresh
\treturn nil
}
`;

        fs.writeFileSync(path.join(this.outputDir, 'auth', 'auth.go'), authCode);
    }

    generateUtilities() {
        console.log('📝 Generating utilities...');
        
        const utilsCode = `package utils

import (
\t"encoding/json"
\t"fmt"
\t"time"
)

// PrettyPrint prints a struct in a pretty JSON format
func PrettyPrint(v interface{}) {
\tb, err := json.MarshalIndent(v, "", "  ")
\tif err != nil {
\t\tfmt.Printf("Error: %v\\n", err)
\t\treturn
\t}
\tfmt.Println(string(b))
}

// TimePtr returns a pointer to a time.Time
func TimePtr(t time.Time) *time.Time {
\treturn &t
}

// StringPtr returns a pointer to a string
func StringPtr(s string) *string {
\treturn &s
}

// IntPtr returns a pointer to an int
func IntPtr(i int) *int {
\treturn &i
}

// Float64Ptr returns a pointer to a float64
func Float64Ptr(f float64) *float64 {
\treturn &f
}

// BoolPtr returns a pointer to a bool
func BoolPtr(b bool) *bool {
\treturn &b
}
`;

        fs.writeFileSync(path.join(this.outputDir, 'utils', 'utils.go'), utilsCode);
    }

    generateDocumentation(services) {
        console.log('📝 Generating documentation...');
        
        const readmeContent = this.generateReadmeContent(services);
        fs.writeFileSync(path.join(this.outputDir, 'README.md'), readmeContent);
        
        // Generate examples
        const exampleCode = this.generateExampleCode(services);
        fs.writeFileSync(path.join(this.outputDir, 'examples', 'main.go'), exampleCode);
    }

    generateReadmeContent(services) {
        const serviceList = services.map(service => 
            `- **${service.title}**: ${service.contract.info.description?.split('\n')[0] || 'No description'}`
        ).join('\n');

        return `# AtlasMesh Fleet OS Go SDK

Official Go client library for the AtlasMesh Fleet OS API.

## Installation

\`\`\`bash
go get github.com/atlasmesh/fleet-os-sdk-go
\`\`\`

## Quick Start

\`\`\`go
package main

import (
    "context"
    "fmt"
    "log"
    
    "github.com/atlasmesh/fleet-os-sdk-go"
    "github.com/atlasmesh/fleet-os-sdk-go/config"
)

func main() {
    // Create client with API key
    client := atlasmesh.NewClientWithAPIKey("your-api-key",
        config.WithBaseURL("https://api.atlasmesh.com"),
        config.WithDebug(true),
    )
    
    // Use the client
    ctx := context.Background()
    
    // Example: List vehicles
    vehicles, err := client.FleetManager.ListVehicles(ctx, nil)
    if err != nil {
        log.Fatal(err)
    }
    
    fmt.Printf("Found %d vehicles\\n", len(vehicles.Vehicles))
}
\`\`\`

## Services

This SDK provides clients for the following AtlasMesh Fleet OS services:

${serviceList}

## Configuration

The SDK can be configured using various options:

\`\`\`go
client := atlasmesh.NewClient(config.NewConfig(
    config.WithBaseURL("https://your-instance.atlasmesh.com"),
    config.WithAPIKey("your-api-key"),
    config.WithTimeout(60 * time.Second),
    config.WithDebug(true),
))
\`\`\`

## Authentication

The SDK supports multiple authentication methods:

### API Key Authentication

\`\`\`go
client := atlasmesh.NewClientWithAPIKey("your-api-key")
\`\`\`

### JWT Token Authentication

\`\`\`go
client := atlasmesh.NewClientWithJWT("your-jwt-token")
\`\`\`

## Error Handling

All API methods return an error as the second return value. Always check for errors:

\`\`\`go
result, err := client.TripService.CreateTrip(ctx, request)
if err != nil {
    // Handle error
    log.Printf("API error: %v", err)
    return
}
\`\`\`

## Examples

See the [examples](./examples/) directory for more usage examples.

## Documentation

- [API Reference](https://docs.atlasmesh.com/api/)
- [SDK Documentation](https://pkg.go.dev/github.com/atlasmesh/fleet-os-sdk-go)

## License

This SDK is licensed under the same terms as AtlasMesh Fleet OS.
`;
    }

    generateExampleCode(services) {
        return `package main

import (
\t"context"
\t"fmt"
\t"log"
\t"time"
\t
\t"github.com/atlasmesh/fleet-os-sdk-go"
\t"github.com/atlasmesh/fleet-os-sdk-go/config"
\t"github.com/atlasmesh/fleet-os-sdk-go/utils"
)

func main() {
\t// Create client
\tclient := atlasmesh.NewClientWithAPIKey("your-api-key",
\t\tconfig.WithBaseURL("https://api.atlasmesh.com"),
\t\tconfig.WithDebug(true),
\t)
\t
\tctx := context.Background()
\t
\t// Example 1: List vehicles
\tfmt.Println("=== Listing Vehicles ===")
\tvehicles, err := client.FleetManager.ListVehicles(ctx, nil)
\tif err != nil {
\t\tlog.Printf("Error listing vehicles: %v", err)
\t} else {
\t\tfmt.Printf("Found %d vehicles\\n", len(vehicles.Vehicles))
\t\tfor _, vehicle := range vehicles.Vehicles {
\t\t\tfmt.Printf("- %s (%s): %s\\n", vehicle.Name, vehicle.Id, vehicle.Status)
\t\t}
\t}
\t
\t// Example 2: Create a trip
\tfmt.Println("\\n=== Creating Trip ===")
\ttripRequest := &CreateTripRequest{
\t\tOrigin: &Location{
\t\t\tLatitude:  25.2048,
\t\t\tLongitude: 55.2708,
\t\t\tAddress:   utils.StringPtr("Dubai Marina"),
\t\t},
\t\tDestination: &Location{
\t\t\tLatitude:  25.1972,
\t\t\tLongitude: 55.2744,
\t\t\tAddress:   utils.StringPtr("Dubai Mall"),
\t\t},
\t\tSector:   "ridehail",
\t\tPriority: "normal",
\t}
\t
\ttrip, err := client.TripService.CreateTrip(ctx, tripRequest)
\tif err != nil {
\t\tlog.Printf("Error creating trip: %v", err)
\t} else {
\t\tfmt.Printf("Created trip: %s\\n", trip.Id)
\t\tutils.PrettyPrint(trip)
\t}
\t
\t// Example 3: Health check
\tfmt.Println("\\n=== Health Check ===")
\thealth, err := client.Health(ctx)
\tif err != nil {
\t\tlog.Printf("Error checking health: %v", err)
\t} else {
\t\tfmt.Printf("Overall status: %s\\n", health.OverallStatus)
\t\tfor service, status := range health.Services {
\t\t\tfmt.Printf("- %s: %s\\n", service, status)
\t\t}
\t}
}
`;
    }

    generateModuleFiles() {
        console.log('📝 Generating module files...');
        
        // Generate go.mod
        const goMod = `module github.com/atlasmesh/fleet-os-sdk-go

go 1.21

require (
\tgithub.com/golang/protobuf v1.5.3
\tgolang.org/x/oauth2 v0.10.0
)

require (
\tgithub.com/golang/protobuf v1.5.3 // indirect
\tgolang.org/x/net v0.12.0 // indirect
\tgoogle.golang.org/appengine v1.6.7 // indirect
\tgoogle.golang.org/protobuf v1.31.0 // indirect
)
`;
        fs.writeFileSync(path.join(this.outputDir, 'go.mod'), goMod);
        
        // Generate .gitignore
        const gitignore = `# Binaries for programs and plugins
*.exe
*.exe~
*.dll
*.so
*.dylib

# Test binary, built with \`go test -c\`
*.test

# Output of the go coverage tool, specifically when used with LiteIDE
*.out

# Dependency directories (remove the comment below to include it)
# vendor/

# Go workspace file
go.work

# IDE files
.vscode/
.idea/
*.swp
*.swo

# OS files
.DS_Store
Thumbs.db
`;
        fs.writeFileSync(path.join(this.outputDir, '.gitignore'), gitignore);
        
        // Generate Makefile
        const makefile = `# AtlasMesh Fleet OS Go SDK Makefile

.PHONY: build test lint fmt vet clean install examples

# Build the SDK
build:
\tgo build ./...

# Run tests
test:
\tgo test -v ./...

# Run tests with coverage
test-coverage:
\tgo test -v -coverprofile=coverage.out ./...
\tgo tool cover -html=coverage.out -o coverage.html

# Lint the code
lint:
\tgolangci-lint run

# Format the code
fmt:
\tgo fmt ./...

# Vet the code
vet:
\tgo vet ./...

# Clean build artifacts
clean:
\tgo clean
\trm -f coverage.out coverage.html

# Install dependencies
install:
\tgo mod download
\tgo mod tidy

# Run examples
examples:
\tgo run examples/main.go

# Generate documentation
docs:
\tgodoc -http=:6060

# Run all checks
check: fmt vet lint test

# Build for multiple platforms
build-all:
\tGOOS=linux GOARCH=amd64 go build -o bin/sdk-linux-amd64 ./examples
\tGOOS=darwin GOARCH=amd64 go build -o bin/sdk-darwin-amd64 ./examples
\tGOOS=windows GOARCH=amd64 go build -o bin/sdk-windows-amd64.exe ./examples
`;
        fs.writeFileSync(path.join(this.outputDir, 'Makefile'), makefile);
    }

    toPascalCase(str) {
        return str.replace(/(^\w|_\w)/g, (match) => 
            match.replace('_', '').toUpperCase()
        );
    }
}

// Main execution
if (require.main === module) {
    const generator = new GoSDKGenerator();
    generator.generateSDK().catch(console.error);
}

module.exports = GoSDKGenerator;
