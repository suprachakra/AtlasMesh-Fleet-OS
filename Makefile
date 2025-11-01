# AtlasMesh Fleet OS - Build and Development Makefile
# Provides standardized commands for development, testing, and deployment

.PHONY: help setup build test lint clean docker deploy gates evidence

# Default target
.DEFAULT_GOAL := help

# Variables
PROJECT_NAME := atlasmesh-fleet-os
VERSION := $(shell git describe --tags --always --dirty)
COMMIT_HASH := $(shell git rev-parse --short HEAD)
BUILD_DATE := $(shell date -u +"%Y-%m-%dT%H:%M:%SZ")
GO_VERSION := 1.21
NODE_VERSION := 18
PYTHON_VERSION := 3.10

# Docker configuration
DOCKER_REGISTRY := ghcr.io/atlasmesh
DOCKER_TAG := $(VERSION)

# Colors for output
RED := \033[31m
GREEN := \033[32m
YELLOW := \033[33m
BLUE := \033[34m
RESET := \033[0m

help: ## Show this help message
	@echo "$(BLUE)AtlasMesh Fleet OS - Development Commands$(RESET)"
	@echo "=========================================="
	@echo ""
	@echo "$(GREEN)Setup Commands:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep -E "(setup|install)" | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(RESET) %s\n", $$1, $$2}'
	@echo ""
	@echo "$(GREEN)Development Commands:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep -E "(build|dev|run)" | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(RESET) %s\n", $$1, $$2}'
	@echo ""
	@echo "$(GREEN)Testing Commands:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep -E "(test|lint|check)" | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(RESET) %s\n", $$1, $$2}'
	@echo ""
	@echo "$(GREEN)CI/CD Commands:$(RESET)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | grep -E "(gates|evidence|docker|deploy)" | awk 'BEGIN {FS = ":.*?## "}; {printf "  $(YELLOW)%-20s$(RESET) %s\n", $$1, $$2}'
	@echo ""

# Setup Commands
setup: ## Set up development environment
	@echo "$(BLUE)Setting up AtlasMesh Fleet OS development environment...$(RESET)"
	@./scripts/setup_dev.sh

install-deps: ## Install all dependencies
	@echo "$(BLUE)Installing dependencies...$(RESET)"
	@npm ci
	@cd services && find . -name "go.mod" -execdir go mod download \;
	@pip install -r requirements.txt

# Development Commands
build: ## Build all services and components
	@echo "$(BLUE)Building AtlasMesh Fleet OS...$(RESET)"
	@echo "Version: $(VERSION)"
	@echo "Commit: $(COMMIT_HASH)"
	@echo "Build Date: $(BUILD_DATE)"
	@nx run-many --target=build --all --parallel=3

build-go: ## Build Go services
	@echo "$(BLUE)Building Go services...$(RESET)"
	@cd services && find . -name "*.go" -path "*/cmd/*" -execdir go build -ldflags "-X main.version=$(VERSION) -X main.commit=$(COMMIT_HASH) -X main.buildDate=$(BUILD_DATE)" \;

build-ui: ## Build UI components
	@echo "$(BLUE)Building UI components...$(RESET)"
	@nx run-many --target=build --projects=tag:ui --parallel=2

dev: ## Start development environment
	@echo "$(BLUE)Starting development environment...$(RESET)"
	@docker-compose -f deployment/manifests/docker-compose.dev.yml up -d
	@echo "$(GREEN)Development environment started!$(RESET)"
	@echo "Control Center: http://localhost:3000"
	@echo "API Gateway: http://localhost:8080"

dev-stop: ## Stop development environment
	@echo "$(BLUE)Stopping development environment...$(RESET)"
	@docker-compose -f deployment/manifests/docker-compose.dev.yml down

run-local: ## Run services locally (no Docker)
	@echo "$(BLUE)Running services locally...$(RESET)"
	@./scripts/run_local.sh

# Testing Commands
test: ## Run all tests
	@echo "$(BLUE)Running all tests...$(RESET)"
	@nx run-many --target=test --all --parallel=3 --coverage

test-unit: ## Run unit tests only
	@echo "$(BLUE)Running unit tests...$(RESET)"
	@nx run-many --target=test:unit --all --parallel=3

test-integration: ## Run integration tests
	@echo "$(BLUE)Running integration tests...$(RESET)"
	@nx run-many --target=test:integration --all --parallel=2

test-e2e: ## Run end-to-end tests
	@echo "$(BLUE)Running E2E tests...$(RESET)"
	@nx run-many --target=e2e --all

test-contract: ## Run contract tests for adapters
	@echo "$(BLUE)Running contract tests...$(RESET)"
	@nx run-many --target=test:contract --projects=tag:adapter

test-sim: ## Run simulation tests
	@echo "$(BLUE)Running simulation tests...$(RESET)"
	@cd sim && python -m pytest testing/ -v

test-performance: ## Run performance tests
	@echo "$(BLUE)Running performance tests...$(RESET)"
	@./scripts/run_performance_tests.sh

lint: ## Run linting on all code
	@echo "$(BLUE)Running linters...$(RESET)"
	@nx run-many --target=lint --all --parallel=3

lint-fix: ## Fix linting issues automatically
	@echo "$(BLUE)Fixing linting issues...$(RESET)"
	@nx run-many --target=lint --all --parallel=3 --fix

check-types: ## Run type checking
	@echo "$(BLUE)Running type checks...$(RESET)"
	@nx run-many --target=typecheck --all --parallel=3

check-security: ## Run security scans
	@echo "$(BLUE)Running security scans...$(RESET)"
	@npm audit --audit-level=high
	@./scripts/security_scan.sh

# CI/CD Commands
gates: ## Run all CI gates locally
	@echo "$(BLUE)Running CI gates locally...$(RESET)"
	@node tools/variant-budget-analyzer.js
	@npm run lint
	@npm run test
	@npm run build
	@echo "$(GREEN)All CI gates passed!$(RESET)"

gates-variant-budget: ## Check variant budget compliance
	@echo "$(BLUE)Checking variant budget...$(RESET)"
	@node tools/variant-budget-analyzer.js

gates-performance: ## Check performance budgets
	@echo "$(BLUE)Checking performance budgets...$(RESET)"
	@./scripts/check_performance_budgets.sh

gates-accessibility: ## Check accessibility compliance
	@echo "$(BLUE)Checking accessibility compliance...$(RESET)"
	@./scripts/check_accessibility.sh

evidence: ## Generate evidence bundle
	@echo "$(BLUE)Generating evidence bundle...$(RESET)"
	@node tools/evidence-bundle-generator.js
	@echo "$(GREEN)Evidence bundle generated successfully!$(RESET)"

evidence-validate: ## Validate evidence bundle completeness
	@echo "$(BLUE)Validating evidence bundle...$(RESET)"
	@./scripts/validate_evidence_bundle.sh

# Docker Commands
docker-build: ## Build Docker images
	@echo "$(BLUE)Building Docker images...$(RESET)"
	@docker build -t $(DOCKER_REGISTRY)/api-gateway:$(DOCKER_TAG) -f services/api-gateway/Dockerfile .
	@docker build -t $(DOCKER_REGISTRY)/mission-management:$(DOCKER_TAG) -f services/mission-management/Dockerfile .
	@docker build -t $(DOCKER_REGISTRY)/dispatch-service:$(DOCKER_TAG) -f services/dispatch-service/Dockerfile .
	@docker build -t $(DOCKER_REGISTRY)/fleet-manager:$(DOCKER_TAG) -f services/fleet-manager/Dockerfile .
	@docker build -t $(DOCKER_REGISTRY)/control-center:$(DOCKER_TAG) -f ui/control-center/Dockerfile .

docker-push: ## Push Docker images to registry
	@echo "$(BLUE)Pushing Docker images...$(RESET)"
	@docker push $(DOCKER_REGISTRY)/api-gateway:$(DOCKER_TAG)
	@docker push $(DOCKER_REGISTRY)/mission-management:$(DOCKER_TAG)
	@docker push $(DOCKER_REGISTRY)/dispatch-service:$(DOCKER_TAG)
	@docker push $(DOCKER_REGISTRY)/fleet-manager:$(DOCKER_TAG)
	@docker push $(DOCKER_REGISTRY)/control-center:$(DOCKER_TAG)

docker-scan: ## Scan Docker images for vulnerabilities
	@echo "$(BLUE)Scanning Docker images...$(RESET)"
	@docker scout cves $(DOCKER_REGISTRY)/api-gateway:$(DOCKER_TAG)
	@docker scout cves $(DOCKER_REGISTRY)/mission-management:$(DOCKER_TAG)

# Deployment Commands
deploy-dev: ## Deploy to development environment
	@echo "$(BLUE)Deploying to development environment...$(RESET)"
	@kubectl apply -f deployment/manifests/kubernetes/dev/ --namespace=atlasmesh-dev

deploy-staging: ## Deploy to staging environment
	@echo "$(BLUE)Deploying to staging environment...$(RESET)"
	@kubectl apply -f deployment/manifests/kubernetes/staging/ --namespace=atlasmesh-staging

deploy-prod: ## Deploy to production environment (requires approval)
	@echo "$(RED)Production deployment requires manual approval!$(RESET)"
	@echo "Run: kubectl apply -f deployment/manifests/kubernetes/prod/ --namespace=atlasmesh-prod"

# Database Commands
db-migrate: ## Run database migrations
	@echo "$(BLUE)Running database migrations...$(RESET)"
	@./scripts/db_migrate.sh

db-seed: ## Seed database with test data
	@echo "$(BLUE)Seeding database with test data...$(RESET)"
	@./scripts/db_seed.sh

db-reset: ## Reset database (development only)
	@echo "$(YELLOW)Resetting database...$(RESET)"
	@./scripts/db_reset.sh

# Utility Commands
clean: ## Clean build artifacts and dependencies
	@echo "$(BLUE)Cleaning build artifacts...$(RESET)"
	@rm -rf dist/
	@rm -rf node_modules/.cache/
	@rm -rf .nx/cache/
	@cd services && find . -name "*.exe" -delete
	@cd services && find . -type d -name "dist" -exec rm -rf {} +

clean-docker: ## Clean Docker images and containers
	@echo "$(BLUE)Cleaning Docker resources...$(RESET)"
	@docker system prune -f
	@docker image prune -f

format: ## Format all code
	@echo "$(BLUE)Formatting code...$(RESET)"
	@nx run-many --target=format --all --parallel=3
	@cd services && find . -name "*.go" -exec gofmt -w {} \;
	@black . --exclude="/(\.git|\.venv|node_modules)/"

validate-schemas: ## Validate all JSON schemas
	@echo "$(BLUE)Validating schemas...$(RESET)"
	@node tools/validate-schemas.js

validate-configs: ## Validate configuration files
	@echo "$(BLUE)Validating configurations...$(RESET)"
	@./scripts/validate_configs.sh

# Documentation Commands
docs-build: ## Build documentation
	@echo "$(BLUE)Building documentation...$(RESET)"
	@mkdocs build

docs-serve: ## Serve documentation locally
	@echo "$(BLUE)Serving documentation at http://localhost:8000$(RESET)"
	@mkdocs serve

docs-deploy: ## Deploy documentation
	@echo "$(BLUE)Deploying documentation...$(RESET)"
	@mkdocs gh-deploy

# Monitoring Commands
logs: ## Show logs from development environment
	@docker-compose -f deployment/manifests/docker-compose.dev.yml logs -f

logs-service: ## Show logs for specific service (usage: make logs-service SERVICE=api-gateway)
	@docker-compose -f deployment/manifests/docker-compose.dev.yml logs -f $(SERVICE)

status: ## Show status of all services
	@echo "$(BLUE)Service Status:$(RESET)"
	@docker-compose -f deployment/manifests/docker-compose.dev.yml ps

# Version Commands
version: ## Show version information
	@echo "$(BLUE)AtlasMesh Fleet OS Version Information:$(RESET)"
	@echo "Version: $(VERSION)"
	@echo "Commit: $(COMMIT_HASH)"
	@echo "Build Date: $(BUILD_DATE)"
	@echo "Go Version: $(GO_VERSION)"
	@echo "Node Version: $(NODE_VERSION)"
	@echo "Python Version: $(PYTHON_VERSION)"

version-bump: ## Bump version (usage: make version-bump TYPE=patch|minor|major)
	@echo "$(BLUE)Bumping version...$(RESET)"
	@npm version $(TYPE) --no-git-tag-version
	@git add package.json
	@git commit -m "chore: bump version to $(shell node -p "require('./package.json').version")"
	@git tag v$(shell node -p "require('./package.json').version")

# Release Commands
release-prepare: ## Prepare release (run tests, build, generate evidence)
	@echo "$(BLUE)Preparing release...$(RESET)"
	@make test
	@make build
	@make evidence
	@make docker-build

release-notes: ## Generate release notes
	@echo "$(BLUE)Generating release notes...$(RESET)"
	@./scripts/generate_release_notes.sh

# Edge Development Commands
.PHONY: edge-dev edge-dev-stop edge-dev-logs edge-simulation edge-provision edge-status edge-clean

edge-dev: ## Start edge development environment
	@echo "$(BLUE)Starting AtlasMesh Edge Development Environment...$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml up -d
	@echo "$(GREEN)Edge development environment started!$(RESET)"

edge-dev-stop: ## Stop edge development environment
	@echo "$(BLUE)Stopping AtlasMesh Edge Development Environment...$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml down
	@echo "$(GREEN)Edge development environment stopped$(RESET)"

edge-dev-logs: ## Show edge development logs
	@echo "$(BLUE)Showing Edge Development Logs...$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml logs -f

edge-simulation: ## Start edge environment with simulation
	@echo "$(BLUE)Starting Edge Environment with Simulation...$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml --profile simulation up -d
	@echo "$(GREEN)Edge simulation environment started!$(RESET)"

edge-provision: ## Provision a new vehicle
	@echo "$(BLUE)Provisioning Vehicle...$(RESET)"
	@read -p "Enter Vehicle ID: " VEHICLE_ID; \
	read -p "Enter Vehicle Type (ugv_themis): " VEHICLE_TYPE; \
	VEHICLE_TYPE=$${VEHICLE_TYPE:-ugv_themis}; \
	read -p "Enter Sector (logistics): " SECTOR; \
	SECTOR=$${SECTOR:-logistics}; \
	python3 tools/garage/vehicle_provisioner.py --vehicle-id $$VEHICLE_ID --vehicle-type $$VEHICLE_TYPE --sector $$SECTOR --verbose

edge-status: ## Show edge environment status
	@echo "$(BLUE)Edge Environment Status:$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml ps

edge-clean: ## Clean edge environment
	@echo "$(BLUE)Cleaning Edge Environment...$(RESET)"
	@cd edge && docker-compose -f docker-compose.edge.yml down -v --remove-orphans
	@docker system prune -f

# Security Commands
.PHONY: security-init security-vault security-opa security-certs security-verify security-policies

security-init: ## Initialize complete security infrastructure
	@echo "$(BLUE)Initializing AtlasMesh Security Infrastructure...$(RESET)"
	@chmod +x scripts/init-security.sh
	@./scripts/init-security.sh init
	@echo "$(GREEN)Security infrastructure initialized!$(RESET)"

security-vault: ## Initialize Vault secrets management
	@echo "$(BLUE)Initializing Vault...$(RESET)"
	@chmod +x scripts/init-security.sh
	@./scripts/init-security.sh vault-only
	@echo "$(GREEN)Vault initialized!$(RESET)"

security-opa: ## Load OPA authorization policies
	@echo "$(BLUE)Loading OPA Policies...$(RESET)"
	@chmod +x scripts/init-security.sh
	@./scripts/init-security.sh opa-only
	@echo "$(GREEN)OPA policies loaded!$(RESET)"

security-certs: ## Generate mTLS service certificates
	@echo "$(BLUE)Generating Service Certificates...$(RESET)"
	@chmod +x scripts/init-security.sh
	@./scripts/init-security.sh certs-only
	@echo "$(GREEN)Service certificates generated!$(RESET)"

security-verify: ## Verify security infrastructure setup
	@echo "$(BLUE)Verifying Security Setup...$(RESET)"
	@chmod +x scripts/init-security.sh
	@./scripts/init-security.sh verify
	@echo "$(GREEN)Security verification completed!$(RESET)"

security-policies: ## Test authorization policies
	@echo "$(BLUE)Testing Security Policies...$(RESET)"
	@curl -X POST http://localhost:8181/v1/data/atlasmesh/rbac/allow \
		-H "Content-Type: application/json" \
		-d '{"input": {"user": {"roles": ["admin"]}, "action": "read", "resource": {"type": "vehicle"}}}' | jq '.'
	@echo "$(GREEN)Policy test completed!$(RESET)"

# ROS2 Development Commands
.PHONY: ros2-build ros2-test ros2-launch

ros2-build: ## Build ROS2 packages
	@echo "$(BLUE)Building ROS2 Packages...$(RESET)"
	@cd edge/vehicle-agent && colcon build --symlink-install

ros2-test: ## Run ROS2 tests
	@echo "$(BLUE)Running ROS2 Tests...$(RESET)"
	@cd edge/vehicle-agent && colcon test

ros2-launch: ## Launch vehicle agent
	@echo "$(BLUE)Launching Vehicle Agent...$(RESET)"
	@read -p "Enter Vehicle ID (vehicle_001): " VEHICLE_ID; \
	VEHICLE_ID=$${VEHICLE_ID:-vehicle_001}; \
	read -p "Enter Vehicle Type (ugv_themis): " VEHICLE_TYPE; \
	VEHICLE_TYPE=$${VEHICLE_TYPE:-ugv_themis}; \
	cd edge/vehicle-agent && ros2 launch atlasmesh_vehicle_agent vehicle_agent.launch.py vehicle_id:=$$VEHICLE_ID vehicle_type:=$$VEHICLE_TYPE

# OTA Development Commands
.PHONY: ota-check ota-download ota-install

ota-check: ## Check for OTA updates
	@echo "$(BLUE)Checking for OTA Updates...$(RESET)"
	@curl -X POST http://localhost:8081/api/v1/ota/check

ota-download: ## Download OTA update
	@echo "$(BLUE)Downloading OTA Update...$(RESET)"
	@read -p "Enter Package ID: " PACKAGE_ID; \
	curl -X POST http://localhost:8081/api/v1/ota/download -d '{"package_id":"'$$PACKAGE_ID'"}'

ota-install: ## Install OTA update
	@echo "$(BLUE)Installing OTA Update...$(RESET)"
	@read -p "Enter Package ID: " PACKAGE_ID; \
	curl -X POST http://localhost:8081/api/v1/ota/install -d '{"package_id":"'$$PACKAGE_ID'"}'

# Environment-specific targets
.PHONY: sector-defense sector-mining sector-logistics sector-ridehail

sector-defense: ## Run with defense sector configuration
	@echo "$(BLUE)Starting with Defense sector configuration...$(RESET)"
	@SECTOR=defense ./scripts/run_local.sh

sector-mining: ## Run with mining sector configuration
	@echo "$(BLUE)Starting with Mining sector configuration...$(RESET)"
	@SECTOR=mining ./scripts/run_local.sh

sector-logistics: ## Run with logistics sector configuration
	@echo "$(BLUE)Starting with Logistics sector configuration...$(RESET)"
	@SECTOR=logistics ./scripts/run_local.sh

sector-ridehail: ## Run with ride-hail sector configuration
	@echo "$(BLUE)Starting with Ride-hail sector configuration...$(RESET)"
	@SECTOR=ridehail ./scripts/run_local.sh

# API Contract & SDK Management
.PHONY: api-contracts-validate api-contracts-validate-staging api-contracts-validate-prod
.PHONY: api-version-create api-version-deprecate api-version-status api-version-list
.PHONY: sdk-generate-go sdk-generate-typescript sdk-generate-python sdk-generate-all sdk-test sdk-publish

api-contracts-validate: ## Validate API contracts against development environment
	@echo "$(BLUE)Validating API contracts (development)...$(RESET)"
	@node tools/contract-testing/contract-validator.js development

api-contracts-validate-staging: ## Validate API contracts against staging environment
	@echo "$(BLUE)Validating API contracts (staging)...$(RESET)"
	@node tools/contract-testing/contract-validator.js staging

api-contracts-validate-prod: ## Validate API contracts against production environment
	@echo "$(BLUE)Validating API contracts (production)...$(RESET)"
	@node tools/contract-testing/contract-validator.js production

api-version-create: ## Create new API version (interactive)
	@echo "$(BLUE)Creating new API version...$(RESET)"
	@read -p "Service name: " service; \
	read -p "Version type (patch/minor/major): " type; \
	read -p "Description: " desc; \
	node tools/api-versioning/version-manager.js create $$service $$type "$$desc"

api-version-deprecate: ## Deprecate API version (interactive)
	@echo "$(YELLOW)Deprecating API version...$(RESET)"
	@read -p "Service name: " service; \
	read -p "Version to deprecate: " version; \
	read -p "Reason: " reason; \
	node tools/api-versioning/version-manager.js deprecate $$service $$version "$$reason"

api-version-status: ## Show API version status
	@echo "$(BLUE)API version status...$(RESET)"
	@node tools/api-versioning/version-manager.js status

api-version-list: ## List all API versions
	@echo "$(BLUE)Listing API versions...$(RESET)"
	@node tools/api-versioning/version-manager.js list

sdk-generate-go: ## Generate Go SDK from API contracts
	@echo "$(BLUE)Generating Go SDK...$(RESET)"
	@node tools/sdk-generator/generate-go-sdk.js

sdk-generate-typescript: ## Generate TypeScript SDK from API contracts
	@echo "$(BLUE)Generating TypeScript SDK...$(RESET)"
	@node tools/sdk-generator/generate-typescript-sdk.js

sdk-generate-python: ## Generate Python SDK from API contracts
	@echo "$(BLUE)Generating Python SDK...$(RESET)"
	@node tools/sdk-generator/generate-python-sdk.js

sdk-generate-all: ## Generate all SDKs (Go, TypeScript, Python)
	@echo "$(BLUE)Generating all SDKs...$(RESET)"
	@$(MAKE) sdk-generate-go
	@$(MAKE) sdk-generate-typescript
	@$(MAKE) sdk-generate-python
	@echo "$(GREEN)All SDKs generated successfully!$(RESET)"

sdk-test: ## Test all generated SDKs
	@echo "$(BLUE)Testing generated SDKs...$(RESET)"
	@if [ -d "sdks/go" ]; then cd sdks/go && make test; fi
	@if [ -d "sdks/typescript" ]; then cd sdks/typescript && npm test; fi
	@if [ -d "sdks/python" ]; then cd sdks/python && python -m pytest; fi
	@echo "$(GREEN)SDK tests completed!$(RESET)"

sdk-publish: ## Publish SDKs to package repositories
	@echo "$(BLUE)Publishing SDKs...$(RESET)"
	@echo "$(YELLOW)⚠️  This will publish to public repositories. Continue? [y/N]$(RESET)"
	@read -p "" confirm; \
	if [ "$$confirm" = "y" ] || [ "$$confirm" = "Y" ]; then \
		if [ -d "sdks/go" ]; then cd sdks/go && git tag v$$(grep "version:" go.mod | cut -d' ' -f2) && git push --tags; fi; \
		if [ -d "sdks/typescript" ]; then cd sdks/typescript && npm publish; fi; \
		if [ -d "sdks/python" ]; then cd sdks/python && python setup.py sdist bdist_wheel && twine upload dist/*; fi; \
		echo "$(GREEN)SDKs published successfully!$(RESET)"; \
	else \
		echo "$(YELLOW)SDK publishing cancelled.$(RESET)"; \
	fi

# Performance Budget Management
.PHONY: perf-budgets-enforce perf-budgets-enforce-staging perf-budgets-enforce-prod
.PHONY: perf-budgets-baseline perf-budgets-report perf-budgets-gate

perf-budgets-enforce: ## Enforce performance budgets (development)
	@echo "$(BLUE)Enforcing performance budgets (development)...$(RESET)"
	@node tools/performance-budgets/budget-enforcer.js development

perf-budgets-enforce-staging: ## Enforce performance budgets (staging)
	@echo "$(BLUE)Enforcing performance budgets (staging)...$(RESET)"
	@node tools/performance-budgets/budget-enforcer.js staging

perf-budgets-enforce-prod: ## Enforce performance budgets (production)
	@echo "$(BLUE)Enforcing performance budgets (production)...$(RESET)"
	@node tools/performance-budgets/budget-enforcer.js production

perf-budgets-baseline: ## Update performance baseline
	@echo "$(BLUE)Updating performance baseline...$(RESET)"
	@$(MAKE) perf-budgets-enforce
	@echo "$(GREEN)Performance baseline updated!$(RESET)"

perf-budgets-report: ## Generate performance budget report
	@echo "$(BLUE)Generating performance budget report...$(RESET)"
	@open tools/performance-budgets/reports/performance-budget-development-latest.html || \
	 xdg-open tools/performance-budgets/reports/performance-budget-development-latest.html || \
	 echo "Report available at: tools/performance-budgets/reports/performance-budget-development-latest.html"

perf-budgets-gate: ## Run performance gate (CI/CD)
	@echo "$(BLUE)Running performance gate...$(RESET)"
	@node ci/performance-gates/performance-gate.js

# Feature Flags Management
.PHONY: feature-flags-start feature-flags-stop feature-flags-logs feature-flags-test
.PHONY: feature-flags-create feature-flags-list feature-flags-evaluate feature-flags-kill

feature-flags-start: ## Start feature flags service
	@echo "$(BLUE)Starting feature flags service...$(RESET)"
	@docker-compose up -d feature-flags

feature-flags-stop: ## Stop feature flags service
	@echo "$(BLUE)Stopping feature flags service...$(RESET)"
	@docker-compose stop feature-flags

feature-flags-logs: ## Show feature flags service logs
	@echo "$(BLUE)Feature flags service logs:$(RESET)"
	@docker-compose logs -f feature-flags

feature-flags-test: ## Test feature flags service
	@echo "$(BLUE)Testing feature flags service...$(RESET)"
	@curl -f http://localhost:8095/health && echo "$(GREEN)Feature flags service is healthy!$(RESET)"

feature-flags-create: ## Create a new feature flag (interactive)
	@echo "$(BLUE)Creating new feature flag...$(RESET)"
	@read -p "Flag key: " flag_key; \
	read -p "Flag name: " flag_name; \
	read -p "Description: " description; \
	read -p "Enabled (true/false): " enabled; \
	curl -X POST http://localhost:8095/api/v1/flags \
		-H "Content-Type: application/json" \
		-d '{"key":"'$$flag_key'","name":"'$$flag_name'","description":"'$$description'","enabled":'$$enabled',"variations":[{"key":"on","name":"On","value":true},{"key":"off","name":"Off","value":false}]}'

feature-flags-list: ## List all feature flags
	@echo "$(BLUE)Listing feature flags...$(RESET)"
	@curl -s http://localhost:8095/api/v1/flags | jq '.'

feature-flags-evaluate: ## Evaluate a feature flag (interactive)
	@echo "$(BLUE)Evaluating feature flag...$(RESET)"
	@read -p "Flag key: " flag_key; \
	read -p "Entity ID: " entity_id; \
	read -p "Entity type (USER/VEHICLE/TENANT): " entity_type; \
	curl -X POST http://localhost:8095/api/v1/evaluate \
		-H "Content-Type: application/json" \
		-d '{"flag_key":"'$$flag_key'","context":{"entity_id":"'$$entity_id'","entity_type":"'$$entity_type'"},"default_value":false}' | jq '.'

feature-flags-kill: ## Activate kill switch for a flag (interactive)
	@echo "$(YELLOW)Activating kill switch...$(RESET)"
	@read -p "Flag key: " flag_key; \
	read -p "Reason: " reason; \
	read -p "Default variation key: " default_var; \
	curl -X POST http://localhost:8095/api/v1/flags/$$flag_key/kill \
		-H "Content-Type: application/json" \
		-d '{"reason":"'$$reason'","default_variation_key":"'$$default_var'"}'

# Accessibility & UX Gates
.PHONY: accessibility-validate accessibility-validate-staging accessibility-validate-prod
.PHONY: accessibility-report accessibility-gate

accessibility-validate: ## Run accessibility validation (development)
	@echo "$(BLUE)Running accessibility validation (development)...$(RESET)"
	@cd tools/accessibility-gates && node accessibility-validator.js development

accessibility-validate-staging: ## Run accessibility validation (staging)
	@echo "$(BLUE)Running accessibility validation (staging)...$(RESET)"
	@cd tools/accessibility-gates && node accessibility-validator.js staging

accessibility-validate-prod: ## Run accessibility validation (production)
	@echo "$(BLUE)Running accessibility validation (production)...$(RESET)"
	@cd tools/accessibility-gates && node accessibility-validator.js production

accessibility-report: ## Open accessibility report
	@echo "$(BLUE)Opening accessibility report...$(RESET)"
	@if [ -f tools/accessibility-gates/reports/accessibility-ux-development-latest.html ]; then \
		open tools/accessibility-gates/reports/accessibility-ux-development-latest.html || \
		xdg-open tools/accessibility-gates/reports/accessibility-ux-development-latest.html || \
		echo "Report available at: tools/accessibility-gates/reports/accessibility-ux-development-latest.html"; \
	else \
		echo "$(YELLOW)No accessibility report found. Run 'make accessibility-validate' first.$(RESET)"; \
	fi

accessibility-gate: ## Run accessibility CI gate
	@echo "$(BLUE)Running accessibility CI gate...$(RESET)"
	@cd tools/accessibility-gates && node accessibility-validator.js development && \
		echo "$(GREEN)✅ Accessibility gate passed$(RESET)" || \
		(echo "$(RED)❌ Accessibility gate failed$(RESET)" && exit 1)

# Edge OS Management
.PHONY: edge-os-build edge-os-run edge-os-stop edge-os-logs edge-os-shell edge-os-health

edge-os-build: ## Build hardened edge OS image
	@echo "$(BLUE)Building AtlasMesh Edge OS...$(RESET)"
	@docker build -f edge/os-baseline/Dockerfile.edge-os -t atlasmesh/edge-os:latest edge/os-baseline/
	@echo "$(GREEN)Edge OS image built successfully!$(RESET)"

edge-os-run: ## Run edge OS container
	@echo "$(BLUE)Starting AtlasMesh Edge OS...$(RESET)"
	@docker run -d \
		--name atlasmesh-edge-os \
		--privileged \
		--pid host \
		--network host \
		-v /dev:/dev \
		-v /sys/fs/cgroup:/sys/fs/cgroup:rw \
		-v /var/lib/atlasmesh:/var/lib/atlasmesh \
		-e VEHICLE_ID=$${VEHICLE_ID:-dev-vehicle-001} \
		-e SECTOR=$${SECTOR:-development} \
		-e ENVIRONMENT=$${ENVIRONMENT:-development} \
		atlasmesh/edge-os:latest
	@echo "$(GREEN)Edge OS container started!$(RESET)"

edge-os-stop: ## Stop edge OS container
	@echo "$(BLUE)Stopping AtlasMesh Edge OS...$(RESET)"
	@docker stop atlasmesh-edge-os || true
	@docker rm atlasmesh-edge-os || true
	@echo "$(GREEN)Edge OS container stopped!$(RESET)"

edge-os-logs: ## Show edge OS logs
	@echo "$(BLUE)Edge OS logs:$(RESET)"
	@docker logs -f atlasmesh-edge-os

edge-os-shell: ## Access edge OS shell
	@echo "$(BLUE)Accessing Edge OS shell...$(RESET)"
	@docker exec -it atlasmesh-edge-os /bin/bash

edge-os-health: ## Check edge OS health
	@echo "$(BLUE)Checking Edge OS health...$(RESET)"
	@docker exec atlasmesh-edge-os /usr/local/bin/health-check.sh

edge-os-restart: ## Restart edge OS container
	@echo "$(BLUE)Restarting AtlasMesh Edge OS...$(RESET)"
	@$(MAKE) edge-os-stop
	@$(MAKE) edge-os-run

edge-os-clean: ## Clean edge OS containers and images
	@echo "$(BLUE)Cleaning Edge OS containers and images...$(RESET)"
	@docker stop atlasmesh-edge-os || true
	@docker rm atlasmesh-edge-os || true
	@docker rmi atlasmesh/edge-os:latest || true
	@echo "$(GREEN)Edge OS cleanup completed!$(RESET)"