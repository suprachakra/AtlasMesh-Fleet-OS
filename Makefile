# AtlasMesh Fleet OS - Development Makefile

.PHONY: help dev build test lint clean setup validate

# Default target
help: ## Show this help message
	@echo "AtlasMesh Fleet OS - Available Commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

setup: ## Setup development environment
	@echo "🚀 Setting up AtlasMesh Fleet OS development environment..."
	@chmod +x scripts/setup_dev.sh
	@./scripts/setup_dev.sh

dev: ## Start development environment
	@echo "🔧 Starting development environment..."
	@docker-compose -f deploy/docker-compose.dev.yml up -d
	@echo "✅ Development environment started"
	@echo "   - Fleet Ops UI: http://localhost:3000"
	@echo "   - Control Center: http://localhost:3001" 
	@echo "   - API Gateway: http://localhost:8080"
	@echo "   - Simulation: http://localhost:8090"

build: ## Build all services and applications
	@echo "🏗️ Building all services..."
	@nx run-many --target=build --all --parallel=3

test: ## Run all tests
	@echo "🧪 Running tests..."
	@nx run-many --target=test --all --parallel=3

test-e2e: ## Run end-to-end tests
	@echo "🎯 Running E2E tests..."
	@nx run-many --target=e2e --all

test-acceptance: ## Run sector acceptance tests
	@echo "✅ Running acceptance tests..."
	@npm run test -- tests/e2e/acceptance/

lint: ## Run linting on all code
	@echo "🔍 Linting code..."
	@nx run-many --target=lint --all

validate: ## Validate schemas and compatibility
	@echo "🔍 Validating schemas and compatibility..."
	@npm run validate:schemas
	@npm run validate:compat

simulate: ## Run digital twin simulation
	@echo "🎮 Starting digital twin simulation..."
	@docker-compose -f deploy/docker-compose.sim.yml up

clean: ## Clean build artifacts and containers
	@echo "🧹 Cleaning up..."
	@docker-compose -f deploy/docker-compose.dev.yml down -v
	@docker-compose -f deploy/docker-compose.sim.yml down -v
	@nx reset
	@rm -rf dist/ node_modules/.cache/

stop: ## Stop all services
	@echo "⏹️ Stopping all services..."
	@docker-compose -f deploy/docker-compose.dev.yml down
	@docker-compose -f deploy/docker-compose.sim.yml down

logs: ## Show logs from all services
	@docker-compose -f deploy/docker-compose.dev.yml logs -f

# Safety and compliance targets
safety-check: ## Run safety case validation
	@echo "🛡️ Running safety case validation..."
	@./tools/safety/validate_safety_case.sh

compliance-check: ## Generate compliance reports
	@echo "📋 Generating compliance reports..."
	@./tools/compliance/generate_reports.sh

audit-bundle: ## Generate audit bundle for release
	@echo "📦 Generating audit bundle..."
	@./tools/release/generate_audit_bundle.sh

# Deployment targets
deploy-staging: ## Deploy to staging environment
	@echo "🚀 Deploying to staging..."
	@kubectl apply -k deploy/kustomize/overlays/staging/

deploy-prod: ## Deploy to production environment
	@echo "🚀 Deploying to production..."
	@kubectl apply -k deploy/kustomize/overlays/production/

# Documentation
docs-serve: ## Serve documentation locally
	@echo "📚 Serving documentation..."
	@mkdocs serve

docs-build: ## Build documentation site
	@echo "📚 Building documentation..."
	@mkdocs build
