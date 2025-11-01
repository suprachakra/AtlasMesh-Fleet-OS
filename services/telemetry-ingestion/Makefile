# Makefile for Telemetry Ingestion Service

# Variables
SERVICE_NAME := telemetry-ingestion
VERSION := 1.0.0
DOCKER_IMAGE := $(SERVICE_NAME):$(VERSION)
DOCKER_REGISTRY := registry.atlasmesh.ae
FULL_IMAGE := $(DOCKER_REGISTRY)/$(DOCKER_IMAGE)

# Go variables
GO := go
GOFMT := gofmt
GOLINT := golangci-lint
GOTEST := go test
GOCOVER := go test -cover

# Build variables
BUILD_DIR := build
BINARY_NAME := $(SERVICE_NAME)
BINARY_PATH := $(BUILD_DIR)/$(BINARY_NAME)

# Docker variables
DOCKER := docker
DOCKER_BUILD := $(DOCKER) build
DOCKER_RUN := $(DOCKER) run
DOCKER_PUSH := $(DOCKER) push
DOCKER_TAG := $(DOCKER) tag

# Kubernetes variables
KUBECTL := kubectl
K8S_DIR := k8s
K8S_NAMESPACE := atlasmesh

# Default target
.PHONY: all
all: build test

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build          - Build the service binary"
	@echo "  test           - Run unit tests"
	@echo "  test-coverage  - Run tests with coverage"
	@echo "  lint           - Run linter"
	@echo "  fmt            - Format code"
	@echo "  clean          - Clean build artifacts"
	@echo "  docker-build   - Build Docker image"
	@echo "  docker-run     - Run Docker container"
	@echo "  docker-push    - Push Docker image"
	@echo "  k8s-deploy     - Deploy to Kubernetes"
	@echo "  k8s-delete     - Delete from Kubernetes"
	@echo "  dev-deps       - Start development dependencies"
	@echo "  dev-stop       - Stop development dependencies"
	@echo "  run            - Run the service locally"

# Build target
.PHONY: build
build:
	@echo "Building $(SERVICE_NAME)..."
	@mkdir -p $(BUILD_DIR)
	@$(GO) build -o $(BINARY_PATH) ./cmd/main.go
	@echo "Build complete: $(BINARY_PATH)"

# Test targets
.PHONY: test
test:
	@echo "Running tests..."
	@$(GOTEST) -v ./...

.PHONY: test-coverage
test-coverage:
	@echo "Running tests with coverage..."
	@$(GOCOVER) -v ./...

.PHONY: test-integration
test-integration:
	@echo "Running integration tests..."
	@$(GOTEST) -v -tags=integration ./...

.PHONY: test-load
test-load:
	@echo "Running load tests..."
	@$(GOTEST) -v -tags=load ./...

# Lint and format targets
.PHONY: lint
lint:
	@echo "Running linter..."
	@$(GOLINT) run

.PHONY: fmt
fmt:
	@echo "Formatting code..."
	@$(GOFMT) -w .

.PHONY: fmt-check
fmt-check:
	@echo "Checking code formatting..."
	@$(GOFMT) -d .

# Clean target
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@$(GO) clean

# Docker targets
.PHONY: docker-build
docker-build:
	@echo "Building Docker image..."
	@$(DOCKER_BUILD) -t $(DOCKER_IMAGE) .
	@echo "Docker image built: $(DOCKER_IMAGE)"

.PHONY: docker-run
docker-run:
	@echo "Running Docker container..."
	@$(DOCKER_RUN) -p 8080:8080 --rm $(DOCKER_IMAGE)

.PHONY: docker-push
docker-push:
	@echo "Pushing Docker image..."
	@$(DOCKER_TAG) $(DOCKER_IMAGE) $(FULL_IMAGE)
	@$(DOCKER_PUSH) $(FULL_IMAGE)
	@echo "Docker image pushed: $(FULL_IMAGE)"

# Kubernetes targets
.PHONY: k8s-deploy
k8s-deploy:
	@echo "Deploying to Kubernetes..."
	@$(KUBECTL) apply -f $(K8S_DIR)/ -n $(K8S_NAMESPACE)
	@echo "Deployment complete"

.PHONY: k8s-delete
k8s-delete:
	@echo "Deleting from Kubernetes..."
	@$(KUBECTL) delete -f $(K8S_DIR)/ -n $(K8S_NAMESPACE)
	@echo "Deletion complete"

.PHONY: k8s-status
k8s-status:
	@echo "Checking Kubernetes status..."
	@$(KUBECTL) get pods -l app=$(SERVICE_NAME) -n $(K8S_NAMESPACE)

# Development targets
.PHONY: dev-deps
dev-deps:
	@echo "Starting development dependencies..."
	@docker-compose -f docker-compose.dev.yml up -d
	@echo "Development dependencies started"

.PHONY: dev-stop
dev-stop:
	@echo "Stopping development dependencies..."
	@docker-compose -f docker-compose.dev.yml down
	@echo "Development dependencies stopped"

.PHONY: dev-logs
dev-logs:
	@echo "Showing development logs..."
	@docker-compose -f docker-compose.dev.yml logs -f

# Run target
.PHONY: run
run:
	@echo "Running $(SERVICE_NAME) locally..."
	@$(GO) run ./cmd/main.go

# Install target
.PHONY: install
install: build
	@echo "Installing $(SERVICE_NAME)..."
	@cp $(BINARY_PATH) /usr/local/bin/$(BINARY_NAME)
	@echo "Installation complete"

# Uninstall target
.PHONY: uninstall
uninstall:
	@echo "Uninstalling $(SERVICE_NAME)..."
	@rm -f /usr/local/bin/$(BINARY_NAME)
	@echo "Uninstallation complete"

# Security targets
.PHONY: security-scan
security-scan:
	@echo "Running security scan..."
	@$(GO) list -json -deps ./... | nancy sleuth

.PHONY: security-audit
security-audit:
	@echo "Running security audit..."
	@$(GO) list -json -deps ./... | nancy sleuth

# Dependencies target
.PHONY: deps
deps:
	@echo "Installing dependencies..."
	@$(GO) mod download
	@$(GO) mod tidy

# Update dependencies target
.PHONY: deps-update
deps-update:
	@echo "Updating dependencies..."
	@$(GO) get -u ./...
	@$(GO) mod tidy

# Generate targets
.PHONY: generate
generate:
	@echo "Generating code..."
	@$(GO) generate ./...

# Benchmark target
.PHONY: benchmark
benchmark:
	@echo "Running benchmarks..."
	@$(GOTEST) -bench=. -benchmem ./...

# Profile target
.PHONY: profile
profile:
	@echo "Running profiling..."
	@$(GOTEST) -cpuprofile=cpu.prof -memprofile=mem.prof ./...

# Documentation target
.PHONY: docs
docs:
	@echo "Generating documentation..."
	@$(GO) doc -all ./...

# Version target
.PHONY: version
version:
	@echo "$(SERVICE_NAME) version $(VERSION)"

# Status target
.PHONY: status
status:
	@echo "Service: $(SERVICE_NAME)"
	@echo "Version: $(VERSION)"
	@echo "Binary: $(BINARY_PATH)"
	@echo "Docker Image: $(DOCKER_IMAGE)"
	@echo "Full Image: $(FULL_IMAGE)"

# All-in-one targets
.PHONY: ci
ci: fmt-check lint test build

.PHONY: release
release: clean test build docker-build docker-push

.PHONY: dev
dev: dev-deps run

.PHONY: prod
prod: build docker-build docker-push k8s-deploy
