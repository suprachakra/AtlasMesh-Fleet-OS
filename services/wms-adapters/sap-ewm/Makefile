# SAP EWM Adapter Makefile

# Variables
SERVICE_NAME := sap-ewm-adapter
DOCKER_IMAGE := atlasmesh/$(SERVICE_NAME)
VERSION := latest
GO_VERSION := 1.21
BINARY_NAME := main

# Build variables
BUILD_DIR := build
DIST_DIR := dist

# Docker variables
DOCKER_REGISTRY := localhost:5000
DOCKER_TAG := $(VERSION)

# Go variables
GOOS := linux
GOARCH := amd64
CGO_ENABLED := 0

# Default target
.PHONY: all
all: clean build test

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR) $(DIST_DIR)
	@go clean -cache

# Install dependencies
.PHONY: deps
deps:
	@echo "Installing dependencies..."
	@go mod download
	@go mod tidy

# Run tests
.PHONY: test
test:
	@echo "Running tests..."
	@go test -v -race -coverprofile=coverage.out ./...

# Run tests with coverage
.PHONY: test-coverage
test-coverage: test
	@echo "Generating coverage report..."
	@go tool cover -html=coverage.out -o coverage.html
	@echo "Coverage report generated: coverage.html"

# Build the application
.PHONY: build
build: deps
	@echo "Building $(SERVICE_NAME)..."
	@mkdir -p $(BUILD_DIR)
	@CGO_ENABLED=$(CGO_ENABLED) GOOS=$(GOOS) GOARCH=$(GOARCH) go build \
		-ldflags="-w -s" \
		-o $(BUILD_DIR)/$(BINARY_NAME) \
		./cmd/main.go

# Build for multiple platforms
.PHONY: build-all
build-all: deps
	@echo "Building for multiple platforms..."
	@mkdir -p $(DIST_DIR)
	@for os in linux darwin windows; do \
		for arch in amd64 arm64; do \
			echo "Building for $$os/$$arch..."; \
			CGO_ENABLED=0 GOOS=$$os GOARCH=$$arch go build \
				-ldflags="-w -s" \
				-o $(DIST_DIR)/$(BINARY_NAME)-$$os-$$arch \
				./cmd/main.go; \
		done; \
	done

# Run the application locally
.PHONY: run
run: build
	@echo "Running $(SERVICE_NAME)..."
	@cd $(BUILD_DIR) && ./$(BINARY_NAME)

# Run with hot reload (requires air)
.PHONY: dev
dev:
	@echo "Running in development mode..."
	@air

# Lint the code
.PHONY: lint
lint:
	@echo "Linting code..."
	@golangci-lint run

# Format the code
.PHONY: fmt
fmt:
	@echo "Formatting code..."
	@go fmt ./...
	@goimports -w .

# Security scan
.PHONY: security
security:
	@echo "Running security scan..."
	@gosec ./...

# Build Docker image
.PHONY: docker-build
docker-build:
	@echo "Building Docker image..."
	@docker build -t $(DOCKER_IMAGE):$(DOCKER_TAG) .

# Build Docker image for multiple platforms
.PHONY: docker-buildx
docker-buildx:
	@echo "Building multi-platform Docker image..."
	@docker buildx build --platform linux/amd64,linux/arm64 \
		-t $(DOCKER_IMAGE):$(DOCKER_TAG) \
		--push .

# Run Docker container
.PHONY: docker-run
docker-run: docker-build
	@echo "Running Docker container..."
	@docker run --rm -p 8080:8080 \
		-e DB_HOST=host.docker.internal \
		-e SAP_EWM_HOST=host.docker.internal \
		$(DOCKER_IMAGE):$(DOCKER_TAG)

# Run with Docker Compose
.PHONY: docker-compose-up
docker-compose-up:
	@echo "Starting services with Docker Compose..."
	@docker-compose up -d

# Stop Docker Compose services
.PHONY: docker-compose-down
docker-compose-down:
	@echo "Stopping Docker Compose services..."
	@docker-compose down

# View Docker Compose logs
.PHONY: docker-compose-logs
docker-compose-logs:
	@echo "Viewing Docker Compose logs..."
	@docker-compose logs -f

# Deploy to Kubernetes
.PHONY: k8s-deploy
k8s-deploy:
	@echo "Deploying to Kubernetes..."
	@kubectl apply -f k8s/

# Delete from Kubernetes
.PHONY: k8s-delete
k8s-delete:
	@echo "Deleting from Kubernetes..."
	@kubectl delete -f k8s/

# View Kubernetes logs
.PHONY: k8s-logs
k8s-logs:
	@echo "Viewing Kubernetes logs..."
	@kubectl logs -f deployment/$(SERVICE_NAME) -n atlasmesh

# Database migration
.PHONY: db-migrate
db-migrate:
	@echo "Running database migration..."
	@docker-compose exec postgres psql -U atlasmesh -d atlasmesh_ewm -f /docker-entrypoint-initdb.d/001_create_tasks_table.sql

# Database reset
.PHONY: db-reset
db-reset:
	@echo "Resetting database..."
	@docker-compose down -v
	@docker-compose up -d postgres
	@sleep 10
	@make db-migrate

# Generate API documentation
.PHONY: docs
docs:
	@echo "Generating API documentation..."
	@swag init -g cmd/main.go -o docs/

# Install development tools
.PHONY: install-tools
install-tools:
	@echo "Installing development tools..."
	@go install github.com/cosmtrek/air@latest
	@go install github.com/golangci/golangci-lint/cmd/golangci-lint@latest
	@go install github.com/securecodewarrior/gosec/v2/cmd/gosec@latest
	@go install golang.org/x/tools/cmd/goimports@latest
	@go install github.com/swaggo/swag/cmd/swag@latest

# Check if tools are installed
.PHONY: check-tools
check-tools:
	@echo "Checking development tools..."
	@which air || echo "air not installed"
	@which golangci-lint || echo "golangci-lint not installed"
	@which gosec || echo "gosec not installed"
	@which goimports || echo "goimports not installed"
	@which swag || echo "swag not installed"

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all              - Clean, build, and test"
	@echo "  clean            - Clean build artifacts"
	@echo "  deps             - Install dependencies"
	@echo "  test             - Run tests"
	@echo "  test-coverage    - Run tests with coverage report"
	@echo "  build            - Build the application"
	@echo "  build-all        - Build for multiple platforms"
	@echo "  run              - Run the application locally"
	@echo "  dev              - Run with hot reload"
	@echo "  lint             - Lint the code"
	@echo "  fmt              - Format the code"
	@echo "  security         - Run security scan"
	@echo "  docker-build     - Build Docker image"
	@echo "  docker-buildx    - Build multi-platform Docker image"
	@echo "  docker-run       - Run Docker container"
	@echo "  docker-compose-up    - Start services with Docker Compose"
	@echo "  docker-compose-down  - Stop Docker Compose services"
	@echo "  docker-compose-logs  - View Docker Compose logs"
	@echo "  k8s-deploy       - Deploy to Kubernetes"
	@echo "  k8s-delete       - Delete from Kubernetes"
	@echo "  k8s-logs         - View Kubernetes logs"
	@echo "  db-migrate       - Run database migration"
	@echo "  db-reset         - Reset database"
	@echo "  docs             - Generate API documentation"
	@echo "  install-tools    - Install development tools"
	@echo "  check-tools      - Check if tools are installed"
	@echo "  help             - Show this help message"
