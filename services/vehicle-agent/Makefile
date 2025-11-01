# Makefile for vehicle-agent service

# Variables
SERVICE_NAME := vehicle-agent
DOCKER_IMAGE := atlasmesh/$(SERVICE_NAME)
VERSION := latest
GO_VERSION := 1.21

# Default target
.PHONY: all
all: build test

# Build the service
.PHONY: build
build:
	@echo "Building $(SERVICE_NAME)..."
	go build -o bin/$(SERVICE_NAME) ./cmd/main.go

# Run the service
.PHONY: run
run:
	@echo "Running $(SERVICE_NAME)..."
	go run ./cmd/main.go

# Test the service
.PHONY: test
test:
	@echo "Testing $(SERVICE_NAME)..."
	go test -v ./...

# Test with coverage
.PHONY: test-coverage
test-coverage:
	@echo "Testing $(SERVICE_NAME) with coverage..."
	go test -v -coverprofile=coverage.out ./...
	go tool cover -html=coverage.out -o coverage.html

# Benchmark tests
.PHONY: benchmark
benchmark:
	@echo "Running benchmarks for $(SERVICE_NAME)..."
	go test -bench=. -benchmem ./...

# Lint the code
.PHONY: lint
lint:
	@echo "Linting $(SERVICE_NAME)..."
	golangci-lint run

# Format the code
.PHONY: fmt
fmt:
	@echo "Formatting $(SERVICE_NAME)..."
	go fmt ./...

# Tidy dependencies
.PHONY: tidy
tidy:
	@echo "Tidying dependencies for $(SERVICE_NAME)..."
	go mod tidy

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning $(SERVICE_NAME)..."
	rm -rf bin/
	rm -f coverage.out coverage.html

# Docker build
.PHONY: docker-build
docker-build:
	@echo "Building Docker image for $(SERVICE_NAME)..."
	docker build -t $(DOCKER_IMAGE):$(VERSION) .

# Docker run
.PHONY: docker-run
docker-run:
	@echo "Running Docker container for $(SERVICE_NAME)..."
	docker run --rm -p 8080:8080 $(DOCKER_IMAGE):$(VERSION)

# Docker push
.PHONY: docker-push
docker-push:
	@echo "Pushing Docker image for $(SERVICE_NAME)..."
	docker push $(DOCKER_IMAGE):$(VERSION)

# Kubernetes deploy
.PHONY: k8s-deploy
k8s-deploy:
	@echo "Deploying $(SERVICE_NAME) to Kubernetes..."
	kubectl apply -f k8s/

# Kubernetes delete
.PHONY: k8s-delete
k8s-delete:
	@echo "Deleting $(SERVICE_NAME) from Kubernetes..."
	kubectl delete -f k8s/

# Development setup
.PHONY: dev-setup
dev-setup:
	@echo "Setting up development environment for $(SERVICE_NAME)..."
	go mod download
	go install github.com/golangci/golangci-lint/cmd/golangci-lint@latest

# Install dependencies
.PHONY: deps
deps:
	@echo "Installing dependencies for $(SERVICE_NAME)..."
	go mod download

# Generate mocks
.PHONY: generate-mocks
generate-mocks:
	@echo "Generating mocks for $(SERVICE_NAME)..."
	go generate ./...

# Security scan
.PHONY: security-scan
security-scan:
	@echo "Running security scan for $(SERVICE_NAME)..."
	gosec ./...

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build          - Build the service"
	@echo "  run            - Run the service"
	@echo "  test           - Run tests"
	@echo "  test-coverage  - Run tests with coverage"
	@echo "  benchmark      - Run benchmark tests"
	@echo "  lint           - Lint the code"
	@echo "  fmt            - Format the code"
	@echo "  tidy           - Tidy dependencies"
	@echo "  clean          - Clean build artifacts"
	@echo "  docker-build   - Build Docker image"
	@echo "  docker-run     - Run Docker container"
	@echo "  docker-push    - Push Docker image"
	@echo "  k8s-deploy     - Deploy to Kubernetes"
	@echo "  k8s-delete     - Delete from Kubernetes"
	@echo "  dev-setup      - Setup development environment"
	@echo "  deps           - Install dependencies"
	@echo "  generate-mocks - Generate mocks"
	@echo "  security-scan  - Run security scan"
	@echo "  help           - Show this help"
