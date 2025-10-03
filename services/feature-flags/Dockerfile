# Build stage
FROM golang:1.21-alpine AS builder

# Install build dependencies
RUN apk add --no-cache git ca-certificates tzdata

# Set working directory
WORKDIR /app

# Copy go mod files
COPY go.mod go.sum ./

# Download dependencies
RUN go mod download

# Copy source code
COPY . .

# Build the application
RUN CGO_ENABLED=0 GOOS=linux go build -a -installsuffix cgo -o feature-flags ./cmd/main.go

# Final stage
FROM alpine:3.18

# Install runtime dependencies
RUN apk --no-cache add ca-certificates curl

# Create non-root user
RUN addgroup -g 1001 -S atlasmesh && \
    adduser -u 1001 -S atlasmesh -G atlasmesh

# Set working directory
WORKDIR /app

# Copy binary from builder stage
COPY --from=builder /app/feature-flags .

# Change ownership
RUN chown -R atlasmesh:atlasmesh /app

# Switch to non-root user
USER atlasmesh

# Expose ports
EXPOSE 8080 9090

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1

# Run the application
CMD ["./feature-flags"]
