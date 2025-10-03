# AtlasMesh Control Center - Multi-stage Docker build
# Optimized for production deployment with security and performance

# Build stage
FROM node:18-alpine AS builder

# Set working directory
WORKDIR /app

# Install build dependencies
RUN apk add --no-cache \
    python3 \
    make \
    g++ \
    git

# Copy package files
COPY package*.json ./
COPY tsconfig*.json ./
COPY vite.config.ts ./
COPY tailwind.config.js ./
COPY postcss.config.js ./

# Install dependencies
RUN npm ci --only=production --silent

# Copy source code
COPY src/ ./src/
COPY public/ ./public/
COPY index.html ./

# Set build environment variables
ARG VITE_API_URL=http://localhost:8080
ARG VITE_WS_URL=ws://localhost:8080
ARG VITE_APP_VERSION=1.0.0
ARG VITE_COMMIT_HASH=unknown
ARG VITE_BUILD_DATE
ENV VITE_API_URL=$VITE_API_URL
ENV VITE_WS_URL=$VITE_WS_URL
ENV VITE_APP_VERSION=$VITE_APP_VERSION
ENV VITE_COMMIT_HASH=$VITE_COMMIT_HASH
ENV VITE_BUILD_DATE=$VITE_BUILD_DATE

# Build the application
RUN npm run build

# Production stage
FROM nginx:1.25-alpine AS production

# Install security updates
RUN apk upgrade --no-cache

# Create non-root user
RUN addgroup -g 1001 -S atlasmesh && \
    adduser -S atlasmesh -u 1001 -G atlasmesh

# Copy custom nginx configuration
COPY nginx.conf /etc/nginx/nginx.conf
COPY nginx-default.conf /etc/nginx/conf.d/default.conf

# Copy built application from builder stage
COPY --from=builder /app/dist /usr/share/nginx/html

# Copy health check script
COPY healthcheck.sh /usr/local/bin/healthcheck.sh
RUN chmod +x /usr/local/bin/healthcheck.sh

# Set ownership and permissions
RUN chown -R atlasmesh:atlasmesh /usr/share/nginx/html && \
    chown -R atlasmesh:atlasmesh /var/cache/nginx && \
    chown -R atlasmesh:atlasmesh /var/log/nginx && \
    chown -R atlasmesh:atlasmesh /etc/nginx/conf.d

# Create nginx PID directory
RUN mkdir -p /var/run/nginx && \
    chown -R atlasmesh:atlasmesh /var/run/nginx

# Switch to non-root user
USER atlasmesh

# Expose port
EXPOSE 3000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD /usr/local/bin/healthcheck.sh

# Labels for metadata
LABEL maintainer="AtlasMesh Engineering <engineering@atlasmesh.com>"
LABEL description="AtlasMesh Fleet OS Control Center"
LABEL version="1.0.0"
LABEL org.opencontainers.image.title="AtlasMesh Control Center"
LABEL org.opencontainers.image.description="Web-based control center for AtlasMesh Fleet OS"
LABEL org.opencontainers.image.vendor="AtlasMesh"
LABEL org.opencontainers.image.licenses="Proprietary"
LABEL org.opencontainers.image.source="https://github.com/atlasmesh/fleet-os"

# Start nginx
CMD ["nginx", "-g", "daemon off;"]
