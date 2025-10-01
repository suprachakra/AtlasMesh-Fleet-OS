package middleware

import (
	"context"
	"crypto/subtle"
	"fmt"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/golang-jwt/jwt/v5"
	"go.uber.org/zap"
	"golang.org/x/time/rate"

	"github.com/atlasmesh/fleet-os/services/auth-service/internal/config"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/metrics"
)

// SecurityMiddleware provides comprehensive security middleware functions
// SECURITY: Implements defense-in-depth with multiple security layers
// CONCURRENCY: All middleware functions are goroutine-safe
// PERF: Rate limiting uses token bucket algorithm for efficient request throttling
type SecurityMiddleware struct {
	logger  *zap.Logger        // Structured logging for security events
	config  *config.Config     // Security configuration and policies
	metrics *metrics.Collector // Security metrics for monitoring and alerting
}

// NewSecurityMiddleware creates a new security middleware instance
// SECURITY: Initializes security policies and rate limiters
func NewSecurityMiddleware(logger *zap.Logger, config *config.Config, metrics *metrics.Collector) *SecurityMiddleware {
	return &SecurityMiddleware{
		logger:  logger,
		config:  config,
		metrics: metrics,
	}
}

// CORS middleware with strict security settings
// SECURITY: Implements strict CORS policy to prevent cross-origin attacks
// INTEGRATION CONTRACT: Whitelist of allowed origins for production deployment
func CORS() gin.HandlerFunc {
	return func(c *gin.Context) {
		origin := c.Request.Header.Get("Origin")
		
		// In production, maintain a whitelist of allowed origins
		allowedOrigins := []string{
			"https://control-center.atlasmesh.com",
			"https://dashboard.atlasmesh.com",
			"http://localhost:3000", // Development only
		}
		
		originAllowed := false
		for _, allowed := range allowedOrigins {
			if origin == allowed {
				originAllowed = true
				break
			}
		}
		
		if originAllowed {
			c.Header("Access-Control-Allow-Origin", origin)
		}
		
		c.Header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		c.Header("Access-Control-Allow-Headers", "Origin, Content-Type, Accept, Authorization, X-Requested-With, X-Request-ID")
		c.Header("Access-Control-Allow-Credentials", "true")
		c.Header("Access-Control-Max-Age", "86400") // 24 hours
		
		if c.Request.Method == "OPTIONS" {
			c.AbortWithStatus(http.StatusNoContent)
			return
		}
		
		c.Next()
	}
}

// SecurityHeaders adds comprehensive security headers
func SecurityHeaders() gin.HandlerFunc {
	return func(c *gin.Context) {
		// Prevent MIME type sniffing
		c.Header("X-Content-Type-Options", "nosniff")
		
		// Prevent clickjacking
		c.Header("X-Frame-Options", "DENY")
		
		// Enable XSS protection
		c.Header("X-XSS-Protection", "1; mode=block")
		
		// Strict Transport Security (HTTPS only)
		c.Header("Strict-Transport-Security", "max-age=31536000; includeSubDomains; preload")
		
		// Content Security Policy
		csp := "default-src 'self'; " +
			"script-src 'self' 'unsafe-inline' https://cdn.jsdelivr.net; " +
			"style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; " +
			"font-src 'self' https://fonts.gstatic.com; " +
			"img-src 'self' data: https:; " +
			"connect-src 'self' wss: https:; " +
			"frame-ancestors 'none'; " +
			"base-uri 'self'; " +
			"form-action 'self'"
		c.Header("Content-Security-Policy", csp)
		
		// Referrer Policy
		c.Header("Referrer-Policy", "strict-origin-when-cross-origin")
		
		// Permissions Policy
		permissions := "camera=(), microphone=(), geolocation=(), payment=(), usb=(), magnetometer=(), gyroscope=(), accelerometer=()"
		c.Header("Permissions-Policy", permissions)
		
		// Remove server information
		c.Header("Server", "")
		
		c.Next()
	}
}

// RateLimiter implements rate limiting with different tiers
func RateLimiter(config config.RateLimitConfig) gin.HandlerFunc {
	// Create different limiters for different endpoints
	limiters := map[string]*rate.Limiter{
		"login":    rate.NewLimiter(rate.Every(time.Minute/time.Duration(config.LoginRPS)), config.LoginBurst),
		"api":      rate.NewLimiter(rate.Every(time.Second/time.Duration(config.APIRPS)), config.APIBurst),
		"upload":   rate.NewLimiter(rate.Every(time.Minute/time.Duration(config.UploadRPS)), config.UploadBurst),
		"download": rate.NewLimiter(rate.Every(time.Second/time.Duration(config.DownloadRPS)), config.DownloadBurst),
	}
	
	return func(c *gin.Context) {
		// Determine rate limit tier based on endpoint
		var limiter *rate.Limiter
		path := c.Request.URL.Path
		
		switch {
		case strings.Contains(path, "/auth/login"):
			limiter = limiters["login"]
		case strings.Contains(path, "/upload"):
			limiter = limiters["upload"]
		case strings.Contains(path, "/download"):
			limiter = limiters["download"]
		default:
			limiter = limiters["api"]
		}
		
		// Check rate limit
		if !limiter.Allow() {
			c.Header("X-RateLimit-Limit", fmt.Sprintf("%.0f", limiter.Limit()))
			c.Header("X-RateLimit-Remaining", "0")
			c.Header("X-RateLimit-Reset", fmt.Sprintf("%d", time.Now().Add(time.Minute).Unix()))
			
			c.JSON(http.StatusTooManyRequests, gin.H{
				"error": "Rate limit exceeded",
				"retry_after": 60,
			})
			c.Abort()
			return
		}
		
		c.Next()
	}
}

// RequestID generates and adds request ID for tracing
func RequestID() gin.HandlerFunc {
	return func(c *gin.Context) {
		requestID := c.GetHeader("X-Request-ID")
		if requestID == "" {
			requestID = generateRequestID()
		}
		
		c.Header("X-Request-ID", requestID)
		c.Set("request_id", requestID)
		
		c.Next()
	}
}

// AuditLogging logs security-relevant events
func AuditLogging(logger *zap.Logger) gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()
		
		// Capture request information
		requestID := c.GetString("request_id")
		clientIP := c.ClientIP()
		userAgent := c.GetHeader("User-Agent")
		method := c.Request.Method
		path := c.Request.URL.Path
		
		c.Next()
		
		// Log after request completion
		duration := time.Since(start)
		statusCode := c.Writer.Status()
		
		// Determine if this is a security-relevant event
		isSecurityEvent := isSecurityRelevantPath(path) || 
			statusCode == http.StatusUnauthorized ||
			statusCode == http.StatusForbidden ||
			statusCode >= http.StatusInternalServerError
		
		if isSecurityEvent {
			logger.Info("Security audit log",
				zap.String("request_id", requestID),
				zap.String("client_ip", clientIP),
				zap.String("user_agent", userAgent),
				zap.String("method", method),
				zap.String("path", path),
				zap.Int("status_code", statusCode),
				zap.Duration("duration", duration),
				zap.String("event_type", "http_request"),
			)
		}
	}
}

// JWTAuth validates JWT tokens and extracts user information
func JWTAuth(authService interface{}) gin.HandlerFunc {
	return func(c *gin.Context) {
		// Extract token from Authorization header
		authHeader := c.GetHeader("Authorization")
		if authHeader == "" {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Authorization header required"})
			c.Abort()
			return
		}
		
		// Check Bearer token format
		tokenParts := strings.Split(authHeader, " ")
		if len(tokenParts) != 2 || tokenParts[0] != "Bearer" {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid authorization header format"})
			c.Abort()
			return
		}
		
		tokenString := tokenParts[1]
		
		// Parse and validate JWT token
		token, err := jwt.Parse(tokenString, func(token *jwt.Token) (interface{}, error) {
			// Verify signing method
			if _, ok := token.Method.(*jwt.SigningMethodRSA); !ok {
				return nil, fmt.Errorf("unexpected signing method: %v", token.Header["alg"])
			}
			
			// Return public key for verification
			// In a real implementation, this would fetch the public key
			// from the auth service or a key management system
			return nil, fmt.Errorf("public key retrieval not implemented")
		})
		
		if err != nil {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid token"})
			c.Abort()
			return
		}
		
		// Extract claims
		if claims, ok := token.Claims.(jwt.MapClaims); ok && token.Valid {
			// Verify token expiration
			if exp, ok := claims["exp"].(float64); ok {
				if time.Now().Unix() > int64(exp) {
					c.JSON(http.StatusUnauthorized, gin.H{"error": "Token expired"})
					c.Abort()
					return
				}
			}
			
			// Store user information in context
			c.Set("user_id", claims["sub"])
			c.Set("user_email", claims["email"])
			c.Set("user_roles", claims["roles"])
			c.Set("user_permissions", claims["permissions"])
			c.Set("user_sector", claims["sector"])
			c.Set("mfa_verified", claims["mfa_verified"])
		} else {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid token claims"})
			c.Abort()
			return
		}
		
		c.Next()
	}
}

// RequireRole ensures user has required role
func RequireRole(requiredRole string) gin.HandlerFunc {
	return func(c *gin.Context) {
		userRoles, exists := c.Get("user_roles")
		if !exists {
			c.JSON(http.StatusForbidden, gin.H{"error": "No roles found"})
			c.Abort()
			return
		}
		
		roles, ok := userRoles.([]interface{})
		if !ok {
			c.JSON(http.StatusForbidden, gin.H{"error": "Invalid roles format"})
			c.Abort()
			return
		}
		
		// Check if user has required role
		hasRole := false
		for _, role := range roles {
			if roleStr, ok := role.(string); ok && roleStr == requiredRole {
				hasRole = true
				break
			}
		}
		
		if !hasRole {
			c.JSON(http.StatusForbidden, gin.H{"error": "Insufficient privileges"})
			c.Abort()
			return
		}
		
		c.Next()
	}
}

// RequirePermission ensures user has required permission
func RequirePermission(requiredPermission string) gin.HandlerFunc {
	return func(c *gin.Context) {
		userPermissions, exists := c.Get("user_permissions")
		if !exists {
			c.JSON(http.StatusForbidden, gin.H{"error": "No permissions found"})
			c.Abort()
			return
		}
		
		permissions, ok := userPermissions.([]interface{})
		if !ok {
			c.JSON(http.StatusForbidden, gin.H{"error": "Invalid permissions format"})
			c.Abort()
			return
		}
		
		// Check if user has required permission
		hasPermission := false
		for _, perm := range permissions {
			if permStr, ok := perm.(string); ok && permStr == requiredPermission {
				hasPermission = true
				break
			}
		}
		
		if !hasPermission {
			c.JSON(http.StatusForbidden, gin.H{"error": "Insufficient permissions"})
			c.Abort()
			return
		}
		
		c.Next()
	}
}

// RequireServiceAccount ensures request is from a valid service account
func RequireServiceAccount() gin.HandlerFunc {
	return func(c *gin.Context) {
		// Check for service account token or certificate
		serviceToken := c.GetHeader("X-Service-Token")
		if serviceToken == "" {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Service token required"})
			c.Abort()
			return
		}
		
		// Validate service token (placeholder implementation)
		if !isValidServiceToken(serviceToken) {
			c.JSON(http.StatusUnauthorized, gin.H{"error": "Invalid service token"})
			c.Abort()
			return
		}
		
		c.Set("service_account", true)
		c.Next()
	}
}

// Metrics middleware for collecting security metrics
func Metrics(collector *metrics.Collector) gin.HandlerFunc {
	return func(c *gin.Context) {
		start := time.Now()
		
		c.Next()
		
		duration := time.Since(start)
		statusCode := c.Writer.Status()
		method := c.Request.Method
		path := c.FullPath()
		
		// Record HTTP request metrics
		collector.RecordHTTPRequest(method, path, statusCode, duration)
		
		// Record security-specific metrics
		if statusCode == http.StatusUnauthorized {
			collector.RecordAuthFailure("jwt_validation", c.ClientIP())
		}
		
		if statusCode == http.StatusForbidden {
			collector.RecordAuthFailure("authorization", c.ClientIP())
		}
		
		if statusCode == http.StatusTooManyRequests {
			collector.RecordRateLimitHit(c.ClientIP(), path)
		}
	}
}

// Helper functions

func generateRequestID() string {
	// Generate a unique request ID
	return fmt.Sprintf("%d", time.Now().UnixNano())
}

func isSecurityRelevantPath(path string) bool {
	securityPaths := []string{
		"/auth/",
		"/admin/",
		"/api/v1/users/",
		"/api/v1/roles/",
		"/api/v1/permissions/",
	}
	
	for _, secPath := range securityPaths {
		if strings.Contains(path, secPath) {
			return true
		}
	}
	
	return false
}

func isValidServiceToken(token string) bool {
	// Placeholder implementation
	// In a real system, this would validate the service token
	// against a known list or cryptographic signature
	validTokens := []string{
		"service-api-gateway-token",
		"service-policy-engine-token",
		"service-trip-service-token",
	}
	
	for _, validToken := range validTokens {
		if subtle.ConstantTimeCompare([]byte(token), []byte(validToken)) == 1 {
			return true
		}
	}
	
	return false
}
