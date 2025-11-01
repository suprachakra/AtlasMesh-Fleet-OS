package auth

import (
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"database/sql"
	"encoding/base64"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/golang-jwt/jwt/v5"
	"github.com/google/uuid"
	"go.uber.org/zap"
	"golang.org/x/crypto/bcrypt"

	"github.com/atlasmesh/fleet-os/services/auth-service/internal/config"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/metrics"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/models"
	"github.com/atlasmesh/fleet-os/services/auth-service/internal/vault"
)

// Service handles authentication and authorization
type Service struct {
	logger      *zap.Logger
	db          *sql.DB
	vaultClient *vault.Client
	metrics     *metrics.Collector
	config      config.AuthConfig
	
	// JWT signing keys
	privateKey *rsa.PrivateKey
	publicKey  *rsa.PublicKey
	keyID      string
	
	// Service state
	ready bool
}

// ServiceConfig holds configuration for the auth service
type ServiceConfig struct {
	Logger      *zap.Logger
	Database    *sql.DB
	VaultClient *vault.Client
	Metrics     *metrics.Collector
	Config      config.AuthConfig
}

// NewService creates a new authentication service
func NewService(cfg ServiceConfig) *Service {
	service := &Service{
		logger:      cfg.Logger,
		db:          cfg.Database,
		vaultClient: cfg.VaultClient,
		metrics:     cfg.Metrics,
		config:      cfg.Config,
		keyID:       uuid.New().String(),
	}
	
	// Initialize JWT signing keys
	if err := service.initializeKeys(); err != nil {
		cfg.Logger.Fatal("Failed to initialize JWT keys", zap.Error(err))
	}
	
	service.ready = true
	return service
}

// initializeKeys generates or loads JWT signing keys
func (s *Service) initializeKeys() error {
	// Try to load existing keys from Vault
	privateKeyPEM, err := s.vaultClient.GetSecret("auth/jwt-private-key")
	if err == nil && privateKeyPEM != "" {
		// Parse existing private key
		block, _ := pem.Decode([]byte(privateKeyPEM))
		if block == nil {
			return fmt.Errorf("failed to decode PEM block")
		}
		
		privateKey, err := x509.ParsePKCS1PrivateKey(block.Bytes)
		if err != nil {
			return fmt.Errorf("failed to parse private key: %w", err)
		}
		
		s.privateKey = privateKey
		s.publicKey = &privateKey.PublicKey
		
		s.logger.Info("Loaded existing JWT signing keys from Vault")
		return nil
	}
	
	// Generate new keys
	s.logger.Info("Generating new JWT signing keys")
	
	privateKey, err := rsa.GenerateKey(rand.Reader, 2048)
	if err != nil {
		return fmt.Errorf("failed to generate private key: %w", err)
	}
	
	s.privateKey = privateKey
	s.publicKey = &privateKey.PublicKey
	
	// Store private key in Vault
	privateKeyPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "RSA PRIVATE KEY",
		Bytes: x509.MarshalPKCS1PrivateKey(privateKey),
	})
	
	if err := s.vaultClient.SetSecret("auth/jwt-private-key", string(privateKeyPEM)); err != nil {
		return fmt.Errorf("failed to store private key in Vault: %w", err)
	}
	
	// Store public key in Vault
	publicKeyPKIX, err := x509.MarshalPKIXPublicKey(s.publicKey)
	if err != nil {
		return fmt.Errorf("failed to marshal public key: %w", err)
	}
	
	publicKeyPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "PUBLIC KEY",
		Bytes: publicKeyPKIX,
	})
	
	if err := s.vaultClient.SetSecret("auth/jwt-public-key", string(publicKeyPEM)); err != nil {
		return fmt.Errorf("failed to store public key in Vault: %w", err)
	}
	
	s.logger.Info("Generated and stored new JWT signing keys")
	return nil
}

// Login authenticates a user and returns JWT tokens
func (s *Service) Login(c *gin.Context) {
	var req models.LoginRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		s.metrics.RecordAuthAttempt("login", "invalid_request", "")
		c.JSON(400, gin.H{"error": "Invalid request format"})
		return
	}
	
	// Rate limiting check
	if s.isRateLimited(c.ClientIP(), "login") {
		s.metrics.RecordAuthAttempt("login", "rate_limited", req.Email)
		c.JSON(429, gin.H{"error": "Too many login attempts"})
		return
	}
	
	// Authenticate user
	user, err := s.authenticateUser(req.Email, req.Password)
	if err != nil {
		s.logger.Warn("Authentication failed", 
			zap.String("email", req.Email),
			zap.String("ip", c.ClientIP()),
			zap.Error(err),
		)
		s.metrics.RecordAuthAttempt("login", "failed", req.Email)
		c.JSON(401, gin.H{"error": "Invalid credentials"})
		return
	}
	
	// Check if user is active
	if !user.Active {
		s.metrics.RecordAuthAttempt("login", "user_disabled", req.Email)
		c.JSON(401, gin.H{"error": "Account disabled"})
		return
	}
	
	// Check MFA if enabled
	if user.MFAEnabled && req.MFACode == "" {
		s.metrics.RecordAuthAttempt("login", "mfa_required", req.Email)
		c.JSON(200, gin.H{
			"mfa_required": true,
			"mfa_methods": user.MFAMethods,
		})
		return
	}
	
	if user.MFAEnabled && req.MFACode != "" {
		if !s.verifyMFACode(user.ID, req.MFACode) {
			s.metrics.RecordAuthAttempt("login", "mfa_failed", req.Email)
			c.JSON(401, gin.H{"error": "Invalid MFA code"})
			return
		}
	}
	
	// Generate tokens
	accessToken, refreshToken, err := s.generateTokens(user)
	if err != nil {
		s.logger.Error("Failed to generate tokens", zap.Error(err))
		s.metrics.RecordAuthAttempt("login", "token_generation_failed", req.Email)
		c.JSON(500, gin.H{"error": "Internal server error"})
		return
	}
	
	// Create session
	session := &models.Session{
		ID:           uuid.New().String(),
		UserID:       user.ID,
		RefreshToken: refreshToken,
		IPAddress:    c.ClientIP(),
		UserAgent:    c.GetHeader("User-Agent"),
		CreatedAt:    time.Now(),
		ExpiresAt:    time.Now().Add(s.config.RefreshTokenTTL),
		Active:       true,
	}
	
	if err := s.createSession(session); err != nil {
		s.logger.Error("Failed to create session", zap.Error(err))
		c.JSON(500, gin.H{"error": "Internal server error"})
		return
	}
	
	// Update last login
	if err := s.updateLastLogin(user.ID, c.ClientIP()); err != nil {
		s.logger.Warn("Failed to update last login", zap.Error(err))
	}
	
	s.logger.Info("User logged in successfully",
		zap.String("userId", user.ID),
		zap.String("email", user.Email),
		zap.String("ip", c.ClientIP()),
	)
	
	s.metrics.RecordAuthAttempt("login", "success", req.Email)
	
	c.JSON(200, gin.H{
		"access_token":  accessToken,
		"refresh_token": refreshToken,
		"token_type":    "Bearer",
		"expires_in":    int(s.config.AccessTokenTTL.Seconds()),
		"user": gin.H{
			"id":       user.ID,
			"email":    user.Email,
			"name":     user.Name,
			"roles":    user.Roles,
			"sector":   user.Sector,
			"mfa_enabled": user.MFAEnabled,
		},
	})
}

// RefreshToken generates new access token using refresh token
func (s *Service) RefreshToken(c *gin.Context) {
	var req models.RefreshTokenRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(400, gin.H{"error": "Invalid request format"})
		return
	}
	
	// Validate refresh token
	session, err := s.validateRefreshToken(req.RefreshToken)
	if err != nil {
		s.logger.Warn("Invalid refresh token", zap.Error(err))
		c.JSON(401, gin.H{"error": "Invalid refresh token"})
		return
	}
	
	// Get user
	user, err := s.getUserByID(session.UserID)
	if err != nil {
		s.logger.Error("Failed to get user", zap.Error(err))
		c.JSON(500, gin.H{"error": "Internal server error"})
		return
	}
	
	// Check if user is still active
	if !user.Active {
		c.JSON(401, gin.H{"error": "Account disabled"})
		return
	}
	
	// Generate new access token
	accessToken, err := s.generateAccessToken(user)
	if err != nil {
		s.logger.Error("Failed to generate access token", zap.Error(err))
		c.JSON(500, gin.H{"error": "Internal server error"})
		return
	}
	
	// Optionally rotate refresh token
	var newRefreshToken string
	if s.config.RotateRefreshTokens {
		newRefreshToken, err = s.generateRefreshToken(user)
		if err != nil {
			s.logger.Error("Failed to generate refresh token", zap.Error(err))
			c.JSON(500, gin.H{"error": "Internal server error"})
			return
		}
		
		// Update session with new refresh token
		session.RefreshToken = newRefreshToken
		session.ExpiresAt = time.Now().Add(s.config.RefreshTokenTTL)
		if err := s.updateSession(session); err != nil {
			s.logger.Error("Failed to update session", zap.Error(err))
		}
	}
	
	response := gin.H{
		"access_token": accessToken,
		"token_type":   "Bearer",
		"expires_in":   int(s.config.AccessTokenTTL.Seconds()),
	}
	
	if newRefreshToken != "" {
		response["refresh_token"] = newRefreshToken
	}
	
	c.JSON(200, response)
}

// Logout invalidates the current session
func (s *Service) Logout(c *gin.Context) {
	var req models.LogoutRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(400, gin.H{"error": "Invalid request format"})
		return
	}
	
	// Invalidate refresh token session
	if req.RefreshToken != "" {
		if err := s.invalidateRefreshToken(req.RefreshToken); err != nil {
			s.logger.Warn("Failed to invalidate refresh token", zap.Error(err))
		}
	}
	
	// Add access token to blacklist
	if req.AccessToken != "" {
		if err := s.blacklistAccessToken(req.AccessToken); err != nil {
			s.logger.Warn("Failed to blacklist access token", zap.Error(err))
		}
	}
	
	s.logger.Info("User logged out", zap.String("ip", c.ClientIP()))
	c.JSON(200, gin.H{"message": "Logged out successfully"})
}

// Helper methods

func (s *Service) authenticateUser(email, password string) (*models.User, error) {
	query := `
		SELECT id, email, name, password_hash, active, mfa_enabled, mfa_secret, 
		       sector, created_at, updated_at, last_login_at, last_login_ip
		FROM users 
		WHERE email = $1 AND deleted_at IS NULL
	`
	
	user := &models.User{}
	var passwordHash string
	var lastLoginAt sql.NullTime
	var lastLoginIP sql.NullString
	
	err := s.db.QueryRow(query, email).Scan(
		&user.ID, &user.Email, &user.Name, &passwordHash,
		&user.Active, &user.MFAEnabled, &user.MFASecret,
		&user.Sector, &user.CreatedAt, &user.UpdatedAt,
		&lastLoginAt, &lastLoginIP,
	)
	
	if err != nil {
		if err == sql.ErrNoRows {
			return nil, fmt.Errorf("user not found")
		}
		return nil, fmt.Errorf("database error: %w", err)
	}
	
	// Verify password
	if err := bcrypt.CompareHashAndPassword([]byte(passwordHash), []byte(password)); err != nil {
		return nil, fmt.Errorf("invalid password")
	}
	
	// Load user roles
	roles, err := s.getUserRoles(user.ID)
	if err != nil {
		return nil, fmt.Errorf("failed to load user roles: %w", err)
	}
	user.Roles = roles
	
	// Load user permissions
	permissions, err := s.getUserPermissions(user.ID)
	if err != nil {
		return nil, fmt.Errorf("failed to load user permissions: %w", err)
	}
	user.Permissions = permissions
	
	if lastLoginAt.Valid {
		user.LastLoginAt = &lastLoginAt.Time
	}
	if lastLoginIP.Valid {
		user.LastLoginIP = &lastLoginIP.String
	}
	
	return user, nil
}

func (s *Service) generateTokens(user *models.User) (accessToken, refreshToken string, err error) {
	accessToken, err = s.generateAccessToken(user)
	if err != nil {
		return "", "", err
	}
	
	refreshToken, err = s.generateRefreshToken(user)
	if err != nil {
		return "", "", err
	}
	
	return accessToken, refreshToken, nil
}

func (s *Service) generateAccessToken(user *models.User) (string, error) {
	now := time.Now()
	claims := jwt.MapClaims{
		"iss":         "atlasmesh-auth",
		"sub":         user.ID,
		"aud":         "atlasmesh-fleet-os",
		"exp":         now.Add(s.config.AccessTokenTTL).Unix(),
		"iat":         now.Unix(),
		"nbf":         now.Unix(),
		"jti":         uuid.New().String(),
		"email":       user.Email,
		"name":        user.Name,
		"roles":       user.Roles,
		"permissions": user.Permissions,
		"sector":      user.Sector,
		"mfa_verified": user.MFAEnabled, // Assume MFA was verified during login
	}
	
	token := jwt.NewWithClaims(jwt.SigningMethodRS256, claims)
	token.Header["kid"] = s.keyID
	
	return token.SignedString(s.privateKey)
}

func (s *Service) generateRefreshToken(user *models.User) (string, error) {
	// Generate a random refresh token
	tokenBytes := make([]byte, 32)
	if _, err := rand.Read(tokenBytes); err != nil {
		return "", err
	}
	
	return base64.URLEncoding.EncodeToString(tokenBytes), nil
}

// Additional helper methods would be implemented here...
// (getUserRoles, getUserPermissions, createSession, etc.)

func (s *Service) IsReady() bool {
	return s.ready
}

func (s *Service) GetHealthChecks() map[string]interface{} {
	checks := make(map[string]interface{})
	
	// Database health check
	if err := s.db.Ping(); err != nil {
		checks["database"] = map[string]interface{}{
			"status": "unhealthy",
			"error":  err.Error(),
		}
	} else {
		checks["database"] = map[string]interface{}{
			"status": "healthy",
		}
	}
	
	// Vault health check
	if err := s.vaultClient.HealthCheck(); err != nil {
		checks["vault"] = map[string]interface{}{
			"status": "unhealthy",
			"error":  err.Error(),
		}
	} else {
		checks["vault"] = map[string]interface{}{
			"status": "healthy",
		}
	}
	
	return checks
}

// Background processors
func (s *Service) StartTokenCleanup(ctx context.Context) {
	ticker := time.NewTicker(1 * time.Hour)
	defer ticker.Stop()
	
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.cleanupExpiredTokens()
		}
	}
}

func (s *Service) StartSessionMonitoring(ctx context.Context) {
	ticker := time.NewTicker(5 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.monitorSessions()
		}
	}
}

func (s *Service) StartSecurityEventProcessor(ctx context.Context) {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			s.processSecurityEvents()
		}
	}
}

// Placeholder implementations for helper methods
func (s *Service) isRateLimited(ip, action string) bool { return false }
func (s *Service) verifyMFACode(userID, code string) bool { return true }
func (s *Service) getUserByID(userID string) (*models.User, error) { return nil, nil }
func (s *Service) validateRefreshToken(token string) (*models.Session, error) { return nil, nil }
func (s *Service) createSession(session *models.Session) error { return nil }
func (s *Service) updateSession(session *models.Session) error { return nil }
func (s *Service) updateLastLogin(userID, ip string) error { return nil }
func (s *Service) getUserRoles(userID string) ([]string, error) { return []string{}, nil }
func (s *Service) getUserPermissions(userID string) ([]string, error) { return []string{}, nil }
func (s *Service) invalidateRefreshToken(token string) error { return nil }
func (s *Service) blacklistAccessToken(token string) error { return nil }
func (s *Service) cleanupExpiredTokens() {}
func (s *Service) monitorSessions() {}
func (s *Service) processSecurityEvents() {}

// Placeholder handlers for remaining endpoints
func (s *Service) ForgotPassword(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ResetPassword(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) VerifyEmail(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) OAuthLogin(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) OAuthCallback(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetJWKS(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetPublicKey(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetCurrentUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UpdateCurrentUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ChangePassword(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) EnableMFA(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DisableMFA(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) VerifyMFA(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetUserSessions(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) RevokeSession(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ListUsers(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) CreateUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UpdateUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DeleteUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) AdminResetPassword(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DisableUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) EnableUser(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ListRoles(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) CreateRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UpdateRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DeleteRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) AssignRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UnassignRole(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ListPermissions(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) CreatePermission(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetPermission(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UpdatePermission(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DeletePermission(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetAuditLogs(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetSecurityEvents(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetActiveSessions(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) RevokeAllSessions(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ServiceAuthenticate(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ServiceAuthorize(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetServiceCertificate(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) RenewServiceCertificate(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) EvaluatePolicy(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) EvaluatePolicyBatch(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) ListPolicies(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) CreatePolicy(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) GetPolicy(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) UpdatePolicy(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
func (s *Service) DeletePolicy(c *gin.Context) { c.JSON(501, gin.H{"error": "Not implemented"}) }
