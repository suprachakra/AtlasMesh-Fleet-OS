package vault

import (
	"context"
	"fmt"
	"time"

	"github.com/hashicorp/vault/api"
	"go.uber.org/zap"

	"github.com/atlasmesh/fleet-os/services/auth-service/internal/config"
)

// Client wraps HashiCorp Vault client with AtlasMesh-specific functionality
type Client struct {
	client *api.Client
	logger *zap.Logger
	config config.VaultConfig
	
	// Authentication
	token      string
	tokenLease *api.Secret
	
	// State
	authenticated bool
}

// NewClient creates a new Vault client
func NewClient(cfg config.VaultConfig, logger *zap.Logger) (*Client, error) {
	// Create Vault client configuration
	vaultConfig := api.DefaultConfig()
	vaultConfig.Address = cfg.Address
	vaultConfig.Timeout = cfg.Timeout
	
	// Configure TLS if enabled
	if cfg.TLS.Enabled {
		tlsConfig := &api.TLSConfig{
			CACert:        cfg.TLS.CACert,
			ClientCert:    cfg.TLS.ClientCert,
			ClientKey:     cfg.TLS.ClientKey,
			Insecure:      cfg.TLS.InsecureSkipVerify,
			ServerName:    cfg.TLS.ServerName,
		}
		
		if err := vaultConfig.ConfigureTLS(tlsConfig); err != nil {
			return nil, fmt.Errorf("failed to configure Vault TLS: %w", err)
		}
	}
	
	// Create Vault client
	client, err := api.NewClient(vaultConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create Vault client: %w", err)
	}
	
	vaultClient := &Client{
		client: client,
		logger: logger,
		config: cfg,
	}
	
	// Authenticate with Vault
	if err := vaultClient.authenticate(); err != nil {
		return nil, fmt.Errorf("failed to authenticate with Vault: %w", err)
	}
	
	// Start token renewal goroutine
	go vaultClient.startTokenRenewal()
	
	logger.Info("Successfully connected to Vault", zap.String("address", cfg.Address))
	return vaultClient, nil
}

// authenticate performs initial authentication with Vault
func (c *Client) authenticate() error {
	switch c.config.AuthMethod {
	case "token":
		return c.authenticateWithToken()
	case "kubernetes":
		return c.authenticateWithKubernetes()
	case "aws":
		return c.authenticateWithAWS()
	case "azure":
		return c.authenticateWithAzure()
	case "gcp":
		return c.authenticateWithGCP()
	default:
		return fmt.Errorf("unsupported auth method: %s", c.config.AuthMethod)
	}
}

// authenticateWithToken uses a static token for authentication
func (c *Client) authenticateWithToken() error {
	if c.config.Token == "" {
		return fmt.Errorf("token is required for token auth method")
	}
	
	c.client.SetToken(c.config.Token)
	c.token = c.config.Token
	c.authenticated = true
	
	// Verify token is valid
	secret, err := c.client.Auth().Token().LookupSelf()
	if err != nil {
		return fmt.Errorf("failed to verify token: %w", err)
	}
	
	c.tokenLease = secret
	c.logger.Info("Authenticated with Vault using token")
	return nil
}

// authenticateWithKubernetes uses Kubernetes service account for authentication
func (c *Client) authenticateWithKubernetes() error {
	// Read service account token
	jwt, err := c.readServiceAccountToken()
	if err != nil {
		return fmt.Errorf("failed to read service account token: %w", err)
	}
	
	// Authenticate with Vault
	data := map[string]interface{}{
		"jwt":  jwt,
		"role": c.config.KubernetesRole,
	}
	
	secret, err := c.client.Logical().Write("auth/kubernetes/login", data)
	if err != nil {
		return fmt.Errorf("failed to authenticate with Kubernetes: %w", err)
	}
	
	if secret == nil || secret.Auth == nil {
		return fmt.Errorf("no auth info returned from Vault")
	}
	
	c.client.SetToken(secret.Auth.ClientToken)
	c.token = secret.Auth.ClientToken
	c.tokenLease = secret
	c.authenticated = true
	
	c.logger.Info("Authenticated with Vault using Kubernetes service account")
	return nil
}

// authenticateWithAWS uses AWS IAM for authentication
func (c *Client) authenticateWithAWS() error {
	// This would implement AWS IAM authentication
	// For now, return not implemented
	return fmt.Errorf("AWS authentication not implemented")
}

// authenticateWithAzure uses Azure managed identity for authentication
func (c *Client) authenticateWithAzure() error {
	// This would implement Azure managed identity authentication
	// For now, return not implemented
	return fmt.Errorf("Azure authentication not implemented")
}

// authenticateWithGCP uses GCP service account for authentication
func (c *Client) authenticateWithGCP() error {
	// This would implement GCP service account authentication
	// For now, return not implemented
	return fmt.Errorf("GCP authentication not implemented")
}

// GetSecret retrieves a secret from Vault
func (c *Client) GetSecret(path string) (string, error) {
	if !c.authenticated {
		return "", fmt.Errorf("not authenticated with Vault")
	}
	
	secret, err := c.client.Logical().Read(c.config.SecretPath + "/" + path)
	if err != nil {
		return "", fmt.Errorf("failed to read secret %s: %w", path, err)
	}
	
	if secret == nil || secret.Data == nil {
		return "", fmt.Errorf("secret not found: %s", path)
	}
	
	// Handle KV v2 secrets
	if data, ok := secret.Data["data"].(map[string]interface{}); ok {
		if value, exists := data["value"].(string); exists {
			return value, nil
		}
	}
	
	// Handle KV v1 secrets
	if value, exists := secret.Data["value"].(string); exists {
		return value, nil
	}
	
	return "", fmt.Errorf("secret value not found in path: %s", path)
}

// SetSecret stores a secret in Vault
func (c *Client) SetSecret(path, value string) error {
	if !c.authenticated {
		return fmt.Errorf("not authenticated with Vault")
	}
	
	data := map[string]interface{}{
		"data": map[string]interface{}{
			"value": value,
		},
	}
	
	_, err := c.client.Logical().Write(c.config.SecretPath+"/data/"+path, data)
	if err != nil {
		return fmt.Errorf("failed to write secret %s: %w", path, err)
	}
	
	c.logger.Debug("Secret stored in Vault", zap.String("path", path))
	return nil
}

// GetSecretWithMetadata retrieves a secret with its metadata
func (c *Client) GetSecretWithMetadata(path string) (*SecretWithMetadata, error) {
	if !c.authenticated {
		return nil, fmt.Errorf("not authenticated with Vault")
	}
	
	secret, err := c.client.Logical().Read(c.config.SecretPath + "/" + path)
	if err != nil {
		return nil, fmt.Errorf("failed to read secret %s: %w", path, err)
	}
	
	if secret == nil {
		return nil, fmt.Errorf("secret not found: %s", path)
	}
	
	result := &SecretWithMetadata{
		Path:      path,
		CreatedAt: time.Now(), // This would be parsed from Vault metadata
		UpdatedAt: time.Now(), // This would be parsed from Vault metadata
		Version:   1,          // This would be parsed from Vault metadata
	}
	
	// Handle KV v2 secrets
	if data, ok := secret.Data["data"].(map[string]interface{}); ok {
		result.Data = data
		if metadata, ok := secret.Data["metadata"].(map[string]interface{}); ok {
			if createdTime, ok := metadata["created_time"].(string); ok {
				if t, err := time.Parse(time.RFC3339, createdTime); err == nil {
					result.CreatedAt = t
				}
			}
			if version, ok := metadata["version"].(float64); ok {
				result.Version = int(version)
			}
		}
	} else {
		// Handle KV v1 secrets
		result.Data = secret.Data
	}
	
	return result, nil
}

// DeleteSecret removes a secret from Vault
func (c *Client) DeleteSecret(path string) error {
	if !c.authenticated {
		return fmt.Errorf("not authenticated with Vault")
	}
	
	_, err := c.client.Logical().Delete(c.config.SecretPath + "/data/" + path)
	if err != nil {
		return fmt.Errorf("failed to delete secret %s: %w", path, err)
	}
	
	c.logger.Debug("Secret deleted from Vault", zap.String("path", path))
	return nil
}

// GenerateSecret generates a new secret using Vault's secret engines
func (c *Client) GenerateSecret(engine, path string, options map[string]interface{}) (*GeneratedSecret, error) {
	if !c.authenticated {
		return nil, fmt.Errorf("not authenticated with Vault")
	}
	
	fullPath := fmt.Sprintf("%s/%s", engine, path)
	secret, err := c.client.Logical().Write(fullPath, options)
	if err != nil {
		return nil, fmt.Errorf("failed to generate secret at %s: %w", fullPath, err)
	}
	
	if secret == nil || secret.Data == nil {
		return nil, fmt.Errorf("no data returned from secret generation")
	}
	
	result := &GeneratedSecret{
		Path:      fullPath,
		Data:      secret.Data,
		LeaseID:   secret.LeaseID,
		Renewable: secret.Renewable,
		TTL:       time.Duration(secret.LeaseDuration) * time.Second,
	}
	
	return result, nil
}

// RenewSecret renews a lease for a dynamic secret
func (c *Client) RenewSecret(leaseID string, increment int) error {
	if !c.authenticated {
		return fmt.Errorf("not authenticated with Vault")
	}
	
	_, err := c.client.Sys().Renew(leaseID, increment)
	if err != nil {
		return fmt.Errorf("failed to renew secret lease %s: %w", leaseID, err)
	}
	
	c.logger.Debug("Secret lease renewed", zap.String("leaseID", leaseID))
	return nil
}

// RevokeSecret revokes a lease for a dynamic secret
func (c *Client) RevokeSecret(leaseID string) error {
	if !c.authenticated {
		return fmt.Errorf("not authenticated with Vault")
	}
	
	err := c.client.Sys().Revoke(leaseID)
	if err != nil {
		return fmt.Errorf("failed to revoke secret lease %s: %w", leaseID, err)
	}
	
	c.logger.Debug("Secret lease revoked", zap.String("leaseID", leaseID))
	return nil
}

// HealthCheck verifies Vault connectivity and authentication
func (c *Client) HealthCheck() error {
	if !c.authenticated {
		return fmt.Errorf("not authenticated with Vault")
	}
	
	// Check Vault health
	health, err := c.client.Sys().Health()
	if err != nil {
		return fmt.Errorf("Vault health check failed: %w", err)
	}
	
	if !health.Initialized {
		return fmt.Errorf("Vault is not initialized")
	}
	
	if health.Sealed {
		return fmt.Errorf("Vault is sealed")
	}
	
	// Verify token is still valid
	_, err = c.client.Auth().Token().LookupSelf()
	if err != nil {
		return fmt.Errorf("token validation failed: %w", err)
	}
	
	return nil
}

// startTokenRenewal starts a goroutine to automatically renew the Vault token
func (c *Client) startTokenRenewal() {
	if c.tokenLease == nil || c.tokenLease.Auth == nil {
		c.logger.Warn("No token lease information available for renewal")
		return
	}
	
	if !c.tokenLease.Auth.Renewable {
		c.logger.Info("Token is not renewable")
		return
	}
	
	// Calculate renewal interval (typically half of the lease duration)
	leaseDuration := time.Duration(c.tokenLease.Auth.LeaseDuration) * time.Second
	renewalInterval := leaseDuration / 2
	
	if renewalInterval < time.Minute {
		renewalInterval = time.Minute
	}
	
	c.logger.Info("Starting token renewal",
		zap.Duration("leaseDuration", leaseDuration),
		zap.Duration("renewalInterval", renewalInterval),
	)
	
	ticker := time.NewTicker(renewalInterval)
	defer ticker.Stop()
	
	for range ticker.C {
		if err := c.renewToken(); err != nil {
			c.logger.Error("Failed to renew Vault token", zap.Error(err))
			// In production, you might want to implement exponential backoff
			// and eventually re-authenticate if renewal continues to fail
		}
	}
}

// renewToken renews the current Vault token
func (c *Client) renewToken() error {
	secret, err := c.client.Auth().Token().RenewSelf(0) // 0 means use default increment
	if err != nil {
		return fmt.Errorf("failed to renew token: %w", err)
	}
	
	c.tokenLease = secret
	c.logger.Debug("Token renewed successfully",
		zap.Duration("newLeaseDuration", time.Duration(secret.Auth.LeaseDuration)*time.Second),
	)
	
	return nil
}

// readServiceAccountToken reads the Kubernetes service account token
func (c *Client) readServiceAccountToken() (string, error) {
	// This would read from /var/run/secrets/kubernetes.io/serviceaccount/token
	// For now, return a placeholder
	return "", fmt.Errorf("service account token reading not implemented")
}

// SecretWithMetadata represents a secret with its metadata
type SecretWithMetadata struct {
	Path      string                 `json:"path"`
	Data      map[string]interface{} `json:"data"`
	CreatedAt time.Time              `json:"created_at"`
	UpdatedAt time.Time              `json:"updated_at"`
	Version   int                    `json:"version"`
}

// GeneratedSecret represents a dynamically generated secret
type GeneratedSecret struct {
	Path      string                 `json:"path"`
	Data      map[string]interface{} `json:"data"`
	LeaseID   string                 `json:"lease_id"`
	Renewable bool                   `json:"renewable"`
	TTL       time.Duration          `json:"ttl"`
}
