package vault

import (
	"context"
	"crypto/tls"
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/hashicorp/vault/api"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"go.opentelemetry.io/otel"
	"go.opentelemetry.io/otel/trace"
)

// VaultClient provides secure secrets management integration
type VaultClient struct {
	client      *api.Client
	config      VaultConfig
	metrics     *VaultMetrics
	tracer      trace.Tracer
	authMethod  AuthMethod
	renewer     *TokenRenewer
}

// VaultConfig holds Vault configuration
type VaultConfig struct {
	Address     string        `json:"address"`
	Token       string        `json:"token"`
	Namespace   string        `json:"namespace"`
	Timeout     time.Duration `json:"timeout"`
	RetryCount  int           `json:"retry_count"`
	RetryDelay  time.Duration `json:"retry_delay"`
	
	// TLS Configuration
	TLSConfig   TLSConfig     `json:"tls_config"`
	
	// Authentication
	AuthMethod  string        `json:"auth_method"`
	AuthPath    string        `json:"auth_path"`
	
	// Secrets Engine
	SecretsPath string        `json:"secrets_path"`
	KVVersion   int           `json:"kv_version"`
}

// TLSConfig holds TLS configuration for Vault
type TLSConfig struct {
	InsecureSkipVerify bool   `json:"insecure_skip_verify"`
	CertFile          string `json:"cert_file"`
	KeyFile           string `json:"key_file"`
	CAFile            string `json:"ca_file"`
}

// VaultMetrics tracks Vault-related metrics
type VaultMetrics struct {
	RequestsTotal     *prometheus.CounterVec
	RequestDuration   *prometheus.HistogramVec
	ErrorsTotal       *prometheus.CounterVec
	TokenRenewals     *prometheus.CounterVec
	SecretsRetrieved  *prometheus.CounterVec
	SecretsStored     *prometheus.CounterVec
	SecretsDeleted    *prometheus.CounterVec
}

// AuthMethod interface for different authentication methods
type AuthMethod interface {
	Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error)
	RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error)
}

// TokenRenewer handles automatic token renewal
type TokenRenewer struct {
	client    *api.Client
	authMethod AuthMethod
	interval  time.Duration
	stopChan  chan struct{}
}

// SecretData represents secret data
type SecretData struct {
	Path      string            `json:"path"`
	Data      map[string]interface{} `json:"data"`
	Metadata  map[string]interface{} `json:"metadata"`
	Version   int               `json:"version"`
	CreatedAt time.Time         `json:"created_at"`
	UpdatedAt time.Time         `json:"updated_at"`
}

// NewVaultClient creates a new Vault client
func NewVaultClient(config VaultConfig) (*VaultClient, error) {
	// Create Vault client
	clientConfig := api.DefaultConfig()
	clientConfig.Address = config.Address
	clientConfig.Timeout = config.Timeout
	
	// Configure TLS
	if config.TLSConfig.CertFile != "" && config.TLSConfig.KeyFile != "" {
		cert, err := tls.LoadX509KeyPair(config.TLSConfig.CertFile, config.TLSConfig.KeyFile)
		if err != nil {
			return nil, fmt.Errorf("failed to load TLS certificate: %w", err)
		}
		
		clientConfig.HttpClient.Transport = &http.Transport{
			TLSClientConfig: &tls.Config{
				Certificates:       []tls.Certificate{cert},
				InsecureSkipVerify: config.TLSConfig.InsecureSkipVerify,
			},
		}
	}
	
	client, err := api.NewClient(clientConfig)
	if err != nil {
		return nil, fmt.Errorf("failed to create Vault client: %w", err)
	}
	
	// Set namespace if provided
	if config.Namespace != "" {
		client.SetNamespace(config.Namespace)
	}
	
	// Set initial token
	if config.Token != "" {
		client.SetToken(config.Token)
	}
	
	// Initialize metrics
	metrics := &VaultMetrics{
		RequestsTotal: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_requests_total",
				Help: "Total number of Vault requests",
			},
			[]string{"method", "path", "status"},
		),
		RequestDuration: promauto.NewHistogramVec(
			prometheus.HistogramOpts{
				Name: "vault_request_duration_seconds",
				Help: "Vault request duration in seconds",
				Buckets: []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0},
			},
			[]string{"method", "path"},
		),
		ErrorsTotal: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_errors_total",
				Help: "Total number of Vault errors",
			},
			[]string{"error_type", "path"},
		),
		TokenRenewals: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_token_renewals_total",
				Help: "Total number of token renewals",
			},
			[]string{"status"},
		),
		SecretsRetrieved: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_secrets_retrieved_total",
				Help: "Total number of secrets retrieved",
			},
			[]string{"path"},
		),
		SecretsStored: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_secrets_stored_total",
				Help: "Total number of secrets stored",
			},
			[]string{"path"},
		),
		SecretsDeleted: promauto.NewCounterVec(
			prometheus.CounterOpts{
				Name: "vault_secrets_deleted_total",
				Help: "Total number of secrets deleted",
			},
			[]string{"path"},
		),
	}
	
	// Initialize auth method
	var authMethod AuthMethod
	switch config.AuthMethod {
	case "kubernetes":
		authMethod = NewKubernetesAuth(config.AuthPath)
	case "aws":
		authMethod = NewAWSAuth(config.AuthPath)
	case "azure":
		authMethod = NewAzureAuth(config.AuthPath)
	case "gcp":
		authMethod = NewGCPAuth(config.AuthPath)
	default:
		authMethod = NewTokenAuth(config.Token)
	}
	
	// Initialize token renewer
	renewer := &TokenRenewer{
		client:     client,
		authMethod: authMethod,
		interval:   5 * time.Minute,
		stopChan:   make(chan struct{}),
	}
	
	return &VaultClient{
		client:     client,
		config:     config,
		metrics:    metrics,
		tracer:     otel.Tracer("vault-client"),
		authMethod: authMethod,
		renewer:    renewer,
	}, nil
}

// Authenticate authenticates with Vault
func (vc *VaultClient) Authenticate(ctx context.Context) error {
	ctx, span := vc.tracer.Start(ctx, "vault-authenticate")
	defer span.End()
	
	start := time.Now()
	defer func() {
		vc.metrics.RequestDuration.WithLabelValues("authenticate", "auth").Observe(time.Since(start).Seconds())
	}()
	
	secret, err := vc.authMethod.Authenticate(ctx, vc.client)
	if err != nil {
		vc.metrics.ErrorsTotal.WithLabelValues("authentication_failed", "auth").Inc()
		return fmt.Errorf("authentication failed: %w", err)
	}
	
	// Set the token
	vc.client.SetToken(secret.Auth.ClientToken)
	
	// Start token renewal
	go vc.renewer.Start()
	
	vc.metrics.RequestsTotal.WithLabelValues("authenticate", "auth", "success").Inc()
	return nil
}

// GetSecret retrieves a secret from Vault
func (vc *VaultClient) GetSecret(ctx context.Context, path string) (*SecretData, error) {
	ctx, span := vc.tracer.Start(ctx, "vault-get-secret")
	defer span.End()
	
	start := time.Now()
	defer func() {
		vc.metrics.RequestDuration.WithLabelValues("get", path).Observe(time.Since(start).Seconds())
	}()
	
	// Construct full path
	fullPath := fmt.Sprintf("%s/data/%s", vc.config.SecretsPath, path)
	
	secret, err := vc.client.Logical().Read(fullPath)
	if err != nil {
		vc.metrics.ErrorsTotal.WithLabelValues("read_failed", path).Inc()
		return nil, fmt.Errorf("failed to read secret: %w", err)
	}
	
	if secret == nil {
		vc.metrics.ErrorsTotal.WithLabelValues("secret_not_found", path).Inc()
		return nil, fmt.Errorf("secret not found: %s", path)
	}
	
	// Extract data
	data := secret.Data["data"].(map[string]interface{})
	metadata := secret.Data["metadata"].(map[string]interface{})
	
	secretData := &SecretData{
		Path:     path,
		Data:     data,
		Metadata: metadata,
		Version:  int(metadata["version"].(float64)),
		UpdatedAt: time.Now(),
	}
	
	vc.metrics.SecretsRetrieved.WithLabelValues(path).Inc()
	vc.metrics.RequestsTotal.WithLabelValues("get", path, "success").Inc()
	
	return secretData, nil
}

// StoreSecret stores a secret in Vault
func (vc *VaultClient) StoreSecret(ctx context.Context, path string, data map[string]interface{}) error {
	ctx, span := vc.tracer.Start(ctx, "vault-store-secret")
	defer span.End()
	
	start := time.Now()
	defer func() {
		vc.metrics.RequestDuration.WithLabelValues("store", path).Observe(time.Since(start).Seconds())
	}()
	
	// Construct full path
	fullPath := fmt.Sprintf("%s/data/%s", vc.config.SecretsPath, path)
	
	// Prepare data for KV v2
	secretData := map[string]interface{}{
		"data": data,
	}
	
	_, err := vc.client.Logical().Write(fullPath, secretData)
	if err != nil {
		vc.metrics.ErrorsTotal.WithLabelValues("write_failed", path).Inc()
		return fmt.Errorf("failed to store secret: %w", err)
	}
	
	vc.metrics.SecretsStored.WithLabelValues(path).Inc()
	vc.metrics.RequestsTotal.WithLabelValues("store", path, "success").Inc()
	
	return nil
}

// DeleteSecret deletes a secret from Vault
func (vc *VaultClient) DeleteSecret(ctx context.Context, path string) error {
	ctx, span := vc.tracer.Start(ctx, "vault-delete-secret")
	defer span.End()
	
	start := time.Now()
	defer func() {
		vc.metrics.RequestDuration.WithLabelValues("delete", path).Observe(time.Since(start).Seconds())
	}()
	
	// Construct full path
	fullPath := fmt.Sprintf("%s/data/%s", vc.config.SecretsPath, path)
	
	_, err := vc.client.Logical().Delete(fullPath)
	if err != nil {
		vc.metrics.ErrorsTotal.WithLabelValues("delete_failed", path).Inc()
		return fmt.Errorf("failed to delete secret: %w", err)
	}
	
	vc.metrics.SecretsDeleted.WithLabelValues(path).Inc()
	vc.metrics.RequestsTotal.WithLabelValues("delete", path, "success").Inc()
	
	return nil
}

// ListSecrets lists secrets in a path
func (vc *VaultClient) ListSecrets(ctx context.Context, path string) ([]string, error) {
	ctx, span := vc.tracer.Start(ctx, "vault-list-secrets")
	defer span.End()
	
	start := time.Now()
	defer func() {
		vc.metrics.RequestDuration.WithLabelValues("list", path).Observe(time.Since(start).Seconds())
	}()
	
	// Construct full path
	fullPath := fmt.Sprintf("%s/metadata/%s", vc.config.SecretsPath, path)
	
	secret, err := vc.client.Logical().List(fullPath)
	if err != nil {
		vc.metrics.ErrorsTotal.WithLabelValues("list_failed", path).Inc()
		return nil, fmt.Errorf("failed to list secrets: %w", err)
	}
	
	if secret == nil {
		return []string{}, nil
	}
	
	keys := secret.Data["keys"].([]interface{})
	result := make([]string, len(keys))
	for i, key := range keys {
		result[i] = key.(string)
	}
	
	vc.metrics.RequestsTotal.WithLabelValues("list", path, "success").Inc()
	
	return result, nil
}

// GetSecretValue retrieves a specific value from a secret
func (vc *VaultClient) GetSecretValue(ctx context.Context, path, key string) (string, error) {
	secret, err := vc.GetSecret(ctx, path)
	if err != nil {
		return "", err
	}
	
	value, exists := secret.Data[key]
	if !exists {
		return "", fmt.Errorf("key %s not found in secret %s", key, path)
	}
	
	return value.(string), nil
}

// RotateSecret rotates a secret by generating a new value
func (vc *VaultClient) RotateSecret(ctx context.Context, path string, generator SecretGenerator) error {
	ctx, span := vc.tracer.Start(ctx, "vault-rotate-secret")
	defer span.End()
	
	// Get current secret
	secret, err := vc.GetSecret(ctx, path)
	if err != nil {
		return fmt.Errorf("failed to get current secret: %w", err)
	}
	
	// Generate new value
	newValue, err := generator.Generate()
	if err != nil {
		return fmt.Errorf("failed to generate new secret: %w", err)
	}
	
	// Update secret
	secret.Data["value"] = newValue
	secret.Data["rotated_at"] = time.Now().Format(time.RFC3339)
	
	// Store updated secret
	err = vc.StoreSecret(ctx, path, secret.Data)
	if err != nil {
		return fmt.Errorf("failed to store rotated secret: %w", err)
	}
	
	return nil
}

// SecretGenerator interface for generating new secret values
type SecretGenerator interface {
	Generate() (string, error)
}

// StartTokenRenewal starts automatic token renewal
func (tr *TokenRenewer) Start() {
	ticker := time.NewTicker(tr.interval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			if err := tr.renewToken(); err != nil {
				// Log error but continue
				fmt.Printf("Token renewal failed: %v\n", err)
			}
		case <-tr.stopChan:
			return
		}
	}
}

// StopTokenRenewal stops automatic token renewal
func (tr *TokenRenewer) Stop() {
	close(tr.stopChan)
}

// renewToken renews the current token
func (tr *TokenRenewer) renewToken() error {
	ctx := context.Background()
	
	secret, err := tr.authMethod.RenewToken(ctx, tr.client, tr.client.Token())
	if err != nil {
		return fmt.Errorf("token renewal failed: %w", err)
	}
	
	// Update token if provided
	if secret.Auth != nil && secret.Auth.ClientToken != "" {
		tr.client.SetToken(secret.Auth.ClientToken)
	}
	
	return nil
}

// Close closes the Vault client
func (vc *VaultClient) Close() {
	if vc.renewer != nil {
		vc.renewer.Stop()
	}
}

// TokenAuth implements token-based authentication
type TokenAuth struct {
	token string
}

// NewTokenAuth creates a new token auth method
func NewTokenAuth(token string) *TokenAuth {
	return &TokenAuth{token: token}
}

// Authenticate authenticates using token
func (ta *TokenAuth) Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error) {
	client.SetToken(ta.token)
	
	// Verify token is valid
	secret, err := client.Auth().Token().LookupSelf()
	if err != nil {
		return nil, fmt.Errorf("token validation failed: %w", err)
	}
	
	return secret, nil
}

// RenewToken renews the token
func (ta *TokenAuth) RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error) {
	return client.Auth().Token().RenewSelf(0)
}

// KubernetesAuth implements Kubernetes authentication
type KubernetesAuth struct {
	role   string
	jwt    string
	path   string
}

// NewKubernetesAuth creates a new Kubernetes auth method
func NewKubernetesAuth(path string) *KubernetesAuth {
	return &KubernetesAuth{
		path: path,
		role: "atlasmesh-service",
	}
}

// Authenticate authenticates using Kubernetes service account
func (ka *KubernetesAuth) Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error) {
	// Read Kubernetes service account token
	jwt, err := readKubernetesToken()
	if err != nil {
		return nil, fmt.Errorf("failed to read Kubernetes token: %w", err)
	}
	
	// Authenticate with Vault
	secret, err := client.Logical().Write(fmt.Sprintf("auth/%s/login", ka.path), map[string]interface{}{
		"role": ka.role,
		"jwt":  jwt,
	})
	if err != nil {
		return nil, fmt.Errorf("Kubernetes authentication failed: %w", err)
	}
	
	return secret, nil
}

// RenewToken renews the token
func (ka *KubernetesAuth) RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error) {
	return client.Auth().Token().RenewSelf(0)
}

// AWSAuth implements AWS authentication
type AWSAuth struct {
	role   string
	path   string
}

// NewAWSAuth creates a new AWS auth method
func NewAWSAuth(path string) *AWSAuth {
	return &AWSAuth{
		path: path,
		role: "atlasmesh-service",
	}
}

// Authenticate authenticates using AWS IAM
func (aa *AWSAuth) Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error) {
	// Get AWS credentials
	creds, err := getAWSCredentials()
	if err != nil {
		return nil, fmt.Errorf("failed to get AWS credentials: %w", err)
	}
	
	// Authenticate with Vault
	secret, err := client.Logical().Write(fmt.Sprintf("auth/%s/login", aa.path), map[string]interface{}{
		"role": aa.role,
		"iam_http_request_method": "POST",
		"iam_request_url":        "https://sts.amazonaws.com/",
		"iam_request_body":       "Action=GetCallerIdentity&Version=2011-06-15",
		"iam_request_headers":    creds,
	})
	if err != nil {
		return nil, fmt.Errorf("AWS authentication failed: %w", err)
	}
	
	return secret, nil
}

// RenewToken renews the token
func (aa *AWSAuth) RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error) {
	return client.Auth().Token().RenewSelf(0)
}

// AzureAuth implements Azure authentication
type AzureAuth struct {
	role   string
	path   string
}

// NewAzureAuth creates a new Azure auth method
func NewAzureAuth(path string) *AzureAuth {
	return &AzureAuth{
		path: path,
		role: "atlasmesh-service",
	}
}

// Authenticate authenticates using Azure managed identity
func (za *AzureAuth) Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error) {
	// Get Azure managed identity token
	token, err := getAzureManagedIdentityToken()
	if err != nil {
		return nil, fmt.Errorf("failed to get Azure token: %w", err)
	}
	
	// Authenticate with Vault
	secret, err := client.Logical().Write(fmt.Sprintf("auth/%s/login", za.path), map[string]interface{}{
		"role": za.role,
		"jwt":  token,
	})
	if err != nil {
		return nil, fmt.Errorf("Azure authentication failed: %w", err)
	}
	
	return secret, nil
}

// RenewToken renews the token
func (za *AzureAuth) RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error) {
	return client.Auth().Token().RenewSelf(0)
}

// GCPAuth implements GCP authentication
type GCPAuth struct {
	role   string
	path   string
}

// NewGCPAuth creates a new GCP auth method
func NewGCPAuth(path string) *GCPAuth {
	return &GCPAuth{
		path: path,
		role: "atlasmesh-service",
	}
}

// Authenticate authenticates using GCP service account
func (ga *GCPAuth) Authenticate(ctx context.Context, client *api.Client) (*api.Secret, error) {
	// Get GCP service account token
	token, err := getGCPServiceAccountToken()
	if err != nil {
		return nil, fmt.Errorf("failed to get GCP token: %w", err)
	}
	
	// Authenticate with Vault
	secret, err := client.Logical().Write(fmt.Sprintf("auth/%s/login", ga.path), map[string]interface{}{
		"role": ga.role,
		"jwt":  token,
	})
	if err != nil {
		return nil, fmt.Errorf("GCP authentication failed: %w", err)
	}
	
	return secret, nil
}

// RenewToken renews the token
func (ga *GCPAuth) RenewToken(ctx context.Context, client *api.Client, token string) (*api.Secret, error) {
	return client.Auth().Token().RenewSelf(0)
}

// Helper functions for cloud provider authentication
func readKubernetesToken() (string, error) {
	// Implementation for reading Kubernetes service account token
	// This would typically read from /var/run/secrets/kubernetes.io/serviceaccount/token
	return "kubernetes-token", nil
}

func getAWSCredentials() (map[string]interface{}, error) {
	// Implementation for getting AWS credentials
	// This would typically use AWS SDK to get credentials
	return map[string]interface{}{
		"Authorization": "AWS4-HMAC-SHA256 ...",
	}, nil
}

func getAzureManagedIdentityToken() (string, error) {
	// Implementation for getting Azure managed identity token
	// This would typically call Azure metadata service
	return "azure-token", nil
}

func getGCPServiceAccountToken() (string, error) {
	// Implementation for getting GCP service account token
	// This would typically use GCP metadata service
	return "gcp-token", nil
}
