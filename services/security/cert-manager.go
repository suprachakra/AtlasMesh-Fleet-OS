// AtlasMesh Security Service - Certificate Management and Rotation
package main

import (
	"crypto/rand"
	"crypto/rsa"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"fmt"
	"log"
	"math/big"
	"net/http"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
)

type CertificateManager struct {
	mu           sync.RWMutex
	certificates map[string]*CertificateEntry
	caKey        *rsa.PrivateKey
	caCert       *x509.Certificate
	config       *SecurityConfig
}

type CertificateEntry struct {
	Certificate *x509.Certificate
	PrivateKey  *rsa.PrivateKey
	PEMCert     []byte
	PEMKey      []byte
	ExpiresAt   time.Time
	CreatedAt   time.Time
	Subject     string
	Usage       []string
}

type SecurityConfig struct {
	CertDir         string        `json:"cert_dir"`
	KeySize         int           `json:"key_size"`
	CertValidity    time.Duration `json:"cert_validity"`
	RotationWindow  time.Duration `json:"rotation_window"`
	AutoRotate      bool          `json:"auto_rotate"`
	MTLSEnabled     bool          `json:"mtls_enabled"`
	VaultEnabled    bool          `json:"vault_enabled"`
	VaultAddress    string        `json:"vault_address"`
	VaultToken      string        `json:"vault_token"`
}

func main() {
	log.Println("AtlasMesh Security Service starting...")

	config := &SecurityConfig{
		CertDir:        getEnv("CERT_DIR", "./certs"),
		KeySize:        2048,
		CertValidity:   24 * time.Hour * 365, // 1 year
		RotationWindow: 24 * time.Hour * 30,  // 30 days before expiry
		AutoRotate:     true,
		MTLSEnabled:    true,
		VaultEnabled:   getEnv("VAULT_ENABLED", "false") == "true",
		VaultAddress:   getEnv("VAULT_ADDR", "http://localhost:8200"),
		VaultToken:     getEnv("VAULT_TOKEN", ""),
	}

	certManager, err := NewCertificateManager(config)
	if err != nil {
		log.Fatalf("Failed to initialize certificate manager: %v", err)
	}

	// Start certificate rotation monitoring
	go certManager.StartRotationMonitor()

	// Setup HTTP server
	router := gin.New()
	router.Use(gin.Logger(), gin.Recovery())

	// Health endpoints
	router.GET("/health", func(c *gin.Context) {
		c.JSON(200, gin.H{"status": "healthy", "service": "security"})
	})

	// Certificate management endpoints
	v1 := router.Group("/api/v1")
	{
		v1.POST("/certificates", certManager.IssueCertificate)
		v1.GET("/certificates/:subject", certManager.GetCertificate)
		v1.DELETE("/certificates/:subject", certManager.RevokeCertificate)
		v1.POST("/certificates/:subject/rotate", certManager.RotateCertificate)
		v1.GET("/ca/certificate", certManager.GetCACertificate)
		v1.GET("/certificates", certManager.ListCertificates)
	}

	// Start server
	server := &http.Server{
		Addr:    ":8082",
		Handler: router,
		TLSConfig: &tls.Config{
			MinVersion: tls.VersionTLS12,
			CipherSuites: []uint16{
				tls.TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384,
				tls.TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305,
				tls.TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,
			},
		},
	}

	log.Println("Security service listening on port 8082")
	if config.MTLSEnabled {
		log.Fatal(server.ListenAndServeTLS(
			filepath.Join(config.CertDir, "server.crt"),
			filepath.Join(config.CertDir, "server.key"),
		))
	} else {
		log.Fatal(server.ListenAndServe())
	}
}

func NewCertificateManager(config *SecurityConfig) (*CertificateManager, error) {
	cm := &CertificateManager{
		certificates: make(map[string]*CertificateEntry),
		config:       config,
	}

	// Ensure certificate directory exists
	if err := os.MkdirAll(config.CertDir, 0755); err != nil {
		return nil, fmt.Errorf("failed to create cert directory: %w", err)
	}

	// Initialize or load CA
	if err := cm.initializeCA(); err != nil {
		return nil, fmt.Errorf("failed to initialize CA: %w", err)
	}

	// Load existing certificates
	if err := cm.loadExistingCertificates(); err != nil {
		log.Printf("Warning: failed to load existing certificates: %v", err)
	}

	return cm, nil
}

func (cm *CertificateManager) initializeCA() error {
	caKeyPath := filepath.Join(cm.config.CertDir, "ca.key")
	caCertPath := filepath.Join(cm.config.CertDir, "ca.crt")

	// Check if CA already exists
	if _, err := os.Stat(caKeyPath); err == nil {
		return cm.loadCA(caKeyPath, caCertPath)
	}

	// Generate new CA
	return cm.generateCA(caKeyPath, caCertPath)
}

func (cm *CertificateManager) generateCA(keyPath, certPath string) error {
	// Generate CA private key
	caKey, err := rsa.GenerateKey(rand.Reader, cm.config.KeySize)
	if err != nil {
		return fmt.Errorf("failed to generate CA key: %w", err)
	}

	// Create CA certificate template
	template := x509.Certificate{
		SerialNumber: big.NewInt(1),
		Subject: pkix.Name{
			Organization:  []string{"AtlasMesh Fleet OS"},
			Country:       []string{"US"},
			Province:      []string{""},
			Locality:      []string{"San Francisco"},
			StreetAddress: []string{""},
			PostalCode:    []string{""},
		},
		NotBefore:             time.Now(),
		NotAfter:              time.Now().Add(cm.config.CertValidity * 2), // CA lives longer
		IsCA:                  true,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageClientAuth, x509.ExtKeyUsageServerAuth},
		KeyUsage:              x509.KeyUsageDigitalSignature | x509.KeyUsageCertSign,
		BasicConstraintsValid: true,
	}

	// Create certificate
	caCertDER, err := x509.CreateCertificate(rand.Reader, &template, &template, &caKey.PublicKey, caKey)
	if err != nil {
		return fmt.Errorf("failed to create CA certificate: %w", err)
	}

	// Parse certificate
	caCert, err := x509.ParseCertificate(caCertDER)
	if err != nil {
		return fmt.Errorf("failed to parse CA certificate: %w", err)
	}

	// Save CA key
	keyFile, err := os.Create(keyPath)
	if err != nil {
		return fmt.Errorf("failed to create CA key file: %w", err)
	}
	defer keyFile.Close()

	keyPEM := &pem.Block{Type: "RSA PRIVATE KEY", Bytes: x509.MarshalPKCS1PrivateKey(caKey)}
	if err := pem.Encode(keyFile, keyPEM); err != nil {
		return fmt.Errorf("failed to write CA key: %w", err)
	}

	// Save CA certificate
	certFile, err := os.Create(certPath)
	if err != nil {
		return fmt.Errorf("failed to create CA cert file: %w", err)
	}
	defer certFile.Close()

	certPEM := &pem.Block{Type: "CERTIFICATE", Bytes: caCertDER}
	if err := pem.Encode(certFile, certPEM); err != nil {
		return fmt.Errorf("failed to write CA certificate: %w", err)
	}

	cm.caKey = caKey
	cm.caCert = caCert

	log.Println("Generated new CA certificate")
	return nil
}

func (cm *CertificateManager) loadCA(keyPath, certPath string) error {
	// Load CA key
	keyData, err := os.ReadFile(keyPath)
	if err != nil {
		return fmt.Errorf("failed to read CA key: %w", err)
	}

	keyBlock, _ := pem.Decode(keyData)
	if keyBlock == nil {
		return fmt.Errorf("failed to decode CA key PEM")
	}

	caKey, err := x509.ParsePKCS1PrivateKey(keyBlock.Bytes)
	if err != nil {
		return fmt.Errorf("failed to parse CA key: %w", err)
	}

	// Load CA certificate
	certData, err := os.ReadFile(certPath)
	if err != nil {
		return fmt.Errorf("failed to read CA certificate: %w", err)
	}

	certBlock, _ := pem.Decode(certData)
	if certBlock == nil {
		return fmt.Errorf("failed to decode CA certificate PEM")
	}

	caCert, err := x509.ParseCertificate(certBlock.Bytes)
	if err != nil {
		return fmt.Errorf("failed to parse CA certificate: %w", err)
	}

	cm.caKey = caKey
	cm.caCert = caCert

	log.Println("Loaded existing CA certificate")
	return nil
}

func (cm *CertificateManager) loadExistingCertificates() error {
	// Implementation would scan cert directory and load existing certificates
	log.Println("Loading existing certificates...")
	return nil
}

func (cm *CertificateManager) IssueCertificate(c *gin.Context) {
	var req struct {
		Subject string   `json:"subject" binding:"required"`
		Usage   []string `json:"usage"`
		DNSNames []string `json:"dns_names"`
		IPAddresses []string `json:"ip_addresses"`
	}

	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(400, gin.H{"error": err.Error()})
		return
	}

	cert, err := cm.generateCertificate(req.Subject, req.Usage, req.DNSNames, req.IPAddresses)
	if err != nil {
		c.JSON(500, gin.H{"error": err.Error()})
		return
	}

	c.JSON(201, gin.H{
		"subject":    cert.Subject,
		"expires_at": cert.ExpiresAt,
		"created_at": cert.CreatedAt,
		"usage":      cert.Usage,
	})
}

func (cm *CertificateManager) generateCertificate(subject string, usage, dnsNames, ipAddresses []string) (*CertificateEntry, error) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	// Generate private key
	privKey, err := rsa.GenerateKey(rand.Reader, cm.config.KeySize)
	if err != nil {
		return nil, fmt.Errorf("failed to generate private key: %w", err)
	}

	// Create certificate template
	template := x509.Certificate{
		SerialNumber: big.NewInt(time.Now().Unix()),
		Subject: pkix.Name{
			CommonName:   subject,
			Organization: []string{"AtlasMesh Fleet OS"},
		},
		NotBefore:    time.Now(),
		NotAfter:     time.Now().Add(cm.config.CertValidity),
		KeyUsage:     x509.KeyUsageKeyEncipherment | x509.KeyUsageDigitalSignature,
		ExtKeyUsage:  []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth, x509.ExtKeyUsageClientAuth},
	}

	// Add DNS names and IP addresses
	for _, dns := range dnsNames {
		template.DNSNames = append(template.DNSNames, dns)
	}

	// Create certificate
	certDER, err := x509.CreateCertificate(rand.Reader, &template, cm.caCert, &privKey.PublicKey, cm.caKey)
	if err != nil {
		return nil, fmt.Errorf("failed to create certificate: %w", err)
	}

	cert, err := x509.ParseCertificate(certDER)
	if err != nil {
		return nil, fmt.Errorf("failed to parse certificate: %w", err)
	}

	// Convert to PEM
	certPEM := pem.EncodeToMemory(&pem.Block{Type: "CERTIFICATE", Bytes: certDER})
	keyPEM := pem.EncodeToMemory(&pem.Block{Type: "RSA PRIVATE KEY", Bytes: x509.MarshalPKCS1PrivateKey(privKey)})

	entry := &CertificateEntry{
		Certificate: cert,
		PrivateKey:  privKey,
		PEMCert:     certPEM,
		PEMKey:      keyPEM,
		ExpiresAt:   cert.NotAfter,
		CreatedAt:   time.Now(),
		Subject:     subject,
		Usage:       usage,
	}

	cm.certificates[subject] = entry

	// Save to disk
	if err := cm.saveCertificateToDisk(subject, entry); err != nil {
		log.Printf("Warning: failed to save certificate to disk: %v", err)
	}

	log.Printf("Issued certificate for subject: %s", subject)
	return entry, nil
}

func (cm *CertificateManager) saveCertificateToDisk(subject string, entry *CertificateEntry) error {
	certPath := filepath.Join(cm.config.CertDir, fmt.Sprintf("%s.crt", subject))
	keyPath := filepath.Join(cm.config.CertDir, fmt.Sprintf("%s.key", subject))

	if err := os.WriteFile(certPath, entry.PEMCert, 0644); err != nil {
		return err
	}

	if err := os.WriteFile(keyPath, entry.PEMKey, 0600); err != nil {
		return err
	}

	return nil
}

func (cm *CertificateManager) GetCertificate(c *gin.Context) {
	subject := c.Param("subject")
	
	cm.mu.RLock()
	cert, exists := cm.certificates[subject]
	cm.mu.RUnlock()

	if !exists {
		c.JSON(404, gin.H{"error": "Certificate not found"})
		return
	}

	c.JSON(200, gin.H{
		"subject":     cert.Subject,
		"expires_at":  cert.ExpiresAt,
		"created_at":  cert.CreatedAt,
		"usage":       cert.Usage,
		"certificate": string(cert.PEMCert),
	})
}

func (cm *CertificateManager) RevokeCertificate(c *gin.Context) {
	subject := c.Param("subject")
	
	cm.mu.Lock()
	delete(cm.certificates, subject)
	cm.mu.Unlock()

	c.JSON(200, gin.H{"status": "revoked"})
}

func (cm *CertificateManager) RotateCertificate(c *gin.Context) {
	subject := c.Param("subject")
	
	cm.mu.RLock()
	oldCert, exists := cm.certificates[subject]
	cm.mu.RUnlock()

	if !exists {
		c.JSON(404, gin.H{"error": "Certificate not found"})
		return
	}

	// Generate new certificate
	newCert, err := cm.generateCertificate(subject, oldCert.Usage, nil, nil)
	if err != nil {
		c.JSON(500, gin.H{"error": err.Error()})
		return
	}

	c.JSON(200, gin.H{
		"status":     "rotated",
		"subject":    newCert.Subject,
		"expires_at": newCert.ExpiresAt,
	})
}

func (cm *CertificateManager) GetCACertificate(c *gin.Context) {
	caPEM := pem.EncodeToMemory(&pem.Block{Type: "CERTIFICATE", Bytes: cm.caCert.Raw})
	
	c.JSON(200, gin.H{
		"certificate": string(caPEM),
		"expires_at":  cm.caCert.NotAfter,
	})
}

func (cm *CertificateManager) ListCertificates(c *gin.Context) {
	cm.mu.RLock()
	defer cm.mu.RUnlock()

	certs := make([]map[string]interface{}, 0, len(cm.certificates))
	for _, cert := range cm.certificates {
		certs = append(certs, map[string]interface{}{
			"subject":    cert.Subject,
			"expires_at": cert.ExpiresAt,
			"created_at": cert.CreatedAt,
			"usage":      cert.Usage,
		})
	}

	c.JSON(200, gin.H{"certificates": certs})
}

func (cm *CertificateManager) StartRotationMonitor() {
	if !cm.config.AutoRotate {
		return
	}

	ticker := time.NewTicker(24 * time.Hour) // Check daily
	defer ticker.Stop()

	log.Println("Started certificate rotation monitor")

	for {
		select {
		case <-ticker.C:
			cm.checkAndRotateCertificates()
		}
	}
}

func (cm *CertificateManager) checkAndRotateCertificates() {
	cm.mu.RLock()
	toRotate := make([]string, 0)
	
	for subject, cert := range cm.certificates {
		if time.Until(cert.ExpiresAt) < cm.config.RotationWindow {
			toRotate = append(toRotate, subject)
		}
	}
	cm.mu.RUnlock()

	for _, subject := range toRotate {
		log.Printf("Auto-rotating certificate for subject: %s", subject)
		oldCert := cm.certificates[subject]
		if _, err := cm.generateCertificate(subject, oldCert.Usage, nil, nil); err != nil {
			log.Printf("Failed to rotate certificate for %s: %v", subject, err)
		}
	}
}

func getEnv(key, defaultValue string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return defaultValue
}
