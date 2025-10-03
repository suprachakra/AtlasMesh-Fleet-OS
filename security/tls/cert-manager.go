package tls

import (
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"fmt"
	"math/big"
	"net"
	"sync"
	"time"

	"go.uber.org/zap"
)

// CertificateManager handles TLS certificate generation, rotation, and management
type CertificateManager struct {
	logger     *zap.Logger
	config     Config
	caCert     *x509.Certificate
	caKey      *rsa.PrivateKey
	
	// Certificate storage
	certificates map[string]*CertificateInfo
	certMutex    sync.RWMutex
	
	// Renewal tracking
	renewalQueue chan string
	stopChan     chan struct{}
}

// Config holds certificate manager configuration
type Config struct {
	// CA Configuration
	CAKeySize         int           `yaml:"ca_key_size" default:"4096"`
	CACertValidityDays int          `yaml:"ca_cert_validity_days" default:"3650"` // 10 years
	
	// Service Certificate Configuration
	ServiceKeySize         int           `yaml:"service_key_size" default:"2048"`
	ServiceCertValidityDays int          `yaml:"service_cert_validity_days" default:"90"`
	
	// Renewal Configuration
	RenewalThresholdDays int           `yaml:"renewal_threshold_days" default:"30"`
	RenewalCheckInterval time.Duration `yaml:"renewal_check_interval" default:"24h"`
	
	// Certificate Storage
	CertStorePath string `yaml:"cert_store_path" default:"/etc/atlasmesh/certs"`
	
	// SPIFFE Configuration
	SPIFFETrustDomain string `yaml:"spiffe_trust_domain" default:"atlasmesh.local"`
	
	// Organization Information
	Organization       string `yaml:"organization" default:"AtlasMesh"`
	OrganizationalUnit string `yaml:"organizational_unit" default:"Fleet OS"`
	Country            string `yaml:"country" default:"US"`
	Province           string `yaml:"province" default:"CA"`
	Locality           string `yaml:"locality" default:"San Francisco"`
}

// CertificateInfo holds certificate and key information
type CertificateInfo struct {
	Certificate *x509.Certificate
	PrivateKey  *rsa.PrivateKey
	CertPEM     []byte
	KeyPEM      []byte
	CreatedAt   time.Time
	ExpiresAt   time.Time
	ServiceName string
	SPIFFEID    string
}

// NewCertificateManager creates a new certificate manager
func NewCertificateManager(config Config, logger *zap.Logger) (*CertificateManager, error) {
	cm := &CertificateManager{
		logger:       logger,
		config:       config,
		certificates: make(map[string]*CertificateInfo),
		renewalQueue: make(chan string, 100),
		stopChan:     make(chan struct{}),
	}
	
	// Initialize or load CA certificate
	if err := cm.initializeCA(); err != nil {
		return nil, fmt.Errorf("failed to initialize CA: %w", err)
	}
	
	// Start renewal worker
	go cm.renewalWorker()
	
	// Start renewal checker
	go cm.renewalChecker()
	
	logger.Info("Certificate manager initialized successfully")
	return cm, nil
}

// initializeCA initializes or loads the Certificate Authority
func (cm *CertificateManager) initializeCA() error {
	// Try to load existing CA certificate and key
	caCertPEM, caKeyPEM, err := cm.loadCAFromStorage()
	if err == nil {
		// Parse existing CA certificate
		caCertBlock, _ := pem.Decode(caCertPEM)
		if caCertBlock == nil {
			return fmt.Errorf("failed to decode CA certificate PEM")
		}
		
		caCert, err := x509.ParseCertificate(caCertBlock.Bytes)
		if err != nil {
			return fmt.Errorf("failed to parse CA certificate: %w", err)
		}
		
		// Parse existing CA private key
		caKeyBlock, _ := pem.Decode(caKeyPEM)
		if caKeyBlock == nil {
			return fmt.Errorf("failed to decode CA private key PEM")
		}
		
		caKey, err := x509.ParsePKCS1PrivateKey(caKeyBlock.Bytes)
		if err != nil {
			return fmt.Errorf("failed to parse CA private key: %w", err)
		}
		
		cm.caCert = caCert
		cm.caKey = caKey
		
		cm.logger.Info("Loaded existing CA certificate",
			zap.String("subject", caCert.Subject.String()),
			zap.Time("expires", caCert.NotAfter),
		)
		
		return nil
	}
	
	// Generate new CA certificate and key
	cm.logger.Info("Generating new CA certificate")
	
	// Generate CA private key
	caKey, err := rsa.GenerateKey(rand.Reader, cm.config.CAKeySize)
	if err != nil {
		return fmt.Errorf("failed to generate CA private key: %w", err)
	}
	
	// Create CA certificate template
	caTemplate := &x509.Certificate{
		SerialNumber: big.NewInt(1),
		Subject: pkix.Name{
			Organization:       []string{cm.config.Organization},
			OrganizationalUnit: []string{cm.config.OrganizationalUnit + " CA"},
			Country:            []string{cm.config.Country},
			Province:           []string{cm.config.Province},
			Locality:           []string{cm.config.Locality},
			CommonName:         "AtlasMesh Fleet OS Root CA",
		},
		NotBefore:             time.Now(),
		NotAfter:              time.Now().AddDate(0, 0, cm.config.CACertValidityDays),
		KeyUsage:              x509.KeyUsageKeyEncipherment | x509.KeyUsageDigitalSignature | x509.KeyUsageCertSign,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth, x509.ExtKeyUsageClientAuth},
		BasicConstraintsValid: true,
		IsCA:                  true,
		MaxPathLen:            2,
	}
	
	// Create CA certificate
	caCertDER, err := x509.CreateCertificate(rand.Reader, caTemplate, caTemplate, &caKey.PublicKey, caKey)
	if err != nil {
		return fmt.Errorf("failed to create CA certificate: %w", err)
	}
	
	// Parse the created certificate
	caCert, err := x509.ParseCertificate(caCertDER)
	if err != nil {
		return fmt.Errorf("failed to parse created CA certificate: %w", err)
	}
	
	cm.caCert = caCert
	cm.caKey = caKey
	
	// Save CA certificate and key to storage
	if err := cm.saveCAToStorage(caCert, caKey); err != nil {
		cm.logger.Warn("Failed to save CA to storage", zap.Error(err))
	}
	
	cm.logger.Info("Generated new CA certificate",
		zap.String("subject", caCert.Subject.String()),
		zap.Time("expires", caCert.NotAfter),
	)
	
	return nil
}

// GenerateServiceCertificate generates a new certificate for a service
func (cm *CertificateManager) GenerateServiceCertificate(serviceName string, dnsNames []string, ipAddresses []net.IP) (*CertificateInfo, error) {
	cm.logger.Info("Generating service certificate",
		zap.String("service", serviceName),
		zap.Strings("dnsNames", dnsNames),
	)
	
	// Generate service private key
	serviceKey, err := rsa.GenerateKey(rand.Reader, cm.config.ServiceKeySize)
	if err != nil {
		return nil, fmt.Errorf("failed to generate service private key: %w", err)
	}
	
	// Create SPIFFE ID
	spiffeID := fmt.Sprintf("spiffe://%s/service/%s", cm.config.SPIFFETrustDomain, serviceName)
	
	// Create certificate template
	template := &x509.Certificate{
		SerialNumber: big.NewInt(time.Now().UnixNano()),
		Subject: pkix.Name{
			Organization:       []string{cm.config.Organization},
			OrganizationalUnit: []string{cm.config.OrganizationalUnit},
			Country:            []string{cm.config.Country},
			Province:           []string{cm.config.Province},
			Locality:           []string{cm.config.Locality},
			CommonName:         serviceName,
		},
		NotBefore:    time.Now(),
		NotAfter:     time.Now().AddDate(0, 0, cm.config.ServiceCertValidityDays),
		KeyUsage:     x509.KeyUsageKeyEncipherment | x509.KeyUsageDigitalSignature,
		ExtKeyUsage:  []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth, x509.ExtKeyUsageClientAuth},
		DNSNames:     dnsNames,
		IPAddresses:  ipAddresses,
	}
	
	// Add SPIFFE ID as SAN URI
	template.URIs = []*net.URL{{Scheme: "spiffe", Host: cm.config.SPIFFETrustDomain, Path: "/service/" + serviceName}}
	
	// Create certificate
	certDER, err := x509.CreateCertificate(rand.Reader, template, cm.caCert, &serviceKey.PublicKey, cm.caKey)
	if err != nil {
		return nil, fmt.Errorf("failed to create service certificate: %w", err)
	}
	
	// Parse the created certificate
	cert, err := x509.ParseCertificate(certDER)
	if err != nil {
		return nil, fmt.Errorf("failed to parse created certificate: %w", err)
	}
	
	// Encode certificate to PEM
	certPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "CERTIFICATE",
		Bytes: certDER,
	})
	
	// Encode private key to PEM
	keyPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "RSA PRIVATE KEY",
		Bytes: x509.MarshalPKCS1PrivateKey(serviceKey),
	})
	
	// Create certificate info
	certInfo := &CertificateInfo{
		Certificate: cert,
		PrivateKey:  serviceKey,
		CertPEM:     certPEM,
		KeyPEM:      keyPEM,
		CreatedAt:   cert.NotBefore,
		ExpiresAt:   cert.NotAfter,
		ServiceName: serviceName,
		SPIFFEID:    spiffeID,
	}
	
	// Store certificate
	cm.certMutex.Lock()
	cm.certificates[serviceName] = certInfo
	cm.certMutex.Unlock()
	
	// Save certificate to storage
	if err := cm.saveCertificateToStorage(serviceName, certInfo); err != nil {
		cm.logger.Warn("Failed to save certificate to storage", zap.Error(err))
	}
	
	cm.logger.Info("Generated service certificate",
		zap.String("service", serviceName),
		zap.String("spiffeID", spiffeID),
		zap.Time("expires", cert.NotAfter),
	)
	
	return certInfo, nil
}

// GetServiceCertificate retrieves a service certificate
func (cm *CertificateManager) GetServiceCertificate(serviceName string) (*CertificateInfo, error) {
	cm.certMutex.RLock()
	certInfo, exists := cm.certificates[serviceName]
	cm.certMutex.RUnlock()
	
	if !exists {
		return nil, fmt.Errorf("certificate not found for service: %s", serviceName)
	}
	
	// Check if certificate needs renewal
	if cm.needsRenewal(certInfo) {
		cm.logger.Info("Certificate needs renewal", zap.String("service", serviceName))
		select {
		case cm.renewalQueue <- serviceName:
		default:
			cm.logger.Warn("Renewal queue full, skipping renewal request", zap.String("service", serviceName))
		}
	}
	
	return certInfo, nil
}

// GetTLSConfig returns a TLS configuration for a service
func (cm *CertificateManager) GetTLSConfig(serviceName string) (*tls.Config, error) {
	certInfo, err := cm.GetServiceCertificate(serviceName)
	if err != nil {
		return nil, err
	}
	
	// Create TLS certificate
	tlsCert, err := tls.X509KeyPair(certInfo.CertPEM, certInfo.KeyPEM)
	if err != nil {
		return nil, fmt.Errorf("failed to create TLS certificate: %w", err)
	}
	
	// Create CA certificate pool
	caCertPool := x509.NewCertPool()
	caCertPEM := pem.EncodeToMemory(&pem.Block{
		Type:  "CERTIFICATE",
		Bytes: cm.caCert.Raw,
	})
	caCertPool.AppendCertsFromPEM(caCertPEM)
	
	// Create TLS configuration
	tlsConfig := &tls.Config{
		Certificates: []tls.Certificate{tlsCert},
		ClientAuth:   tls.RequireAndVerifyClientCert,
		ClientCAs:    caCertPool,
		RootCAs:      caCertPool,
		MinVersion:   tls.VersionTLS12,
		CipherSuites: []uint16{
			tls.TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384,
			tls.TLS_ECDHE_RSA_WITH_CHACHA20_POLY1305,
			tls.TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384,
			tls.TLS_ECDHE_ECDSA_WITH_CHACHA20_POLY1305,
		},
		PreferServerCipherSuites: true,
	}
	
	return tlsConfig, nil
}

// GetCACertificate returns the CA certificate in PEM format
func (cm *CertificateManager) GetCACertificate() []byte {
	return pem.EncodeToMemory(&pem.Block{
		Type:  "CERTIFICATE",
		Bytes: cm.caCert.Raw,
	})
}

// needsRenewal checks if a certificate needs renewal
func (cm *CertificateManager) needsRenewal(certInfo *CertificateInfo) bool {
	renewalThreshold := time.Duration(cm.config.RenewalThresholdDays) * 24 * time.Hour
	return time.Until(certInfo.ExpiresAt) < renewalThreshold
}

// renewalWorker processes certificate renewal requests
func (cm *CertificateManager) renewalWorker() {
	for {
		select {
		case serviceName := <-cm.renewalQueue:
			if err := cm.renewServiceCertificate(serviceName); err != nil {
				cm.logger.Error("Failed to renew certificate",
					zap.String("service", serviceName),
					zap.Error(err),
				)
			}
		case <-cm.stopChan:
			return
		}
	}
}

// renewalChecker periodically checks for certificates that need renewal
func (cm *CertificateManager) renewalChecker() {
	ticker := time.NewTicker(cm.config.RenewalCheckInterval)
	defer ticker.Stop()
	
	for {
		select {
		case <-ticker.C:
			cm.checkForRenewals()
		case <-cm.stopChan:
			return
		}
	}
}

// checkForRenewals checks all certificates for renewal needs
func (cm *CertificateManager) checkForRenewals() {
	cm.certMutex.RLock()
	servicesToRenew := make([]string, 0)
	
	for serviceName, certInfo := range cm.certificates {
		if cm.needsRenewal(certInfo) {
			servicesToRenew = append(servicesToRenew, serviceName)
		}
	}
	cm.certMutex.RUnlock()
	
	// Queue renewals
	for _, serviceName := range servicesToRenew {
		select {
		case cm.renewalQueue <- serviceName:
			cm.logger.Info("Queued certificate for renewal", zap.String("service", serviceName))
		default:
			cm.logger.Warn("Renewal queue full, skipping renewal", zap.String("service", serviceName))
		}
	}
}

// renewServiceCertificate renews a service certificate
func (cm *CertificateManager) renewServiceCertificate(serviceName string) error {
	cm.certMutex.RLock()
	oldCertInfo, exists := cm.certificates[serviceName]
	cm.certMutex.RUnlock()
	
	if !exists {
		return fmt.Errorf("certificate not found for service: %s", serviceName)
	}
	
	// Extract DNS names and IP addresses from old certificate
	dnsNames := oldCertInfo.Certificate.DNSNames
	ipAddresses := oldCertInfo.Certificate.IPAddresses
	
	// Generate new certificate
	newCertInfo, err := cm.GenerateServiceCertificate(serviceName, dnsNames, ipAddresses)
	if err != nil {
		return fmt.Errorf("failed to generate new certificate: %w", err)
	}
	
	cm.logger.Info("Successfully renewed certificate",
		zap.String("service", serviceName),
		zap.Time("oldExpiry", oldCertInfo.ExpiresAt),
		zap.Time("newExpiry", newCertInfo.ExpiresAt),
	)
	
	return nil
}

// Stop stops the certificate manager
func (cm *CertificateManager) Stop() {
	close(cm.stopChan)
	cm.logger.Info("Certificate manager stopped")
}

// Placeholder methods for storage operations
func (cm *CertificateManager) loadCAFromStorage() ([]byte, []byte, error) {
	// This would load CA certificate and key from persistent storage
	// For now, return error to trigger generation
	return nil, nil, fmt.Errorf("CA not found in storage")
}

func (cm *CertificateManager) saveCAToStorage(cert *x509.Certificate, key *rsa.PrivateKey) error {
	// This would save CA certificate and key to persistent storage
	cm.logger.Debug("Saving CA to storage (placeholder)")
	return nil
}

func (cm *CertificateManager) saveCertificateToStorage(serviceName string, certInfo *CertificateInfo) error {
	// This would save service certificate to persistent storage
	cm.logger.Debug("Saving certificate to storage (placeholder)", zap.String("service", serviceName))
	return nil
}
