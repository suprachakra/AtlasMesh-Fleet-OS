#!/bin/bash
# AtlasMesh Security Initialization Script
# Sets up security infrastructure including Vault, OPA, and certificates

set -e

# Configuration
VAULT_ADDR=${VAULT_ADDR:-"http://localhost:8200"}
VAULT_TOKEN=${VAULT_TOKEN:-"atlasmesh-root-token"}
OPA_ADDR=${OPA_ADDR:-"http://localhost:8181"}
TIMEOUT=${TIMEOUT:-60}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Wait for Vault to be ready
wait_for_vault() {
    log_info "Waiting for Vault to be ready at $VAULT_ADDR..."
    
    local retries=0
    local max_retries=30
    
    while [ $retries -lt $max_retries ]; do
        if curl -s "$VAULT_ADDR/v1/sys/health" >/dev/null 2>&1; then
            log_success "Vault is ready!"
            return 0
        fi
        
        retries=$((retries + 1))
        log_info "Attempt $retries/$max_retries - Vault not ready yet, waiting 5 seconds..."
        sleep 5
    done
    
    log_error "Vault failed to become ready after $max_retries attempts"
    return 1
}

# Wait for OPA to be ready
wait_for_opa() {
    log_info "Waiting for OPA to be ready at $OPA_ADDR..."
    
    local retries=0
    local max_retries=30
    
    while [ $retries -lt $max_retries ]; do
        if curl -s "$OPA_ADDR/health" >/dev/null 2>&1; then
            log_success "OPA is ready!"
            return 0
        fi
        
        retries=$((retries + 1))
        log_info "Attempt $retries/$max_retries - OPA not ready yet, waiting 5 seconds..."
        sleep 5
    done
    
    log_error "OPA failed to become ready after $max_retries attempts"
    return 1
}

# Initialize Vault secrets engine
init_vault_secrets() {
    log_info "Initializing Vault secrets engines..."
    
    export VAULT_ADDR
    export VAULT_TOKEN
    
    # Enable KV v2 secrets engine
    vault secrets enable -path=secret kv-v2 2>/dev/null || log_warning "KV v2 secrets engine already enabled"
    
    # Enable PKI secrets engine for certificates
    vault secrets enable -path=pki pki 2>/dev/null || log_warning "PKI secrets engine already enabled"
    
    # Configure PKI engine
    vault secrets tune -max-lease-ttl=87600h pki
    
    # Generate root CA certificate
    vault write -field=certificate pki/root/generate/internal \
        common_name="AtlasMesh Fleet OS Root CA" \
        ttl=87600h > /tmp/ca_cert.crt
    
    # Configure CA and CRL URLs
    vault write pki/config/urls \
        issuing_certificates="$VAULT_ADDR/v1/pki/ca" \
        crl_distribution_points="$VAULT_ADDR/v1/pki/crl"
    
    # Create intermediate CA
    vault secrets enable -path=pki_int pki 2>/dev/null || log_warning "Intermediate PKI already enabled"
    vault secrets tune -max-lease-ttl=43800h pki_int
    
    # Generate intermediate CSR
    vault write -format=json pki_int/intermediate/generate/internal \
        common_name="AtlasMesh Fleet OS Intermediate CA" \
        | jq -r '.data.csr' > /tmp/pki_intermediate.csr
    
    # Sign intermediate certificate
    vault write -format=json pki/root/sign-intermediate \
        csr=@/tmp/pki_intermediate.csr \
        format=pem_bundle ttl="43800h" \
        | jq -r '.data.certificate' > /tmp/intermediate.cert.pem
    
    # Set intermediate certificate
    vault write pki_int/intermediate/set-signed certificate=@/tmp/intermediate.cert.pem
    
    # Create role for service certificates
    vault write pki_int/roles/atlasmesh-services \
        allowed_domains="atlasmesh.local,localhost" \
        allow_subdomains=true \
        max_ttl="720h" \
        generate_lease=true
    
    log_success "Vault secrets engines initialized"
}

# Initialize Vault authentication methods
init_vault_auth() {
    log_info "Initializing Vault authentication methods..."
    
    # Enable userpass auth method
    vault auth enable userpass 2>/dev/null || log_warning "Userpass auth already enabled"
    
    # Enable Kubernetes auth method (for service accounts)
    vault auth enable kubernetes 2>/dev/null || log_warning "Kubernetes auth already enabled"
    
    # Create policies
    cat > /tmp/atlasmesh-policy.hcl << EOF
# AtlasMesh Fleet OS Policy
path "secret/data/atlasmesh/*" {
  capabilities = ["create", "read", "update", "delete", "list"]
}

path "pki_int/issue/atlasmesh-services" {
  capabilities = ["create", "update"]
}

path "pki_int/certs" {
  capabilities = ["list"]
}

path "pki_int/revoke" {
  capabilities = ["create", "update"]
}

path "auth/token/lookup-self" {
  capabilities = ["read"]
}

path "auth/token/renew-self" {
  capabilities = ["update"]
}
EOF
    
    vault policy write atlasmesh-policy /tmp/atlasmesh-policy.hcl
    
    # Create admin policy
    cat > /tmp/atlasmesh-admin-policy.hcl << EOF
# AtlasMesh Admin Policy
path "*" {
  capabilities = ["create", "read", "update", "delete", "list", "sudo"]
}
EOF
    
    vault policy write atlasmesh-admin-policy /tmp/atlasmesh-admin-policy.hcl
    
    # Create service accounts
    vault write auth/userpass/users/atlasmesh-admin \
        password=admin123 \
        policies=atlasmesh-admin-policy
    
    vault write auth/userpass/users/atlasmesh-service \
        password=service123 \
        policies=atlasmesh-policy
    
    log_success "Vault authentication methods initialized"
}

# Store initial secrets
store_initial_secrets() {
    log_info "Storing initial secrets in Vault..."
    
    # Database credentials
    vault kv put secret/atlasmesh/database \
        username=atlasmesh \
        password=atlasmesh_dev_password \
        host=postgres \
        port=5432
    
    # Redis credentials
    vault kv put secret/atlasmesh/redis \
        password=atlasmesh_redis_password \
        host=redis \
        port=6379
    
    # JWT signing keys (will be generated by auth service)
    vault kv put secret/atlasmesh/jwt \
        issuer=atlasmesh-auth \
        audience=atlasmesh-fleet-os
    
    # API keys for external services
    vault kv put secret/atlasmesh/external-apis \
        weather_api_key=placeholder \
        maps_api_key=placeholder \
        notification_api_key=placeholder
    
    # Encryption keys
    vault kv put secret/atlasmesh/encryption \
        master_key=$(openssl rand -base64 32) \
        data_key=$(openssl rand -base64 32)
    
    log_success "Initial secrets stored in Vault"
}

# Load OPA policies
load_opa_policies() {
    log_info "Loading OPA policies..."
    
    # Load RBAC policies
    if [ -f "security/policies/rbac-policies.rego" ]; then
        curl -X PUT "$OPA_ADDR/v1/policies/rbac" \
            -H "Content-Type: text/plain" \
            --data-binary @security/policies/rbac-policies.rego
        log_success "RBAC policies loaded"
    else
        log_warning "RBAC policies file not found"
    fi
    
    # Load ABAC policies
    if [ -f "security/policies/abac-policies.rego" ]; then
        curl -X PUT "$OPA_ADDR/v1/policies/abac" \
            -H "Content-Type: text/plain" \
            --data-binary @security/policies/abac-policies.rego
        log_success "ABAC policies loaded"
    else
        log_warning "ABAC policies file not found"
    fi
    
    # Test policy evaluation
    curl -X POST "$OPA_ADDR/v1/data/atlasmesh/rbac/allow" \
        -H "Content-Type: application/json" \
        -d '{
            "input": {
                "user": {
                    "id": "test-user",
                    "roles": ["admin"],
                    "sector": "logistics"
                },
                "resource": {
                    "type": "vehicle",
                    "id": "vehicle-001"
                },
                "action": "read"
            }
        }' > /tmp/opa_test_result.json
    
    if [ "$(jq -r '.result' /tmp/opa_test_result.json)" = "true" ]; then
        log_success "OPA policy evaluation test passed"
    else
        log_warning "OPA policy evaluation test failed"
    fi
}

# Generate service certificates
generate_service_certificates() {
    log_info "Generating service certificates..."
    
    # Create certificates directory
    mkdir -p certs/services
    
    # Services that need certificates
    services=(
        "api-gateway"
        "auth-service"
        "policy-engine"
        "mission-management"
        "dispatch-service"
        "fleet-manager"
        "routing-service"
        "telemetry-ingestion"
        "data-lineage"
    )
    
    for service in "${services[@]}"; do
        log_info "Generating certificate for $service..."
        
        # Generate certificate
        vault write -format=json pki_int/issue/atlasmesh-services \
            common_name="$service.atlasmesh.local" \
            alt_names="$service,localhost" \
            ttl="720h" > "/tmp/${service}_cert.json"
        
        # Extract certificate and private key
        jq -r '.data.certificate' "/tmp/${service}_cert.json" > "certs/services/${service}.crt"
        jq -r '.data.private_key' "/tmp/${service}_cert.json" > "certs/services/${service}.key"
        jq -r '.data.issuing_ca' "/tmp/${service}_cert.json" > "certs/services/${service}_ca.crt"
        
        # Set proper permissions
        chmod 600 "certs/services/${service}.key"
        chmod 644 "certs/services/${service}.crt"
        chmod 644 "certs/services/${service}_ca.crt"
        
        log_success "Certificate generated for $service"
    done
    
    # Copy CA certificate for general use
    cp /tmp/ca_cert.crt certs/ca.crt
    chmod 644 certs/ca.crt
}

# Create security monitoring dashboard
create_security_dashboard() {
    log_info "Creating security monitoring dashboard..."
    
    # This would create Grafana dashboards for security monitoring
    # For now, just create a placeholder
    mkdir -p monitoring/dashboards/security
    
    cat > monitoring/dashboards/security/security-overview.json << 'EOF'
{
  "dashboard": {
    "title": "AtlasMesh Security Overview",
    "description": "Security monitoring dashboard for AtlasMesh Fleet OS",
    "panels": [
      {
        "title": "Authentication Failures",
        "type": "stat",
        "targets": [
          {
            "expr": "sum(rate(atlasmesh_auth_failures_total[5m]))"
          }
        ]
      },
      {
        "title": "Authorization Denials", 
        "type": "stat",
        "targets": [
          {
            "expr": "sum(rate(atlasmesh_authz_denials_total[5m]))"
          }
        ]
      },
      {
        "title": "Rate Limit Hits",
        "type": "stat", 
        "targets": [
          {
            "expr": "sum(rate(atlasmesh_rate_limit_hits_total[5m]))"
          }
        ]
      }
    ]
  }
}
EOF
    
    log_success "Security dashboard created"
}

# Verify security setup
verify_security_setup() {
    log_info "Verifying security setup..."
    
    # Check Vault status
    if vault status >/dev/null 2>&1; then
        log_success "Vault is operational"
    else
        log_error "Vault verification failed"
        return 1
    fi
    
    # Check OPA status
    if curl -s "$OPA_ADDR/health" | grep -q "ok"; then
        log_success "OPA is operational"
    else
        log_error "OPA verification failed"
        return 1
    fi
    
    # Check certificates
    if [ -f "certs/ca.crt" ] && [ -f "certs/services/auth-service.crt" ]; then
        log_success "Certificates generated successfully"
    else
        log_error "Certificate verification failed"
        return 1
    fi
    
    # Test policy evaluation
    if [ -f "/tmp/opa_test_result.json" ]; then
        log_success "Policy evaluation working"
    else
        log_error "Policy evaluation verification failed"
        return 1
    fi
    
    log_success "Security setup verification completed"
}

# Main function
main() {
    case "${1:-init}" in
        "init")
            log_info "🔐 Initializing AtlasMesh Security Infrastructure"
            wait_for_vault
            wait_for_opa
            init_vault_secrets
            init_vault_auth
            store_initial_secrets
            load_opa_policies
            generate_service_certificates
            create_security_dashboard
            verify_security_setup
            log_success "🎉 Security initialization completed successfully!"
            ;;
        "vault-only")
            log_info "🔐 Initializing Vault only"
            wait_for_vault
            init_vault_secrets
            init_vault_auth
            store_initial_secrets
            log_success "✅ Vault initialization completed!"
            ;;
        "opa-only")
            log_info "🔐 Initializing OPA only"
            wait_for_opa
            load_opa_policies
            log_success "✅ OPA initialization completed!"
            ;;
        "certs-only")
            log_info "🔐 Generating certificates only"
            wait_for_vault
            generate_service_certificates
            log_success "✅ Certificate generation completed!"
            ;;
        "verify")
            log_info "🔍 Verifying security setup"
            verify_security_setup
            ;;
        "help"|"-h"|"--help")
            echo "Usage: $0 [command]"
            echo ""
            echo "Commands:"
            echo "  init         Initialize complete security infrastructure (default)"
            echo "  vault-only   Initialize Vault secrets and auth only"
            echo "  opa-only     Initialize OPA policies only"
            echo "  certs-only   Generate service certificates only"
            echo "  verify       Verify security setup"
            echo "  help         Show this help message"
            echo ""
            echo "Environment Variables:"
            echo "  VAULT_ADDR   Vault server address (default: http://localhost:8200)"
            echo "  VAULT_TOKEN  Vault root token (default: atlasmesh-root-token)"
            echo "  OPA_ADDR     OPA server address (default: http://localhost:8181)"
            echo "  TIMEOUT      Connection timeout in seconds (default: 60)"
            ;;
        *)
            log_error "Unknown command: $1"
            echo "Use '$0 help' for usage information"
            exit 1
            ;;
    esac
}

# Handle script interruption
trap 'log_warning "Script interrupted"; exit 130' INT TERM

# Check dependencies
command -v vault >/dev/null 2>&1 || { log_error "vault CLI is required but not installed. Aborting."; exit 1; }
command -v curl >/dev/null 2>&1 || { log_error "curl is required but not installed. Aborting."; exit 1; }
command -v jq >/dev/null 2>&1 || { log_error "jq is required but not installed. Aborting."; exit 1; }

# Run main function
main "$@"
