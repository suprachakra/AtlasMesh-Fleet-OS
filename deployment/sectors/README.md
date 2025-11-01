# Sector-Specific Deployments

This directory contains Kubernetes deployment manifests and scripts for deploying AtlasMesh Fleet OS across different sectors.

## Overview

Each sector has its own deployment configuration optimized for its specific operational requirements:

- **Logistics** - Warehouse automation with indoor navigation and precision control
- **Defense** - Military UGV operations with encrypted communications and tactical features
- **Mining** - Heavy equipment automation with production optimization and safety features
- **Ride-Hail** - Urban passenger transport with real-time dispatch and passenger safety

## Directory Structure

```
deployments/sectors/
├── deploy.sh                    # Main deployment script
├── README.md                    # This file
├── logistics/                   # Logistics sector deployments
│   └── k8s/
│       ├── namespace.yaml       # Namespace and resource quotas
│       ├── configmap.yaml       # Configuration data
│       ├── secrets.yaml         # Secrets and credentials
│       ├── deployments.yaml     # Application deployments
│       └── services.yaml        # Service definitions
├── defense/                     # Defense sector deployments
│   └── k8s/
│       ├── namespace.yaml
│       ├── configmap.yaml
│       ├── secrets.yaml
│       ├── deployments.yaml
│       └── services.yaml
├── mining/                      # Mining sector deployments
│   └── k8s/
│       ├── namespace.yaml
│       ├── configmap.yaml
│       ├── secrets.yaml
│       ├── deployments.yaml
│       └── services.yaml
└── ride_hail/                   # Ride-Hail sector deployments
    └── k8s/
        ├── namespace.yaml
        ├── configmap.yaml
        ├── secrets.yaml
        ├── deployments.yaml
        └── services.yaml
```

## Prerequisites

### Kubernetes Cluster
- Kubernetes 1.20 or later
- kubectl configured and connected to cluster
- Sufficient resources for the selected sector

### Required Resources by Sector

#### Logistics
- CPU: 4 cores (requests) / 8 cores (limits)
- Memory: 8Gi (requests) / 16Gi (limits)
- Storage: 10 PVCs
- Services: 20

#### Defense
- CPU: 8 cores (requests) / 16 cores (limits)
- Memory: 16Gi (requests) / 32Gi (limits)
- Storage: 20 PVCs
- Services: 30
- Security: Classified level access

#### Mining
- CPU: 12 cores (requests) / 24 cores (limits)
- Memory: 24Gi (requests) / 48Gi (limits)
- Storage: 30 PVCs
- Services: 40

#### Ride-Hail
- CPU: 16 cores (requests) / 32 cores (limits)
- Memory: 32Gi (requests) / 64Gi (limits)
- Storage: 50 PVCs
- Services: 60

## Quick Start

### Deploy a Specific Sector

```bash
# Deploy Logistics sector
./deploy.sh -s logistics

# Deploy Defense sector
./deploy.sh -s defense

# Deploy Mining sector
./deploy.sh -s mining

# Deploy Ride-Hail sector
./deploy.sh -s ride_hail
```

### Deploy with Custom Options

```bash
# Deploy with custom namespace
./deploy.sh -s logistics -n my-logistics-namespace

# Deploy to staging environment
./deploy.sh -s defense -e staging

# Perform dry run
./deploy.sh -s mining -d

# Enable verbose output
./deploy.sh -s ride_hail -v
```

## Deployment Script Usage

The `deploy.sh` script provides a comprehensive deployment solution with the following options:

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-s, --sector` | Sector to deploy (logistics, defense, mining, ride_hail) | Required |
| `-e, --environment` | Environment (production, staging, development) | production |
| `-n, --namespace` | Kubernetes namespace | atlasmesh-{sector} |
| `-d, --dry-run` | Perform dry run without applying changes | false |
| `-v, --verbose` | Enable verbose output | false |
| `-h, --help` | Show help message | - |

### Examples

```bash
# Basic deployment
./deploy.sh -s logistics

# Staging deployment with custom namespace
./deploy.sh -s defense -e staging -n defense-staging

# Dry run to validate configuration
./deploy.sh -s mining -d -v

# Production deployment with verbose output
./deploy.sh -s ride_hail -e production -v
```

## Sector-Specific Configurations

### Logistics Sector
- **Focus**: Warehouse automation and indoor operations
- **Key Features**: Indoor navigation, pallet detection, battery management
- **Vehicle Types**: Forklifts, tuggers, pallet trucks, reach trucks
- **Max Fleet Size**: 50 vehicles
- **Precision**: High precision required for warehouse operations

### Defense Sector
- **Focus**: Military UGV operations and tactical missions
- **Key Features**: Encrypted communications, threat detection, GPS-denied navigation
- **Vehicle Types**: UGVs, tactical vehicles, reconnaissance vehicles
- **Max Fleet Size**: 20 vehicles
- **Security**: Classified level with encrypted communications

### Mining Sector
- **Focus**: Heavy equipment automation and production optimization
- **Key Features**: Haul cycle optimization, blast zone safety, heavy vehicle control
- **Vehicle Types**: Haul trucks, bulldozers, excavators, loaders
- **Max Fleet Size**: 100 vehicles
- **Production**: Focus on production targets and efficiency

### Ride-Hail Sector
- **Focus**: Urban passenger transport and real-time dispatch
- **Key Features**: Passenger safety, urban navigation, dynamic pricing
- **Vehicle Types**: Robotaxis, shuttles, autonomous vehicles
- **Max Fleet Size**: 200 vehicles
- **Operations**: Real-time dispatch and passenger experience

## Security Considerations

### Defense Sector
- All communications are encrypted
- Classified security level required
- Additional encryption keys for sensitive data
- Security context restrictions on containers
- Non-root user execution

### All Sectors
- TLS certificates for secure communications
- Secrets management for credentials
- Resource quotas and limits
- Network policies (recommended)

## Monitoring and Observability

Each deployment includes:

- **Prometheus Metrics**: Service and application metrics
- **Logging**: Structured logging with Zap
- **Health Checks**: Readiness and liveness probes
- **Resource Monitoring**: CPU, memory, and storage usage

## Troubleshooting

### Common Issues

1. **Insufficient Resources**
   - Check cluster capacity
   - Adjust resource quotas
   - Scale cluster if needed

2. **Image Pull Errors**
   - Verify image availability
   - Check image pull secrets
   - Ensure proper registry access

3. **Configuration Errors**
   - Validate YAML syntax
   - Check ConfigMap values
   - Verify secret encoding

4. **Network Issues**
   - Check service definitions
   - Verify port configurations
   - Test network connectivity

### Debug Commands

```bash
# Check pod status
kubectl get pods -n atlasmesh-{sector}

# View pod logs
kubectl logs -f deployment/{service-name} -n atlasmesh-{sector}

# Describe resources
kubectl describe deployment/{service-name} -n atlasmesh-{sector}

# Check events
kubectl get events -n atlasmesh-{sector} --sort-by='.lastTimestamp'
```

## Customization

### Adding New Sectors

1. Create sector directory: `deployments/sectors/{new-sector}/`
2. Create Kubernetes manifests in `k8s/` subdirectory
3. Update deployment script to include new sector
4. Add sector-specific configuration

### Modifying Existing Deployments

1. Edit the appropriate YAML files in the sector directory
2. Update configuration values as needed
3. Test with dry run: `./deploy.sh -s {sector} -d`
4. Deploy changes: `./deploy.sh -s {sector}`

## Best Practices

1. **Always use dry run first** to validate configurations
2. **Test in staging environment** before production deployment
3. **Monitor resource usage** and adjust quotas as needed
4. **Keep secrets secure** and rotate regularly
5. **Use version control** for all deployment files
6. **Document customizations** and changes

## Support

For issues and questions:

1. Check the troubleshooting section above
2. Review Kubernetes documentation
3. Check AtlasMesh Fleet OS documentation
4. Contact the development team

## License

This deployment configuration is part of AtlasMesh Fleet OS and follows the same license terms.

